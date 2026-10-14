#!/usr/bin/env python3
"""
DeepSeek API Client for competition task solving.
Uses OpenAI-compatible API to call deepseek-v4-flash for structured JSON output.
"""

import json
import os
import time
import yaml
import requests
from pathlib import Path

from aic_task.question_source import (
    calculate_generated_answer,
    calculate_generated_evidence,
    extract_result_variables,
)
from aic_task.mapping_contract import (MappingRuleError, resolve_mapping, validate_mapping_interpretation)




class DeepSeekClient:
    """Client for DeepSeek Flash API with retry logic and structured JSON output."""

    def __init__(self, config_path=None):
        self.config = self._load_config(config_path)
        self.api_key = self.config["deepseek"]["api_key"]
        self.base_url = self.config["deepseek"]["base_url"]
        self.model = self.config["deepseek"]["model"]
        self.connect_timeout = self.config["deepseek"]["connect_timeout_sec"]
        self.read_timeout = self.config["deepseek"]["read_timeout_sec"]
        self.max_retries = self.config["deepseek"]["max_retries"]

    def _load_config(self, config_path=None):
        if config_path is None:
            config_path = os.environ.get("AIC_API_CONFIG_PATH") or None
        if config_path is None:
            # Search multiple locations for config (symlink-install safe, security-conscious)
            search_paths = [
                # Resolve symlink-install to this candidate before legacy fixed paths.
                Path(__file__).resolve().parents[2] / "config" / "api_secrets.yaml",
                # 1. Source directory (preferred - stays in source, not in install)
                Path("/home/ros/dev_ws/src/yzbot/aic_task/config/api_secrets.yaml"),
                # 2. Install share directory (for non-symlink installs)
                Path("/home/ros/dev_ws/install/aic_task/share/aic_task/config/api_secrets.yaml"),
                # 3. Relative to this file (legacy fallback)
                Path(__file__).parent.parent.parent / "config" / "api_secrets.yaml",
            ]
            for p in search_paths:
                if p.exists():
                    config_path = p
                    break
            else:
                raise FileNotFoundError(
                    f"api_secrets.yaml not found. Searched: {[str(p) for p in search_paths]}\n"
                    "Copy api_secrets.example.yaml to api_secrets.yaml and set your key."
                )
        with open(config_path, "r") as f:
            return yaml.safe_load(f)

    def _make_request(self, messages, temperature=0.0):
        """Make a single API request."""
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        data = {
            "model": self.model,
            "messages": messages,
            "response_format": {"type": "json_object"},
            "temperature": temperature,
            "max_tokens": 1024,
            "thinking": {"type": "disabled"},
            "stream": False,
        }
        response = requests.post(
            f"{self.base_url}/v1/chat/completions",
            headers=headers,
            json=data,
            timeout=(self.connect_timeout, self.read_timeout),
        )
        return response

    def _model_serves_requests(self):
        """Probe whether the configured model name is actually usable.

        `/models` lists canonical ids only. The live API also accepts aliases
        that are absent from that list: measured 2026-09-23, both
        `deepseek-v4-flash` and `deepseek-chat` return HTTP 200 and are served
        as `deepseek-flash`, while a genuinely bogus name returns HTTP 400
        naming the supported ids.

        Without this probe the periodic health check compared the configured
        name against the `/models` list, declared a working configuration
        broken, and republished `API_ERROR` over `/competition/api_status`
        every 60 s for the whole run -- a topic `start_competition.py` uses for
        its readiness gates (observed in run_20260923_112338). A false negative
        there is worse than a missing check, so send one minimal request and
        trust the HTTP result.
        """
        body = {
            "model": self.model,
            "messages": [{"role": "user", "content": "ping"}],
            "max_tokens": 1,
            "stream": False,
        }
        try:
            response = requests.post(
                f"{self.base_url}/chat/completions",
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=body,
                timeout=(self.connect_timeout, self.read_timeout),
            )
        except requests.exceptions.Timeout:
            return False, "MODEL_PROBE_TIMEOUT"
        except requests.exceptions.ConnectionError:
            return False, "MODEL_PROBE_CONNECTION_ERROR"
        except Exception as exc:
            return False, f"MODEL_PROBE_ERROR: {str(exc)[:160]}"
        if response.status_code == 200:
            return True, None
        return False, f"MODEL_PROBE_HTTP_{response.status_code}"

    def check_available(self):
        """Verify the configured account and model without exposing credentials."""
        headers = {"Authorization": f"Bearer {self.api_key}"}
        try:
            response = requests.get(
                f"{self.base_url}/models",
                headers=headers,
                timeout=(self.connect_timeout, self.read_timeout),
            )
            if response.status_code != 200:
                return False, f"MODEL_LIST_HTTP_{response.status_code}"
            payload = response.json()
            model_ids = {
                str(entry.get("id"))
                for entry in payload.get("data", [])
                if isinstance(entry, dict)
            }
            if self.model not in model_ids:
                # Absent from the canonical list -- it may still be a working
                # alias, so confirm with a real minimal request before failing.
                return self._model_serves_requests()
            return True, None
        except requests.exceptions.Timeout:
            return False, "MODEL_LIST_TIMEOUT"
        except requests.exceptions.ConnectionError:
            return False, "MODEL_LIST_CONNECTION_ERROR"
        except Exception as exc:
            return False, f"MODEL_LIST_ERROR: {str(exc)[:160]}"

    @staticmethod
    def _build_messages(question_text, mapping_rules_text, generator_order=None):
        """Build current-round evidence, formulas and independently resolved mapping."""
        system_prompt = (
            "You solve robot competition questions. Return one JSON object, without markdown. "
            "The current original question and mapping are authoritative. Never reuse another round. "
            "Calculate demands per category before assigning colours and zones. '都' multiplies "
            "the stated demand by the number of people; '和某人一样' copies that person's demand; "
            "'换成' transfers its quantity to the named category; '双倍' doubles the specified "
            "demand; '少一' subtracts one from the specified copied demand. "
            "For each category use exact integer ceiling: count=(total+capacity-1)//capacity. "
            "If a rule compares x/y, compare their computed counts, not the raw demand totals. "
            "Rule clauses can choose colour by variable, threshold, or comparison, and destination "
            "by variable, colour, or threshold. Apply the current clauses independently. "
            "Equal values have no larger/smaller unless the rule explicitly defines a tie. "
            "Return question, mapping_rule, calculation, ordered_tasks, mapping_interpretation. "
            "ordered_tasks has exactly one entry per requested variable in the original order, "
            "with consecutive sequence starting at 1, variable, color (red/blue), count (1..5), "
            "zone (A/B/C). Each count must equal calculation[variable]. "
            "mapping_interpretation must contain variable_to_color and variable_to_zone. "
            "Do not infer zones from count alone: equal counts can have different zones. "
            "The independently checked arithmetic and mapping provided below must agree with "
            "your output. If the question or rules are ambiguous, do not invent an executable plan."
        )

        order_context = str(generator_order or "").strip()
        if not order_context or order_context == str(question_text).strip():
            order_context = "No separate order line; preserve the result-variable order in the question."

        user_prompt = (
            f"CURRENT ROUND QUESTION:\n{question_text}\n\n"
            f"CURRENT ROUND GENERATOR ORDER:\n{order_context}\n\n"
            f"CURRENT ROUND MAPPING RULES:\n{mapping_rules_text}\n\n"
            "Solve only the current round. Follow the generator order when it is explicit. "
            "Apply its mapping after calculating its x/y "
            "values, and output JSON with ordered_tasks, calculation, and metadata."
        )

        evidence = calculate_generated_evidence(question_text)
        if evidence is not None:
            user_prompt += "\nINDEPENDENT ARITHMETIC CHECK:\n" + json.dumps(evidence, ensure_ascii=False)
            try:
                contract = resolve_mapping(mapping_rules_text, evidence["calculation"])
            except MappingRuleError:
                contract = None
            if contract is not None:
                user_prompt += "\nINDEPENDENT MAPPING CHECK:\n" + json.dumps(contract, ensure_ascii=False)

        return [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ]

    def solve_task(
        self,
        question_text,
        mapping_rules_text,
        mode="competition",
        generator_order=None,
    ):
        """
        Send question and mapping rules to DeepSeek, get structured task JSON.

        Args:
            question_text: Raw output from TMSCQtest binary
            mapping_rules_text: Color/zone mapping rules
            mode: retained for caller compatibility; invalid plans fail in both modes

        Returns:
            dict: Structured task with ordered_tasks, or None on failure
            str: Error message if failed
        """
        # Refuse unsupported/ambiguous input before spending an API request.
        calculation = calculate_generated_answer(question_text)
        if calculation is None:
            return None, "UNVERIFIED_QUESTION: original question is outside the supported complete grammar"
        try:
            verified_mapping = resolve_mapping(mapping_rules_text, calculation)
        except MappingRuleError as exc:
            return None, f"UNVERIFIED_MAPPING: {exc}"

        inventory = {"red": 0, "blue": 0}
        for variable, count in calculation.items():
            inventory[verified_mapping["variable_to_color"][variable]] += count
        if any(count > 5 for count in inventory.values()):
            return None, "INSUFFICIENT_INVENTORY: independently resolved colour totals exceed 5"
        order = extract_result_variables(generator_order) if generator_order else extract_result_variables(question_text)
        if not order or len(order) != len(calculation) or set(order) != set(calculation):
            return None, "UNVERIFIED_ORDER: result-variable order cannot be confirmed"

        messages = self._build_messages(
            question_text, mapping_rules_text, generator_order=generator_order
        )

        last_error = None
        for attempt in range(self.max_retries + 1):
            try:
                if attempt > 0:
                    time.sleep(min(2 ** attempt, 8))  # Exponential backoff

                response = self._make_request(messages)
                status = response.status_code

                if status == 200:
                    body = response.json()
                    content = body["choices"][0]["message"].get("content", "")
                    if not content:
                        last_error = "DeepSeek returned empty content"
                        continue

                    try:
                        result = json.loads(content)
                        if not isinstance(result, dict):
                            last_error = "Result is not a JSON object"
                            continue
                        # Preserve the exact evidence sent to the model instead of
                        # trusting it to echo these fields without alteration.
                        result["question"] = question_text
                        result["mapping_rule"] = mapping_rules_text
                        result["solver"] = {
                            "provider": "deepseek",
                            "model": self.model,
                        }
                        # Validate structure
                        errors = self._validate_task(
                            result,
                            mapping_rules_text,
                            generator_order=generator_order,
                        )
                        if errors:
                            last_error = f"Validation errors: {errors}"
                            messages.extend(
                                [
                                    {"role": "assistant", "content": content},
                                    {
                                        "role": "user",
                                        "content": (
                                            "The JSON failed validation: "
                                            f"{'; '.join(errors)}. Return a corrected JSON object "
                                            "that follows every rule, especially one task per "
                                            "variable and count == calculation[variable]."
                                        ),
                                    },
                                ]
                            )
                            continue
                        result["mapping_interpretation"] = verified_mapping
                        return result, None
                    except json.JSONDecodeError as e:
                        last_error = f"Invalid JSON: {e}"
                        continue

                elif status == 401:
                    return None, "API_KEY_INVALID"
                elif status == 402:
                    return None, "INSUFFICIENT_BALANCE"
                elif status == 429:
                    last_error = "RATE_LIMITED"
                    time.sleep(5)
                    continue
                elif status >= 500:
                    last_error = f"SERVER_ERROR_{status}"
                    continue
                else:
                    last_error = f"HTTP_{status}: {response.text[:200]}"
                    continue

            except requests.exceptions.Timeout:
                last_error = "TIMEOUT"
                continue
            except requests.exceptions.ConnectionError:
                last_error = "CONNECTION_ERROR"
                continue
            except Exception as e:
                last_error = f"UNEXPECTED: {str(e)[:200]}"
                continue

        return None, last_error

    def _validate_task(self, task, mapping_rules_text="", generator_order=None):
        """Validate structured task output. Returns list of error strings."""
        errors = []

        if not isinstance(task, dict):
            return ["Result is not a dict"]

        if "ordered_tasks" not in task:
            errors.append("Missing ordered_tasks")
            return errors

        tasks = task.get("ordered_tasks", [])
        if not isinstance(tasks, list) or not tasks:
            errors.append("ordered_tasks is empty")
            return errors

        valid_colors = {"red", "blue"}
        valid_zones = {"A", "B", "C"}
        seen_sequences = set()
        seen_variables = set()
        inventory_by_color = {"red": 0, "blue": 0}
        calculation = task.get("calculation", {})
        if not isinstance(calculation, dict):
            errors.append("calculation is not an object")
            calculation = {}

        for i, t in enumerate(tasks):
            if not isinstance(t, dict):
                errors.append(f"Task {i + 1}: entry is not an object")
                continue
            seq = t.get("sequence")
            if not isinstance(seq, int) or isinstance(seq, bool):
                errors.append(f"Task {i + 1}: invalid sequence")
                continue
            if seq in seen_sequences:
                errors.append(f"Duplicate sequence {seq}")
            seen_sequences.add(seq)
            if seq != i + 1:
                errors.append(f"Task {seq}: expected consecutive sequence {i + 1}")

            variable = str(t.get("variable", "")).strip()
            if not variable:
                errors.append(f"Task {seq}: missing variable")
            elif variable in seen_variables:
                errors.append(f"Task {seq}: duplicate variable '{variable}'")
            seen_variables.add(variable)

            color = str(t.get("color", "")).lower()
            if color not in valid_colors:
                errors.append(f"Task {seq}: invalid color '{color}'")

            zone = str(t.get("zone", "")).upper()
            if zone not in valid_zones:
                errors.append(f"Task {seq}: invalid zone '{zone}'")

            count = t.get("count", 0)
            if not isinstance(count, int) or isinstance(count, bool) or not 1 <= count <= 5:
                errors.append(f"Task {seq}: invalid count {count}")
            elif color in inventory_by_color:
                inventory_by_color[color] += count
            if variable not in calculation:
                errors.append(f"Task {seq}: variable '{variable}' missing from calculation")
            else:
                calculated = calculation[variable]
                if (
                    not isinstance(calculated, int)
                    or isinstance(calculated, bool)
                    or not 1 <= calculated <= 5
                ):
                    errors.append(
                        f"Task {seq}: invalid calculation[{variable}]={calculated}"
                    )
                elif calculated != count:
                    errors.append(
                        f"Task {seq}: count {count} does not match "
                        f"calculation[{variable}]={calculated}"
                    )

        extra_variables = set(calculation) - seen_variables
        if extra_variables:
            errors.append(
                f"calculation has variables without tasks: {sorted(extra_variables)}"
            )

        for color, total in inventory_by_color.items():
            if total > 5:
                errors.append(
                    f"requested {total} {color} cubes exceeds available inventory 5"
                )

        expected_order = extract_result_variables(generator_order)
        if not expected_order:
            expected_order = extract_result_variables(task.get("question", ""))
        actual_order = [str(item.get("variable", "")).strip() for item in tasks if isinstance(item, dict)]
        if expected_order and actual_order != expected_order:
            errors.append(
                f"task variable order {actual_order} does not match "
                f"generator order {expected_order}"
            )

        expected_calculation = calculate_generated_answer(task.get("question", ""))
        if expected_calculation is None:
            errors.append("Original question cannot be independently verified")
        else:
            for variable, expected in expected_calculation.items():
                actual = calculation.get(variable)
                if actual != expected:
                    errors.append(
                        f"question requires calculation[{variable}]={expected}, "
                        f"got {actual}"
                    )

            missing_expected = set(expected_calculation) - seen_variables
            if missing_expected:
                errors.append(
                    f"question variables missing from tasks: {sorted(missing_expected)}"
                )

        errors.extend(validate_mapping_interpretation(task, mapping_rules_text))

        return errors


def main():
    """Standalone test."""
    import sys
    client = DeepSeekClient()
    question = "Calculate x=4 and y=1, in x then y order."
    mapping = (
        "x represents red and y represents blue; "
        "a result count >= 3 goes to zone A; "
        "a result count < 3 goes to zone C."
    )
    print(f"Solving: {question}")
    result, error = client.solve_task(question, mapping, mode="development")
    if error:
        print(f"Error: {error}")
    if result:
        print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

