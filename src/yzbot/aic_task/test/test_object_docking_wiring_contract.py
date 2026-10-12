"""节点接线契约测试（源码/AST 级）。

独立复核在节点接线处发现过一个 P0：`TaskOrchestratorNode._navigate_to` 覆写没有
`evidence` 形参，而 `_world2_dock_candidate` 用 `evidence=evidence` 调它 ——
运行期必抛 `TypeError`，而当时 70 个 docking 单测**完全碰不到这条路径**。

本文件用 AST 静态锁死这类接线，防止回归：
1. 模块内所有 `self._navigate_to(...)` 的关键字实参都必须被最派生覆写签名接受；
2. 覆写必须把 `evidence` 转发给 `super()`；
3. `_publish_docking_twist` 必须走 `pub_precision_cmd`（= /cmd_vel_nav），不得绕过安全链；
4. `object_docking_enabled` 的 ROS 参数默认必须是 False（v1/world3 隔离）；
5. `_apply_object_docking_points()` 必须在 `_load_points` 之后调用（points 覆盖参数）；
6. `_world2_dock_candidate` 的 release gate 不得再硬编码 `velocity_chain_active=True`。
"""
import ast
import importlib
import sys
import unittest
from pathlib import Path

NODE_SRC = (
    Path(__file__).resolve().parents[1] / "aic_task" / "task_orchestrator_node.py"
)
FLOW_SRC_PATH = (
    Path(__file__).resolve().parents[1]
    / "aic_task"
    / "navigation"
    / "object_docking"
    / "flow.py"
)
FLOW_SOURCE = FLOW_SRC_PATH.read_text(encoding="utf-8")
FLOW_TREE = ast.parse(FLOW_SOURCE)
FLOW_LINES = FLOW_SOURCE.splitlines()


def _flow_function(name):
    for node in ast.walk(FLOW_TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return "\n".join(FLOW_LINES[node.lineno - 1 : node.end_lineno])
    raise AssertionError(f"flow function {name} not found")
SOURCE = NODE_SRC.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)
LINES = SOURCE.splitlines()


def _segment(method):
    """取方法源码行（注意 SOURCE 是字符串，必须按行取，按行号切会切成字符）。"""
    return "\n".join(LINES[method.lineno - 1 : method.end_lineno])


def _class(name):
    for node in ast.walk(TREE):
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    raise AssertionError(f"class {name} not found")


def _method(cls, name):
    for node in cls.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"method {name} not found in {cls.name}")


def _arg_names(func):
    args = func.args
    names = [a.arg for a in args.posonlyargs + args.args + args.kwonlyargs]
    return [n for n in names if n != "self"]


def _self_calls(root, method_name):
    """收集 `self.<method_name>(...)` 的所有调用。"""
    out = []
    for node in ast.walk(root):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr != method_name:
            continue
        if not isinstance(func.value, ast.Name) or func.value.id != "self":
            continue
        out.append(node)
    return out


class NavigateToWiringTest(unittest.TestCase):
    def setUp(self):
        self.cls = _class("TaskOrchestratorNode")
        self.override = _method(self.cls, "_navigate_to")
        self.accepted = set(_arg_names(self.override))

    def test_override_accepts_evidence(self):
        self.assertIn(
            "evidence",
            self.accepted,
            "P0 回归：_navigate_to 覆写必须有 evidence 形参，否则调用方 TypeError",
        )

    def test_override_forwards_evidence_to_super(self):
        forwarded = False
        for node in ast.walk(self.override):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not (
                isinstance(func, ast.Attribute)
                and func.attr == "_navigate_to"
                and isinstance(func.value, ast.Call)
                and isinstance(func.value.func, ast.Name)
                and func.value.func.id == "super"
            ):
                continue
            # 位置参数里必须出现 evidence（按 operations.py 的形参顺序是第 7 个）
            if any(
                isinstance(arg, ast.Name) and arg.id == "evidence"
                for arg in node.args
            ):
                forwarded = True
            if any(
                kw.arg == "evidence" for kw in node.keywords
            ):
                forwarded = True
        self.assertTrue(
            forwarded, "覆写必须把 evidence 转发给 super()._navigate_to"
        )

    def _all_package_calls(self, method_name):
        """扫**整个包**（不只节点类）：MRO 里任何 mixin 的 self._navigate_to(...)
        都解析到节点覆写，所以都必须符合同一签名。"""
        pkg = NODE_SRC.parent
        out = []
        for path in sorted(pkg.rglob("*.py")):
            if "__pycache__" in str(path):
                continue
            try:
                tree = ast.parse(path.read_text(encoding="utf-8"))
            except SyntaxError:
                continue
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                func = node.func
                if not isinstance(func, ast.Attribute) or func.attr != method_name:
                    continue
                if not isinstance(func.value, ast.Name) or func.value.id != "self":
                    continue
                out.append((path, node))
        return out

    def test_every_navigate_to_call_kwarg_is_accepted(self):
        """**全包**所有 self._navigate_to(...) 的 kwargs 必须被覆写签名接受。"""
        calls = self._all_package_calls("_navigate_to")
        self.assertTrue(calls, "expected at least one _navigate_to call")
        for path, call in calls:
            for keyword in call.keywords:
                if keyword.arg is None:  # **kwargs 展开，跳过
                    continue
                self.assertIn(
                    keyword.arg,
                    self.accepted,
                    f"{path.name}:{call.lineno} 传了未声明的关键字实参 "
                    f"{keyword.arg!r}；节点覆写签名只有 {sorted(self.accepted)}",
                )

    def test_positional_call_arity_fits(self):
        """**全包**位置参数个数不得超过覆写签名能接受的个数。"""
        limit = len(_arg_names(self.override))
        for path, call in self._all_package_calls("_navigate_to"):
            self.assertLessEqual(
                len(call.args),
                limit,
                f"{path.name}:{call.lineno} 位置参数过多（上限 {limit}）",
            )


class DockingWiringTest(unittest.TestCase):
    def setUp(self):
        self.cls = _class("TaskOrchestratorNode")

    def test_docking_twist_goes_through_the_safety_chain(self):
        """必须走 pub_precision_cmd（/cmd_vel_nav），不得直接发 /cmd_vel_safe。"""
        method = _method(self.cls, "_publish_docking_twist")
        published = [
            node
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "publish"
        ]
        self.assertTrue(published, "expected a publish call")
        for call in published:
            owner = call.func.value
            self.assertTrue(
                isinstance(owner, ast.Attribute) and owner.attr == "pub_precision_cmd",
                "_publish_docking_twist 必须发布到 pub_precision_cmd(/cmd_vel_nav)，"
                "否则绕过 velocity_smoother / collision_monitor",
            )
        # 发布对象只允许 pub_precision_cmd（等价于"不直接发 /cmd_vel_safe"）
        owners = {
            call.func.value.attr
            for call in published
            if isinstance(call.func.value, ast.Attribute)
        }
        self.assertEqual(owners, {"pub_precision_cmd"})

    def test_object_docking_enabled_defaults_to_false(self):
        """默认必须是 False，只有场景配置能开启（v1/world3 隔离）。"""
        found = []
        for node in ast.walk(self.cls):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not (isinstance(func, ast.Attribute) and func.attr == "declare_parameter"):
                continue
            if not node.args:
                continue
            first = node.args[0]
            if isinstance(first, ast.Constant) and first.value == "object_docking_enabled":
                found.append(node)
        self.assertTrue(found, "object_docking_enabled 参数未声明")
        default = found[0].args[1] if len(found[0].args) > 1 else None
        self.assertIsInstance(default, ast.Constant)
        self.assertIs(
            default.value, False, "object_docking_enabled 默认必须为 False"
        )

    def test_points_override_runs_after_points_are_loaded(self):
        """`_apply_object_docking_points()` 必须在 `self.points = ...` 之后。"""
        init = _method(self.cls, "__init__")
        load_line = None
        apply_line = None
        for node in ast.walk(init):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if (
                        isinstance(target, ast.Attribute)
                        and target.attr == "points"
                        and isinstance(target.value, ast.Name)
                        and target.value.id == "self"
                    ):
                        load_line = node.lineno
            if isinstance(node, ast.Call):
                func = node.func
                if (
                    isinstance(func, ast.Attribute)
                    and func.attr == "_apply_object_docking_points"
                ):
                    apply_line = node.lineno
        self.assertIsNotNone(load_line, "self.points 赋值未找到")
        self.assertIsNotNone(apply_line, "_apply_object_docking_points 未被调用")
        self.assertGreater(
            apply_line,
            load_line,
            "points.yaml 覆盖必须在 self.points 载入之后，否则读不到配置",
        )

    def test_release_gate_does_not_hardcode_velocity_chain(self):
        """fail-open 回归：release gate 不得写死 velocity_chain_active=True。"""
        seq = _flow_function("handoff_then_dock")
        self.assertNotIn("velocity_chain_active=True", seq)
        self.assertIn("velocity_chain_active=velocity_chain_active()", seq)
        # 节点必须把真实探针接进去
        candidate = _segment(_method(self.cls, "_world2_dock_candidate"))
        self.assertIn("velocity_chain_active=self._velocity_chain_active", candidate)

    def test_docking_path_guards_the_item_budget(self):
        """A5：交接段可能超预算，必须有 deadline 守卫，且 item_flow 要传它。"""
        seq = _flow_function("handoff_then_dock")
        self.assertIn("STATUS_BUDGET_EXHAUSTED", seq)
        self.assertIn("budget_left", seq)
        candidate = _segment(_method(self.cls, "_world2_dock_candidate"))
        self.assertIn("deadline", candidate)
        self.assertIn("object_docking_minimum_budget_sec", candidate)
        self.assertIn("budget_left=", candidate)
        flow_src = (
            Path(__file__).resolve().parents[1]
            / "aic_task"
            / "orchestration"
            / "item_flow.py"
        ).read_text(encoding="utf-8")
        self.assertIn("navigation_deadline", flow_src.split("_world2_dock_candidate")[1][:200])

    def test_release_path_uses_the_tested_pure_helpers(self):
        """被单测覆盖的纯函数必须是生产路径真正调用的，不能是死代码。

        （第一轮的 85 个 docking 单测全都碰不到节点路径，P0/P1 就是这样溜进去的。）
        """
        candidate = _segment(_method(self.cls, "_world2_dock_candidate"))
        self.assertIn("handoff_then_dock(", candidate)
        seq = _flow_function("handoff_then_dock")
        self.assertIn("already_in_band_evidence(", seq)
        self.assertIn("handoff_from_navigation_evidence(", seq)
        self.assertIn("docking_release_checks(", seq)
        active = _segment(_method(self.cls, "_velocity_chain_state"))
        self.assertIn("velocity_chain_verdict(", active)

    def test_velocity_chain_probe_uses_lifecycle_not_topic_freshness(self):
        """A1 回归守卫：不得退回"话题新鲜度"判据。

        velocity_smoother 收到零速后停止发布、collision_monitor 有
        stop_pub_timeout(2.0 s)，而软交接发生在刚停车那一刻 —— 话题新鲜度会把
        合法交接 100% 拦死（自锁）。判据必须是 lifecycle 状态。
        """
        self.assertIn('"/{node_name}/get_state"', SOURCE)
        self.assertIn("GetState", SOURCE)
        active = _segment(_method(self.cls, "_velocity_chain_active"))
        self.assertNotIn("_velocity_chain_age", active)
        self.assertNotIn("_last_cmd_vel_safe_at", active)
        self.assertNotIn("_on_cmd_vel_safe", SOURCE)

    def test_velocity_chain_unknown_does_not_self_lock(self):
        """查不到状态必须放行（打事件留痕），否则会重演 A1 自锁。"""
        active = _segment(_method(self.cls, "_velocity_chain_active"))
        self.assertIn("pass_with_log", active)
        self.assertIn("return True", active)

    def test_velocity_chain_check_has_a_config_switch(self):
        """A3：必须有开关与节点列表配置入口，现场能关能调。"""
        self.assertIn('"object_docking_velocity_chain_check"', SOURCE)
        self.assertIn('"object_docking_velocity_chain_nodes"', SOURCE)
        self.assertIn(
            "object_docking_velocity_chain_check",
            _segment(_method(self.cls, "_apply_object_docking_points")),
        )


class NodeImportExportContractTest(unittest.TestCase):
    """节点从 docking 包导入了什么，包就必须真的导出什么。

    2026-09-28 在 VM 同步后实测踩到：`ObjectDockingMixin` 没被 `__init__.py` 导出，
    而节点是 `from aic_task.navigation.object_docking import ObjectDockingMixin` ——
    **orchestrator 直接 ImportError、栈起不来**。本地 206 个测试全用 importlib 直载
    子模块，从没走过 `__init__.py`，所以完全没覆盖到。

    用**纯静态 AST**（不 import）：其它测试会往 sys.modules 塞桩模块，真 import 会被
    遮蔽，导致测试顺序相关。
    """

    @classmethod
    def setUpClass(cls):
        init_path = FLOW_SRC_PATH.parent / "__init__.py"
        init_tree = ast.parse(init_path.read_text(encoding="utf-8"))
        cls.init_path = init_path
        # 包内 `from ... import (...)` 收进来的名字 = 包根实际可访问的名字
        imported = set()
        for node in ast.walk(init_tree):
            if isinstance(node, ast.ImportFrom):
                imported.update(alias.name for alias in node.names)
        cls.imported = imported
        # __all__ 字面量
        all_names = set()
        for node in ast.walk(init_tree):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if isinstance(target, ast.Name) and target.id == "__all__":
                        all_names.update(
                            elt.value for elt in node.value.elts
                            if isinstance(elt, ast.Constant)
                        )
        cls.all_names = all_names

    def _node_docking_imports(self):
        """节点里对 docking 包（及其子模块）的导入名。"""
        root_names, sub_names = set(), set()
        for node in ast.walk(TREE):
            if not isinstance(node, ast.ImportFrom):
                continue
            module = node.module or ""
            if module == "aic_task.navigation.object_docking":
                root_names.update(alias.name for alias in node.names)
            elif module.startswith("aic_task.navigation.object_docking."):
                sub_names.add(module)
        return root_names, sub_names

    def test_package_root_exports_everything_the_node_imports(self):
        root_names, _ = self._node_docking_imports()
        self.assertTrue(root_names, "节点没有从 docking 包根导入？")
        missing = sorted(root_names - self.imported)
        self.assertEqual(
            missing,
            [],
            f"{self.init_path.name} 没有收进这些名字，节点会 ImportError：{missing}",
        )

    def test_all_export_matches_reality(self):
        """`__all__` 里每个名字都必须真被导入进来（防笔误/漏导入）。"""
        missing = sorted(self.all_names - self.imported)
        self.assertEqual(missing, [], f"__all__ 里有但没导入的名字：{missing}")

    def test_submodule_imports_are_declared(self):
        """子模块导入（如 .flow）必须真的存在对应文件。"""
        _, sub_modules = self._node_docking_imports()
        pkg_dir = FLOW_SRC_PATH.parent
        for module in sub_modules:
            leaf = module.rsplit(".", 1)[-1]
            self.assertTrue(
                (pkg_dir / f"{leaf}.py").is_file(),
                f"节点导入了 {module}，但 {pkg_dir / (leaf + '.py')} 不存在",
            )

    def test_package_is_ros_free(self):
        """docking 包必须保持无 ROS 依赖，否则单测无法加载它。"""
        for src in (
            FLOW_SRC_PATH,
            FLOW_SRC_PATH.parent / "controller.py",
            FLOW_SRC_PATH.parent / "handoff.py",
            FLOW_SRC_PATH.parent / "__init__.py",
        ):
            text = src.read_text(encoding="utf-8")
            self.assertNotIn("import rclpy", text, src.name)
            self.assertNotIn("from nav2_msgs", text, src.name)
            self.assertNotIn("from lifecycle_msgs", text, src.name)
            self.assertNotIn("from std_msgs", text, src.name)


if __name__ == "__main__":
    unittest.main()
