import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aic_task.task_input.question_source import QuestionSource
from aic_task.task_input.deepseek_client import DeepSeekClient


class CompetitionQuestionModeTest(unittest.TestCase):
    def test_competition_passes_token_and_development_keeps_plain_call(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'generator'
            binary.write_text('#!/bin/sh\nprintf "ARGS:%s\\n" "$*"\n')
            binary.chmod(0o700)
            for mode, expected in [(True, 'ARGS:11470507'), (False, 'ARGS:')]:
                source = QuestionSource(str(binary), competition_mode=mode)
                stdout, stderr, error = source.run()
                self.assertIsNone(error)
                self.assertEqual(stdout, expected)
                self.assertEqual(source.binary_info['mode'], 'competition' if mode else 'development')

    def test_explicit_existing_config_path_does_not_need_snapshot_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'example.yaml'
            config.write_text('deepseek:\n  api_key: ""\n  base_url: https://example.invalid\n  model: test\n  connect_timeout_sec: 1\n  read_timeout_sec: 1\n  max_retries: 0\n')
            with patch.dict(os.environ, {'AIC_API_CONFIG_PATH': str(config)}):
                client = DeepSeekClient()
                self.assertEqual(client.model, 'test')
                self.assertEqual(client.api_key, '')
