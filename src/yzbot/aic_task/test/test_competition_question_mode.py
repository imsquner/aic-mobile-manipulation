import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aic_task.task_input.question_source import (
    QUESTION_BINARY_TMPDIR,
    QUESTION_MODE_ENV,
    QuestionSource,
    binary_subprocess_env,
    resolve_generator_args,
)
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

    def test_random_question_mode_drops_the_official_token(self):
        # The 30-round acceptance plan asks the generator for its random
        # (non-competition) question set while the stack stays --mode competition.
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'generator'
            binary.write_text('#!/bin/sh\nprintf "ARGS:%s\\n" "$*"\n')
            binary.chmod(0o700)
            with patch.dict(os.environ, {QUESTION_MODE_ENV: 'random'}, clear=False):
                source = QuestionSource(str(binary), competition_mode=True)
                stdout, stderr, error = source.run()
        self.assertIsNone(error)
        self.assertEqual(stdout, 'ARGS:')
        self.assertEqual(source.binary_info['args'], [])
        self.assertEqual(source.binary_info['question_mode'], 'random')
        self.assertEqual(source.binary_info['mode'], 'competition')

    def test_official_and_unset_question_mode_keep_the_token(self):
        for value in (None, '', 'official', 'OFFICIAL'):
            env = {} if value is None else {QUESTION_MODE_ENV: value}
            with self.subTest(value=value):
                self.assertEqual(resolve_generator_args(True, env), ['11470507'])
                self.assertEqual(resolve_generator_args(False, env), [])
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'generator'
            binary.write_text('#!/bin/sh\nprintf "ARGS:%s\\n" "$*"\n')
            binary.chmod(0o700)
            with patch.dict(os.environ, {QUESTION_MODE_ENV: 'official'}, clear=False):
                source = QuestionSource(str(binary), competition_mode=True)
            self.assertEqual(source.binary_info['question_mode'], 'official')
            self.assertEqual(source.binary_info['args'], ['11470507'])

    def test_unknown_question_mode_is_refused(self):
        with self.assertRaises(ValueError):
            resolve_generator_args(True, {QUESTION_MODE_ENV: 'rndom'})

    def test_non_ascii_tmpdir_is_replaced_for_the_bundle(self):
        # PyInstaller one-file bundle: a Chinese TMPDIR makes it print nothing
        # (measured: exit 0 with empty stdout/stderr), which broke the launcher.
        with patch.dict(os.environ, {'TMPDIR': '/home/ros/桌面/临时'}, clear=False):
            env = binary_subprocess_env()
        self.assertEqual(env['TMPDIR'], QUESTION_BINARY_TMPDIR)
        self.assertTrue(env['TMPDIR'].isascii())
        self.assertTrue(Path(env['TMPDIR']).is_dir())
        self.assertEqual(env['PYTHONIOENCODING'], 'utf-8')

    def test_ascii_tmpdir_is_preserved(self):
        with patch.dict(os.environ, {'TMPDIR': '/tmp/keep-me'}, clear=False):
            self.assertEqual(binary_subprocess_env()['TMPDIR'], '/tmp/keep-me')

    def test_run_hands_the_generator_a_sanitized_environment(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'generator'
            binary.write_text('#!/bin/sh\nprintf "题目: 1\\n"\n')
            binary.chmod(0o700)
            source = QuestionSource(str(binary), competition_mode=True)
            with patch(
                'aic_task.task_input.question_source.subprocess.run',
                return_value=SimpleNamespace(returncode=0, stdout='题目: 1\n', stderr=''),
            ) as run, patch.dict(os.environ, {'TMPDIR': '/home/ros/桌面/临时'}, clear=False):
                stdout, stderr, error = source.run()
        self.assertIsNone(error)
        self.assertEqual(stdout, '题目: 1')
        env = run.call_args.kwargs['env']
        self.assertTrue(env['TMPDIR'].isascii())
        self.assertEqual(env['PYTHONIOENCODING'], 'utf-8')

    def test_explicit_existing_config_path_does_not_need_snapshot_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / 'example.yaml'
            config.write_text('deepseek:\n  api_key: ""\n  base_url: https://example.invalid\n  model: test\n  connect_timeout_sec: 1\n  read_timeout_sec: 1\n  max_retries: 0\n')
            with patch.dict(os.environ, {'AIC_API_CONFIG_PATH': str(config)}):
                client = DeepSeekClient()
                self.assertEqual(client.model, 'test')
                self.assertEqual(client.api_key, '')
