"""隔离桌面候选不得偷偷读取旧工作区，且symlink-install能找到源配置。"""
import ast
import os
from pathlib import Path
from types import SimpleNamespace

import yaml

ROOT = Path(__file__).parents[1] / 'aic_task'


def test_api_loads_config_beside_resolved_source_file(tmp_path):
    module = tmp_path / 'source/aic_task/aic_task/task_input/deepseek_client.py'
    module.parent.mkdir(parents=True)
    module.touch()
    config = module.parents[2] / 'config/api_secrets.yaml'
    config.parent.mkdir()
    config.write_text('deepseek:\n  model: local-source-fixture\n')
    installed = tmp_path / 'install/alias.py'
    installed.parent.mkdir()
    installed.symlink_to(module)
    original = ROOT / 'task_input/deepseek_client.py'
    method = next(n for n in ast.walk(ast.parse(original.read_text())) if isinstance(n, ast.FunctionDef) and n.name == '_load_config')
    ns = {'Path': Path, 'yaml': yaml, 'os': os, '__file__': str(installed)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(original), 'exec'), ns)
    assert ns['_load_config'](SimpleNamespace())['deepseek']['model'] == 'local-source-fixture'


def test_question_generator_search_starts_with_current_workspace():
    original = ROOT / 'task_input/question_source.py'
    tree = ast.parse(original.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'QuestionSource')
    assignment = next(n for n in cls.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'DEFAULT_BINARY_PATHS' for t in n.targets))
    paths = eval(compile(ast.Expression(body=assignment.value), str(original), 'eval'), {'Path':Path, '__file__':str(original)})
    assert Path(paths[0]) == original.resolve().parents[5] / 'TMSCQtest_x86_x64.bin'


def test_mapping_search_resolves_symlink_source():
    original = ROOT / 'task_input/parser_node.py'
    tree = ast.parse(original.read_text())
    search = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'search_paths' for t in n.targets))
    first = search.value.elts[0]
    assert any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'resolve' for n in ast.walk(first))
