"""统一面板端口：默认避开Mihomo，拒绝非法端口。"""
import ast
import os
from pathlib import Path

import pytest

LAUNCHER = Path(__file__).parents[1] / 'start_competition.py'


def load_port_helper():
    tree = ast.parse(LAUNCHER.read_text())
    function = next((n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'rosbridge_port'), None)
    assert function is not None, 'launcher must resolve the same ROSBRIDGE_PORT as its shell child'
    namespace = {'os': os}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(LAUNCHER), 'exec'), namespace)
    return namespace['rosbridge_port']


def test_default_avoids_mihomo(monkeypatch):
    monkeypatch.delenv('ROSBRIDGE_PORT', raising=False)
    assert load_port_helper()() == 9091


def test_explicit_port_is_shared(monkeypatch):
    monkeypatch.setenv('ROSBRIDGE_PORT', '9191')
    assert load_port_helper()() == 9191


@pytest.mark.parametrize('value', ['invalid', '0', '65536', '-1'])
def test_invalid_port_rejected(monkeypatch, value):
    monkeypatch.setenv('ROSBRIDGE_PORT', value)
    with pytest.raises(ValueError):
        load_port_helper()()


def load_build_helper():
    import hashlib
    tree = ast.parse(LAUNCHER.read_text())
    function = next((n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'workspace_build_base'), None)
    assert function is not None, '--setup must avoid ROSIDL non-ASCII output paths'
    namespace = {'Path': Path, 'hashlib': hashlib}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(LAUNCHER), 'exec'), namespace)
    return namespace['workspace_build_base']


def test_fresh_chinese_workspace_gets_ascii_build_base(tmp_path):
    workspace = tmp_path / '中文/dev_ws'
    base = load_build_helper()(workspace)
    assert str(base).isascii()
    assert base != workspace / 'build'
    assert base == load_build_helper()(workspace)


def test_existing_ascii_cache_symlink_is_reused(tmp_path):
    workspace = tmp_path / '中文/dev_ws'
    workspace.mkdir(parents=True)
    cache = tmp_path / 'ascii-build'
    cache.mkdir()
    (workspace / 'build').symlink_to(cache, target_is_directory=True)
    assert load_build_helper()(workspace) == cache
