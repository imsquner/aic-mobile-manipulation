#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编排器的结构守护测试：防止「模块级 def 插进类体」这类静默截断。

先失败证明：`approach_minimum_cubes` 最初被插到了类体中间，col-0 的 def 会
**结束 class body**，于是从 `_load_points` 往后的所有成员都变成了那个函数的
嵌套函数。py_compile 不报错（嵌套 def 合法），AST 兜底查导入也查不出，
直到 VM 起栈才炸：

    AttributeError: 'TaskOrchestratorNode' object has no attribute '_load_points'

用法：python3 test_orchestrator_structure.py
"""
import ast
import os
import sys
from pathlib import Path

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, "task_orchestrator_node.py")
PACKAGE_SOURCE_FILES = sorted(
    str(path)
    for path in Path(HERE).rglob("*.py")
    if not path.name.startswith("test_")
)
STATE_MODULE = os.path.join(HERE, "orchestration", "states.py")

CLASS_NAME = "TaskOrchestratorNode"
REQUIRED_METHODS = (
    "_load_points",
    "_transition",
    "_tick",
    "_publish_state_heartbeat",
    "_emit_item_timing",
    "_set_navigation_profile",
    "_non_target_obstacle_points",
    "_publish_non_target_obstacles",
    "_wait_for_base_still",
    "_publish_placed_cargo_obstacles",
    "_execute_single_item",
)
REQUIRED_TOP_FUNCTIONS = ("approach_minimum_cubes", "main")
MIN_METHODS = 100

FAIL = []
PASS = [0]


def check(name, expected, actual):
    if expected == actual:
        PASS[0] += 1
    else:
        FAIL.append("%s  期望=%r 实际=%r" % (name, expected, actual))


def invalid_task_state_references(source_tree, declared_states):
    referenced_states = set()
    for node in ast.walk(source_tree):
        if not isinstance(node, ast.Attribute):
            continue
        if isinstance(node.value, ast.Name) and node.value.id == "TaskState":
            referenced_states.add(node.attr)
        elif (
            isinstance(node.value, ast.Attribute)
            and node.value.attr == "TaskState"
            and isinstance(node.value.value, ast.Name)
            and node.value.value.id == "self"
        ):
            referenced_states.add(node.attr)
    return sorted(referenced_states - declared_states)


with open(TARGET, "r", encoding="utf-8") as handle:
    source = handle.read()
tree = ast.parse(source, filename=TARGET)

classes = [n for n in tree.body if isinstance(n, ast.ClassDef)]
class_names = [n.name for n in classes]
check("编排器类存在", True, CLASS_NAME in class_names)

if CLASS_NAME in class_names:
    node = classes[class_names.index(CLASS_NAME)]
    methods = {
        child.name for child in node.body if isinstance(child, ast.FunctionDef)
    }
    for source_file in PACKAGE_SOURCE_FILES:
        with open(source_file, "r", encoding="utf-8") as handle:
            module_tree = ast.parse(handle.read(), filename=source_file)
        for class_def in module_tree.body:
            if isinstance(class_def, ast.ClassDef) and class_def.name.endswith("Mixin"):
                methods.update(
                    child.name for child in class_def.body
                    if isinstance(child, ast.FunctionDef)
                )
    check("节点与功能模块方法数量 >= %d" % MIN_METHODS, True, len(methods) >= MIN_METHODS)
    for name in REQUIRED_METHODS:
        check("节点可调用方法 %s" % name, True, name in methods)

    # 核心守护：类体行号区间内不允许出现**模块级**的 def/class。
    # 这正是「col-0 def 截断类体」的指纹。
    top_level_defs = [
        n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
    ]
    intruders = [
        n.name
        for n in top_level_defs
        if n is not node and node.lineno < n.lineno < (node.end_lineno or 0)
    ]
    check("类体区间内没有模块级 def/class", [], intruders)
    # 类体之后只应剩少量模块级语句（文件尾部有 main() 启动块）
    trailing = len(source.splitlines()) - (node.end_lineno or 0)
    check("类体之后剩余行数 <= 40", True, trailing <= 40)

with open(STATE_MODULE, "r", encoding="utf-8") as handle:
    state_tree = ast.parse(handle.read(), filename=STATE_MODULE)
task_state_classes = [
    n for n in state_tree.body
    if isinstance(n, ast.ClassDef) and n.name == "TaskState"
]
if task_state_classes:
    state_class = task_state_classes[0]
    declared_states = {
        target.id
        for statement in state_class.body
        if isinstance(statement, ast.Assign)
        for target in statement.targets
        if isinstance(target, ast.Name)
        and isinstance(statement.value, ast.Constant)
        and isinstance(statement.value.value, str)
    }
    invalid_refs = []
    for source_file in PACKAGE_SOURCE_FILES:
        with open(source_file, "r", encoding="utf-8") as handle:
            source_tree = ast.parse(handle.read(), filename=source_file)
        for state_name in invalid_task_state_references(source_tree, declared_states):
            invalid_refs.append("%s:%s" % (os.path.basename(source_file), state_name))
    check("所有源码模块的 TaskState 引用都已声明", [], invalid_refs)
    fake_tree = ast.parse("self.TaskState.__MISSING_CONTRACT_TEST__")
    check(
        "契约检查能拒绝伪造的未知 TaskState 成员",
        ["__MISSING_CONTRACT_TEST__"],
        invalid_task_state_references(fake_tree, declared_states),
    )
else:
    check("TaskState 枚举存在", True, False)

top_funcs = [
    n.name for n in tree.body if isinstance(n, ast.FunctionDef)
]
for name in REQUIRED_TOP_FUNCTIONS:
    check("模块级函数 %s" % name, True, name in top_funcs)

print()
print("通过 %d 项，失败 %d 项" % (PASS[0], len(FAIL)))
for line in FAIL:
    print("FAIL: %s" % line)
def test_orchestrator_structure_checks():
    assert not FAIL, "\n".join(FAIL)


if __name__ == "__main__":
    sys.exit(1 if FAIL else 0)
