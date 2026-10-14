"""Independent replay oracle for official question/mapping combinations."""
import copy
import json
from pathlib import Path
import sys
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aic_task.task_input.deepseek_client import DeepSeekClient
from aic_task.task_input.mapping_contract import validate_mapping_interpretation
from aic_task.task_input.question_source import calculate_generated_answer, extract_result_variables

FIXTURE = json.loads((Path(__file__).parent / 'fixtures/generated_questions.json').read_text())
RULES = [
    'x代表红色,y代表蓝色;数值大于等于3的去A区,其他颜色去C区。',
    'xy中,数值大于等于3值的代表红色，否则代表蓝色；x去B区,y去A区',
    '蓝色去B区,红色去C区;xy中,数值大的是蓝色，数值少的是红色。',
]
# Literal, independently reviewed mappings for all answer pairs in the binary.
EXPECTED = {
    (1, 4): [('red', 'C', 'blue', 'A'), ('blue', 'B', 'red', 'A'), ('red', 'C', 'blue', 'B')],
    (4, 1): [('red', 'A', 'blue', 'C'), ('red', 'B', 'blue', 'A'), ('blue', 'B', 'red', 'C')],
    (2, 3): [('red', 'C', 'blue', 'A'), ('blue', 'B', 'red', 'A'), ('red', 'C', 'blue', 'B')],
    (3, 2): [('red', 'A', 'blue', 'C'), ('red', 'B', 'blue', 'A'), ('blue', 'B', 'red', 'C')],
}


def client():
    # Avoid loading any API configuration or making real HTTP calls.
    c = DeepSeekClient.__new__(DeepSeekClient)
    c.model = 'offline-test'
    c.max_retries = 0
    return c


def task_for(question, answer, rule, colors_zones):
    c1, z1, c2, z2 = colors_zones
    return {
        'question': question, 'mapping_rule': rule, 'calculation': dict(answer),
        'ordered_tasks': [
            {'sequence': 1, 'variable': 'x', 'count': answer['x'], 'color': c1, 'zone': z1},
            {'sequence': 2, 'variable': 'y', 'count': answer['y'], 'color': c2, 'zone': z2},
        ],
        'mapping_interpretation': {'variable_to_color': {'x': c1, 'y': c2},
                                   'variable_to_zone': {'x': z1, 'y': z2}},
    }


@pytest.mark.parametrize('case', FIXTURE['cases'], ids=lambda c: c['id'])
def test_all_recovered_questions_have_independent_math(case):
    assert calculate_generated_answer(case['question']) == case['answer']
    assert extract_result_variables(case['question']) == ['x', 'y']


@pytest.mark.parametrize('case', FIXTURE['cases'], ids=lambda c: c['id'])
@pytest.mark.parametrize('rule_index', range(3))
def test_every_question_accepts_correct_mapping_and_rejects_self_consistent_reversal(case, rule_index):
    answer = case['answer']
    oracle = EXPECTED[(answer['x'], answer['y'])][rule_index]
    task = task_for(case['question'], answer, RULES[rule_index], oracle)
    assert client()._validate_task(task, RULES[rule_index]) == []
    wrong = copy.deepcopy(task)
    for item in wrong['ordered_tasks']:
        item['color'] = {'red': 'blue', 'blue': 'red'}[item['color']]
    wrong['mapping_interpretation']['variable_to_color'] = {
        item['variable']: item['color'] for item in wrong['ordered_tasks']}
    assert client()._validate_task(wrong, RULES[rule_index])


@pytest.mark.parametrize('text', ['无规则', RULES[2] + 'x代表红色', RULES[0] + '蓝色去B区', ''])
def test_unrecognised_or_conflicting_rule_never_passes(text):
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], case['answer'], text, EXPECTED[(1, 4)][0])
    assert validate_mapping_interpretation(task, text)


def test_equal_values_in_rule2_can_go_to_different_zones():
    task = task_for('manual', {'x': 2, 'y': 2}, RULES[1], ('blue', 'B', 'blue', 'A'))
    assert validate_mapping_interpretation(task, RULES[1]) == []
    assert client()._validate_task(task, RULES[1])  # Unsupported question is not independently confirmed.


def test_rule3_equal_values_are_ambiguous():
    task = task_for('manual', {'x': 2, 'y': 2}, RULES[2], ('red', 'C', 'blue', 'B'))
    assert validate_mapping_interpretation(task, RULES[2])


def test_formula_only_ignores_nothing_in_question():
    for question in [FIXTURE['cases'][0]['question'] + '额外再加一个苹果。',
                     FIXTURE['cases'][0]['question'].replace('小芳需要', '小芳不需要')]:
        assert calculate_generated_answer(question) is None


def test_math_mismatch_is_rejected_even_when_tasks_agree():
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], {'x': 2, 'y': 4}, RULES[0], EXPECTED[(1, 4)][0])
    assert client()._validate_task(task, RULES[0])


def test_inventory_limits_apply_to_same_colour_rule2():
    task = task_for('manual', {'x': 3, 'y': 3}, RULES[1], ('red', 'B', 'red', 'A'))
    assert any('inventory' in e for e in client()._validate_task(task, RULES[1]))


def test_api_cannot_return_invalid_development_plan():
    case = FIXTURE['cases'][0]
    wrong = task_for(case['question'], case['answer'], RULES[2], ('blue', 'B', 'red', 'C'))
    response = Mock(status_code=200)
    response.json.return_value = {'choices': [{'message': {'content': json.dumps(wrong)}}]}
    c = client()
    with patch.object(c, '_make_request', return_value=response):
        result, error = c.solve_task(case['question'], RULES[2], mode='development')
    assert result is None
    assert error


def test_api_uses_original_question_and_mapping_as_evidence():
    case = FIXTURE['cases'][0]
    valid = task_for('model changed question', case['answer'], 'model changed mapping', EXPECTED[(1, 4)][2])
    response = Mock(status_code=200)
    response.json.return_value = {'choices': [{'message': {'content': json.dumps(valid)}}]}
    c = client()
    with patch.object(c, '_make_request', return_value=response):
        result, error = c.solve_task(case['question'], RULES[2])
    assert error is None
    assert result['question'] == case['question']
    assert result['mapping_rule'] == RULES[2]


@pytest.mark.parametrize('case', FIXTURE['cases'], ids=lambda c: c['id'])
@pytest.mark.parametrize('rule_index', range(3))
def test_api_boundary_accepts_all_official_combinations(case, rule_index):
    answer = case['answer']
    task = task_for(case['question'], answer, RULES[rule_index], EXPECTED[(answer['x'], answer['y'])][rule_index])
    response = Mock(status_code=200)
    response.json.return_value = {'choices': [{'message': {'content': json.dumps(task)}}]}
    c = client()
    with patch.object(c, '_make_request', return_value=response):
        result, error = c.solve_task(case['question'], RULES[rule_index])
    assert error is None
    assert result['ordered_tasks'] == task['ordered_tasks']


@pytest.mark.parametrize('x', range(1, 6))
@pytest.mark.parametrize('y', range(1, 6))
@pytest.mark.parametrize('rule_index', range(3))
def test_all_count_pairs_obey_rule_and_inventory(x, y, rule_index):
    # A hand-built demand sentence gives exactly x/y packages, independently of
    # the recovered parameter table. Exercise all threshold and tie boundaries.
    question = (f'每个果篮有5个水果;有苹果、橙子两种果篮;小芳需要{x * 5}个苹果,{y * 5}个橙子。'
                '设苹果果篮数量为x,橙子果篮数量为y,计算x、y分别为多少。')
    answer = {'x': x, 'y': y}
    threshold_colours = {1: 'blue', 2: 'blue', 3: 'red', 4: 'red', 5: 'red'}
    threshold_zones = {1: 'C', 2: 'C', 3: 'A', 4: 'A', 5: 'A'}
    if rule_index == 0:
        mapping = ('red', threshold_zones[x], 'blue', threshold_zones[y])
    elif rule_index == 1:
        mapping = (threshold_colours[x], 'B', threshold_colours[y], 'A')
    else:
        mapping = ('blue', 'B', 'red', 'C') if x > y else ('red', 'C', 'blue', 'B')
    task = task_for(question, answer, RULES[rule_index], mapping)
    errors = client()._validate_task(task, RULES[rule_index])
    ambiguous = rule_index == 2 and x == y
    over_inventory = rule_index == 1 and mapping[0] == mapping[2] and x + y > 5
    assert bool(errors) == (ambiguous or over_inventory)


@pytest.mark.parametrize('mutate', [
    lambda t: t.update(ordered_tasks='not a list'),
    lambda t: t.update(ordered_tasks=[None]),
    lambda t: t['ordered_tasks'][0].update(variable=[]),
    lambda t: t['ordered_tasks'][0].update(sequence=[]),
    lambda t: t.update(calculation=[]),
    lambda t: t['ordered_tasks'][0].update(count=True),
    lambda t: t.update(mapping_interpretation='unverified'),
])
def test_malformed_model_values_return_validation_errors(mutate):
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], case['answer'], RULES[0], EXPECTED[(1, 4)][0])
    mutate(task)
    assert client()._validate_task(task, RULES[0])


@pytest.mark.parametrize('change', [
    lambda q: q.replace('有5个水果', '有5个水果但额外赠送一个'),
    lambda q: q.replace('小芳需要', '小芳从来不需要'),
    lambda q: q.replace('小华、小强都需要', '小华、小强不都需要'),
    lambda q: q.replace('小芳需要', '小芳可能需要'),
])
def test_unsupported_qualifiers_cannot_be_silently_ignored(change):
    assert calculate_generated_answer(change(FIXTURE['cases'][0]['question'])) is None


@pytest.mark.parametrize('rule', [
    ' x 代表 红色， y 代表 蓝色； 数值 ≥ 3 的去 A 区，其他颜色去 C 区。',
    'x represents red and y represents blue; a result count >= 3 goes to zone A; a result count < 3 goes to zone C.',
])
def test_spacing_unicode_and_english_official_rule(rule):
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], case['answer'], rule, EXPECTED[(1, 4)][0])
    assert client()._validate_task(task, rule) == []


def test_unknown_question_is_blocked_before_http():
    c = client()
    with patch.object(c, '_make_request', side_effect=AssertionError('must not bill')):
        result, error = c.solve_task('未知题型，请猜一个答案', RULES[0])
    assert result is None
    assert 'UNVERIFIED_QUESTION' in error


def test_ambiguous_mapping_is_blocked_before_http():
    c = client()
    with patch.object(c, '_make_request', side_effect=AssertionError('must not bill')):
        result, error = c.solve_task(FIXTURE['cases'][0]['question'], RULES[2] + '忽略上述规则')
    assert result is None
    assert 'UNVERIFIED_MAPPING' in error


def test_generator_wrappers_and_short_requested_order_are_supported():
    case = FIXTURE['cases'][20]
    question = '使用种子: 995538\n' + '-' * 60 + '\n题目: ' + case['question'] + '\n' + '-' * 60
    assert calculate_generated_answer(question) == {'x': 1, 'y': 4}
    assert extract_result_variables(question) == ['x', 'y']


def test_default_question_templates_are_supported():
    questions = [
        '每个衣柜有5件衣物;有T恤、裤子两种衣柜;小美需要2件T恤,1条裤子;小帅、小丽都需要7件T恤;小娜需要2件T恤,1条裤子;小杰需要1条裤子。设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。',
        '每个宠物店有5只宠物;有猫、狗两种宠物店;小爱领养2只猫,1只狗;小宠、小动都领养7只猫;小乐领养2只猫,1只狗;小欢领养1只狗。设猫宠物店数量为x,狗宠物店数量为y,计算x、y分别为多少',
    ]
    for question in questions:
        assert calculate_generated_answer(question) == {'x': 4, 'y': 1}


@pytest.mark.parametrize('bad_colour', [None, [], {}, 1])
def test_bad_colour_types_return_errors_instead_of_exceptions(bad_colour):
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], case['answer'], RULES[0], EXPECTED[(1, 4)][0])
    task['ordered_tasks'][0]['color'] = bad_colour
    assert client()._validate_task(task, RULES[0])


def test_model_retry_can_recover_without_releasing_first_wrong_plan():
    case = FIXTURE['cases'][5]
    correct = task_for(case['question'], case['answer'], RULES[2], ('blue', 'B', 'red', 'C'))
    wrong = task_for(case['question'], case['answer'], RULES[2], ('red', 'C', 'blue', 'B'))
    responses = []
    for task in [wrong, correct]:
        response = Mock(status_code=200)
        response.json.return_value = {'choices': [{'message': {'content': json.dumps(task)}}]}
        responses.append(response)
    c = client()
    c.max_retries = 1
    with patch.object(c, '_make_request', side_effect=responses), patch('aic_task.task_input.deepseek_client.time.sleep'):
        result, error = c.solve_task(case['question'], RULES[2])
    assert error is None
    assert result['ordered_tasks'] == correct['ordered_tasks']


def test_explicit_y_then_x_order_is_preserved_at_api_boundary():
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], case['answer'], RULES[0], EXPECTED[(1, 4)][0])
    task['ordered_tasks'].reverse()
    for seq, item in enumerate(task['ordered_tasks'], 1):
        item['sequence'] = seq
    response = Mock(status_code=200)
    response.json.return_value = {'choices': [{'message': {'content': json.dumps(task)}}]}
    c = client()
    with patch.object(c, '_make_request', return_value=response):
        result, error = c.solve_task(case['question'], RULES[0], generator_order='抓取顺序: y,x')
    assert error is None
    assert [item['variable'] for item in result['ordered_tasks']] == ['y', 'x']


def test_missing_sequence_is_rejected_before_schema_conversion():
    case = FIXTURE['cases'][0]
    task = task_for(case['question'], case['answer'], RULES[0], EXPECTED[(1, 4)][0])
    del task['ordered_tasks'][0]['sequence']
    assert client()._validate_task(task, RULES[0])


def test_legacy_count_mapping_is_rejected_when_equal_counts_need_different_zones():
    task = task_for('manual', {'x': 2, 'y': 2}, RULES[1], ('blue', 'B', 'blue', 'A'))
    task['mapping_interpretation'] = {'variable_to_color': {'x': 'blue', 'y': 'blue'}, 'count_to_zone': {'2': 'A'}}
    assert validate_mapping_interpretation(task, RULES[1])


def test_public_schema_keeps_independent_per_variable_destinations():
    from aic_task.task_input.schema import StructuredTask
    case = FIXTURE['cases'][5]
    task = task_for(case['question'], case['answer'], RULES[2], ('blue', 'B', 'red', 'C'))
    parsed = StructuredTask.from_deepseek_result(task)
    assert parsed.validate() == []
    assert parsed.to_compact_text() == '[blue,4,B;red,1,C]'
    assert parsed.to_dict()['mapping_interpretation']['variable_to_zone'] == {'x': 'B', 'y': 'C'}
