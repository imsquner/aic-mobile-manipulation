"""Offline regressions: execute production methods without spinning ROS or API."""
import ast
import json
import threading
import time
import unittest
import uuid
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1] / 'aic_task'
PERCEPTION = ROOT.parents[1] / 'aic_perception' / 'aic_perception'

class Message:
    def __init__(self, data='', **kwargs):
        self.data = data
        self.__dict__.update(kwargs)

class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)

class Logger:
    def __getattr__(self, name): return lambda *args, **kwargs: None

def production(path, name, **extra):
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    ns = dict(time=time, threading=threading, json=json, String=Message, Bool=Message,
              Int32=Message, Float32=Message, Node=object, StructuredTask=object)
    ns.update(extra)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), ns)
    return ns[name]

def task_states():
    return production(ROOT / 'orchestration/states.py', 'TaskState')

class ControlSafetyTest(unittest.TestCase):
    def test_scene_reset_failure_never_finishes_ready(self):
        cls = production(ROOT / 'orchestration/lifecycle.py', 'LifecycleMixin')
        node = cls()
        node._reset_scene = lambda: (False, 'cube failed')
        node.pub_last_error = Publisher()
        node._reset_lock = threading.Lock()
        node._finish_reset = lambda: self.fail('failed scene entered READY path')
        node._transition = lambda state: setattr(node, 'state', state)
        node._event = lambda *a, **k: None
        cls._scene_reset_worker.__globals__['TaskState'] = task_states()
        node._scene_reset_worker()
        self.assertNotEqual(getattr(node, 'state', ''), 'READY')

    def test_cancelled_plan_wait_does_not_become_api_error(self):
        states = task_states()
        cls = production(ROOT / 'orchestration/round_flow.py', 'RoundFlowMixin',
                         TaskState=states, TERMINAL_TASK_STATES={'CANCELLED', 'TIME_LIMIT'})
        node = cls()
        node._exec_lock = threading.Lock()
        node.task_plan_timeout = 120
        node.state = 'CANCELLED'
        node._stop_exec = True
        node._wait_for_task_plan = lambda timeout: False
        node._transition = lambda state: setattr(node, 'state', state)
        node._publish_final_result = lambda: None
        node._event = lambda *a, **k: None
        node.get_logger = lambda: Logger()
        node._execution_loop()
        self.assertEqual(node.state, 'CANCELLED')

    def test_ik_wait_wakes_on_cancel(self):
        gate = production(PERCEPTION / 'pick_sequence_orchestrator.py', 'ValidationGate')()
        gate.start()
        result = []
        worker = threading.Thread(target=lambda: result.append(gate.wait(.5)))
        worker.start()
        time.sleep(.02)
        gate.cancel()
        worker.join(.15)
        alive = worker.is_alive()
        worker.join(1)
        self.assertFalse(alive, 'cancel left IK wait blocked')
        self.assertEqual(result, [None])

    def test_api_probe_does_not_change_parsing_state(self):
        cls = production(ROOT / 'task_input/parser_node.py', 'TaskParserNode',
                         DeepSeekClient=lambda: SimpleNamespace(check_available=lambda: (True, None), model='mock'))
        node = cls.__new__(cls)
        node.current_task = None
        node.pub_api_model = Publisher()
        node.pub_api_connection = Publisher()
        node._set_api_connection = lambda status: None
        node._set_status = lambda status: self.fail('probe overwrote parsing with ' + status)
        node._set_error = lambda reason: self.fail(reason)
        node._check_api()

    def test_old_execution_cannot_overwrite_cancelling(self):
        cls = production(ROOT / 'orchestration/states.py', 'StatePublishingMixin')
        node = cls()
        node.state = 'CANCELLING'
        node._control_mode = 'cancel'
        node._control_thread = object()
        node._control_lock = threading.RLock()
        node._phase_timer = SimpleNamespace(enter=lambda s: None)
        node.pub_state = Publisher()
        node._event = lambda *a, **k: None
        node.get_logger = lambda: Logger()
        node._transition('ITEM_FAILED')
        self.assertEqual(node.state, 'CANCELLING')

    def test_unknown_ui_state_does_not_display_idle(self):
        ui = json.loads((ROOT.parents[1] / 'foxglove/window_dashboard.json').read_text())
        self.assertNotEqual(ui['configById']['Indicator!state']['fallbackLabel'], '空闲')
        self.assertNotEqual(ui['configById']['Indicator!nav']['fallbackLabel'], '空闲')

    def test_stop_grasp_does_not_wait_for_operation_lock(self):
        cls = production(PERCEPTION / 'grasp_executor.py', 'GraspExecutor',
                         Parameter=object)
        node = cls.__new__(cls)
        node._operation_lock = threading.Lock(); node._operation_lock.acquire()
        node._stop_event = threading.Event()
        node._active_goals = []; node._pending_calls = []; node._pending_sends = []
        node._motion_lock = threading.RLock()
        node._publish = lambda *a, **k: None
        response = Message(success=False, message='')
        node._on_stop(None, response)
        self.assertTrue(node._stop_event.is_set())
        self.assertFalse(response.success)

class ParserContractTest(unittest.TestCase):
    def parser(self):
        cls = production(ROOT / 'task_input/parser_node.py', 'TaskParserNode', uuid=uuid)
        node = cls.__new__(cls)
        node._parser_lock = threading.RLock()
        node._generation = 1; node._round_id = 'round-one'; node._system_state = 'READY'
        node._status = 'EMPTY'; node._worker = None; node._last_plan_payload = {}
        node.current_question_text = ''; node.current_mapping_rule = ''; node.current_generator_order = None
        node.current_task = None; node.mode = 'competition'; node._task_frozen = False
        node.get_logger = lambda: Logger()
        for attr in ('pub_question','pub_mapping','pub_task_text','pub_task_plan','pub_api_status',
                     'pub_last_error','pub_red_total','pub_blue_total','pub_total_items'):
            setattr(node,attr,Publisher())
        return node

    def test_plan_ready_requires_coordinator_acceptance(self):
        node=self.parser()
        task=SimpleNamespace(validate=lambda:[], to_json=lambda:'{"ordered_tasks":[]}',
                             to_compact_text=lambda:'[]',red_count=1,blue_count=0,total_items=1)
        node._question_flow.__globals__['StructuredTask']=SimpleNamespace(from_deepseek_result=lambda result:task)
        node.ds_client=SimpleNamespace(solve_task=lambda *args,**kwargs:({},None))
        node._question_flow(1,'question',None,'mapping')
        self.assertEqual(node._status,'VALIDATING_TASK')
        node._on_plan_acceptance(Message(data=json.dumps({'round_id':'round-one','generation':1,'accepted':True})))
        self.assertEqual(node._status,'PLAN_READY')

    def test_reset_publishes_empty_content_and_invalid_plan(self):
        node=self.parser();node.current_question_text='old';node.current_mapping_rule='old'
        node._on_reset(Message(data=True))
        self.assertEqual(node.pub_question.messages[-1].data,'')
        self.assertEqual(node.pub_mapping.messages[-1].data,'')
        payload=json.loads(node.pub_task_plan.messages[-1].data)
        self.assertFalse(payload['valid']);self.assertEqual(payload['ordered_tasks'],[])
        self.assertEqual(node._status,'EMPTY')

    def test_frozen_parser_rejects_mapping_even_before_running_state_arrives(self):
        node=self.parser();node._status='PLAN_READY';node.current_task=object()
        node._task_frozen=True;node.current_mapping_rule='original'
        node._on_mapping_input(Message(data='late edit'))
        self.assertEqual(node.current_mapping_rule,'original')
        self.assertEqual(node._status,'PLAN_READY')

    def test_mapping_change_invalidates_current_plan(self):
        node=self.parser();node._status='PLAN_READY';node.current_task=object()
        node._on_mapping_input(Message(data='new mapping'))
        self.assertIsNone(node.current_task);self.assertEqual(node._status,'PLAN_INVALIDATED')
        self.assertFalse(json.loads(node.pub_task_plan.messages[-1].data)['valid'])

    def test_old_worker_error_cannot_overwrite_reset(self):
        node=self.parser();node._on_reset(Message(data=True))
        node._set_status('PARSE_FAILED',1);node._set_error('old error',1)
        self.assertEqual(node._status,'EMPTY');self.assertEqual(node.pub_last_error.messages,[])

    def test_initialization_failure_has_fault_state(self):
        tree=ast.parse((ROOT/'task_orchestrator_node.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='TaskOrchestratorNode')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_initialize_recognition_pose')
        ns={'TaskState':task_states(),'String':Message}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'startup','exec'),ns)
        node=SimpleNamespace(_arm_client=SimpleNamespace(wait_for_server=lambda **k:False),
                             pub_last_error=Publisher(),_event=lambda *a,**k:None)
        node._transition=lambda state:setattr(node,'state',state)
        ns['_initialize_recognition_pose'](node)
        self.assertEqual(node.state,'INITIALIZATION_FAILED')

    def test_node_accepts_control_transition_keyword(self):
        tree=ast.parse((ROOT/'task_orchestrator_node.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='TaskOrchestratorNode')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_transition')
        self.assertIn('control',[arg.arg for arg in method.args.args])

class GraspStopTest(unittest.TestCase):
    def executor(self):
        cls=production(PERCEPTION/'grasp_executor.py','GraspExecutor',Parameter=object,
                       AttachLink=SimpleNamespace(Request=Message), DetachLink=SimpleNamespace(Request=Message))
        node=cls.__new__(cls)
        node._operation_lock=threading.Lock();node._motion_lock=threading.RLock()
        node._stop_event=threading.Event();node._pending_calls=[];node._pending_sends=[];node._active_goals=[]
        node._attached=False;node._attachment_unknown=False;node.allow_attach=True;node.allow_detach=True
        node._at_contact=True;node.cube_model='red_cube_1';node._publish=lambda *a,**k:None
        node._clear_plan=lambda:None;node.detach_timeout=.5
        return node

    def test_late_attach_is_settled_before_stop_and_then_released(self):
        node=self.executor();future=Future();entered=threading.Event()
        def attach_call(request): entered.set();return future
        node._attach=SimpleNamespace(wait_for_service=lambda **k:True,call_async=attach_call)
        response=Message(success=False,message='')
        thread=threading.Thread(target=lambda:node._on_attach(None,response))
        thread.start();self.assertTrue(entered.wait(1))
        stop=Message(success=False,message='');node._on_stop(None,stop)
        self.assertFalse(stop.success);thread.join(1);self.assertFalse(thread.is_alive())
        self.assertTrue(node._attachment_unknown)
        future.set_result(Message(success=True,message='attached'))
        node._on_stop(None,stop);self.assertTrue(stop.success);self.assertTrue(node._attached)
        detached=Future();detached.set_result(Message(success=True,message='detached'))
        node._detach=SimpleNamespace(wait_for_service=lambda **k:True,call_async=lambda r:detached)
        release=Message(success=False,message='');node._on_abort(None,release)
        self.assertTrue(release.success);self.assertFalse(node._attached)

    def test_reset_does_not_clear_unsettled_service_or_attached_state(self):
        node=self.executor();pending=Future();node._pending_calls.append(pending)
        response=Message(success=False,message='');node._on_reset(None,response)
        self.assertFalse(response.success)
        pending.set_result(None);node._attached=True
        node._on_reset(None,response);self.assertFalse(response.success);self.assertTrue(node._attached)

    def test_grasp_state_publisher_matches_durable_subscriber(self):
        tree=ast.parse((PERCEPTION/'grasp_executor.py').read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)
               and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher'
               and any(isinstance(arg,ast.Constant) and arg.value=='/competition/grasp_execution_status' for arg in n.args)]
        self.assertEqual(len(calls),1)
        self.assertFalse(isinstance(calls[0].args[2],ast.Constant),
                         'durable subscriber cannot match a volatile publisher')

    def test_stopped_sequence_rejects_late_execute_request(self):
        cls=production(PERCEPTION/'pick_sequence_orchestrator.py','PickSequenceOrchestrator')
        node=cls.__new__(cls);node._stop_event=threading.Event();node._stop_event.set()
        response=Message(success=False,message='');node._on_execute(None,response)
        self.assertFalse(response.success);self.assertIn('stopped',response.message)


if __name__ == "__main__": unittest.main()
