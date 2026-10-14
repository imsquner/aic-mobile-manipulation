"""Behavioral tests for cleanup ordering and late action responses; no ROS graph."""
import ast
from datetime import datetime
from zoneinfo import ZoneInfo
import math
import threading
import time
import unittest
from concurrent.futures import Future
from types import SimpleNamespace

from test_control_ui_safety import ROOT, Message, Publisher, Logger, production, task_states

STATES = task_states()
CONTROL = production(ROOT / 'orchestration/control_flow.py', 'ControlFlowMixin',
                     TaskState=STATES, Future=Future, math=math, datetime=datetime, ZoneInfo=ZoneInfo,
                     ARM_JOINTS=[f'joint{i}' for i in range(1,7)],
                     POSE_PRE_GRASP=[0,.4,.9,0,-1.2,0],
                     Trigger=SimpleNamespace(Request=Message))
STATE_PUBLISHER = production(ROOT / 'orchestration/states.py', 'StatePublishingMixin')

def node_wait_method():
    tree = ast.parse((ROOT / 'task_orchestrator_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskOrchestratorNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_wait_future')
    ns = {'time': time}
    exec(compile(ast.Module(body=[method], type_ignores=[]), 'production_wait', 'exec'), ns)
    return ns['_wait_future']

class Client:
    def __init__(self, success=True, message='round-new'):
        self.success, self.message, self.calls = success, message, 0
    def service_is_ready(self): return True
    def wait_for_service(self, timeout_sec): return True
    def call_async(self, request):
        self.calls += 1
        future = Future(); future.set_result(Message(success=self.success, message=self.message))
        return future

class Harness(CONTROL, STATE_PUBLISHER):
    _wait_future = node_wait_method()
    def __init__(self):
        self._init_control_flow({})
        self.state = 'GRASPING'
        self._phase_timer = SimpleNamespace(enter=lambda state: None)
        self._data_lock = threading.RLock()
        self._exec_thread = self._startup_pose_thread = None
        self._stop_exec = True
        self._attached_cube = 'red_cube_1'
        self._attachment_state = 'ATTACHED'
        self.start_time = None
        self._frozen_elapsed = 12
        self._task_plan_frozen = True
        self._parser_enabled = True
        self.reset_scene_on_reset = True
        self.current_task = object(); self.expanded_items = [1]
        self.completed_count = 2
        self.events = []; self.steps = []
        self._placed_cube_regions = {'red_cube_2': {'x':1}}
        self._placed_cube_obstacle_positions = {'red_cube_2':(1,1)}
        for attr in ('pub_state','pub_step','pub_attachment','pub_updated','pub_navigation','pub_manipulation',
                     'pub_last_error','pub_current_color','pub_current_zone','pub_current_item','pub_current_group_count','pub_total'):
            setattr(self, attr, Publisher())
        self._parser_clear_client = Client(); self._parser_cancel_client = Client()
        self._grasp_abort_client = Client(); self._pick_stop_client = Client(); self._grasp_stop_client = Client()
        self._latest_joint_positions = dict(zip([f'joint{i}' for i in range(1,7)], [0,.4,.9,0,-1.2,0]))
        self._latest_joint_time = time.monotonic()
    def _event(self,*a,**k): self.events.append((a,k))
    def get_logger(self): return Logger()
    def _publish_precision_stop(self): pass
    def _cancel_navigation(self): self._cancel_tracked_goals()
    def _cancel_trajectories(self): self._cancel_tracked_goals()
    def _should_stop(self):
        if self._is_control_worker():
            self._control_checkpoint(); return False
        return self._stop_exec
    def _wait_control_stopped(self): self.steps.append('stopped'); return True
    def _wait_for_base_still(self,**kwargs): return True
    def _wait_arm_still(self): return True
    def _arm(self,*args): self.steps.append('pose'); return True
    def _reset_scene(self): self.steps.append('scene'); return True, {}
    def _finalize_reset_localization(self,*args):return True, {"verified":True}
    def _set_navigation_profile(self,*args): self.steps.append('profile'); return True
    def _clear_navigation_costmaps(self): self.steps.append('costmap'); return True
    def _publish_placed_cargo_obstacles(self): pass
    def _reset_run_counters(self):
        self.steps.append('counters'); self.completed_count = 0
        self._placed_cube_regions.clear(); self._placed_cube_obstacle_positions.clear()
    def run(self, mode):
        self._control_mode = mode
        self.state = 'RESETTING' if mode == 'reset' else 'CANCELLING'
        self._control_thread = threading.current_thread()
        self._control_worker()

class Goal:
    accepted = True
    def __init__(self): self.result = Future(); self.cancel_count = 0
    def get_result_async(self): return self.result
    def cancel_goal_async(self): self.cancel_count += 1; return Future()

class ControlFlowTest(unittest.TestCase):
    def test_cancel_preserves_scene_and_counts(self):
        node=Harness(); task=node.current_task; node.run('cancel')
        self.assertEqual(node.state,'CANCELLED')
        self.assertIs(node.current_task,task)
        self.assertEqual(node.completed_count,2)
        self.assertNotIn('scene',node.steps)
        self.assertEqual(node._attached_cube,None)
        self.assertEqual(node.steps,['stopped','pose','profile'])

    def test_reset_commits_only_after_scene_pose_and_navigation_verified(self):
        node=Harness(); node.run('reset')
        self.assertEqual(node.state,'READY')
        self.assertEqual(node.steps,['stopped','scene','pose','profile','costmap','counters'])
        self.assertIsNone(node.current_task)
        self.assertEqual(node.expanded_items,[])
        self.assertEqual(node.completed_count,0)
        self.assertFalse(node._stop_exec)
        self.assertFalse(node._task_plan_frozen)

    def test_release_failure_does_not_move_arm_or_claim_cancelled(self):
        node=Harness(); node._grasp_abort_client.success=False; node.run('cancel')
        self.assertEqual(node.state,'CANCEL_FAILED')
        self.assertNotIn('pose',node.steps)
        self.assertEqual(node._attached_cube,'red_cube_1')

    def test_scene_failure_preserves_occupancy_and_counts(self):
        node=Harness(); node._reset_scene=lambda:(False,'cube failed'); node.run('reset')
        self.assertEqual(node.state,'RESET_FAILED')
        self.assertEqual(node.completed_count,2)
        self.assertIn('red_cube_2',node._placed_cube_regions)
        self.assertNotIn('pose',node.steps)
        self.assertIsNone(node.current_task)

    def test_late_goal_is_cancelled_and_not_declared_stopped_early(self):
        node=Harness(); pending=Future(); node._track_action_send(pending)
        self.assertFalse(node._motion_settled())
        goal=Goal(); pending.set_result(goal)
        self.assertEqual(goal.cancel_count,1)
        self.assertFalse(node._motion_settled())
        goal.result.set_result(Message())
        self.assertTrue(node._motion_settled())

    def test_recovery_goal_is_allowed_despite_task_stop_flag(self):
        node=Harness(); node._control_thread=threading.current_thread()
        node._control_local.generation=node._control_generation
        pending=Future(); node._track_action_send(pending); goal=Goal(); pending.set_result(goal)
        self.assertEqual(goal.cancel_count,0)

    def test_timed_out_send_is_cancelled_even_without_operator_cancel(self):
        node=Harness(); node._stop_exec=False; pending=Future(); node._track_action_send(pending)
        self.assertFalse(node._wait_future(pending,0))
        goal=Goal(); pending.set_result(goal)
        self.assertEqual(goal.cancel_count,1)

    def test_repeated_cancel_is_merged_and_reset_upgrades_same_worker(self):
        node=Harness(); entered=threading.Event(); release=threading.Event()
        def stopped():
            entered.set(); release.wait(1); node._control_checkpoint(); return True
        node._wait_control_stopped=stopped
        node._request_control('cancel'); self.assertTrue(entered.wait(1))
        worker=node._control_thread; generation=node._control_generation
        node._request_control('cancel'); self.assertEqual(node._control_generation,generation)
        node._request_control('reset'); self.assertIs(node._control_thread,worker)
        release.set(); worker.join(2)
        self.assertFalse(worker.is_alive()); self.assertEqual(node.state,'READY')

    def test_cancel_during_reset_does_not_commit_reset_counters(self):
        node=Harness(); entered=threading.Event(); release=threading.Event()
        def scene():
            entered.set(); release.wait(1); node._control_checkpoint(); return True,{}
        node._reset_scene=scene
        node._request_control('reset'); self.assertTrue(entered.wait(1))
        worker=node._control_thread; node._request_control('cancel'); release.set(); worker.join(2)
        self.assertFalse(worker.is_alive()); self.assertEqual(node.state,'CANCELLED')
        self.assertEqual(node.completed_count,2)

    def test_completed_result_exception_is_not_terminal_confirmation(self):
        node=Harness(); pending=Future(); node._track_action_send(pending)
        goal=Goal(); pending.set_result(goal); goal.result.set_exception(RuntimeError('transport failed'))
        self.assertFalse(node._motion_settled())

    def test_single_cached_joint_sample_is_not_proof_of_stationarity(self):
        node=Harness(); node._control_timeouts['pose']=.4
        self.assertFalse(CONTROL._wait_arm_still(node))

if __name__ == '__main__': unittest.main()
