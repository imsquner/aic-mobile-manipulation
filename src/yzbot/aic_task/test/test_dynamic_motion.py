"""Regressions for real-time independent motion, pause continuity and reset proof."""
import importlib,math,sys
from pathlib import Path
import pytest
from test_control_flow import Harness,Client
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def motion():
    assert (ROOT/'aic_task/navigation/obstacle_motion.py').is_file(), 'need bounded simulation-time trajectory'
    cls=importlib.import_module('aic_task.navigation.obstacle_motion').ObstacleMotion
    return cls({'name':'box','z':.15,'yaw':-.7,'dimensions':[.38,.40,.30],
                'safety_radius_m':.65,'motion':{'type':'sine_x','center_x':3.,'center_y':-11.,'amplitude_m':.6,'omega_rad_s':.125,'phase_rad':0.}})

def test_not_moving_until_full_system_ready():
    m=motion();assert m.advance(.1,(0,0),True)==(3.,-11.)
    m.observe_state('READY');assert m.advance(.1,(0,0),True)[0]>3.

def test_simulation_pause_does_not_advance_phase():
    m=motion();m.observe_state('READY');m.advance(.1,(0,0),True);previous=m.position
    for _ in range(100):assert m.advance(0,(0,0),True)==previous

def test_unsafe_pause_resumes_without_phase_jump():
    m=motion();m.observe_state('READY');m.advance(.1,(0,0),True);previous=m.position
    for _ in range(100):assert m.advance(.1,(3.,-11.),True)==previous
    p=m.advance(.1,(0,0),True);assert 0<p[0]-previous[0]<=.0075

def test_unknown_or_stale_robot_cannot_allow_motion():
    m=motion();m.observe_state('READY');assert m.advance(.1,None,True)==(3.,-11.)
    assert m.advance(.1,(0,0),False)==(3.,-11.)

def test_clock_jump_holds_until_verified_reset():
    m=motion();m.observe_state('READY');assert m.advance(-.1,(0,0),True)==(3.,-11.)
    assert m.advance(.1,(0,0),True)==(3.,-11.)
    m.reset();assert m.advance(.1,(0,0),True)[0]>3.

def test_resetting_holds_motion_and_restart_begins_at_center():
    m=motion();m.observe_state('READY');m.advance(.1,(0,0),True);m.observe_state('RESETTING');before=m.position
    assert m.advance(.1,(0,0),True)==before
    m.reset();assert m.position==(3.,-11.);m.observe_state('READY');assert m.advance(.1,(0,0),True)[0]-3.<.008

def test_long_clock_gap_cannot_teleport():
    m=motion();m.observe_state('READY');before=m.position
    assert m.advance(10.,(0,0),True)==before
    assert m.advance(.1,(0,0),True)==before

def test_invalid_configuration_is_rejected():
    assert (ROOT/'aic_task/navigation/obstacle_motion.py').exists()
    cls=importlib.import_module('aic_task.navigation.obstacle_motion').ObstacleMotion
    with pytest.raises(ValueError):cls({'name':'bad','motion':{'type':'sine_x','center_x':float('nan'),'center_y':0,'amplitude_m':.3,'omega_rad_s':.25}})

def test_dynamic_reset_failure_blocks_ready_and_counter_clear():
    node=Harness();node._dynamic_obstacles_enabled=True;node._dynamic_obstacle_names=('moving_box_1','moving_box_2');node._dynamic_reset_client=Client(success=False)
    node.run('reset');assert node.state=='RESET_FAILED';assert 'counters' not in node.steps

def test_dynamic_reset_ack_is_required_not_just_success_boolean():
    node=Harness();node._dynamic_obstacles_enabled=True;node._dynamic_obstacle_names=('moving_box_1','moving_box_2');node._dynamic_reset_client=Client(success=True,message='not physical evidence')
    node.run('reset');assert node.state=='RESET_FAILED'

def test_dynamic_reset_verified_ack_allows_ready():
    node=Harness();node._dynamic_obstacles_enabled=True;node._dynamic_obstacle_names=('moving_box_1','moving_box_2');node._dynamic_reset_client=Client(success=True,message='{"verified":true,"obstacles":{"moving_box_1":{},"moving_box_2":{}}}')
    node.run('reset');assert node.state=='READY';assert node._dynamic_reset_client.calls==1

@pytest.mark.parametrize('fresh_after_reply',[False,True])
def test_reset_requires_model_sample_after_service_reply(fresh_after_reply):
    import ast,threading
    from concurrent.futures import Future
    source=ROOT/'aic_task/navigation/dynamic_obstacle.py';tree=ast.parse(source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DynamicObstacleController');fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_reset')
    timer=SimpleNamespace(value=1.,ticks=0)
    node=SimpleNamespace(_lock=threading.RLock(),_resetting=False,_pending={},_robot_xy=(0,0),_model_receipt=1.,_model_sequence=0,_actual={'box':(3.,-11.,.15,-.7)},obstacles={'box':motion()},_failure='')
    def set_position(name,m):node._pending[name]=Future()
    def sleep(seconds):
        timer.value+=seconds;timer.ticks+=1
        if node._pending:
            node._model_sequence=1
            if timer.ticks>=2:
                for f in node._pending.values():
                    if not f.done():f.set_result(SimpleNamespace(success=True))
            if fresh_after_reply and timer.ticks>=4:node._model_sequence=2
    node._set_position=set_position;node._status=lambda:None;node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000))
    ns={'time':SimpleNamespace(monotonic=lambda:timer.value,sleep=sleep),'math':math,'json':importlib.import_module('json')};exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
    response=ns['_reset'](node,None,SimpleNamespace(success=False,message=''))
    assert response.success is fresh_after_reply


def test_scene_rejects_wrong_obstacle_collision_dimensions(tmp_path):
    import dataclasses,xml.etree.ElementTree as ET
    from aic_task.scene_manager import load_scene,validate_scene,SceneError
    source=ROOT.parents[2];scene=load_scene('world2',ROOT/'config/scenes',source)
    world=source/'src/yzbot/mybot_description/worlds/offic_room.world';tree=ET.parse(world)
    tree.getroot().find("./world/model[@name='moving_box_1']/link/collision/geometry/box/size").text='0.01 0.01 0.01'
    copy=tmp_path/'world.world';tree.write(copy)
    with pytest.raises(SceneError,match='obstacle.*geometry'):
        validate_scene(dataclasses.replace(scene,world_file=copy))


def test_failed_reset_does_not_reuse_previous_obstacle_evidence():
    node=Harness();node._dynamic_obstacles_enabled=True;node._dynamic_obstacle_names=('moving_box_1','moving_box_2');node._dynamic_reset_client=Client(success=False);node._reset_dynamic_evidence={'verified':True}
    assert not node._reset_dynamic_obstacles()
    assert node._reset_dynamic_evidence is None
