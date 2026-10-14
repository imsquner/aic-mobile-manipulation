"""Behavioral regressions for diagnosed reset-reference race and fresh evidence."""
import math
from types import SimpleNamespace
import pytest
from test_control_ui_safety import ROOT, production, Publisher
from test_control_flow import Harness


def gate(**kwargs):
    cls = production(ROOT / 'navigation/localization.py', 'LocalizationMixin', math=math)
    factory = getattr(cls, '_new_reset_localization_gate', None)
    assert callable(factory), 'reset localization needs a new-data/stability gate'
    return factory(SimpleNamespace(), (0., 0., 0.), 1_000_000_000, 0., 7, **kwargs)


def sample(t, reference=(0., 0., 0.), target=(0., 0., 0.)):
    return {'stamp_ns': int((2+t)*1e9), 'map_base': target,
            'odom_base': (0., 0., 0.), 'map_odom': reference}


def observe(g, t, **kwargs):
    return g.observe(kwargs.pop('snapshot', sample(t)),
                     kwargs.pop('amcl', {'sequence': 8, 'receipt_mono': .01, 'stamp_ns': 2_000_000_000,
                                        'frame_id': 'map', 'pose': (0., 0., 0.)}),
                     now_mono=t, ros_now_ns=int((2+t)*1e9),
                     base_speed=kwargs.pop('base_speed', (0., 0.)), odom_age_sec=.01)


@pytest.mark.parametrize('shift', [1.5657128267402427, .769100470398591])
def test_legitimate_reference_rebuild_passes_only_after_stable_fresh_samples(shift):
    g = gate()
    assert not observe(g, 0., snapshot=sample(0., (shift, 0., 0.)))['success']
    for t in [.1, .2, .3, .4, .5]:
        assert not observe(g, t)['success']
    assert observe(g, .61)['success']


def test_cached_target_without_new_amcl_does_not_confirm_reset():
    g = gate()
    result = observe(g, .8, amcl={'sequence': 7, 'receipt_mono': -.1,
                                  'stamp_ns': 0, 'frame_id': 'map', 'pose': (0, 0, 0)})
    assert not result['success']
    assert result['reason'] == 'LOC_NO_NEW_AMCL'


def test_repeated_tf_stamp_cannot_accumulate_stability():
    g = gate()
    fixed = sample(0)
    for t in [0., .2, .4, .6, .8]:
        assert not observe(g, t, snapshot=fixed)['success']


@pytest.mark.parametrize('change,reason', [
    ({'map_base': (.13, 0., 0.)}, 'LOC_TARGET_XY'),
    ({'map_base': (0., 0., math.radians(9))}, 'LOC_TARGET_YAW'),
    ({'stamp_ns': 100}, 'LOC_TF_STALE_OR_EPOCH'),
    ({'map_base': (float('nan'), 0., 0.)}, 'LOC_TF_UNAVAILABLE'),
])
def test_invalid_localization_never_passes(change, reason):
    g = gate(); snap=sample(0); snap.update(change)
    result=observe(g, 0, snapshot=snap)
    assert not result['success']
    assert result['reason']==reason


def test_moving_base_cannot_pass_localization_stability():
    assert observe(gate(), .9, base_speed=(.02, 0))['reason']=='BASE_NOT_STILL'


def test_clock_rollback_invalidates_the_reset_epoch():
    g=gate();observe(g,.1)
    result=g.observe(sample(.2),None,now_mono=.2,ros_now_ns=500_000_000,base_speed=(0,0),odom_age_sec=.01)
    assert result['reason']=='LOC_CLOCK_ROLLBACK'
    assert not observe(g, 1.)['success']


def test_final_result_is_one_correlated_cleared_snapshot():
    node=Harness();node.pub_reset_progress=Publisher();node.pub_reset_result=Publisher();node.pub_reset_summary=Publisher()
    node._finalize_reset_localization=lambda *a:(True, {'verified':True,'stamp_ns':123})
    node.run('reset')
    assert node.state=='READY'
    assert len(node.pub_reset_result.messages)==1
    import json
    result=json.loads(node.pub_reset_result.messages[0].data)
    assert result['success'] and result['state']=='READY'
    assert result['error']=='' and result['elapsed_seconds']==0
    assert result['counts']=={'completed':0,'failed':0,'total':0}
    assert result['reset_id']==node._reset_id()
    assert result['localization']['verified']


def test_costmap_failure_publishes_failed_result_without_ready():
    node=Harness();node.pub_reset_result=Publisher();node.pub_reset_progress=Publisher();node.pub_reset_summary=Publisher()
    node._clear_navigation_costmaps=lambda:False
    node.run('reset')
    assert node.state=='RESET_FAILED'
    import json
    result=json.loads(node.pub_reset_result.messages[-1].data)
    assert not result['success'] and result['error']


def test_stale_generation_cannot_publish_terminal_success():
    node=Harness();node.pub_reset_result=Publisher();node.pub_reset_summary=Publisher();node._control_mode='reset'
    node._control_local.generation=-1
    method=getattr(node,'_publish_reset_result',None)
    assert callable(method)
    assert method(True,{'verified':True}) is None
    assert not node.pub_reset_result.messages


def test_new_amcl_far_from_target_cannot_hide_behind_old_correct_tf():
    g=gate()
    amcl={'sequence':8,'receipt_mono':.01,'stamp_ns':2_000_000_000,
          'frame_id':'map','pose':(.5,0,0)}
    assert observe(g,.1,amcl=amcl)['reason']=='LOC_TARGET_XY'


def test_snapshot_queries_all_frames_at_one_common_timestamp():
    calls=[]
    def lookup(parent,child,at):
        calls.append((parent,child,at.nanoseconds))
        ns=at.nanoseconds or 2_000_000_000
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)),
              transform=SimpleNamespace(translation=SimpleNamespace(x=1.,y=2.),
              rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    fake_ros=SimpleNamespace(time=SimpleNamespace(Time=lambda nanoseconds=0,**k:SimpleNamespace(nanoseconds=nanoseconds)))
    cls=production(ROOT/'navigation/localization.py','LocalizationMixin',math=math,rclpy=fake_ros)
    node=cls();node._tf_buffer=SimpleNamespace(lookup_transform=lookup)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=3_000_000_000))
    result=node._reset_localization_snapshot(None,(0,0,0))
    assert result['stamp_ns']==2_000_000_000
    assert [c[2] for c in calls[1:]]==[2_000_000_000]*3
    assert set(result['lookup_stamps_ns'].values())=={2_000_000_000}


def test_final_localization_failure_cannot_clear_counters_or_publish_ready():
    node=Harness();node.pub_reset_result=Publisher();node.pub_reset_summary=Publisher()
    node._finalize_reset_localization=lambda *a:(False,{'reason':'LOC_TARGET_XY'})
    node.run('reset')
    assert node.state=='RESET_FAILED'
    assert 'counters' not in node.steps
    import json
    result=json.loads(node.pub_reset_result.messages[-1].data)
    assert not result['success'] and 'LOC_TARGET_XY' in result['error']
