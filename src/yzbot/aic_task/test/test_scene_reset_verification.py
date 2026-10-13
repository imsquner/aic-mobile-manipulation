"""Service success cannot substitute for actual position/localization evidence."""
import math
import unittest
from test_control_ui_safety import ROOT, production

RESET=production(ROOT/'scene/reset.py','SceneResetMixin',math=math)

class SceneHarness(RESET):
    def __init__(self, apply=True):
        self.points={'red_cubes':[{'id':'red_cube_1','position':{'x':1.,'y':2.,'z':.015}}],
                     'blue_cubes':[], 'robot_initial_pose':{'x':0.,'y':0.,'yaw':0.}}
        self.robot_model_name='six_arm';self._attached_cube=None
        self._reset_verify_timeout=.01;self._reset_position_tolerance=.03;self._reset_yaw_tolerance=.05
        self.positions={'red_cube_1':(9,9,.015,0),'six_arm':(9,9,0,0)}
        self.apply=apply;self.events=[]
        self._placed_cube_regions={'red_cube_1':{'x':9}}
        self._placement_slot_states={'A1':'verified'}
    def _control_checkpoint(self): pass
    def _set_step(self,text): pass
    def _event(self,*args,**kwargs): self.events.append((args,kwargs))
    def _set_entity_pose(self,name,x,y,z=0,yaw=0):
        if self.apply:self.positions[name]=(x,y,z,yaw)
        return True
    def _entity_pose(self,name):return self.positions.get(name)
    def _invalidate_localization_reference(self,*args):pass
    def _synchronize_localization_to_gazebo(self,*args,**kwargs):return True
    def _localization_consistency(self):return {'drifted':False}
    def _wait_for_base_still(self,**kwargs):return True

class SceneResetEvidenceTest(unittest.TestCase):
    def test_success_response_but_wrong_pose_is_failure(self):
        node=SceneHarness(apply=False);success,detail=node._reset_scene()
        self.assertFalse(success);self.assertIn('red_cube_1',detail['cubes_failed'])
        self.assertIn('red_cube_1',node._placed_cube_regions)
        self.assertEqual(node._placement_slot_states,{'A1':'verified'})
    def test_localization_failure_prevents_success(self):
        node=SceneHarness();node._synchronize_localization_to_gazebo=lambda *args,**kwargs:False
        success,detail=node._reset_scene();self.assertFalse(success)
        self.assertIn('localization',detail);self.assertIn('A1',node._placement_slot_states)
    def test_verified_scene_still_leaves_final_bookkeeping_to_control_owner(self):
        node=SceneHarness();success,detail=node._reset_scene();self.assertTrue(success)
        self.assertEqual(node.positions['red_cube_1'],(1.,2.,.015,0))
        self.assertIn('red_cube_1',node._placed_cube_regions)
        self.assertEqual(node._placement_slot_states,{'A1':'verified'})

if __name__=='__main__':unittest.main()
