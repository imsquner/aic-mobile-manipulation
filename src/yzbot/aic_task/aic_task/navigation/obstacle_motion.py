"""Finite, bounded obstacle trajectories advanced only by simulation time."""
import math
from pathlib import Path
import yaml

HOLD_STATES={'BOOTING','RESETTING','RESET_FAILED','CANCELLING','CANCELLED','CANCEL_FAILED','TASK_SUCCEEDED','TASK_FAILED','TIME_LIMIT','API_ERROR'}

class ObstacleMotion:
    def __init__(self, spec):
        self.name=str(spec['name']);self.z=float(spec.get('z',.15));self.yaw=float(spec.get('yaw',0.))
        self.dimensions=tuple(float(v) for v in spec.get('dimensions',[.4,.4,.5]))
        self.radius=float(spec.get('safety_radius_m',.65));m=spec['motion']
        if m.get('type')!='sine_x':raise ValueError('only sine_x is supported for world2')
        self.center=(float(m['center_x']),float(m['center_y']));self.amplitude=float(m['amplitude_m']);self.omega=float(m['omega_rad_s']);self.phase=float(m.get('phase_rad',0.))
        values=(*self.center,*self.dimensions,self.z,self.yaw,self.radius,self.amplitude,self.omega,self.phase)
        if (not self.name or len(self.dimensions)!=3 or not all(math.isfinite(v) for v in values)
                or min(self.dimensions)<=0 or self.z<=0 or self.amplitude<=0 or self.omega<=0 or self.radius<=0):raise ValueError('invalid obstacle dimensions or trajectory')
        self.ready=False;self.state='BOOTING';self.reset()

    def observe_state(self,state):
        self.state=str(state)
        if state=='READY':self.ready=True

    def reset(self):
        self.elapsed=0.;self.clock_fault=False
        self.position=(self.center[0]+self.amplitude*math.sin(self.phase),self.center[1])

    def advance(self,dt,robot_xy,robot_fresh):
        if not math.isfinite(dt) or dt<0 or dt>.5:self.clock_fault=True
        if (self.clock_fault or not self.ready or self.state in HOLD_STATES or dt==0
                or not robot_fresh or robot_xy is None or not all(math.isfinite(v) for v in robot_xy)):
            return self.position
        candidate=(self.center[0]+self.amplitude*math.sin(self.omega*(self.elapsed+dt)+self.phase),self.center[1])
        # Check the complete short segment, not just its endpoint.
        nearest_x=min(max(robot_xy[0],min(self.position[0],candidate[0])),max(self.position[0],candidate[0]))
        if math.hypot(nearest_x-robot_xy[0],candidate[1]-robot_xy[1])<self.radius:return self.position
        self.elapsed+=dt;self.position=candidate;return candidate


def load_obstacle_motions(path):
    config=yaml.safe_load(Path(path).read_text()) or {};entries=config.get('dynamic_obstacles',[])
    if not entries:raise ValueError(f'{path} defines no dynamic_obstacles')
    result={}
    for spec in entries:
        motion=ObstacleMotion(spec)
        if motion.name in result:raise ValueError('duplicate dynamic obstacle name')
        result[motion.name]=motion
    return result
