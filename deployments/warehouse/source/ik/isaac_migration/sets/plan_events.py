"""OFFLINE: grasp/release events of a verified real plan and the pad pose at each (MuJoCo kinematics only).

A real plan (real_*_plan.py / real_demo_plan.py output) carries q_rad at 15Hz. The gripper command closes on
each part and opens to release it; the pad midpoint (pad_proxy_0/1) and the gripper_link rotation at those
frames say where the robot actually took and left each part on the real cell. Calibrating the simulator
scene to these poses makes the verified plan the simulator's nominal path (scene_from_plan.py).
"""
import json,sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE.parent/'scripts')]


def events(gripper_rad,pad_z_m,drop_deg=4.,window=8,grasp_below_m=.045,lift_m=.03):
    """(close_end, release_start) frame pairs. A close is a gripper fall of more than drop_deg over `window`
    frames with the pad below grasp_below_m (the demonstrations also re-close partly while lifting away after
    a release, high above the cell); it ends where the command stops falling. A release is the next rise of
    more than drop_deg after the part was lifted by lift_m (small rises right after a close only relieve the
    squeeze), starting at the last frame before the rise."""
    g=np.degrees(np.asarray(gripper_rad));z=np.asarray(pad_z_m);n=len(g);out=[];closed=None;k=window
    while k<n-window:
        if closed is None and g[k]-g[k-window]<-drop_deg and z[k]<grasp_below_m:
            while k+1<n and g[k+1]<g[k]-.05:k+=1
            closed=k
        elif closed is not None and g[k+window]-g[k]>drop_deg and z[closed:k+1].max()-z[closed]>lift_m:
            while k+1<n and g[k+1]-g[k]<=.05:k+=1
            out.append((closed,k));closed=None;k+=window
        k+=1
    return out


class Pad:
    def __init__(self):
        from third_stage_path import Kinematics
        from retarget_prefix import KinematicRetarget
        self.ik=KinematicRetarget(Kinematics())

    def __call__(self,q):
        p,r=self.ik.pose(np.asarray(q));d=self.ik.data
        # pad_proxy_0 is on the fixed finger, pad_proxy_1 on the moving jaw: the jaw pushes the part onto the fixed pad.
        return dict(pad_m=p.tolist(),fixed_pad_m=d.site('pad_proxy_0').xpos.tolist(),moving_pad_m=d.site('pad_proxy_1').xpos.tolist(),
                    yaw_deg=float(np.degrees(np.arctan2(r[1,0],r[0,0]))),gripper_z_axis=r[:,2].tolist())


def plan_events(plan,settle=3):
    """Per part: close/release frames, pad pose where it closed on the part (lowest pad) and where it let go."""
    q=np.array(plan['q_rad']);pad=Pad();z=np.array([pad(x)['pad_m'][2] for x in q]);parts=[]
    for i,(c,r) in enumerate(events(q[:,5],z)):
        g=c-20+int(np.argmin(z[c-20:c+settle+1]))   # lowest pad while closing: the slow closes already lift
        parts.append(dict(part=i+1,close_frame=c,release_frame=r,grasp=dict(frame=g,**pad(q[g])),release=dict(frame=r,**pad(q[r]))))
    return parts


if __name__=='__main__':
    for f in sys.argv[1:]:
        print(f);print(json.dumps(plan_events(json.loads(Path(f).read_text())),indent=1))
