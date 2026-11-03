"""OFFLINE: inputs for one placement episode (place_episode.py) of a set part, optionally retargeted.

Writes <out>.json (start/release/end frames, in-hand pose, simulator gripper offset) and, with a shift,
<out>.npz (q_rad of the whole plan with the part's placement window moved by step86 KinematicRetarget, the
same IK as real_plan_shift.py: ramp in from the transport apex, hold through the release, ramp out on the
lift). The shift is the camera-observed jig offset plus an optional policy residual (world XY, mm).
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parent/'scripts'),str(HERE.parent/'real')]
from plan_events import Pad
from scene_from_plan import JOINTS

RAMP_IN,LIFT_HOLD,RAMP_OUT=12,10,12   # frames
CLEAR_Z,LIFT_MAX=.035,150   # the episode ends once the pad is this far above its release height
APEX_BAND=.015
HOLD_CLEARANCE_DEG=3.   # the hand carries the part kinematically: keep the rigid fins just off it
SEAT_CLEARANCE=.0005    # released this far above the seated goal


def episode_frames(q,event,pad):
    """Start where the transport first reaches its high level (pad within APEX_BAND of the highest point between
    the grasp and the release; the earliest such frame leaves room to shift before the descent), end once the
    pad has risen CLEAR_Z above its release height (the audit needs the hand 30mm clear of the part)."""
    c,r=event['close_frame'],event['release_frame'];z=np.array([pad(x)['pad_m'][2] for x in q[c:r]])
    after=np.array([pad(x)['pad_m'][2] for x in q[r:r+LIFT_MAX]]);up=np.flatnonzero(after>=after[0]+CLEAR_Z)
    return dict(start_frame=c+int(np.argmax(z>=z.max()-APEX_BAND)),release_frame=r,end_frame=min(len(q)-1,r+(int(up[0]) if len(up) else LIFT_MAX)))


def in_hand(scene_xml,q_release,goal):
    """Part pose relative to gripper_link such that at the plan's release frame the part sits SEAT_CLEARANCE above
    its seated goal: the real plan placed it there at jig 0, and the grasp itself is not simulated. (A pose
    carried over from the grasp lands 1-2mm off the tight purple pockets through in-hand tilt.)"""
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(scene_xml));d=mujoco.MjData(m)
    d.qpos[[m.joint(n).qposadr[0] for n in JOINTS]]=q_release;mujoco.mj_forward(m,d)
    g=m.body('gripper_link').id;gp=d.xpos[g].copy();gr=R.from_matrix(d.xmat[g].reshape(3,3))
    G=np.array(goal);pos=G[:3,3]+np.r_[0,0,SEAT_CLEARANCE]
    return dict(in_hand_pos_m=gr.inv().apply(pos-gp).tolist(),in_hand_quat_wxyz=np.roll((gr.inv()*R.from_matrix(G[:3,:3])).as_quat(),1).tolist())


def roll_for_yaw(q,yaw_deg,pad):
    """Wrist-roll change (rad) that turns the gripper by yaw_deg about the vertical at pose q (finite difference)."""
    a,b=np.array(q,float).copy(),np.array(q,float).copy();a[4]-=np.radians(1);b[4]+=np.radians(1)
    k=(pad(b)['yaw_deg']-pad(a)['yaw_deg'])/2.
    return np.radians(yaw_deg/k)


def build(scene_dir,part,shift_mm=(0.,0.),yaw_deg=0.):
    cal=json.loads((Path(scene_dir)/'calibration.json').read_text());plan=json.loads(Path(cal['plan']).read_text());q=np.array(plan['q_rad'])
    e=cal['parts'][part]['event'];pad=Pad();f=episode_frames(q,e,pad)
    hand=dict(set=cal['set'],part=part,**f,**in_hand(Path(scene_dir)/'scene.xml',q[f['release_frame']],cal['parts'][part]['calibrated_goal_pose']),
              gripper_offset_deg=(cal['parts'][part]['grip_fit']['gripper_offset_deg'] or 0.)+HOLD_CLEARANCE_DEG,shift_mm=list(map(float,shift_mm)))
    if not any(shift_mm) and not yaw_deg:return hand,None
    from real_lid_plan import place_shift
    s,r=f['start_frame'],f['release_frame']
    ramp=(s,s+RAMP_IN,r+LIFT_HOLD,r+LIFT_HOLD+RAMP_OUT)
    new,info=place_shift(q,np.r_[shift_mm,0.],ramp=ramp) if any(shift_mm) else (q.copy(),{})
    if yaw_deg:
        # A rotated pallet turns the jig pockets: the wrist rolls by the same yaw over the same window.
        k=np.arange(len(q));sm=lambda a,b:np.clip((k-a)/(b-a),0,1)**2*(3-2*np.clip((k-a)/(b-a),0,1))
        w=sm(*ramp[:2])-sm(*ramp[2:]);droll=roll_for_yaw(new[r],yaw_deg,pad);new[:,4]+=droll*w
        info['wrist_roll_deg']=float(np.degrees(droll))
    hand.update(shift_ramp=list(ramp),shift_ik=info,yaw_deg=float(yaw_deg))
    return hand,new


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--part',required=True)
    p.add_argument('--shift-mm',type=float,nargs=2,default=[0.,0.]);p.add_argument('--yaw-deg',type=float,default=0.);p.add_argument('--output',type=Path,required=True,help='path stem: writes .json (+ .npz)')
    a=p.parse_args()
    hand,q=build(a.scene,a.part,a.shift_mm,a.yaw_deg)
    if q is not None:np.savez(a.output.with_suffix('.npz'),q_rad=q);hand['commands']=str(a.output.with_suffix('.npz'))
    a.output.with_suffix('.json').write_text(json.dumps(hand,indent=1)+'\n');print(json.dumps({k:hand[k] for k in ['part','start_frame','release_frame','end_frame','gripper_offset_deg','shift_mm']}))

if __name__=='__main__':main()
