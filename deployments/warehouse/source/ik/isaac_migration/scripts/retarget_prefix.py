"""Offline: shift the verified center lid/cylinder prefix by a camera-observed jig XY.

Only the target jig moves in the 25-location grid (source parts stay). Transport and
placement commands are translated by the observed jig displacement with step86
KinematicRetarget (pad-midpoint IK, wrist orientation held, bounded=3 line search).
Grasps stay unchanged. The weight ramps to one and holds:
  lid:      after the lid clears the source (Isaac frames 523..633, step86 ramp length);
            step86 ramped from 50% of the primitive, which here starts before lift.
  cylinder: step86 retarget window, demo157 target_entry -> Isaac frame 1623, +25 demo frames.
MuJoCo kinematics only. No physics state, no product truth.
"""
import argparse,hashlib,json
from pathlib import Path
import mujoco,numpy as np
from third_stage_path import Kinematics,rotation_error,AUDIT

LID_RAMP=(523,633)
CYL_ANCHOR_DEMO,CYL_START,SLOWDOWN=324,1056,3

def cylinder_ramp():
    record=next(r for r in json.loads((AUDIT/'inventory.json').read_text())['episodes'] if r['episode']==157)
    begin=record['transport_cycles'][1]['target_entry']
    return CYL_START+(begin-CYL_ANCHOR_DEMO)*SLOWDOWN,CYL_START+(begin+25-CYL_ANCHOR_DEMO)*SLOWDOWN

class KinematicRetarget:
    """visual_grid_experiment.KinematicRetarget with bounded=3, per-frame weight given explicitly."""
    def __init__(self,env):
        self.env=env;self.model=env.model;self.data=mujoco.MjData(env.model);self.body=env.model.body('gripper_link').id
    def pose(self,q):
        self.data.qpos[self.env.qadr]=q;mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
        p=(self.data.site('pad_proxy_0').xpos+self.data.site('pad_proxy_1').xpos)/2
        return p.copy(),self.data.xmat[self.body].reshape(3,3).copy()
    def shift(self,qn,delta):
        b=self.env.bounds;qn=np.clip(qn,b[:,0],b[:,1]);p0,r0=self.pose(qn);desired=p0+delta;q=qn.copy();ow=.015
        for _ in range(40):
            p,r=self.pose(q);jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp);mujoco.mj_jac(self.model,self.data,jp,jr,p,self.body)
            J=np.vstack([jp[:,self.env.vadr[:5]],ow*jr[:,self.env.vadr[:5]]]);err=np.r_[desired-p,ow*rotation_error(r0,r)]
            blocked=np.zeros(5,dtype=bool);dq=np.zeros(5)
            for _ in range(6):
                free=~blocked
                if not free.any():break
                A=J[:,free];dq[free]=np.linalg.solve(A.T@A+4e-6*np.eye(free.sum()),A.T@err)
                outward=free&(((q[:5]<=b[:5,0]+1e-8)&(dq<0))|((q[:5]>=b[:5,1]-1e-8)&(dq>0)))
                if not outward.any():break
                blocked|=outward;dq[blocked]=0.
            dq*=min(1.,.12/max(1e-12,abs(dq).max()));cost=float(err@err)
            for _ in range(10):
                proposal=q.copy();proposal[:5]=np.clip(q[:5]+dq,b[:5,0],b[:5,1]);pp,rr=self.pose(proposal);ee=np.r_[desired-pp,ow*rotation_error(r0,rr)]
                if ee@ee<=cost:q=proposal;break
                dq*=.5
            if abs(dq).max()<1e-7:break
        p,r=self.pose(q)
        return q,float(np.linalg.norm(p-desired)*1000),float(np.linalg.norm(rotation_error(r0,r))*180/np.pi)

def weights(n):
    w=np.zeros(n);c0,c1=cylinder_ramp()
    for (a,b),lo,hi in [(LID_RAMP,0,966),((c0,c1),1056,n)]:
        t=np.clip((np.arange(n)-a)/(b-a),0,1);s=t*t*(3-2*t);w[lo:hi]=s[lo:hi]
    return w

def retarget(target_q,observed_offset_mm):
    """Return shifted commands and diagnostics. Frames 966..1055 (live connector) are left unchanged."""
    q=np.asarray(target_q,dtype=float);offset=np.asarray(observed_offset_mm,dtype=float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)!=2148:raise ValueError('Expected the 2148-frame continuous prefix')
    if offset.shape!=(2,) or not np.isfinite(offset).all() or np.max(abs(offset))>12:raise ValueError('Observed jig offset must be finite and within +/-12mm')
    env=Kinematics();ik=KinematicRetarget(env);w=weights(len(q));out=q.copy();errors=[0.];angles=[0.]
    for i in np.flatnonzero(w>0):
        out[i],e,ang=ik.shift(q[i],np.r_[offset/1000,0]*w[i]);errors.append(e);angles.append(ang)
    if np.any(out<env.bounds[:,0]-1e-6) or np.any(out>env.bounds[:,1]+1e-6):raise ValueError('Retarget outside step86 replay bounds')
    diag=dict(observed_offset_mm=offset.tolist(),lid_ramp_frames=list(LID_RAMP),cylinder_ramp_frames=list(cylinder_ramp()),shifted_frames=int((w>0).sum()),
              max_fk_translation_residual_mm=max(errors),max_fk_orientation_change_deg=max(angles),gripper_unchanged=bool(np.array_equal(out[:,5],q[:,5])),
              grasp_frames_unchanged=bool(np.array_equal(out[:LID_RAMP[0]],q[:LID_RAMP[0]]) and np.array_equal(out[966:cylinder_ramp()[0]],q[966:cylinder_ramp()[0]])),
              product_truth_used=False,physics_state_used=False)
    return out,diag

def main():
    p=argparse.ArgumentParser();p.add_argument('--trajectory',type=Path,required=True);p.add_argument('--observed-offset-mm',type=float,nargs=2,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    with np.load(a.trajectory,allow_pickle=False) as z:data={k:z[k].copy() for k in z.files}
    data['target_q_rad'],diag=retarget(data['target_q_rad'],a.observed_offset_mm)
    diag.update(source_trajectory=str(a.trajectory),source_sha256=hashlib.sha256(a.trajectory.read_bytes()).hexdigest())
    a.output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(a.output,**data);a.output.with_suffix('.json').write_text(json.dumps(diag,indent=2)+'\n')

if __name__=='__main__':main()
