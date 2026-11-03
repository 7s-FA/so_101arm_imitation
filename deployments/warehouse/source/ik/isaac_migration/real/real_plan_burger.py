"""OFFLINE: a verified real plan (made on burger m2) retargeted to another docked burger (e.g. m1).

For each part, the placement window (from the transport's high level through the release and the lift) is
moved by that part's measured target-cell offset (burger_<x>/..._offsets.json: m2 -> m1, robot base mm) with
step86 KinematicRetarget, and for square parts the wrist rolls by the pallet yaw - the same edit the Isaac
placement episodes validated (sets/place_commands.py) up to the release; the shift is returned only after the
lift has begun (OUT_RISE). Only changed frames get new raw ticks.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parent/'sets'),str(HERE.parent/'scripts')]
from policy_contract import q_to_raw
from real_lid_plan import REPO,JOINTS,sha,place_shift,MARGIN_TICKS
from plan_events import plan_events,Pad
from place_commands import episode_frames,roll_for_yaw,RAMP_IN,LIFT_HOLD,RAMP_OUT

PARTS={'A_lower':['A_housing','A_ball','A_cap'],'A_upper':['A_housing','A_ball','A_cap'],'B_left':['B_lid','B_TPU','B_base'],'B_right':['B_lid','B_TPU','B_base']}
ORIENTED={'A_housing','B_lid','B_base'}
OUT_RISE=.02
RETREAT_LIFT,RETREAT_CLEAR=.002,.05   # lift = pad 2 mm above where the jaw finished opening; return 50 mm above release


def release_retreat(q,r,pad,offset_mm):
    """Move the opened gripper by offset_mm (base XY) in the hold between the jaw opening and the lift, keep it
    while rising, and return it only RETREAT_CLEAR above the release height (2026-10-30: on the real A_lower
    housing the gripper rubbed the placed part while lifting; the user asked for 5 mm toward D435 up first)."""
    g=q[r:r+30,5];a=r+int(np.argmax(g>=g.max()-np.radians(.5)))
    z=np.array([pad(x)['pad_m'][2] for x in q[a:a+60]]);b=a+int(np.argmax(z>z[0]+RETREAT_LIFT))
    if b-a<3:raise ValueError(f'release retreat needs a hold between the jaw opening ({a}) and the lift ({b})')
    z_release=pad(q[r])['pad_m'][2];high=np.array([pad(x)['pad_m'][2] for x in q[b:b+80]])>=z_release+RETREAT_CLEAR
    if not high.any():raise ValueError('release retreat: the lift never clears 50 mm above the release')
    c=b+int(np.argmax(high));return place_shift(q,np.r_[np.asarray(offset_mm,float),0.],ramp=(a,b,c,c+RAMP_OUT))


def retarget(plan,set_name,offsets,extra=None,retreat=None):
    q=np.array(plan['q_rad']);pad=Pad();edits=[];k=np.arange(len(q))
    sm=lambda a,b:np.clip((k-a)/(b-a),0,1)**2*(3-2*np.clip((k-a)/(b-a),0,1))
    for part,e in zip(PARTS[set_name],plan_events(plan)):
        f=episode_frames(q,e,pad);s,r=f['start_frame'],f['release_frame']
        # Return the shift only once the pad has risen OUT_RISE above its release height: B_left holds the arm
        # still while the gripper opens with wrist_flex at its stop, where a shift cannot keep the wrist pose.
        z=np.array([pad(x)['pad_m'][2] for x in q[r:]]);up=r+int(np.argmax(z>=z[0]+OUT_RISE)) if (z>=z[0]+OUT_RISE).any() else r+LIFT_HOLD
        out=max(r+LIFT_HOLD,up);ramp=(s,s+RAMP_IN,out,out+RAMP_OUT)
        shift=(np.array(offsets['cells'][part]['offset_mm'])+(extra or {}).get(part,[0.,0.])).tolist();q,info=place_shift(q,np.r_[shift,0.],ramp=ramp)
        if part in ORIENTED:
            droll=roll_for_yaw(q[r],offsets['rigid']['rotation_deg'],pad);q[:,4]+=droll*(sm(*ramp[:2])-sm(*ramp[2:]));info['wrist_roll_deg']=float(np.degrees(droll))
        edits.append(dict(info,part=part,offset_mm=shift,extra_mm=(extra or {}).get(part,[0.,0.])))   # info carries the ramp frames
        if (retreat or {}).get(part):
            q,back=release_retreat(q,r,pad,retreat[part]);edits[-1]['release_retreat']=back
    return q,edits


def main():
    p=argparse.ArgumentParser();p.add_argument('plan',type=Path);p.add_argument('--set',choices=PARTS,required=True)
    p.add_argument('--offsets',type=Path,required=True,help='burger offsets json (m2 -> docked burger)');p.add_argument('--extra',type=Path,help='json {part: [x, y] mm} added to the cell offsets (real-robot anchors)')
    p.add_argument('--retreat',type=Path,help='json {part: [x, y] mm}: move the opened gripper this far before lifting');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    plan=json.loads(a.plan.read_text());offsets=json.loads(a.offsets.read_text())
    if sha(plan['robot_calibration'])!=plan['robot_calibration_sha256']:raise ValueError('robot calibration changed since the plan was built')
    settings=json.loads((REPO/'configs/calibration/simulation_settings.json').read_text());cal=json.loads(Path(plan['robot_calibration']).read_text())
    q0=np.array(plan['q_rad']);q,edits=retarget(plan,a.set,offsets,json.loads(a.extra.read_text()) if a.extra else None,
                                                 json.loads(a.retreat.read_text()) if a.retreat else None)
    raw=np.array(plan['raw']);changed=np.flatnonzero(np.abs(q-q0).max(1)>0);raw[changed]=np.rint(q_to_raw(q[changed],settings)).astype(int)
    lo=np.array([cal[j]['range_min'] for j in JOINTS]);hi=np.array([cal[j]['range_max'] for j in JOINTS])
    if np.any(raw<lo+MARGIN_TICKS) or np.any(raw>hi-MARGIN_TICKS):raise ValueError('Retargeted plan comes within the hardware range margin')
    out=dict(plan,raw=raw.tolist(),q_rad=q.tolist(),start_raw=raw[0].tolist(),burger=offsets['burger'],burger_edits=edits,
             source_plan=str(a.plan),source_plan_sha256=sha(a.plan),burger_offsets=str(a.offsets),burger_offsets_sha256=sha(a.offsets))
    rate=np.abs(np.diff(raw,axis=0)).max(0)*plan['command_hz']*360/4096
    out['max_rate_deg_s_after_burger']=dict(zip(JOINTS,np.round(rate,1).tolist()))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=1)+'\n')
    for e in edits:
        print(e['part'],'ramp',e['ramp_frames'],'offset mm',e['offset_mm'],'IK mm %.3f orient deg %.2f'%(e['max_ik_residual_mm'],e['max_orientation_change_deg']),'roll %.2f'%e.get('wrist_roll_deg',0))
        if 'release_retreat' in e:t=e['release_retreat'];print('  release retreat mm',t['place_offset_mm'],'ramp',t['ramp_frames'],'IK mm %.3f'%t['max_ik_residual_mm'])
    print('max rate',out['max_rate_deg_s_after_burger'],'frames',len(raw),round(len(raw)/plan['command_hz'],1),'s')

if __name__=='__main__':main()
