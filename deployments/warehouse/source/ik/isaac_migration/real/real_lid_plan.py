"""OFFLINE: build the first-lid real-robot command plan from the selected Isaac candidate.

No hardware access. Converts the candidate's verified lid commands (frames 0..965 of
the selected two-part prefix, nominal jig) from simulator joint radians to servo raw
ticks with the same calibration mapping the simulator uses, applies the simulator's
20 deg/s target slew, checks every frame against the robot's calibrated hardware
range with a margin, and writes a hashed plan for real_lid_run.py.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np

REPO=Path(__file__).resolve().parents[1];ROOT=REPO.parents[1]
sys.path.insert(0,str(REPO/'scripts'))
from policy_contract import JOINTS,q_to_raw

ARM3=ROOT/'arm3-current/arm3/calibration/arm3.json'   # package: ROOT = <package>, REPO = <package>/ik/isaac_migration
LID_FRAMES=966          # lid 0..920 + hold 921..965 of the continuous prefix
SLEW_DEG_S=20.
MARGIN_TICKS=2
# Isaac relaxes the grip ~3.6 deg after the lid closes (549) until the release opens (862).
# On arm3 that left only ~6 ticks of squeeze in transport; the real option keeps a tighter target there.
TRANSPORT=(549,862)
# Real placement correction: pad-midpoint shift ramps in after the lid clears the source jig
# (pad ~90mm high at 565) and is complete mid-transport, then held through release and hold.
PLACE_RAMP=(565,640)
# Real grasp XY option: shift the pad over the high approach (160..240, pad 103->84mm, gripper closed), back
# out once the lid is lifted (560..600, pad ~90mm). The in-hand change is carried to the placement with the
# FK-predicted compensation (grasp frame 520, release start 861), as in real_cylinder_plan.py.
GRASP_RAMP=(160,240,560,600)
GRASP_FRAME,RELEASE_FRAME=520,861


def place_shift(target,offset_mm,ramp=PLACE_RAMP):
    """Translate the pad midpoint by offset_mm (base XY, or XYZ) with step86 KinematicRetarget."""
    from third_stage_path import Kinematics
    from retarget_prefix import KinematicRetarget
    env=Kinematics();ik=KinematicRetarget(env);out=target.copy();worst=[0.,0.]
    def smooth(a,b):
        t=np.clip((np.arange(len(target))-a)/(b-a),0,1);return t*t*(3-2*t)
    # ramp (a,b): ramp in and hold; (a,b,c,d): ramp in over a..b, back out over c..d.
    w=smooth(*ramp[:2])-(smooth(*ramp[2:]) if len(ramp)==4 else 0)
    for i in np.flatnonzero(w>0):
        q,e,ang=ik.shift(target[i],np.r_[np.asarray(offset_mm,float),np.zeros(3-len(offset_mm))]/1000*w[i]);out[i,:5]=q[:5];worst=[max(worst[0],e),max(worst[1],ang)]
    return out,dict(place_offset_mm=list(map(float,offset_mm)),ramp_frames=list(ramp),max_ik_residual_mm=worst[0],max_orientation_change_deg=worst[1])


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def slew(targets,start,hz=15.):
    """Simulator command path: a 20 deg/s ramp toward each 15Hz target, sampled per frame."""
    out=[];ctrl=np.array(start,float);step=np.deg2rad(SLEW_DEG_S)/hz
    for t in targets:
        ctrl=ctrl+np.clip(np.asarray(t)-ctrl,-step,step);out.append(ctrl.copy())
    return np.array(out)


def build(trajectory,calibration,settings,frames=LID_FRAMES,transport_grip_deg=None,place_offset_mm=None,grasp_offset_mm=None):
    with np.load(trajectory) as z:target=z['target_q_rad'][:frames].copy();q0=z['initial_q_rad'].copy();phase=[str(x) for x in z['phase'][:frames]]
    if transport_grip_deg is not None:
        a,b=TRANSPORT;target[a:b,5]=np.minimum(target[a:b,5],np.deg2rad(transport_grip_deg))
    shift=grasp=None
    if grasp_offset_mm is not None:
        from third_stage_path import Kinematics
        from retarget_prefix import KinematicRetarget
        ik=KinematicRetarget(Kinematics());_,rg=ik.pose(target[GRASP_FRAME]);_,rp=ik.pose(target[RELEASE_FRAME])
        comp=-(rp@rg.T@-np.r_[np.asarray(grasp_offset_mm,float),0])[:2]
        target,grasp=place_shift(target,grasp_offset_mm,ramp=GRASP_RAMP);grasp['place_compensation_mm']=comp.round(2).tolist()
        place_offset_mm=(np.zeros(2) if place_offset_mm is None else np.asarray(place_offset_mm,float))+comp
    if place_offset_mm is not None:target,shift=place_shift(target,place_offset_mm)
    q=slew(target,q0)
    return dict(phase=phase,place_shift=shift,grasp_shift=grasp,**checked(q,calibration,settings))


def checked(q,calibration,settings):
    """Raw ticks with the hardware-range margin check, plus rates and margins."""
    raw=np.rint(q_to_raw(q,settings)).astype(int)
    lo=np.array([calibration[j]['range_min'] for j in JOINTS]);hi=np.array([calibration[j]['range_max'] for j in JOINTS])
    bad=[(int(k),JOINTS[j],int(raw[k,j])) for k,j in zip(*np.nonzero((raw<lo+MARGIN_TICKS)|(raw>hi-MARGIN_TICKS)))]
    if bad:raise ValueError(f'{len(bad)} plan samples within {MARGIN_TICKS} ticks of or outside the hardware range, e.g. {bad[:3]}')
    rate=np.degrees(np.abs(np.diff(q,axis=0))).max(0)*15
    margin={j:int(min(raw[:,i].min()-lo[i],hi[i]-raw[:,i].max())) for i,j in enumerate(JOINTS)}
    return dict(raw=raw.tolist(),q_rad=q.tolist(),start_raw=raw[0].tolist(),max_rate_deg_s=dict(zip(JOINTS,np.round(rate,2).tolist())),
                hardware_margin_ticks=margin)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--trajectory',type=Path,default=ROOT/'diagnostics/isaac_continuation_20261022/fidelity_x4_input/trajectory.npz')
    p.add_argument('--speed',type=float,default=.5,help='playback speed vs the simulator (first real test: 0.5)')
    p.add_argument('--transport-grip-deg',type=float,help=f'cap the gripper target over frames {TRANSPORT[0]}..{TRANSPORT[1]-1} (grasp close is 11.4 deg)')
    p.add_argument('--place-offset-mm',type=float,nargs=2,metavar=('DX','DY'),help='shift the lid placement in base XY (mm), measured from the previous real try')
    p.add_argument('--grasp-offset-mm',type=float,nargs=2,metavar=('DX','DY'),help='shift the pad at the grasp in base XY (mm); the placement is compensated')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    if not .1<=a.speed<=1:raise ValueError('speed must be within [0.1, 1]')
    calibration=json.loads(ARM3.read_text());settings=json.loads((REPO/'configs/calibration/simulation_settings.json').read_text())
    if sha(ARM3)!=sha(REPO/'configs/dataset_encoder_calibration.json'):raise ValueError('Robot calibration differs from the simulator dataset calibration')
    if a.grasp_offset_mm is not None and max(map(abs,a.grasp_offset_mm))>10:raise ValueError('grasp offset must be within +/-10mm per axis')
    if a.transport_grip_deg is not None and not 10<=a.transport_grip_deg<=15:raise ValueError('transport grip must be within [10, 15] deg')
    if a.place_offset_mm is not None and max(map(abs,a.place_offset_mm))>20:raise ValueError('placement offset must be within +/-20mm per axis')
    plan=build(a.trajectory,calibration,settings,transport_grip_deg=a.transport_grip_deg,place_offset_mm=a.place_offset_mm,grasp_offset_mm=a.grasp_offset_mm)
    plan.update(kind='first-lid real command plan',scope='lid pick, transport, place and hold only; nominal jig (0,0); no camera retarget',
        transport_grip_deg=a.transport_grip_deg,transport_frames=list(TRANSPORT) if a.transport_grip_deg is not None else None,
        speed=a.speed,command_hz=15*a.speed,frames=LID_FRAMES,joint_order=list(JOINTS),
        source_trajectory=str(a.trajectory),source_sha256=sha(a.trajectory),robot_calibration=str(ARM3),robot_calibration_sha256=sha(ARM3),
        mapping='policy_contract.q_to_raw with configs/calibration/simulation_settings.json (tentative joint offsets)',
        simulator_evidence='Isaac selected two-part prefix: lid 1.30mm in repeated centre runs; not a real-robot result',
        caveats=['Isaac-tuned lid commands (grip relief/release timing) have not run on hardware','joint offsets and TCP calibration are tentative',
                 'wrist_flex peaks within a few ticks of its hardware limit','jig must be at the nominal position; no camera correction in this plan'],
        hardware_accessed=False)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(plan,indent=1)+'\n')
    print(json.dumps({k:plan[k] for k in ['frames','speed','command_hz','start_raw','max_rate_deg_s','hardware_margin_ticks']},indent=1))

if __name__=='__main__':main()
