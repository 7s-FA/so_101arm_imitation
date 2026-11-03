"""Offline third-part (base) command path; MuJoCo kinematics only, no physics state.

Port of step86 third_path (continuous_assembly86.py) with its dependencies:
demo215 state -> JointMapping.to_q, offset_grasp IK, closing-grip adjust and the
three_product_axis_path84 fixed-grasp Cartesian plan. Goal XY = nominal CAD goal +
camera-observed absolute jig XY + one policy residual. Product truth is never read.
Run with the offline Python; the Isaac runtime calls the CLI as a subprocess.
"""
import argparse,copy,hashlib,json,os,re
from pathlib import Path
import mujoco,numpy as np
from scipy.optimize import least_squares
from policy_contract import PACKAGE,JOINTS

SNAP=PACKAGE/'source_snapshot'
STEP86=SNAP/'simulation/rl_pick_place/continuous_assembly_step86/policy_grid25_12'
AUDIT=SNAP/'diagnostics/full_sequence_audit_20261020'
MODEL=PACKAGE/'assets/mjcf/measured484_mu07.xml'
EPISODE=215

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_model(model_path=MODEL):
    """MjModel via an MJB cache keyed by the XML, its referenced files (size, mtime) and the MuJoCo version.

    Compiling measured484_mu07.xml (654 meshes) took ~8-14 s per call on the Jetson (2026-10-30) and a retarget
    builds Kinematics five times; the binary loads in ~0.2 s and gave bit-identical retargeted plans.
    """
    path=Path(model_path).resolve();xml=path.read_bytes();key=hashlib.sha256(xml+mujoco.__version__.encode())
    for ref in sorted(set(re.findall(rb'file="([^"]+)"',xml))):
        st=(path.parent/ref.decode()).stat();key.update(ref+b'%d:%d'%(st.st_size,st.st_mtime_ns))
    cache=path.parent/'.mjb_cache'/f'{path.stem}_{key.hexdigest()[:16]}.mjb'
    if cache.exists():return mujoco.MjModel.from_binary_path(str(cache))
    model=mujoco.MjModel.from_xml_path(str(path))
    cache.parent.mkdir(exist_ok=True);tmp=cache.with_name(f'{cache.name}.{os.getpid()}.tmp')
    mujoco.mj_saveModel(model,str(tmp));os.replace(tmp,cache)
    for old in cache.parent.glob(f'{path.stem}_*.mjb'):
        if old!=cache:old.unlink(missing_ok=True)
    return model

class Kinematics:
    """MjModel plus step86 PickPlaceEnv joint addresses and replay bounds."""
    def __init__(self,model_path=MODEL):
        self.model=load_model(model_path)
        self.qadr=np.array([int(self.model.joint(n).qposadr[0]) for n in JOINTS])
        self.vadr=np.array([int(self.model.joint(n).dofadr[0]) for n in JOINTS])
        ranges=json.loads((SNAP/'simulation/rl_pick_place/config.json').read_text())['joint_ranges']
        self.bounds=np.array([ranges[n] for n in JOINTS])

def joint_mapping_to_q(values):
    """compare_imitation_ppo.JointMapping.to_q with the preserved calibration copies."""
    c=json.loads((PACKAGE/'configs/dataset_encoder_calibration.json').read_text())
    s=json.loads((PACKAGE/'configs/calibration/simulation_settings.json').read_text())['joints']
    scale=np.array([(c[n]['range_max']-c[n]['range_min'])/100 if n=='gripper' else 4095/360 for n in JOINTS])
    center=np.array([c[n]['range_min'] if n=='gripper' else (c[n]['range_min']+c[n]['range_max'])/2 for n in JOINTS])
    origin=np.array([s[n]['raw_origin'] for n in JOINTS]);rad=np.array([s[n]['radians_per_tick'] for n in JOINTS])
    offset=np.array([s[n]['offset_rad'] for n in JOINTS])
    return (np.asarray(values)*scale+center-origin)*rad+offset

def demo_record(episode=EPISODE):
    record=next(r for r in json.loads((AUDIT/'inventory.json').read_text())['episodes'] if r['episode']==episode)
    record=copy.deepcopy(record)
    for s in record['segments']:
        # Inventory holds original absolute paths; use the frozen snapshot copy.
        s['data']=str(AUDIT/'segments'/Path(s['data']).name)
        if sha(s['data'])!=s['sha256']:raise ValueError(f'Changed demo segment: {s["data"]}')
    return record

def reference(record):
    return joint_mapping_to_q(np.concatenate([np.load(s['data'])['state'] for s in record['segments']]))

def rotation_error(target,actual):
    q1,q2,delta=np.zeros(4),np.zeros(4),np.zeros(3)
    mujoco.mju_mat2Quat(q1,target.ravel());mujoco.mju_mat2Quat(q2,actual.ravel());mujoco.mju_subQuat(delta,q1,q2)
    return actual@delta

def offset_grasp(env,q,delta_mm,window):
    """cylinder_friction_improvement.offset_grasp without rotation; scratch FK only."""
    scratch=mujoco.MjData(env.model);body=env.model.body('gripper_link').id
    def pose(a):
        scratch.qpos[env.qadr]=a;mujoco.mj_kinematics(env.model,scratch);mujoco.mj_comPos(env.model,scratch)
        return scratch.xpos[body].copy(),scratch.xmat[body].reshape(3,3).copy()
    out=q.copy();errors=[];wstart,up_end,down_start,stop=window
    for i in range(wstart,min(len(q),stop)):
        up=np.clip((i-wstart)/(up_end-wstart),0,1);down=np.clip((stop-i)/(stop-down_start),0,1)
        w=(up*up*(3-2*up))*(down*down*(3-2*down))
        p0,r0=pose(q[i]);desired=p0+np.asarray(delta_mm)*w/1000;a=q[i].copy()
        for _ in range(15):
            p,r=pose(a);jp=np.zeros((3,env.model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(env.model,scratch,jp,jr,body)
            J=np.vstack([jp[:,env.vadr[:5]],.025*jr[:,env.vadr[:5]]]);e=np.r_[desired-p,.025*rotation_error(r0,r)]
            dq=np.linalg.solve(J.T@J+1e-7*np.eye(5),J.T@e)
            a[:5]=np.clip(a[:5]+np.clip(dq,-.04,.04),env.bounds[:5,0],env.bounds[:5,1])
        p,r=pose(a);errors.append(float(np.linalg.norm(p-desired)*1000));out[i]=a
    return out,dict(offset_mm=list(delta_mm),max_fk_position_residual_mm=max(errors),scratch_only=True)

def axis_path(env,q,config,strict=True):
    """three_product_axis_path84.build, unchanged numerics."""
    start=int(config.get('start',520));data=mujoco.MjData(env.model)
    bid=env.model.body('gripper_link').id
    transform=config['grasp_transform']
    local_p=np.array(transform['position_local']);local_a=np.array(transform['axis_local'])
    goal=np.array(config['nominal_goal_m'])+np.array(config.get('goal_offset_mm',[0,0,0]))/1000
    desired_axis=np.array(config.get('axis',[0,0,1]),dtype=float);desired_axis/=np.linalg.norm(desired_axis)
    above_axis=np.array(config.get('above_axis',desired_axis),dtype=float);above_axis/=np.linalg.norm(above_axis)
    def fk(a):
        data.qpos[env.qadr]=a;mujoco.mj_kinematics(env.model,data);mujoco.mj_comPos(env.model,data)
        R=data.xmat[bid].reshape(3,3)
        return data.xpos[bid]+R@local_p,R@local_a
    anchor=q[start].copy();p0,a0=fk(anchor);closed=float(anchor[5])+np.deg2rad(config.get('closed_adjust_deg',0));opened=np.deg2rad(config.get('open_deg',32))
    above=goal+[0,0,config.get('above_mm',60)/1000]
    down=goal+[0,0,config.get('release_height_mm',1)/1000]
    lateral=np.asarray(config.get('unhook_mm',[0,0,0]))/1000
    opening_lateral=np.asarray(config.get('opening_lateral_mm',[0,0,0]))/1000
    retreat=goal+[0,0,config.get('retreat_mm',90)/1000]+lateral
    phases=[('align',above,above_axis,closed,config.get('align_s',6)),
            ('descend',down,desired_axis,closed,config.get('descend_s',8)),
            ('open',down+opening_lateral,desired_axis,opened,config.get('open_s',2)),
            ('wait',down+opening_lateral,desired_axis,opened,config.get('wait_s',1)),
            ('retreat',retreat,above_axis,opened,config.get('retreat_s',6))]
    if np.any(lateral):
        phases.insert(-1,('unhook',down+lateral,desired_axis,opened,config.get('unhook_s',4)))
    if config.get('lift_mm',0):
        phases.insert(0,('lift',p0+[0,0,config['lift_mm']/1000],desired_axis,closed,config.get('lift_s',4)))
    result=[x.copy() for x in q[:start+1]];a=anchor.copy();errors=[];phase_records=[];required_axis_errors=[]
    previous_p=p0;previous_axis=a0;previous_grip=float(anchor[5])
    weight=config.get('axis_weight_m',.06)
    for name,endp,enda,endg,seconds in phases:
        begin=len(result);n=round(seconds*5)
        position_only=name=='retreat' and config.get('retreat_position_only',False)
        seed_posture=a[:5].copy()
        for k in range(1,n+1):
            w=k/n;w=w*w*(3-2*w)
            wanted_p=previous_p*(1-w)+endp*w
            wanted_a=previous_axis*(1-w)+enda*w;wanted_a/=np.linalg.norm(wanted_a)
            grip=previous_grip*(1-w)+endg*w
            def error(v):
                qq=np.r_[v,grip];p,axis=fk(qq)
                if position_only:return np.r_[p-wanted_p,.0001*(v-seed_posture)]
                return np.r_[p-wanted_p,weight*(axis-wanted_a)]
            def jac(v):
                p,axis=fk(np.r_[v,grip]);jp=np.zeros((3,env.model.nv));jr=np.zeros_like(jp)
                mujoco.mj_jac(env.model,data,jp,jr,p,bid)
                if position_only:return np.vstack([jp[:,env.vadr[:5]],.0001*np.eye(5)])
                skew=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                return np.vstack([jp[:,env.vadr[:5]],-weight*skew@jr[:,env.vadr[:5]]])
            solved=least_squares(error,a[:5],jac=jac,bounds=(env.bounds[:5,0],env.bounds[:5,1]),max_nfev=80,ftol=1e-9,xtol=1e-9,gtol=1e-9)
            a=np.r_[solved.x,grip];pp,aa=fk(a)
            errors.append([float(np.linalg.norm(pp-wanted_p)*1000),float(np.rad2deg(np.arccos(np.clip(aa@wanted_a,-1,1))))])
            if not position_only:required_axis_errors.append(errors[-1][1])
            result.append(a.copy())
        phase_records.append(dict(name=name,frame_start=begin,frame_stop=len(result),seconds=seconds,
            max_position_mm=max(x[0] for x in errors[-n:]),max_axis_deg=max(x[1] for x in errors[-n:]),end_error=errors[-1]))
        previous_p=endp;previous_axis=enda;previous_grip=endg
    metrics=dict(max_position_residual_mm=max(x[0] for x in errors),max_axis_residual_deg=max(required_axis_errors),
        phases=phase_records,goal_m=goal.tolist(),uses_live_object_pose=False)
    if strict and (metrics['max_position_residual_mm']>1 or metrics['max_axis_residual_deg']>3):
        raise ValueError('Axis plan infeasible: '+json.dumps(metrics))
    return np.array(result),metrics

def controller():
    """Frozen step86 third-stage settings from the selected MuJoCo grid25_12 run."""
    path=STEP86/'effective_controller.json';cfg=json.loads(path.read_text())
    keys=['third_episode','third_slowdown','third_grasp_mm','third_grip_adjust_deg','third_use_jig_image',
          'third_axis_skip_reference_shift','third_axis_plan','third_joint_tracking']
    for k in ['third_place_mm','third_release','third_stop_frame','third_object_retarget','third_grasp_window']:
        if cfg.get(k) is not None:raise ValueError(f'Unported step86 option present: {k}')
    if cfg['third_episode']!=EPISODE or cfg['third_use_jig_image'] is not True or cfg['third_axis_skip_reference_shift'] is not True:
        raise ValueError('Unexpected frozen third-stage controller')
    return {k:copy.deepcopy(cfg[k]) for k in keys},sha(path)

OVERRIDABLE=('third_grasp_mm','third_grip_adjust_deg','third_vertical_lift','third_pregrasp_clearance','third_grasp_transform','third_axis_plan_overrides')
# Release-plan values an Isaac candidate may change (step86 three_product_axis_path84 keys).
PLAN_KEYS={'release_height_mm':(0,30),'above_mm':(10,60),'descend_s':(2,12),'open_s':(1,8),'wait_s':(0,8),'unhook_s':(1,8),'retreat_s':(2,10)}
PLAN_VECTORS={'opening_lateral_mm':12,'unhook_mm':20}

def pregrasp_clearance(env,q,delta_mm,frames,close):
    """Isaac candidate: temporarily shift the pad midpoint by delta_mm during the open-gripper
    descent (smoothstep in a->b, hold, out c->e) so the fixed TPU finger does not rub down the
    base wall; the stick/slip there chaotically chose the in-hand pose. Frames >= e unchanged."""
    from retarget_prefix import KinematicRetarget
    a,b,c,e=map(int,frames)
    if not a<b<=c<e<=close-9:raise ValueError('pregrasp frames must satisfy a<b<=c<e<=close-9')
    ik=KinematicRetarget(env);out=q.copy();errors=[0.];angles=[0.];smooth=lambda t:float(np.clip(t,0,1))**2*(3-2*float(np.clip(t,0,1)))
    for i in range(a+1,e):
        w=smooth((i-a)/(b-a))*smooth((e-i)/(e-c))
        if w>0:out[i],err,ang=ik.shift(q[i],np.asarray(delta_mm,dtype=float)/1000*w);errors.append(err);angles.append(ang)
    return out,dict(delta_mm=list(map(float,delta_mm)),frames=[a,b,c,e],max_fk_residual_mm=max(errors),max_orientation_change_deg=max(angles))

def vertical_lift(env,q,hold_frame,clear_mm,blend_frames,before_frame):
    """Isaac candidate: hold the pad-midpoint XY of hold_frame while the original path rises
    clear_mm, then blend back over blend_frames. Original wrist orientation and Z are kept.
    Avoids dragging the base along the source-jig chamfer during lift. Scratch FK only."""
    from retarget_prefix import KinematicRetarget
    ik=KinematicRetarget(env);out=q.copy();p_hold=ik.pose(q[hold_frame])[0];z=[ik.pose(x)[0][2] for x in q]
    clear=next((i for i in range(hold_frame,len(q)) if z[i]>=p_hold[2]+clear_mm/1000),None)
    if clear is None or clear+blend_frames>=before_frame:raise ValueError('Vertical lift must clear and rejoin before the axis plan')
    errors=[0.]
    for i in range(hold_frame+1,clear+blend_frames+1):
        w=0. if i<=clear else (i-clear)/blend_frames;w=w*w*(3-2*w)
        p=ik.pose(q[i])[0];delta=np.r_[(1-w)*(p_hold[:2]-p[:2]),0]
        if np.any(delta):out[i],e,_=ik.shift(q[i],delta);errors.append(e)
    return out,dict(hold_frame=hold_frame,clear_frame=clear,rejoin_frame=clear+blend_frames,clear_mm=clear_mm,
                    max_xy_hold_mm=float(max(np.linalg.norm(ik.pose(q[i])[0][:2]-p_hold[:2]) for i in range(hold_frame,clear+1))*1000),max_fk_residual_mm=max(errors))

def build(observed_jig_mm,residual_xy_mm,model_path=MODEL,overrides=None):
    """Return (q3, anchor_frame, diagnostics). Inputs: absolute image jig XY (CAD frame) and residual, mm.

    overrides: Isaac development candidates for the grasp only; same for every residual.
    """
    jig=np.asarray(observed_jig_mm,float);residual=np.asarray(residual_xy_mm,float)
    if jig.shape!=(2,) or residual.shape!=(2,) or not np.isfinite(np.r_[jig,residual]).all():raise ValueError('Invalid jig/residual input')
    if np.max(abs(residual))>3.000001:raise ValueError('Residual exceeds policy bound 3mm')
    cfg,cfg_sha=controller();overrides=dict(overrides or {})
    if set(overrides)-set(OVERRIDABLE):raise ValueError(f'Only {OVERRIDABLE} may be overridden')
    if 'third_grasp_mm' in overrides and (len(overrides['third_grasp_mm'])!=3 or not np.isfinite(overrides['third_grasp_mm']).all()):raise ValueError('third_grasp_mm needs three finite values')
    if 'third_vertical_lift' in overrides and set(overrides['third_vertical_lift'])!={'hold_frame','clear_mm','blend_frames'}:raise ValueError('third_vertical_lift needs hold_frame, clear_mm, blend_frames')
    if 'third_pregrasp_clearance' in overrides:
        v=overrides['third_pregrasp_clearance']
        if set(v)!={'delta_mm','frames'} or len(v['delta_mm'])!=3 or len(v['frames'])!=4 or not np.isfinite(v['delta_mm']).all() or max(map(abs,v['delta_mm']))>3:raise ValueError('third_pregrasp_clearance needs delta_mm (3 values, |v|<=3) and frames (4)')
    if 'third_grasp_transform' in overrides:
        g=overrides['third_grasp_transform']
        if set(g)!={'position_local','axis_local'} or len(g['position_local'])!=3 or len(g['axis_local'])!=3 or not np.isfinite(np.r_[g['position_local'],g['axis_local']]).all() or np.linalg.norm(g['position_local'])>.2 or not .99<np.linalg.norm(g['axis_local'])<1.01:raise ValueError('third_grasp_transform needs position_local (m) and unit axis_local')
    if 'third_axis_plan_overrides' in overrides:
        for k,v in overrides['third_axis_plan_overrides'].items():
            if k in PLAN_KEYS:
                lo,hi=PLAN_KEYS[k]
                if not lo<=float(v)<=hi:raise ValueError(f'{k} must be within [{lo},{hi}]')
            elif k in PLAN_VECTORS:
                if len(v)!=3 or not np.isfinite(v).all() or max(map(abs,v))>PLAN_VECTORS[k]:raise ValueError(f'{k} needs 3 values within +/-{PLAN_VECTORS[k]}mm')
            else:raise ValueError(f'Axis-plan key {k} is not an Isaac candidate parameter')
    if 'third_grip_adjust_deg' in overrides and not -12<=float(overrides['third_grip_adjust_deg'])<=20:raise ValueError('third_grip_adjust_deg must be -12..20 (positive opens)')
    cfg.update(overrides);env=Kinematics(model_path);record=demo_record(cfg['third_episode'])
    q=reference(record)
    start=record['segments'][2]['frame_start'];target=record['transport_cycles'][2]['target_entry']
    smooth=np.convolve(q[:,5],np.ones(5)/5,'same')
    candidates=np.arange(record['transport_cycles'][2]['source_entry'],target-8)
    close=int(candidates[np.argmin((smooth[8:]-smooth[:-8])[candidates])])+8
    diag=dict(episode=record['episode'],stage_start=start,closure_estimate=close,target_entry=target)
    q,diag['grasp']=offset_grasp(env,q,cfg['third_grasp_mm'],(start,start+35,close+45,close+80))
    if cfg.get('third_pregrasp_clearance'):
        v=cfg['third_pregrasp_clearance'];q,diag['pregrasp_clearance']=pregrasp_clearance(env,q,v['delta_mm'],v['frames'],close)
    w=np.clip((np.arange(len(q))-(close-10))/10,0,1);q[:,5]+=np.deg2rad(cfg['third_grip_adjust_deg'])*w*w*(3-2*w)
    if cfg.get('third_vertical_lift'):
        v=cfg['third_vertical_lift'];q,diag['vertical_lift']=vertical_lift(env,q,int(v['hold_frame']),float(v['clear_mm']),int(v['blend_frames']),int(cfg['third_axis_plan']['start']))
    plan=copy.deepcopy(cfg['third_axis_plan'])
    if cfg.get('third_grasp_transform'):
        # Same quantity step86 froze from its MuJoCo nominal run, re-measured once in Isaac.
        plan['grasp_transform']=copy.deepcopy(cfg['third_grasp_transform'])
    if cfg.get('third_axis_plan_overrides'):
        plan.update(copy.deepcopy(cfg['third_axis_plan_overrides']))
    plan['nominal_goal_m'][:2]=(np.asarray(plan['nominal_goal_m'][:2])+residual/1000).tolist()
    plan['goal_offset_mm']=(np.asarray(plan.get('goal_offset_mm',[0,0,0]))+np.r_[jig,0]).tolist()
    q,diag['axis_plan']=axis_path(env,q,plan)
    if np.any(q<env.bounds[:,0]-1e-6) or np.any(q>env.bounds[:,1]+1e-6):raise ValueError('Path outside step86 replay bounds')
    diag.update(observed_jig_mm=jig.tolist(),residual_xy_mm=residual.tolist(),controller_sha256=cfg_sha,
                model=str(model_path),model_sha256=sha(model_path),controller_overrides=overrides,grasp_mm=list(cfg['third_grasp_mm']),grip_adjust_deg=float(cfg['third_grip_adjust_deg']),slowdown=cfg['third_slowdown'],
                joint_tracking=cfg['third_joint_tracking'],product_truth_used=False,physics_state_used=False)
    return q,start,diag

def main():
    p=argparse.ArgumentParser();p.add_argument('--observed-jig-mm',type=float,nargs=2,required=True)
    p.add_argument('--residual-mm',type=float,nargs=2,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--overrides',default='{}',help='JSON object; see OVERRIDABLE')
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    q,anchor,diag=build(a.observed_jig_mm,a.residual_mm,overrides=json.loads(a.overrides))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(a.output,q=q,anchor=np.array(anchor),slowdown=np.array(diag['slowdown']))
    a.output.with_suffix('.json').write_text(json.dumps(diag,indent=2)+'\n')

if __name__=='__main__':main()
