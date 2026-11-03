"""OFFLINE: a simulator scene for one real set, calibrated so the verified real plan is its nominal path.

The CAD layout (tasks/config.json) and the real cell differ by up to ~2.5cm where the real plans take and
leave the parts (joint calibration, jig placement, in-hand offsets). Each part of the set is made a free
product and, with its source jig, moved (XY, yaw about the part) to where the real plan closes on it; its
target jig and goal move to where the real plan lets it go, so the transport the plan performs is
reproduced exactly. Heights stay CAD. Parts that were absent when the demonstration started are removed;
other sets stay as static parts.

Grip fit (second pass, MuJoCo geometry on the first-pass scene): the real moving jaw presses a part onto the
fixed finger, and the real TPU fin-ray fingers wrap round it, so (1) each part is shifted along the closing
direction until it clears the open fixed finger by GRIP_CLEARANCE (same in-hand offset carried to the
release), and (2) the angle where the rigid simulated moving finger first touches the part, minus the real
holding angle (plan command + REAL_SQUEEZE_DEG), is that part's gripper offset for the simulator.

Input  : base MJCF (measured484_mu07.xml, the Isaac lid/cylinder/base physics model) and a real plan
Output : <out>/scene.xml (absolute asset dir), <out>/calibration.json
"""
import argparse,copy,json,hashlib,sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

HERE=Path(__file__).resolve().parent;PACKAGE=HERE.parent;sys.path.insert(0,str(HERE))
from plan_events import plan_events

BASE=PACKAGE/'assets/mjcf/measured484_mu07.xml'
TASKS=PACKAGE/'source_snapshot/simulation/tasks/config.json'
# set -> (part ids in plan order, CAD source bank, static parts removed because the demonstration start had them empty)
SETS={'A_lower':(['A_housing','A_ball','A_cap'],'2',[]),
      'A_upper':(['A_housing','A_ball','A_cap'],'1',['passive_A_housing_2','passive_A_ball_2','passive_A_cap_2']),
      'B_left':(['B_lid','B_TPU','B_base'],'1',[]),
      'B_right':(['B_lid','B_TPU','B_base'],'2',['passive_B_lid_2','passive_B_TPU_2','passive_B_base_2'])}
SYMMETRIC={'A_ball','A_cap','B_TPU'}      # round about the vertical: no yaw calibration
GRIP_CLEARANCE=.0005;REAL_SQUEEZE_DEG=1.   # real measured minus commanded while holding: +0.7..+1.3 deg (ep199/184/257)
PRODUCT_COLLISION=dict(contype='4',conaffinity='7',margin='0',gap='0',friction='.7 .005 .0001',solref='.01 1',solimp='.95 .99 .001',condim='4')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def yaw_of(T):return float(np.degrees(np.arctan2(T[1][0],T[0][0])))
def wxyz(r):return np.roll(r.as_quat(),1)
def rot(q):return R.from_quat(np.roll(np.asarray(q,float),-1))


def calibrate(plan,parts,bank,tasks):
    products={p['id']:p for t in tasks['tasks'].values() for p in t['products']}
    events=plan_events(plan)
    if len(events)!=3:raise ValueError(f'expected 3 grasp/release events, found {len(events)}')
    out={}
    for name,e in zip(parts,events):
        src=np.array(products[name]['source_poses'][bank]);goal=np.array(products[name]['goal_pose'])
        turn=e['release']['yaw_deg']-e['grasp']['yaw_deg']
        want=yaw_of(goal)-turn;dyaw=0. if name in SYMMETRIC else ((want-yaw_of(src)+45)%90)-45   # C4 parts: nearest quarter turn
        # Pad midpoint as the part centre: the commanded jaw closes past the part, and the pad proxies are not the
        # contact faces, so a width-based centre is not reliable; the Isaac housing/ball grasps centred within 1mm.
        g=np.array(e['grasp']['pad_m']);r=np.array(e['release']['pad_m'])
        out[name]=dict(event=e,source_shift_m=(g[:2]-src[:2,3]).tolist(),source_yaw_deg=dyaw,target_shift_m=(r[:2]-goal[:2,3]).tolist(),
                       grasp_height_above_source_mm=float((g[2]-src[2,3])*1000),release_height_above_goal_mm=float((r[2]-goal[2,3])*1000),
                       source_pose=src.tolist(),goal_pose=goal.tolist(),mass_kg=products[name]['mass_estimate_g']/1000)
        G=goal.copy();G[:2,3]+=out[name]['target_shift_m'];out[name]['calibrated_goal_pose']=G.tolist()
    return out


def move_body(body,shift_xy,dyaw_deg,center_xy):
    """Rotate the body pose about the vertical through center_xy by dyaw, then translate by shift_xy (world)."""
    p=np.array(body.get('pos').split(),float);q=np.array(body.get('quat','1 0 0 0').split(),float)
    rz=R.from_euler('z',dyaw_deg,degrees=True);c=np.r_[center_xy,p[2]]
    p=c+rz.apply(p-c);p[:2]+=shift_xy
    body.set('pos',' '.join(f'{v:.12g}' for v in p));body.set('quat',' '.join(f'{v:.12g}' for v in wxyz(rz*rot(q))))


def inertial(mesh_file,mass,assetdir):
    import mujoco
    xml=f'''<mujoco><compiler inertiafromgeom="true" assetdir="{assetdir}"/><asset><mesh name="v" file="{mesh_file}" scale=".001 .001 .001"/></asset>
    <worldbody><body name="b"><freejoint/><geom type="mesh" mesh="v" mass="{mass}"/></body></worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);b=m.body('b').id
    return dict(pos=' '.join(f'{v:.9g}' for v in m.body_ipos[b]),quat=' '.join(f'{v:.9g}' for v in m.body_iquat[b]),
                mass=f'{mass:.9g}',diaginertia=' '.join(f'{v:.9g}' for v in m.body_inertia[b]))


JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')


def grip_fit(scene_xml,cal,plan):
    """Per part: in-hand shift off the fixed finger (world XY at grasp and at release) and the simulator gripper offset."""
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(scene_xml));d=mujoco.MjData(m);q=np.array(plan['q_rad']);qadr=[m.joint(n).qposadr[0] for n in JOINTS]
    fin=lambda b:[g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name==b and m.geom_contype[g]]
    fixed,moving=fin('custom_fixed_tpu_fin_link'),fin('custom_moving_tpu_fin_link');ft=np.zeros(6);out={}
    lo,hi=m.jnt_range[m.joint('gripper').id]
    for name,c in cal.items():
        e=c['event'];k=e['grasp']['frame'];close=e['close_frame'];rel=e['release_frame']
        b=m.body('product_'+name).id;pg=[g for g in range(m.ngeom) if m.geom_bodyid[g]==b and m.geom_contype[g]];adr=m.jnt_qposadr[m.body_jntadr[b]]
        opening=float(min(hi,q[max(0,close-40):close+1,5].max()))
        u=np.array(e['grasp']['moving_pad_m'])-np.array(e['grasp']['fixed_pad_m']);u[2]=0;u/=np.linalg.norm(u)
        def gap(geoms,shift,angle):
            mujoco.mj_resetData(m,d);d.qpos[qadr]=q[k];d.qpos[qadr[5]]=angle;d.qpos[adr:adr+3]+=shift*u;mujoco.mj_forward(m,d)
            return min(mujoco.mj_geomDistance(m,d,a,x,.05,ft) for a in geoms for x in pg)
        shift=0.
        while gap(fixed,shift,opening)<GRIP_CLEARANCE and shift<.03:shift+=.00025
        contact=None
        for angle in np.arange(opening,lo,-np.radians(.25)):
            if gap(moving,shift,angle)<=0:contact=float(angle);break
        mujoco.mj_resetData(m,d);d.qpos[qadr]=q[k];mujoco.mj_forward(m,d);Rg=d.xmat[m.body('gripper_link').id].reshape(3,3).copy()
        d.qpos[qadr]=q[rel];mujoco.mj_forward(m,d);Rr=d.xmat[m.body('gripper_link').id].reshape(3,3).copy()
        held=q[(close+rel)//2,5]+np.radians(REAL_SQUEEZE_DEG)
        out[name]=dict(fixed_finger_shift_mm=shift*1000,source_extra_m=(shift*u)[:2].tolist(),target_extra_m=(Rr@Rg.T@(shift*u))[:2].tolist(),
                       open_angle_deg=float(np.degrees(opening)),contact_angle_deg=None if contact is None else float(np.degrees(contact)),
                       real_hold_angle_deg=float(np.degrees(held)),gripper_offset_deg=None if contact is None else float(np.degrees(contact-held)),
                       segment_end_frame=rel)
    return out


def build(set_name,plan_path,base=BASE,fit=None):
    parts,bank,absent=SETS[set_name];tasks=json.loads(TASKS.read_text());plan=json.loads(Path(plan_path).read_text())
    cal=calibrate(plan,parts,bank,tasks)
    for name,f in (fit or {}).items():
        c=cal[name];c['grip_fit']=f
        c['source_shift_m']=(np.array(c['source_shift_m'])+f['source_extra_m']).tolist();c['target_shift_m']=(np.array(c['target_shift_m'])+f['target_extra_m']).tolist()
        G=np.array(c['goal_pose']);G[:2,3]+=c['target_shift_m'];c['calibrated_goal_pose']=G.tolist()
    tree=ET.parse(base);root=tree.getroot();world=root.find('worldbody');assetdir=str(base.parent)+'/'
    root.find('compiler').set('assetdir',assetdir)
    bodies={b.get('name'):b for b in world.findall('body')}
    meshes={m.get('name'):m.get('file') for m in root.find('asset').findall('mesh')}
    for name in absent:
        if name in bodies:world.remove(bodies.pop(name))
    if set_name.startswith('A'):
        for n in ['product_B_lid','product_B_TPU','product_B_base']:   # the orange left parts stay put: static
            b=bodies[n];b.set('name','static_'+n[len('product_'):]+'_1')
            for tag in ['freejoint','inertial']:
                for e in b.findall(tag):b.remove(e)
    elif set_name=='B_right':
        for n in ['passive_B_lid_2','passive_B_TPU_2','passive_B_base_2']:
            if n in bodies:world.remove(bodies.pop(n))
    for name in parts:
        c=cal[name];src=np.array(c['source_pose']);center=src[:2,3]
        if name.startswith('A'):
            b=bodies['passive_'+name+'_'+bank];b.set('name','product_'+name)
            for g in b.findall('geom'):
                if g.get('contype')!='0':
                    for k in ['margin','gap']:g.attrib.pop(k,None)
                    g.attrib.update(PRODUCT_COLLISION)
            visual=next(g for g in b.findall('geom') if g.get('contype')=='0')
            b.insert(0,ET.Element('inertial',inertial(meshes[visual.get('mesh')],c['mass_kg'],assetdir)))
            b.insert(0,ET.Element('freejoint',name='product_'+name+'_free'))
            b.set('pos',' '.join(f'{v:.12g}' for v in src[:3,3]));b.set('quat',' '.join(f'{v:.12g}' for v in wxyz(R.from_matrix(src[:3,:3]))))
        else:
            b=bodies['product_'+name]
            b.set('pos',' '.join(f'{v:.12g}' for v in src[:3,3]));b.set('quat',' '.join(f'{v:.12g}' for v in wxyz(R.from_matrix(src[:3,:3]))))
        move_body(b,c['source_shift_m'],c['source_yaw_deg'],center)
        move_body(bodies[f'source_{name}_{bank}'],c['source_shift_m'],c['source_yaw_deg'],center)
        t=bodies['target_'+name];move_body(t,c['target_shift_m'],0.,np.zeros(2))
    return tree,cal,plan


def main():
    p=argparse.ArgumentParser();p.add_argument('--set',choices=SETS,required=True);p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    tree,cal,plan=build(a.set,a.plan);a.output.mkdir(parents=True)
    ET.indent(tree);tree.write(a.output/'first_pass.xml',encoding='unicode')
    fit=grip_fit(a.output/'first_pass.xml',cal,plan)
    tree,cal,plan=build(a.set,a.plan,fit=fit);ET.indent(tree);tree.write(a.output/'scene.xml',encoding='unicode')
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(a.output/'scene.xml'))
    products=[m.body(i).name for i in range(m.nbody) if m.body(i).name.startswith('product_')]
    report=dict(set=a.set,plan=str(a.plan),plan_sha256=sha(a.plan),base=str(BASE),base_sha256=sha(BASE),tasks=str(TASKS),tasks_sha256=sha(TASKS),
        products=products,parts={k:{x:v[x] for x in v if x!='event'}|{'event':v['event']} for k,v in cal.items()},
        method='part + source jig moved to the pad midpoint where the plan closes, then off the open fixed finger (grip fit); target jig + goal to where it releases with the same in-hand offset; yaw for square parts from the gripper turn; CAD heights; per-part simulator gripper offset = contact angle - real holding angle',
        mujoco_compiled=dict(nbody=m.nbody,ngeom=m.ngeom,nq=m.nq),hardware_accessed=False)
    (a.output/'calibration.json').write_text(json.dumps(report,indent=1)+'\n')
    for k,v in cal.items():
        f=v['grip_fit'];print(k,'fixed-finger shift mm',round(f['fixed_finger_shift_mm'],1),'contact/real hold deg',f['contact_angle_deg'] and round(f['contact_angle_deg'],1),round(f['real_hold_angle_deg'],1),'offset',f['gripper_offset_deg'] and round(f['gripper_offset_deg'],1))
        print(k,'source shift mm',np.round(np.array(v['source_shift_m'])*1000,1),'yaw',round(v['source_yaw_deg'],1),'target shift mm',np.round(np.array(v['target_shift_m'])*1000,1),
              'grasp/release height above CAD mm',round(v['grasp_height_above_source_mm'],1),round(v['release_height_above_goal_mm'],1))
    print('products',products,'nq',m.nq)

if __name__=='__main__':main()
