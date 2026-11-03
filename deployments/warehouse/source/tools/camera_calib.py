"""D435 ChArUco camera check and recalibration for the Arm1 package. Camera only: never opens the motors.

  capture   24 colour frames 1920x1080 (board pose, as the 2026-10-18 baseline) + 15 frames 640x480 (the server stream)
  check     is the active calibration still right for a capture? board corner shift against the baseline, pose change,
            the error it would put on the pallet measurement, and the server's own ChArUco camera check
  solve     new board pose -> candidate folder (camera_transforms.json, vision-map.json, jig-reference-m2.png,
            camera_check_roi, overlay.png); the active files are not touched
  apply     back up the active files, install a candidate;  restore  put a backup back

Only the camera-to-board pose is re-estimated. The board-to-base transform (survey of the fixed tabletop board) stays,
so the board must not have moved. Cell polygons and the m2 reference are carried to the new view through the table
planes they lie on (cell top heights from the scene), not regenerated: hand edits survive and the pallet keeps its
base-frame m2 position. The board region of the new reference is the live frame (the board is fixed by definition).
"""
import argparse,fcntl,hashlib,json,shutil,sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1];ARM1=ROOT/'integration/arm1';KIT=ROOT/'camera_calibration'
CAMERA=ROOT/'ik/isaac_migration/configs/calibration/camera_transforms.json'
VISION=ARM1/'vision-map.json';MODEL_MAP=ARM1/'model-map.json'
DEVICES=ROOT/'arm3-current/arm3/devices.json';LOCK=ROOT/'.hardware-session.lock'
K640=np.array([[607.4738,0,320.5028],[0,607.5820,251.5071],[0,0,1]])    # = jig_cells.K640 (D435 920312070850, colour 640x480)
DEFAULT_ROI=[5,340,175,475]
CHECK_SEARCH=20   # JigCells.camera_check search range: a region closer to the image edge cannot see a shift toward it
LIMITS=dict(max_rms_px=1.5,min_corners=8,ambiguity_rms_px=.2,check_pallet_mm=1.,check_corner_px=2.)
sys.path.insert(0,str(ARM1))


def read(p):return json.loads(Path(p).read_text())
def write(p,v):Path(p).write_text(json.dumps(v,ensure_ascii=False,indent=1)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stamp():return datetime.now().strftime('%Y%m%d_%H%M%S')
def new_dir(parent):
    for n in range(100):
        p=parent/(stamp()+(f'_{n}' if n else ''))
        if not p.exists():p.mkdir(parents=True);return p
    raise FileExistsError(parent)
def burger():return read(MODEL_MAP)['burger_correction']
def reference_path(bc):return ARM1/bc['reference']
def kmat(i):return np.array([[i['fx'],0,i['ppx']],[0,i['fy'],i['ppy']],[0,0,1]],float)
def angle_deg(R):return float(np.degrees(np.arccos(np.clip((np.trace(R)-1)/2,-1,1))))


def hardware_lock():
    """The server (and manual runs) hold this lock while they own the D435 and motors."""
    f=LOCK.open('w')
    try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:sys.exit('관제 서버 또는 수동 실행이 켜져 있습니다. 서버를 끈 뒤 다시 실행하세요(서버를 끄면 토크가 풀립니다).')
    return f


# ---------------------------------------------------------------- capture
def capture():
    import pyrealsense2 as rs
    lock=hardware_lock();out=new_dir(KIT/'captures')
    serial=read(DEVICES)['cameras']['realsense']['serial_number'];meta=dict(serial=serial,captured_at=datetime.now().astimezone().isoformat())
    for (w,h),count,warm in (((1920,1080),24,45),((640,480),15,30)):
        # USB 2.1 D435 exposes 1080p at 8 fps rather than 15 fps. Keep the
        # requested resolution/format and select only a supported frame rate.
        dev=next((d for d in rs.context().query_devices() if d.get_info(rs.camera_info.serial_number)==serial),None)
        if dev is None:raise RuntimeError(f'RealSense device {serial} not found')
        rates=sorted({p.fps() for sensor in dev.query_sensors() for p in sensor.get_stream_profiles()
                      if p.stream_type()==rs.stream.color and p.format()==rs.format.bgr8
                      and p.as_video_stream_profile().width()==w and p.as_video_stream_profile().height()==h})
        fps=15 if 15 in rates else max((r for r in rates if r <= 15),default=0)
        if not fps:raise RuntimeError(f'No supported {w}x{h} BGR stream at <=15fps: {rates}')
        print(f'{w}x{h}: 촬영 {fps}fps',flush=True)
        pipe=rs.pipeline();cfg=rs.config();cfg.enable_device(serial);cfg.enable_stream(rs.stream.color,w,h,rs.format.bgr8,fps)
        profile=pipe.start(cfg)
        try:
            i=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            meta[f'{w}x{h}']=dict(width=i.width,height=i.height,fx=i.fx,fy=i.fy,ppx=i.ppx,ppy=i.ppy,model=str(i.model),coeffs=list(i.coeffs),fps=fps)
            for _ in range(warm):pipe.wait_for_frames(5000)
            frames=[]
            while len(frames)<count:
                f=pipe.wait_for_frames(5000).get_color_frame()
                if f:frames.append(np.asanyarray(f.get_data()).copy())
        finally:pipe.stop()
        np.savez_compressed(out/f'color_{h}.npz',bgr=np.array(frames))
        print(f'{w}x{h}: {count}장')
    cv2.imwrite(str(out/'overview_640.png'),frames[-1]);write(out/'metadata.json',meta);lock.close()
    print('저장:',out)


def load_capture(path):
    path=Path(path) if path else latest_capture()
    meta=read(path/'metadata.json')
    with np.load(path/'color_1080.npz') as d:big=d['bgr']
    with np.load(path/'color_480.npz') as d:small=d['bgr']
    return path,meta,big,small


def latest_capture():
    found=sorted((KIT/'captures').glob('*/metadata.json'))
    if not found:sys.exit('촬영본이 없습니다. 먼저 capture를 실행하세요.')
    return found[-1].parent


# ---------------------------------------------------------------- board pose (same conventions as the 2026-10-18 baseline)
def charuco_shift():
    """Some OpenCV builds add +0.5 px to ChArUco corners (opencv#28380): probe on a synthetic board, as the baseline did."""
    board=cv2.aruco.CharucoBoard((7,5),1.,.6,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000));det=cv2.aruco.CharucoDetector(board);shifts=set()
    for size in (100,150):
        g=cv2.copyMakeBorder(board.generateImage((7*size,5*size)),30,30,30,30,cv2.BORDER_CONSTANT,value=255)
        c,ids,_,_=det.detectBoard(g)
        if ids is None or len(ids)!=24:raise RuntimeError('ChArUco pixel convention probe failed')
        r=c.reshape(-1,2)-np.array([[30+(i%6+1)*size-.5,30+(i//6+1)*size-.5] for i in ids.ravel()])
        if np.abs(r-.5).max()<.05:shifts.add(-.5)
        elif np.abs(r).max()<.05:shifts.add(0.)
        else:raise RuntimeError('Unknown ChArUco pixel convention')
    if len(shifts)!=1:raise RuntimeError('Inconsistent ChArUco pixel convention')
    return shifts.pop()


def make_board(c):
    d=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,c['dictionary']))
    b=cv2.aruco.CharucoBoard((c['squares_x'],c['squares_y']),c['square_length_m'],c['marker_length_m'],d,np.array(c['marker_ids'],np.int32))
    b.setLegacyPattern(bool(c['legacy_pattern']));return b


def board_corners(frames,K,base):
    """Mean sub-pixel corners over the frames: {id: uv}. Board ROI of the baseline first, the whole frame if the board left it."""
    board=make_board(read(KIT/'board.json'));shift=charuco_shift()
    def detect(roi):
        x0,y0,x1,y1=roi;cp=cv2.aruco.CharucoParameters();cp.cameraMatrix=K.copy();cp.cameraMatrix[:2,2]-=[x0,y0];cp.distCoeffs=np.zeros(5)
        det=cv2.aruco.CharucoDetector(board,cp);rows=[]
        for im in frames:
            cc,ci,_,_=det.detectBoard(im[y0:y1,x0:x1])
            if ci is not None and len(ci)>=LIMITS['min_corners']:rows.append({int(i):q for i,q in zip(ci.ravel(),cc.reshape(-1,2)+shift+[x0,y0])})
        return rows
    h,w=frames.shape[1:3];rows=detect(base['roi_1080'])
    if len(rows)<len(frames)//2:rows=detect([0,0,w,h])
    if len(rows)<len(frames)//2:raise ValueError(f'board found in {len(rows)}/{len(frames)} frames: is it covered or out of view?')
    # The baseline corner set: on this small board another subset alone moves the pose by ~0.5 deg (~1.7 mm on the pallet).
    ids=sorted(set.intersection(*(set(r) for r in rows),set(base['corner_ids'])))
    if len(ids)<LIMITS['min_corners']:raise ValueError(f'only {len(ids)} of the baseline board corners seen in every frame')
    if len(ids)<len(base['corner_ids']):print(f"주의: 기준 모서리 {len(base['corner_ids'])}개 중 {len(ids)}개만 보입니다(보드 일부 가림?)")
    a=np.array([[r[i] for i in ids] for r in rows])
    return ids,a.mean(0),float(np.sqrt(np.mean(np.sum((a-a.mean(0))**2,-1)))),len(rows),board


def board_pose(ids,uv,K,board):
    """T_color_from_board: IPPE branches refined by LM, best by RMS; refused when ambiguous or poor."""
    obj=np.array(board.getChessboardCorners(),float)[ids];out=[]
    ok,rs_,ts,_=cv2.solvePnPGeneric(obj,uv,K,np.zeros(5),flags=cv2.SOLVEPNP_IPPE)
    for r,t in zip(rs_,ts):
        r,t=cv2.solvePnPRefineLM(obj,uv,K,np.zeros(5),r.copy(),t.copy());T=np.eye(4);T[:3,:3]=cv2.Rodrigues(r)[0];T[:3,3]=t.ravel()
        if np.any((obj@T[:3,:3].T+T[:3,3])[:,2]<=0):continue
        p=cv2.projectPoints(obj,r,t,K,np.zeros(5))[0].reshape(-1,2);out.append((float(np.sqrt(np.mean(np.sum((p-uv)**2,1)))),T))
    if not out:raise ValueError('no board pose in front of the camera')
    out.sort(key=lambda x:x[0]);rms,T=out[0];alt=out[1][0] if len(out)>1 else None
    if rms>LIMITS['max_rms_px']:raise ValueError(f'board reprojection RMS {rms:.2f} px > {LIMITS["max_rms_px"]}')
    if alt is not None and alt-rms<=LIMITS['ambiguity_rms_px'] and angle_deg(T[:3,:3].T@out[1][1][:3,:3])>1:raise ValueError('two board poses fit equally well (ambiguous view)')
    return T,rms,alt


# ---------------------------------------------------------------- pixel <-> table plane
def to_plane(uv,T_color_from_base,z):
    Tb=np.linalg.inv(T_color_from_base);uv=np.atleast_2d(np.asarray(uv,float))
    d=(Tb[:3,:3]@np.linalg.inv(K640)@np.c_[uv,np.ones(len(uv))].T).T;c=Tb[:3,3]
    return c+d*((z-c[2])/d[:,2])[:,None]


def project(P,T_color_from_base):
    q=np.atleast_2d(P)@T_color_from_base[:3,:3].T+T_color_from_base[:3,3];p=(K640@(q/q[:,2:]).T).T
    return p[:,:2]


def remap(uv,old,new,z):return project(to_plane(uv,old,z),new)


def plane_homography(old,new,z):
    g=np.array([[x,y] for x in np.linspace(0,639,6) for y in np.linspace(0,479,5)],float)
    return cv2.findHomography(g,remap(g,old,new,z))[0]


# ---------------------------------------------------------------- server-side checks
def jig_meter(bc,camera,vision_cells):
    import jig_cells
    jig_cells.CAMERA=Path(camera)
    return jig_cells.JigCells(bc,vision_cells)


def server_view(small,bc,camera,vision_cells):
    """The server's ChArUco camera check and pallet measurement on each 640x480 frame (BGR, as the server passes it)."""
    meter=jig_meter(bc,camera,vision_cells);import jig_cells
    checks=[meter.camera_check(jig_cells.gradient(f)) for f in small];ms=[meter.measure(f) for f in small];good=[m for m in ms if m['ok']]
    pallet=jig_cells.JigCells.median(good) if good else None
    reasons=sorted({m['reason'] for m in ms if not m['ok']})
    return dict(camera_ok=sum(c['ok'] for c in checks),frames=len(small),shift_px=np.median([c['shift_px'] for c in checks],0).round(2).tolist(),
                score=round(float(min(c['score'] for c in checks)),3),
                pallet=None if pallet is None else dict(translation_mm=pallet['rigid']['translation_mm'],rotation_deg=pallet['rigid']['rotation_deg'],
                                                       at_m2=meter.at_reference(pallet),anchor=meter.match_anchor(pallet)),
                refused=reasons[:3])


def check_intrinsics(meta,base):
    out=[]
    for key,K in (('1920x1080',np.array(base['K_1080'])),('640x480',K640)):
        d=float(np.abs(kmat(meta[key])-K).max())
        if d>.5:out.append(f'{key} 내부 파라미터가 기준과 {d:.1f} px 다릅니다(다른 D435이거나 설정이 다름)')
    return out


def check(path):
    path,meta,big,small=load_capture(path);base=read(KIT/'baseline.json');cam=read(CAMERA);bc=burger();cells=read(VISION)['cells']
    warn=check_intrinsics(meta,base);K=kmat(meta['1920x1080'])
    ids,uv,jitter,n,board=board_corners(big,K,base);T,rms,_=board_pose(ids,uv,K,board)
    # Corners of the calibration now in use: the last solve's if it wrote them, else the 2026-10-18 baseline.
    rc=cam.get('board_corners_1080') or dict(ids=base['corner_ids'],mean_px=base['corner_mean_px_1080'])
    common=[i for i in ids if i in rc['ids']];ref=np.array(rc['mean_px'])[[rc['ids'].index(i) for i in common]]
    corner=float(np.sqrt(np.mean(np.sum((uv[[ids.index(i) for i in common]]-ref)**2,1))))
    old=np.array(cam['T_color_from_board']);D=np.linalg.inv(old)@T
    # What the change would do to the pallet measurement: the same pixel read through the old and the new calibration.
    Tbb=np.array(cam['T_base_from_board']);new=T@np.linalg.inv(Tbb);act=np.array(cam['T_color_from_base_link'])
    err=max(float(np.linalg.norm(to_plane(np.array(c['polygon']).mean(0),act,base['cell_top_z_m'][k])-to_plane(np.array(c['polygon']).mean(0),new,base['cell_top_z_m'][k]))*1000)
            for k,c in cells.items() if k.startswith('target_'))
    server=server_view(small,bc,CAMERA,cells)
    ok=server['camera_ok']>=len(small)*.8 and err<=LIMITS['check_pallet_mm'] and corner<=LIMITS['check_corner_px']
    report=dict(capture=str(path),ok=ok,board_corners=len(ids),board_frames=n,board_jitter_px=round(jitter,3),pose_rms_px=round(rms,3),
                corner_shift_from_baseline_px_1080=round(corner,2),pose_change=dict(rotation_deg=round(angle_deg(D[:3,:3]),3),translation_mm=round(float(np.linalg.norm(D[:3,3]))*1000,2)),
                pallet_error_mm=round(err,2),server=server,warnings=warn,limits=LIMITS)
    write(path/'check.json',report)
    print(f"차루코 모서리 {len(ids)}개 ({n}/{len(big)}장), 재투영 {rms:.2f} px, 떨림 {jitter:.2f} px")
    print(f"기준 대비 모서리 이동 {corner:.2f} px(1920x1080), 카메라 자세 변화 {report['pose_change']['rotation_deg']}° / {report['pose_change']['translation_mm']} mm")
    print(f"이 변화가 팔레트 측정에 주는 오차: 최대 {err:.2f} mm (허용 {LIMITS['check_pallet_mm']} mm)")
    print(f"서버 카메라 검사: {server['camera_ok']}/{server['frames']}장 통과, 이동 {server['shift_px']} px, 점수 {server['score']}")
    if server['pallet']:p=server['pallet'];print(f"팔레트 측정: 이동 {p['translation_mm']} mm, 회전 {p['rotation_deg']}°"+(' (m2 기준 위치)' if p['at_m2'] else f" (검증 위치 {p['anchor']})" if p['anchor'] else ''))
    elif server['refused']:print('팔레트 측정 거부:',' / '.join(server['refused']))
    for w in warn:print('주의:',w)
    print('판정:','지금 보정 그대로 사용' if ok else '재보정 필요 → camera-calib.sh solve')
    return 0 if ok else 3


def solve(path):
    path,meta,big,small=load_capture(path);base=read(KIT/'baseline.json');cam=read(CAMERA);bc=burger();vision=read(VISION)
    for w in check_intrinsics(meta,base):print('주의:',w)
    K=kmat(meta['1920x1080']);ids,uv,jitter,n,board=board_corners(big,K,base);T,rms,alt=board_pose(ids,uv,K,board)
    Tbb=np.array(cam['T_base_from_board']);old=np.array(cam['T_color_from_base_link']);new=T@np.linalg.inv(Tbb)
    out=new_dir(KIT/'candidates')
    c=dict(cam);bfc=np.linalg.inv(new)
    c.update(status='JETSON_CHARUCO_RECALIBRATION',created_at=datetime.now().astimezone().isoformat(),T_color_from_board=T.tolist(),
             T_base_link_from_color=bfc.tolist(),T_color_from_base_link=new.tolist(),T_base_link_from_depth=(bfc@np.array(cam['T_color_from_depth'])).tolist(),
             color_intrinsics=dict(meta['1920x1080'],model='distortion.'+meta['1920x1080']['model'].split('.')[-1]),pose_source=str(path),pose_branch=0,
             board_reprojection_RMS_px=rms,alternate_branch_RMS_px=alt,board_corner_count=len(ids),board_corners_1080=dict(ids=ids,mean_px=uv.round(3).tolist()),frame_count=n,capture_metadata=str(path/'metadata.json'),
             previous=dict(status=cam['status'],sha256=sha(CAMERA),T_color_from_board=cam['T_color_from_board']),
             frame_note='Camera-to-board pose re-estimated on the Jetson (camera_calib.py solve); board-to-base survey unchanged.')
    assert np.abs(bfc@T-Tbb).max()<1e-9
    write(out/'camera_transforms.json',c)
    # Vision cells: each polygon moved through its cell's top plane.
    v=json.loads(json.dumps(vision))
    for k,cell in v['cells'].items():cell['polygon']=remap(cell['polygon'],old,new,base['cell_top_z_m'][k]).round(1).tolist()
    v['calibration']['status']=c['status'];v['note']+=f' | {datetime.now():%Y-%m-%d}: polygons carried to the recalibrated D435 view (camera_calib.py).'
    write(out/'vision-map.json',v)
    # m2 reference: warped through the pallet plane; the ChArUco check region taken from the live frame.
    live=np.median(small,0).astype(np.uint8);ref=cv2.imread(str(reference_path(bc)))
    warped=cv2.warpPerspective(ref,plane_homography(old,new,bc.get('plane_z_m',.012)),(640,480),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
    x0,y0,x1,y1=bc.get('camera_check_roi',DEFAULT_ROI);box=remap([[x0,y0],[x1,y0],[x1,y1],[x0,y1]],old,new,Tbb[2,3])
    m=CHECK_SEARCH;roi=[int(max(m,np.floor(box[:,0].min()))),int(max(m,np.floor(box[:,1].min()))),int(min(640-m,np.ceil(box[:,0].max()))),int(min(480-m,np.ceil(box[:,1].max())))]
    if (roi[2]-roi[0])*(roi[3]-roi[1])<.7*(x1-x0)*(y1-y0):raise ValueError(f'ChArUco check region {roi} is mostly outside the 640x480 view: aim the D435 at the board again')
    warped[roi[1]:roi[3],roi[0]:roi[2]]=live[roi[1]:roi[3],roi[0]:roi[2]]
    cv2.imwrite(str(out/'jig-reference-m2.png'),warped);write(out/'model-map-patch.json',dict(camera_check_roi=roi))
    # Check the candidate with the server's own code on the live frames.
    bc2=dict(bc,reference=str(out/'jig-reference-m2.png'),camera_check_roi=roi);server=server_view(small,bc2,out/'camera_transforms.json',v['cells'])
    from vision_judge import VisionJudge
    judge=VisionJudge(out/'vision-map.json');res=judge.evaluate(cv2.cvtColor(live,cv2.COLOR_BGR2RGB));img=judge.annotate(cv2.cvtColor(live,cv2.COLOR_BGR2RGB),res)
    cv2.rectangle(img,tuple(roi[:2]),tuple(roi[2:]),(255,160,0),1);cv2.imwrite(str(out/'overlay.png'),img)
    D=np.linalg.inv(np.array(cam['T_color_from_board']))@T
    report=dict(capture=str(path),board_corners=len(ids),pose_rms_px=round(rms,3),pose_change=dict(rotation_deg=round(angle_deg(D[:3,:3]),3),translation_mm=round(float(np.linalg.norm(D[:3,3]))*1000,2)),
                camera_check_roi=roi,server=server,sets={k:s['count'] for k,s in res['sets'].items()},targets={k:t['filled'] for k,t in res['targets'].items()})
    write(out/'report.json',report)
    print(f"새 보드 자세: 모서리 {len(ids)}개, 재투영 {rms:.2f} px, 기존 대비 {report['pose_change']['rotation_deg']}° / {report['pose_change']['translation_mm']} mm")
    print(f"후보로 서버 검사: 카메라 {server['camera_ok']}/{server['frames']}장 통과",end='')
    if server['pallet']:p=server['pallet'];print(f", 팔레트 이동 {p['translation_mm']} mm 회전 {p['rotation_deg']}°"+(' (m2 기준)' if p['at_m2'] else f" ({p['anchor']})" if p['anchor'] else ''))
    else:print(', 팔레트 측정 거부:',' / '.join(server['refused']))
    print('세트 판정:',', '.join(f"{s['label']} {s['count']}/3" for s in res['sets'].values()))
    print('후보:',out);print('overlay.png에서 칸 테두리가 지그 칸 위에 있는지 확인한 뒤: bash camera-calib.sh apply',out.name)
    return 0


# ---------------------------------------------------------------- install / roll back
def files():return {'camera_transforms.json':CAMERA,'vision-map.json':VISION,'jig-reference-m2.png':reference_path(burger())}


def backup():
    out=new_dir(KIT/'backup')
    for name,p in files().items():shutil.copy2(p,out/name)
    shutil.copy2(MODEL_MAP,out/'model-map.json');return out


def install(src,roi):
    for name,p in files().items():shutil.copy2(src/name,p.with_name(p.name+'.tmp'));p.with_name(p.name+'.tmp').replace(p)
    m=read(MODEL_MAP)
    if roi is None:m['burger_correction'].pop('camera_check_roi',None)
    else:m['burger_correction']['camera_check_roi']=roi
    write(MODEL_MAP.with_name('model-map.json.tmp'),m);MODEL_MAP.with_name('model-map.json.tmp').replace(MODEL_MAP)


def apply(name):
    src=KIT/'candidates'/name
    if not (src/'report.json').exists():sys.exit(f'후보가 없습니다: {src}')
    lock=hardware_lock();b=backup();install(src,read(src/'model-map-patch.json')['camera_check_roi']);lock.close()
    print('적용했습니다. 이전 파일 백업:',b);print('되돌리기: bash camera-calib.sh restore',b.name)
    print('서버를 다시 켜야 반영됩니다. jetson-run.sh verify는 이 파일들을 "수정됨"으로 표시합니다(의도한 변경).')


def restore(name):
    src=KIT/'backup'/name
    if not (src/'model-map.json').exists():sys.exit(f'백업이 없습니다: {src}')
    lock=hardware_lock();b=backup();install(src,read(src/'model-map.json')['burger_correction'].get('camera_check_roi'));lock.close()
    print('되돌렸습니다:',src.name,'| 되돌리기 전 상태 백업:',b.name)


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter);s=p.add_subparsers(dest='cmd',required=True)
    s.add_parser('capture');[s.add_parser(c).add_argument('capture',nargs='?') for c in ('check','solve')]
    s.add_parser('apply').add_argument('candidate');s.add_parser('restore').add_argument('backup')
    a=p.parse_args()
    if a.cmd=='capture':return capture()
    if a.cmd in ('check','solve'):
        try:return (check if a.cmd=='check' else solve)(a.capture)
        except ValueError as e:print('실패:',e);return 2
    return (apply if a.cmd=='apply' else restore)(getattr(a,'candidate',None) or a.backup)


if __name__=='__main__':sys.exit(main() or 0)
