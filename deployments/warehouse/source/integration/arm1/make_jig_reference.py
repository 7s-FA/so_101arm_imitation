"""Write jig-offset.json from a reference overview frame (the pallet where the plans were verified). Read-only.

Coarse pallet edges (image px, 640x480) are refined on the reference frame with the same edge search the meter
uses, so the stored lines are exactly where the meter finds them in that frame (offset 0 by construction).
"""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np

HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from jig_offset import JigOffsetMeter

CAMERA=Path(__file__).resolve().parents[2]/'ik/isaac_migration/configs/calibration/camera_transforms.json'
K640=[[607.4738,0,320.5028],[0,607.5820,251.5071],[0,0,1]]     # D435 serial 920312070850, color 640x480
COARSE={'left':[[436,180],[446,325]],'bottom':[[453,330],[604,330]],'right':[[584,160],[597,245]]}   # the middle seam is two offset segments: not used
LIMITS=dict(band_px=12,min_step=6.,min_points_fraction=.6,max_rms_px=1.,max_offset_mm=8.,max_rot_deg=3.)
PLANE_Z=.012   # pallet top edge height above the base plane (m); a 10mm error scales the result by ~2%


def main():
    p=argparse.ArgumentParser();p.add_argument('--frame',type=Path,required=True,help='reference overview image or video (first frame)')
    p.add_argument('--output',type=Path,default=HERE/'jig-offset.json');a=p.parse_args()
    img=cv2.imread(str(a.frame)) if a.frame.suffix in ('.png','.jpg') else cv2.VideoCapture(str(a.frame)).read()[1]
    if img is None or img.shape[:2]!=(480,640):raise ValueError('need a 640x480 overview frame')
    cam=json.loads(CAMERA.read_text())
    cfg=dict(reference=str(a.frame),K=K640,T_color_from_base_link=cam['T_color_from_base_link'],camera_status=cam['status'],plane_z_m=PLANE_Z,lines=COARSE,limits=LIMITS)
    tmp=a.output.with_suffix('.tmp.json');tmp.write_text(json.dumps(cfg))
    m=JigOffsetMeter(tmp);gray=cv2.GaussianBlur(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY).astype(np.float32),(3,3),0);lines={}
    for name,line in m.lines.items():
        c,u,rms,n=m.fit(m.edge_points(gray,line));L=np.linalg.norm(line[1]-line[0])
        ends=np.array([c+((line[0]-c)@u)*u,c+((line[1]-c)@u)*u]);lines[name]=ends.round(3).tolist()
        print(name,'points',n,'rms px',round(rms,3),'moved px',np.round(np.abs((line-c)@np.array([-u[1],u[0]])).max(),2))
    cfg['lines']=lines;tmp.unlink();a.output.write_text(json.dumps(cfg,indent=1)+'\n')
    cv2.imwrite(str(a.output.with_suffix('.reference.png')),img)
    print('check on the reference itself:',JigOffsetMeter(a.output).measure(img))

if __name__=='__main__':main()
