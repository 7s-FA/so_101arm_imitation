"""IK correction self-test WITHOUT devices: measure a stored D435 frame, then build a retargeted plan.

1. jig_cells measures the m1 burger frame saved on 2026-10-25 (same code as the server PRECHECK).
2. real_plan_burger.py retargets the B_left plan to a stored real docking (2026-10-26 11:33) with the server's
   own subprocess call, using this environment's Python (mujoco + scipy). The result must keep the frame count
   and stay inside the servo margins. Nothing opens a camera, serial port or motor.
"""
import json,subprocess,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];ARM1=ROOT/'integration/arm1'
sys.path.insert(0,str(ARM1))

def main():
    import cv2
    from jig_cells import JigCells
    from vision_judge import VisionJudge
    config=json.loads((ARM1/'model-map.json').read_text());cfg=config['burger_correction']
    jc=JigCells(cfg,VisionJudge().config['cells'])
    m=jc.measure(cv2.imread(str(ARM1/'tests/data/m1_live_20261025_160823.png')))
    print('1) m1 frame measured:',m['ok'],'translation mm',m['rigid']['translation_mm'],'rotation deg',m['rigid']['rotation_deg'],'camera',m['camera'])
    assert m['ok'] and jc.match_anchor(m)=='m1','m1 frame must match the verified m1 burger'
    sample=ROOT/'ik/samples/20261026_113344_B_left';base=Path(config['plans']['B_left'])
    with tempfile.TemporaryDirectory() as tmp:
        out=Path(tmp)/'plan.json';t0=time.monotonic()
        r=subprocess.run([cfg.get('retarget_python') or sys.executable,cfg['retarget_script'],str(base),'--set','B_left','--offsets',str(sample/'offsets.json'),
                          '--extra',str(sample/'extra.json'),'--output',str(out)],capture_output=True,text=True,timeout=cfg.get('retarget_timeout_s',90),cwd=ROOT)
        print(r.stdout.strip()[-600:]);assert r.returncode==0,r.stderr[-800:]
        plan=json.loads(out.read_text());ref=json.loads((ROOT/'ik/samples/20261026_113344_B_left/plan_pc.json').read_text())
        diff=max(abs(a-b) for x,y in zip(plan['raw'],ref['raw']) for a,b in zip(x,y))
        print(f'2) retargeted B_left plan: {len(plan["raw"])} frames in {time.monotonic()-t0:.1f} s; max difference to the PC result {diff} ticks')
        assert len(plan['raw'])==len(ref['raw']) and diff<=2,'retarget differs from the PC result'
    print('IK 보정 시험 통과 (장치 미사용).')

if __name__=='__main__':main()
