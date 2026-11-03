"""Measure repeated fresh ACT predictions, no camera or motor access."""
import argparse,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'integration/arm1'))
from backend import HardwareBackend,load_map

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--device',default='cuda',choices=['cpu','cuda']);p.add_argument('--rounds',type=int,default=10);a=p.parse_args()
    if a.rounds<3:p.error('At least three rounds required')
    b=HardwareBackend(load_map(ROOT/'integration/arm1/model-map.json'),a.device)
    report={}
    for command,(_,_,_,predictor,_) in b.models.items():
        frames={n:(b.np.zeros((480,640,3),dtype=b.np.uint8),time.monotonic()) for n in b.p.CAMERAS}
        durations=[]
        for _ in range(a.rounds):
            predictor.reset() # Never benchmark cached actions as fresh inference.
            start=time.perf_counter();predictor.predict(b.np.zeros(6,dtype=b.np.float32),frames)
            durations.append((time.perf_counter()-start)*1000)
        report[command]={'fresh_prediction_ms':durations,'max_ms':max(durations),'mean_ms':sum(durations)/len(durations)}
    print(json.dumps(report,indent=2))
    (ROOT/'logs').mkdir(exist_ok=True)
    (ROOT/'logs/benchmark.json').write_text(json.dumps(report,indent=2)+'\n')
    print('No devices opened. Full loop also includes camera processing and 6-motor communication; deadline remains 250ms.')
    sys.exit(1 if any(r['max_ms']>=250 for r in report.values()) else 0)
