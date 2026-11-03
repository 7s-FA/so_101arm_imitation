import argparse
from pathlib import Path
from backend import HardwareBackend, load_map

if __name__=='__main__':
    p=argparse.ArgumentParser(description='Load both models without robot/camera access')
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('model-map.json'))
    p.add_argument('--device',choices=['cpu','cuda'],default='cpu')
    a=p.parse_args()
    backend=HardwareBackend(load_map(a.config),a.device)
    assert backend.bus is None and backend.cameras is None
    print('A/B model checks passed. No devices opened.')
