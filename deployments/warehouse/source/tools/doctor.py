"""Inspect environment, imports and device names WITHOUT opening devices."""
import argparse,importlib,importlib.metadata,json,os,platform,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--require-cuda',action='store_true');a=p.parse_args()
    errors=[]
    print('Python:',sys.executable,sys.version.split()[0],'architecture:',platform.machine())
    for path in ('/etc/os-release','/etc/nv_tegra_release','/proc/device-tree/model'):
        f=Path(path)
        if f.is_file():print(path,f.read_text().replace('\0','').strip())
    if sys.version_info[:2]!=(3,12):errors.append('This package targets Python 3.12 + ROS Jazzy on Ubuntu 24.04')
    for name in ('torch','torchvision','lerobot','cv2','numpy','serial','scservo_sdk','pyrealsense2','mujoco','scipy'):
        try:
            m=importlib.import_module(name);print(name,getattr(m,'__version__','import OK'))
        except Exception as e:errors.append(f'{name}: {e}')
    try:
        import torch
        print('CUDA available:',torch.cuda.is_available(),'built CUDA:',torch.version.cuda)
        if torch.cuda.is_available():
            print('GPU:',torch.cuda.get_device_name(0),'capability:',torch.cuda.get_device_capability(0))
            x=torch.ones((32,32),device='cuda');assert (x@x).isfinite().all().item();torch.cuda.synchronize()
        elif a.require_cuda:errors.append('CUDA unavailable: install a JetPack-compatible torch/torchvision build')
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        print('ACT/processor imports OK')
    except Exception as e:errors.append(f'GPU/ACT check: {e}')
    cfg=json.loads((ROOT/'arm3-current/arm3/devices.json').read_text())
    for name,path in [('follower',cfg['robot']['port']),('wrist',cfg['cameras']['rgb']['path'])]:
        print(name,path,'exists:',Path(path).exists(),'read/write:',os.access(path,os.R_OK|os.W_OK))
    print('Expected RealSense serial:',cfg['cameras']['realsense']['serial_number'])
    print('Device paths:',*[str(p) for p in Path('/dev/v4l/by-id').glob('*')])
    print('ROS Jazzy installed:',Path('/opt/ros/jazzy/setup.bash').is_file())
    print('No camera/serial device was opened. Hardware connectivity is NOT confirmed.')
    for error in errors:print('ERROR:',error)
    return bool(errors)
if __name__=='__main__':sys.exit(main())
