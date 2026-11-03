"""Keep working Jetson GPU wheels; install application dependencies under constraints."""
import argparse,json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def run(*args):subprocess.run([str(x) for x in args],check=True)
def valid_python(p):
    try:return subprocess.run([str(p),'-c','import sys; assert sys.version_info[:2]==(3,12)'],capture_output=True).returncode==0
    except OSError:return False
def cuda_python(p):
    try:return subprocess.run([str(p),'-c','import torch,torchvision; assert torch.cuda.is_available()'],capture_output=True).returncode==0
    except OSError:return False

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--python',type=Path)
    parser.add_argument('--torch-wheel');parser.add_argument('--torchvision-wheel');parser.add_argument('--realsense-wheel')
    parser.add_argument('--ik-only',action='store_true',help='upgrade a working 10-08 environment: only add the bundled IK wheels (offline)')
    a=parser.parse_args()
    if bool(a.torch_wheel)!=bool(a.torchvision_wheel):parser.error('Provide BOTH matching torch and torchvision wheels')
    selected=a.python
    if selected is None and (ROOT/'.jetson-python').is_file():selected=Path((ROOT/'.jetson-python').read_text().strip())
    if selected is None:
        # Previous deployment may already have CUDA/RealSense working. Reuse only
        # a confirmed Python 3.12 venv with GPU torch; never its older robot code.
        previous=[Path(f.read_text().strip()) for f in sorted(Path.home().glob('robot3_ws/arm1_jetson_*/.jetson-python')) if f.is_file()]
        for candidate in (ROOT/'.venv/bin/python',*previous,Path.home()/'robot3_ws/arm1_imitation/.venv/bin/python'):
            if valid_python(candidate) and cuda_python(candidate):selected=candidate;break
    if selected is None:
        if not (ROOT/'.venv/bin/python').exists():run('/usr/bin/python3','-m','venv','--system-site-packages',ROOT/'.venv')
        selected=ROOT/'.venv/bin/python'
    # Do NOT resolve venv interpreter symlinks: that would select system Python.
    selected=selected.expanduser().absolute()
    if not valid_python(selected):parser.error('Selected Python must be 3.12')
    prefix=subprocess.check_output([str(selected),'-c','import sys; print(sys.prefix != sys.base_prefix)'],text=True).strip()
    if prefix!='True':parser.error('--python must point to a virtualenv; system pip will not be modified')
    (ROOT/'.jetson-python').write_text(str(selected)+'\n')
    print('Selected Jetson environment:',selected,flush=True)
    def ik_wheels():
        # IK plan generation (real_plan_burger.py): MuJoCo kinematics + scipy, ARM64 wheels shipped in wheels/ (no internet).
        run(selected,'-m','pip','install','--no-index','--find-links',ROOT/'wheels','mujoco==3.3.7','scipy==1.18.1')
    if a.ik_only:
        if not cuda_python(selected):raise SystemExit('Selected environment has no working CUDA torch; run the full installer instead of --ik-only.')
        ik_wheels();run(selected,ROOT/'tools/doctor.py','--require-cuda');return
    if a.torch_wheel:
        run(selected,'-m','pip','install',a.torch_wheel,a.torchvision_wheel)
    if not cuda_python(selected):
        raise SystemExit('CUDA torch/torchvision is missing or incompatible. Keep JetPack unchanged. Use --python with an existing GPU venv, or supply --torch-wheel and --torchvision-wheel matching this JetPack. See README.md official NVIDIA links. No generic torch was installed.')
    versions=json.loads(subprocess.check_output([str(selected),'-c',
        'import importlib.metadata as m,json; print(json.dumps({n:m.version(n) for n in ("torch","torchvision")}))'],text=True))
    constraints=ROOT/'logs/gpu-constraints.txt';constraints.parent.mkdir(exist_ok=True)
    constraints.write_text(''.join(f'{name}=={version}\n' for name,version in versions.items()))
    run(selected,'-m','pip','install','-c',constraints,'-r',ROOT/'requirements-runtime.txt')
    ik_wheels()
    if a.realsense_wheel:run(selected,'-m','pip','install',a.realsense_wheel)
    elif subprocess.run([str(selected),'-c','import pyrealsense2'],capture_output=True).returncode:
        run(selected,'-m','pip','install','--only-binary=:all:','pyrealsense2>=2.56,<3')
    run(selected,ROOT/'tools/doctor.py','--require-cuda')
if __name__=='__main__':main()
