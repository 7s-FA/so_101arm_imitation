"""Own the mock server process and always shut it down after isolated tests."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT.parents[1]
env=dict(os.environ,ROS_DOMAIN_ID='141',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
         ROS_LOG_DIR=str(ROOT/'logs/ros'))
env.pop('ROS_LOCALHOST_ONLY',None)
path=ROOT/'logs/mock-suite-server.log'
with path.open('w') as output:
    process=subprocess.Popen(['bash',str(WORK/'arm1-control.sh'),'serve','--domain-id','141'],
                             env=env,stdout=output,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError(path.read_text())
            if 'ARM1 READY mode=MOCK domain=141' in path.read_text():break
            time.sleep(.1)
        else:raise RuntimeError('Mock server did not become ready')
        result=subprocess.run(['/usr/bin/python3',str(ROOT/'tests/ros_roundtrip.py')],env=env,timeout=90)
        code=result.returncode
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=5)
    if process.returncode not in (0,-signal.SIGINT):raise RuntimeError(f'Mock server shutdown failed: {path}')
raise SystemExit(code)
