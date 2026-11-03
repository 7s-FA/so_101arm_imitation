"""Dispatch A/B using the same model map as the action server."""
import json, os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
def command(which, extra):
    spec=json.loads((ROOT/'integration/arm1/model-map.json').read_text())['models']['SELECT_'+which]
    if any(a=='--model' or a.startswith('--model=') or a=='--completion-profile' or a.startswith('--completion-profile=') for a in extra):
        raise ValueError('Change integration/arm1/model-map.json to select a model for BOTH manual and server use.')
    options=[]
    if not any(a in ('--seconds','--no-time-limit','--until-done') or a.startswith('--seconds=') for a in extra):
        options+=['--until-done']
    if not any(a in ('--seconds','--no-time-limit','--release-on-complete','--return-to-start-on-complete') or a.startswith('--seconds=') for a in extra):
        options+=['--return-to-start-on-complete']
    env=dict(os.environ,ARM3_PROJECT=str(ROOT/'projects'/spec['project']))
    args=[sys.executable,'-u',str(ROOT/'arm3-current/arm3/run_policy.py'),
          '--model',str(ROOT/spec['model']),'--completion-profile',str(ROOT/spec['completion']),*options,*extra]
    return args,env

if __name__=='__main__':
    args,env=command(sys.argv[1],sys.argv[2:])
    os.execve(sys.executable,args,env)
