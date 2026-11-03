"""Offline USB integrity check. Only reads files; no model or hardware imports."""
from pathlib import Path
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1]

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def main():
    manifest=json.loads((ROOT/'SHA256.json').read_text());bad=[]
    for name,expected in manifest.items():
        p=ROOT/name
        if not p.is_file() or digest(p)!=expected:bad.append(name)
    if bad:
        print('Missing/modified files (after intentional config edits this is expected):',*bad,sep='\n')
        return 1
    print(f'USB package verified: {len(manifest)} files')
    return 0
if __name__=='__main__':sys.exit(main())
