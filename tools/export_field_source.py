#!/usr/bin/env python3
"""Reconstruct captured field sources in a new review folder. No hardware I/O."""
import argparse
import json
from pathlib import Path
import subprocess
from public_examples import examples

ROOT = Path(__file__).resolve().parents[1]

def export(device, output):
    target = Path(output).absolute()
    if target.exists():
        raise ValueError('Output already exists; choose a new directory')
    snapshot = ROOT / 'deployments' / device
    meta = json.loads((snapshot / 'snapshot.json').read_text())
    prefix = meta.get('base_prefix', '')
    def git(*args):
        return subprocess.check_output(['git', '-C', str(ROOT), *args])
    entries = []
    for row in git('ls-tree', '-r', '-z', meta['comparison_base']).split(b'\0'):
        if not row:
            continue
        head, name = row.split(b'\t', 1)
        mode, kind, sha = head.decode().split()
        name = name.decode()
        if not name.startswith(prefix):
            continue
        rel = Path(name[len(prefix):])
        if rel.is_absolute() or '..' in rel.parts or kind != 'blob' or mode not in ('100644', '100755'):
            raise ValueError('Unsupported entry: ' + name)
        entries.append((rel, sha, mode))
    target.mkdir(parents=True)
    def write(rel, data, executable=False):
        path = target / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            data = examples(data.decode('utf-8')).encode('utf-8')
        except UnicodeDecodeError:
            pass
        path.write_bytes(data)
        if executable:
            path.chmod(0o755)
    for rel, sha, mode in entries:
        write(rel, git('cat-file', 'blob', sha), mode == '100755')
    for folder in ('source', 'external-linear'):
        for path in (snapshot / folder).rglob('*'):
            if path.is_file():
                rel = path.relative_to(snapshot / folder)
                if folder != 'source':
                    rel = Path(folder) / rel
                write(rel, path.read_bytes(), bool(path.stat().st_mode & 0o111))
    (target / 'DEPLOYMENT_README.md').write_text(
        '# Field review export\n\nBase: ' + meta['comparison_base'] + '\n\n'
        'Set example IPs, accounts and paths for your installation.\n'
        'Missing field files were not interpreted as upstream deletions.\n'
        'External linear files retain a separate folder; review installation paths.\n'
        'This export does not install, restart services or move hardware.\n')
    return target

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('device', choices=['warehouse', 'workcell'])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(export(args.device, args.output))
