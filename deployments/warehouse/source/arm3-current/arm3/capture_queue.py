"""Read completed raw captures awaiting dataset export; no hardware access."""
import json
from pathlib import Path


def pending_sessions(root, *, strict=False):
    result = []
    for path in sorted(Path(root).glob('*/session.json')):
        if path.is_symlink() or path.parent.is_symlink():
            if strict: raise ValueError(f'Capture symlink is not supported: {path}')
            continue
        try:
            manifest = json.loads(path.read_text())
        except (OSError, ValueError):
            if strict: raise
            continue  # A recorder may be updating its current session manifest.
        if (manifest.get('success') and manifest.get('exported_episode') is None
                and not manifest.get('excluded_from_dataset')):
            result.append((path.parent, manifest))
    return result
