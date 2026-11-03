"""Opt-in project paths; no environment selection retains legacy behavior."""
import os
from pathlib import Path

PROJECTS = Path(__file__).resolve().parents[2]/'projects'


def project_root():
    value = os.environ.get('ARM3_PROJECT')
    if not value: return None
    root = Path(value).resolve()
    if root.parent != PROJECTS.resolve() or not (root/'project.json').is_file():
        raise ValueError('Invalid ARM3_PROJECT directory')
    return root


def project_path(name, legacy_root):
    root = project_root()
    if root is None: return Path(legacy_root)/name
    mapping = {'purple_task.json':'task.json', 'start_pose.json':'start_pose.json',
               'captures':'captures', 'logs':'logs', 'training_act.json':'training_act.json',
               'completion/product_a.json':'completion/product_a.json'}
    return root/mapping[name]
