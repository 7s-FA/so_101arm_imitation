"""D435 overview judge: which source sets are complete, which destination cells are filled, which set a
SELECT_A/SELECT_B job takes. Pure numpy/OpenCV on the RGB frames of record_purple.Cameras; no robot access.

A cell is present when at least `present_fraction` of its polygon matches its part color (HSV ranges in
vision-map.json). A set is available only when all three of its cells are present. Selection order is
fixed by vision-map.json: purple lower set first, otherwise upper; orange left set first, otherwise right.
"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


class VisionJudge:
    def __init__(self, path=ROOT/'vision-map.json'):
        self.config = json.loads(Path(path).read_text())
        if self.config.get('version') != 1: raise ValueError('Expected vision map version 1')
        w, h = self.config['image_size']
        self.size = (h, w)
        self.masks = {}
        for name, cell in self.config['cells'].items():
            mask = np.zeros(self.size, np.uint8)
            cv2.fillPoly(mask, [np.array(cell['polygon'], np.int32)], 1)
            if mask.sum() < 20: raise ValueError(f'{name}: polygon too small')
            self.masks[name] = mask.astype(bool)

    def colors(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        out = {}
        for kind, ranges in self.config['colors_hsv'].items():
            ok = np.zeros(self.size, bool)
            for h0, h1, s0, v0, v1 in ranges:
                ok |= ((hsv[..., 0] >= h0) & (hsv[..., 0] <= h1) & (hsv[..., 1] >= s0)
                       & (hsv[..., 2] >= v0) & (hsv[..., 2] <= v1))
            out[kind] = ok
        return out

    def evaluate(self, rgb):
        if rgb is None or rgb.shape[:2] != self.size: raise ValueError(f'Expected a {self.size} overview frame')
        colors, limit = self.colors(rgb), self.config['present_fraction']
        cells = {}
        for name, cell in self.config['cells'].items():
            fraction = float(colors[cell['kind']][self.masks[name]].mean())
            cells[name] = {'fraction': round(fraction, 3), 'present': fraction >= limit}
        sets = {}
        for key, spec in self.config['sets'].items():
            count = sum(cells[c]['present'] for c in spec['cells'])
            sets[key] = {'label': spec['label'], 'count': count, 'complete': count == len(spec['cells'])}
        targets = {product: {'filled': [c for c in names if cells[c]['present']], 'total': len(names)}
                   for product, names in self.config['targets'].items()}
        return {'cells': cells, 'sets': sets, 'targets': targets}

    def select(self, command, result):
        """{'set': key or None, 'label', 'error', 'reason'}; error is None when the job may start."""
        order = self.config['order'].get(command)
        if order is None: return {'set': None, 'label': None, 'error': 'UNKNOWN_COMMAND', 'reason': command}
        for key in order:
            if result['sets'][key]['complete']:
                return {'set': key, 'label': result['sets'][key]['label'], 'error': None,
                        'reason': '세트 선택 규칙: ' + ' → '.join(self.config['sets'][k]['label'] for k in order)}
        partial = [result['sets'][k]['label'] + f" {result['sets'][k]['count']}/3" for k in order if result['sets'][k]['count']]
        return {'set': None, 'label': None, 'error': 'NO_MATERIAL',
                'reason': '완전한 세트 없음' + (' (일부만 있음: ' + ', '.join(partial) + ')' if partial else '')}

    def annotate(self, rgb, result, selection=None):
        """BGR image: green outline = present, red = empty; the selected set's cells drawn thick yellow."""
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        chosen = set(self.config['sets'][selection['set']]['cells']) if selection and selection.get('set') else set()
        for name, cell in self.config['cells'].items():
            poly = np.array(cell['polygon'], np.int32)
            color = (0, 200, 0) if result['cells'][name]['present'] else (0, 0, 230)
            cv2.polylines(img, [poly], True, color, 1)
            if name in chosen: cv2.polylines(img, [poly], True, (0, 230, 255), 3)
        return img
