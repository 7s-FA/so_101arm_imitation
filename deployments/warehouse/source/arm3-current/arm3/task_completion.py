"""Fixed-view visual completion check for the recorded product-A task.

This is a scene-specific appearance check, not a general object/pose estimator.
References and thresholds are built from successful demonstrations, separately
from the ACT weights. Changed camera/jig layouts require recalibration.
"""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
DEFAULT_PROFILE = ROOT/'completion/product_a.json'
# Overview camera, 640x480. Separate checks prevent one placed part dominating.
REGIONS = {
    'destination_top': [443, 137, 510, 201],
    'destination_middle': [463, 211, 508, 261],
    'destination_bottom': [456, 272, 527, 339],
    'source_top_empty': [197, 310, 255, 373],
    'source_middle_empty': [278, 312, 325, 367],
    'source_bottom_empty': [351, 305, 410, 371],
    'jig_anchor': [530, 141, 603, 345],
}
SHAPE_REGIONS = {**{k:v for k,v in REGIONS.items() if k.startswith('destination_')},
                 'jig_clear': [434,135,611,355]}


def patches(rgb, regions):
    if rgb.shape != (480, 640, 3) or rgb.dtype != np.uint8:
        raise ValueError('Completion requires overview RGB uint8 640x480')
    return np.stack([cv2.resize(rgb[y1:y2, x1:x2], (32,32), interpolation=cv2.INTER_AREA)
                     for x1,y1,x2,y2 in regions.values()]).astype(np.float32)/255


class CompletionDetector:
    def __init__(self, profile=DEFAULT_PROFILE):
        self.profile = Path(profile)
        self.config = json.loads(self.profile.read_text())
        self.presence = None
        method = self.config.get('method', 'appearance')
        if method == 'color_presence':
            from part_presence import PartPresence
            self.presence = PartPresence(self.config['presence'])
            self.reset()
            return
        if method != 'appearance':
            raise ValueError('Unknown completion method')
        ref = self.profile.parent/self.config['references']
        if hashlib.sha256(ref.read_bytes()).hexdigest() != self.config['references_sha256']:
            raise ValueError('Completion reference checksum mismatch')
        with np.load(ref, allow_pickle=False) as data:
            self.references = data['patches']
        shape_file = self.profile.parent/self.config['shape_references']
        if hashlib.sha256(shape_file.read_bytes()).hexdigest() != self.config['shape_references_sha256']:
            raise ValueError('Completion shape reference checksum mismatch')
        self.shape_regions = self.config['shape_regions']
        with np.load(shape_file, allow_pickle=False) as data:
            self.shapes = [data[f'gray_{i}'] for i in range(len(self.shape_regions))]
        self.regions = self.config['regions']
        for rect in self.regions.values():
            if len(rect) != 4 or any(type(v) is not int for v in rect):
                raise ValueError('Invalid completion region')
            x1,y1,x2,y2 = rect
            if not (0 <= x1 < x2 <= 640 and 0 <= y1 < y2 <= 480):
                raise ValueError('Completion region outside camera image')
        self.thresholds = np.asarray(self.config['thresholds'], dtype=np.float32)
        if self.references.shape[1:] != (len(self.regions),32,32,3):
            raise ValueError('Invalid completion references')
        if not len(self.references) or not np.isfinite(self.references).all():
            raise ValueError('Invalid completion reference values')
        if self.thresholds.shape != (len(self.regions),) or not np.isfinite(self.thresholds).all() or not ((self.thresholds > 0) & (self.thresholds <= .15)).all():
            raise ValueError('Invalid completion thresholds')
        self.required_colors = self.config.get('required_color_regions', [])
        if not isinstance(self.required_colors, list) or any(k not in self.regions for k in self.required_colors):
            raise ValueError('Unknown required completion color region')
        self.shape_thresholds = {name:self.config['shape_min_score'] for name in self.shape_regions}
        overrides = self.config.get('shape_min_scores', {})
        if not isinstance(overrides, dict) or any(name not in self.shape_regions for name in overrides):
            raise ValueError('Unknown completion shape threshold region')
        self.shape_thresholds.update(overrides)
        if any(not isinstance(v,(int,float)) or not np.isfinite(v) or not 0 < v <= 1 for v in self.shape_thresholds.values()):
            raise ValueError('Invalid completion shape threshold')
        self.expected_blue = None
        if 'expected_blue_mask' in self.config:
            path = self.profile.parent/self.config['expected_blue_mask']
            if hashlib.sha256(path.read_bytes()).hexdigest() != self.config['expected_blue_mask_sha256']:
                raise ValueError('Expected blue mask checksum mismatch')
            with np.load(path, allow_pickle=False) as data:
                self.expected_blue = data['mask']
            if self.expected_blue.shape != (280,207) or self.expected_blue.dtype != np.bool_:
                raise ValueError('Invalid expected blue mask')
        self.reset()

    def validate_identity(self, current):
        if current != self.config['identity']:
            raise ValueError('Completion profile camera/calibration/task identity mismatch')

    def reset(self):
        self.since = self.last_stamp = None
        self.last = {}
        self.best = {}
        self.best_rgb = None
        self.best_score = -1.0

    def shape_scores(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        scores = {}
        for (name,(x1,y1,x2,y2)), refs in zip(self.shape_regions.items(),self.shapes):
            crop = gray[max(0,y1-10):min(480,y2+10),max(0,x1-10):min(640,x2+10)]
            scores[name] = max(float(cv2.minMaxLoc(cv2.matchTemplate(crop,ref,cv2.TM_CCOEFF_NORMED))[1]) for ref in refs)
        return scores

    def evaluate(self, rgb):
        if self.presence is not None:
            return self.presence.evaluate(rgb)
        values = patches(rgb, self.regions)
        distances = np.abs(self.references-values[None]).mean(axis=(2,3,4)).min(axis=0)
        scores = self.shape_scores(rgb)
        # Blue gripper fingers must be outside the expanded destination area.
        hsv = cv2.cvtColor(rgb[110:390, 418:625], cv2.COLOR_RGB2HSV)
        # Exclude purple parts (live AWB moved their hue to 109..112) and
        # low-saturation cable reflections. Shape checks also detect occlusion.
        blue = cv2.inRange(hsv, (80,130,35), (104,255,255))
        if self.expected_blue is not None:
            # Exempt only the demonstrated cylinder footprint. Its appearance
            # and the surrounding jig still have to pass independent checks.
            blue[self.expected_blue] = 0
        blue_fraction = float(np.count_nonzero(blue)/blue.size)
        matches = {k:v >= self.shape_thresholds[k] for k,v in scores.items()}
        color_matches = {k:bool(v) for k,v in zip(self.regions,distances <= self.thresholds)}
        for name in self.required_colors:
            matches[name+'_color'] = color_matches[name]
        result = {'matches': matches, 'shape_scores':scores,
                  'source_advisory': {k:bool(v) for k,v in zip(self.regions,distances <= self.thresholds) if k.startswith('source_')},
                  'distances': {k:float(v) for k,v in zip(self.regions,distances)},
                  'gripper_clear': blue_fraction < .003, 'blue_fraction': blue_fraction}
        result['candidate'] = bool(all(matches.values()) and result['gripper_clear'])
        return result

    def observe(self, rgb, stamp):
        if not np.isfinite(stamp): raise ValueError('Invalid camera timestamp')
        if self.last_stamp is not None and stamp == self.last_stamp:
            return False  # Re-reading one buffered frame cannot confirm completion.
        if self.last_stamp is not None and (stamp < self.last_stamp or stamp-self.last_stamp > .25):
            self.since = None
        self.last_stamp = stamp
        self.last = self.evaluate(rgb)
        scores = self.last.get('presence_scores', self.last.get('shape_scores', {}))
        score = min(scores.values()) if self.last['gripper_clear'] else 0.0
        if score > self.best_score:
            self.best_score = score
            self.best = dict(self.last)
            self.best_rgb = rgb.copy()
        if not self.last['candidate']:
            self.since = None
            return False
        if self.since is None: self.since = stamp
        self.last['stable_seconds'] = stamp-self.since
        return stamp-self.since >= 1.0
