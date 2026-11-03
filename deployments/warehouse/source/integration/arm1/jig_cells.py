"""Target-pallet (burger) pose per destination cell from one D435 overview frame (read-only).

The six destination jig cells of the reference burger (m2, the pose the plans were verified on) are cut from
the reference frame (vision-map polygon + 6px, gradient magnitude) and found again in the current frame by
normalised cross-correlation within +-search_px, to sub-pixel. First the fixed ChArUco board is checked the
same way: if it moved the camera moved, and the measurement is refused. Each cell's image shift is mapped onto the base
plane (ChArUco camera transform, D435 640x480 intrinsics) as that cell's offset in robot base mm; a rigid fit
of the six gives the pallet rotation. Low match scores, a large rigid-fit residual or offsets outside the
supported range are refused (the job then reports an error instead of guessing).

Also: per-part extra shifts interpolated from real-robot anchors (burger_m1_*/real_anchors.json) by the
pallet translation's projection onto the m1 translation - an interim rule until the RL policy provides them.
"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
K640 = np.array([[607.4738, 0, 320.5028], [0, 607.5820, 251.5071], [0, 0, 1]])    # D435 serial 920312070850, color 640x480
CAMERA = ROOT.parents[1]/'ik/isaac_migration/configs/calibration/camera_transforms.json'
M2_SIDE_ALPHA = .6   # Dockings seen so far: m2 0.23-0.53, m1 0.72-0.94 along the m2->m1 direction.


def gradient(bgr):
    g = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32), (3, 3), 0)
    return cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1))


class JigCells:
    def __init__(self, config, cells):
        """config: model-map.json burger_correction; cells: vision-map cells (polygons of target_* used)."""
        self.c = config
        self.reference = cv2.imread(str(ROOT/config['reference']))
        if self.reference is None or self.reference.shape[:2] != (480, 640): raise ValueError('burger reference must be a 640x480 overview frame')
        self.G_ref = gradient(self.reference)
        self.polygons = {k[len('target_'):]: np.array(v['polygon'], float) for k, v in cells.items() if k.startswith('target_')}
        self.T_base_from_color = np.linalg.inv(np.array(json.loads(CAMERA.read_text())['T_color_from_base_link']))
        self.plane_z = config.get('plane_z_m', .012)

    def to_table(self, uv):
        uv = np.atleast_2d(np.asarray(uv, float))
        d = (self.T_base_from_color[:3, :3] @ np.linalg.inv(K640) @ np.c_[uv, np.ones(len(uv))].T).T
        c = self.T_base_from_color[:3, 3]; t = (self.plane_z - c[2])/d[:, 2]
        return (c + d*t[:, None])[:, :2]

    @staticmethod
    def locate(G, G_ref, box, search):
        """Sub-pixel shift (px) and NCC score of the reference patch `box` (x0, y0, x1, y1) within +-search px of it."""
        x0, y0, x1, y1 = map(int, box); wx, wy = max(0, x0 - search), max(0, y0 - search)
        r = cv2.matchTemplate(G[wy:y1 + search, wx:x1 + search], G_ref[y0:y1, x0:x1], cv2.TM_CCOEFF_NORMED)
        _, score, _, (lx, ly) = cv2.minMaxLoc(r); dx = dy = 0.
        if 0 < lx < r.shape[1] - 1: a, b, c = r[ly, lx - 1], r[ly, lx], r[ly, lx + 1]; dx = .5*(a - c)/(a - 2*b + c)
        if 0 < ly < r.shape[0] - 1: a, b, c = r[ly - 1, lx], r[ly, lx], r[ly + 1, lx]; dy = .5*(a - c)/(a - 2*b + c)
        return np.array([wx + lx + dx - x0, wy + ly + dy - y0]), float(score)

    def camera_check(self, G):
        """The fixed ChArUco board must appear where it did in the reference frame: the pixel-to-mm mapping (the
        ChArUco camera calibration) only holds for that camera pose, and a moved camera would read as a moved pallet."""
        shift, score = self.locate(G, self.G_ref, self.c.get('camera_check_roi', [5, 340, 175, 475]), 20)
        ok = score >= self.c.get('camera_check_min_score', .5) and np.abs(shift).max() <= self.c.get('camera_check_max_px', 1.5)
        return dict(ok=bool(ok), shift_px=shift.round(2).tolist(), score=round(score, 3))

    def measure(self, bgr):
        """{'ok', 'cells': {part: {'offset_mm', 'score'}}, 'rigid': {...}, 'camera': {...}} or ok False + reason."""
        G = gradient(bgr); S = self.c.get('search_px', 35); cells = {}; A = []; B = []
        camera = self.camera_check(G)
        if not camera['ok']:
            return dict(ok=False, camera=camera, reason=f"camera moved or board hidden (ChArUco shift {camera['shift_px']} px, score {camera['score']}): check the D435 mount, redo the ChArUco calibration")
        for part, P in self.polygons.items():
            x0, y0 = (P.min(0) - 6).astype(int); x1, y1 = (P.max(0) + 6).astype(int)
            shift, score = self.locate(G, self.G_ref, (x0, y0, x1, y1), S)
            if score < self.c.get('min_score', .3): return dict(ok=False, camera=camera, reason=f'{part}: cell match score {score:.2f}')
            centre = P.mean(0)
            t0, t1 = self.to_table(centre)[0], self.to_table(centre + shift)[0]
            cells[part] = dict(offset_mm=((t1 - t0)*1000).round(2).tolist(), shift_px=shift.round(2).tolist(), score=round(float(score), 3)); A.append(t0); B.append(t1)
        A, B = np.array(A), np.array(B); ca, cb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb)); R = (U @ Vt).T
        if np.linalg.det(R) < 0: Vt[-1] *= -1; R = (U @ Vt).T
        res = np.linalg.norm(B - ((A - ca) @ R.T + cb), axis=1)*1000
        rigid = dict(rotation_deg=round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 3), translation_mm=((cb - ca)*1000).round(2).tolist(),
                     pivot_xy_m=ca.round(5).tolist(), residual_mm=dict(zip(cells, res.round(2).tolist())))
        out = dict(ok=True, cells=cells, rigid=rigid, camera=camera)
        if res.max() > self.c.get('max_cell_residual_mm', 2.5):
            out.update(ok=False, reason=f'cells disagree with a rigid pallet move (residual {res.max():.1f} mm)')
        elif max(abs(v) for c in cells.values() for v in c['offset_mm']) > self.c.get('max_offset_mm', 15) or abs(rigid['rotation_deg']) > self.c.get('max_rot_deg', 4):
            out.update(ok=False, reason=f"pallet offset outside the supported range ({rigid['translation_mm']} mm, {rigid['rotation_deg']} deg)")
        return out

    @staticmethod
    def median(measurements):
        """Per-cell median offsets of several accepted measurements (same structure as measure())."""
        m0 = measurements[0]
        cells = {p: dict(m0['cells'][p], offset_mm=np.median([m['cells'][p]['offset_mm'] for m in measurements], axis=0).round(2).tolist(),
                         shift_px=np.median([m['cells'][p]['shift_px'] for m in measurements], axis=0).round(2).tolist()) for p in m0['cells']}
        rigid = dict(m0['rigid'], rotation_deg=round(float(np.median([m['rigid']['rotation_deg'] for m in measurements])), 3),
                     translation_mm=np.median([m['rigid']['translation_mm'] for m in measurements], axis=0).round(2).tolist())
        return dict(ok=True, cells=cells, rigid=rigid, camera=m0.get('camera'), frames=len(measurements))

    def at_reference(self, m):
        """True when the pallet is where the plans were verified (no retarget needed)."""
        tol = self.c.get('reference_tolerance_mm', .5)
        return max(abs(v) for c in m['cells'].values() for v in c['offset_mm']) <= tol and abs(m['rigid']['rotation_deg']) <= self.c.get('reference_tolerance_deg', .2)

    def match_anchor(self, m):
        """Name of the verified burger (anchors) whose measured cells this pose matches, else None."""
        for name, a in json.loads(Path(self.c['anchors']).read_text())['burgers'].items():
            if not a.get('cell_offset_mm'): continue
            diff = max(abs(x - y) for p, c in m['cells'].items() for x, y in zip(c['offset_mm'], a['cell_offset_mm'][p]))
            if diff <= self.c.get('anchor_tolerance_mm', 1.) and abs(m['rigid']['rotation_deg'] - a['rotation_deg']) <= self.c.get('anchor_tolerance_deg', .4): return name
        return None

    def extra_mm(self, m):
        """Real-robot extra shifts per part, scaled by how far the pallet moved along the m1 direction (0 at m2, 1 at m1).
        On the m2 side a part with its own non-zero m2 extra uses that value unscaled: the pallets need unrelated corrections."""
        burgers = json.loads(Path(self.c['anchors']).read_text())['burgers']; anchors = burgers['m1']
        t1 = np.median(np.array(list(anchors['cell_offset_mm'].values())), axis=0)
        t = np.median(np.array([c['offset_mm'] for c in m['cells'].values()]), axis=0)
        alpha = float(np.clip(t @ t1/(t1 @ t1), 0, 1.2))
        extra = {p: (np.array(v)*alpha).round(2).tolist() for p, v in anchors['extra_mm'].items()}
        if alpha < M2_SIDE_ALPHA:
            extra.update({p: list(v) for p, v in burgers['m2'].get('extra_mm', {}).items() if any(v)})
        return alpha, extra
