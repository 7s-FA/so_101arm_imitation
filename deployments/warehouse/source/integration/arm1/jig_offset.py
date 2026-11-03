"""Target-pallet offset from one D435 overview frame (read-only; no camera or robot access).

The target pallet (all six destination jigs) moves as one rigid piece and carries no marker. Its long straight
edges are found again near where they were in the reference frame (the pallet position at which the plans were
verified): along each reference line, the strongest dark-to-bright step of a short normal profile gives one
sub-pixel edge point, and a robust line is fitted through them. The lines are mapped onto the table plane
(ChArUco camera transform, D435 640x480 intrinsics) and their intersection and directions give the pallet's
rigid displacement: dx, dy (robot base mm) and rotation (deg). Frames whose edges cannot be found reliably
(occlusion, parts in the way, a different scene) are refused rather than guessed.

Config: jig-offset.json (make with make_jig_reference.py from a reference frame).
"""
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


class JigOffsetMeter:
    def __init__(self, config=ROOT/'jig-offset.json'):
        self.config = json.loads(Path(config).read_text())
        c = self.config
        self.K = np.array(c['K']); self.T_color_from_base = np.array(c['T_color_from_base_link'])
        self.T_base_from_color = np.linalg.inv(self.T_color_from_base); self.plane_z = c['plane_z_m']
        self.lines = {k: np.array(v, float) for k, v in c['lines'].items()}     # name -> [[x0,y0],[x1,y1]] image px
        self.limits = c['limits']

    def to_table(self, uv):
        uv = np.atleast_2d(np.asarray(uv, float))
        d = (self.T_base_from_color[:3, :3] @ np.linalg.inv(self.K) @ np.c_[uv, np.ones(len(uv))].T).T
        c = self.T_base_from_color[:3, 3]; t = (self.plane_z - c[2]) / d[:, 2]
        return (c + d*t[:, None])[:, :2]

    def edge_points(self, gray, line):
        """Sub-pixel edge points along a reference line: the strongest step of the normal profile within the band."""
        p0, p1 = line; L = np.linalg.norm(p1 - p0); u = (p1 - p0)/L; n = np.array([-u[1], u[0]])
        band, step = self.limits['band_px'], 1.
        offs = np.arange(-band, band + step, step); pts = []
        for s in np.linspace(.06, .94, int(L/3)):
            c = p0 + s*(p1 - p0); xy = c + offs[:, None]*n
            prof = cv2.remap(gray, xy[:, 0].astype(np.float32)[None], xy[:, 1].astype(np.float32)[None], cv2.INTER_LINEAR)[0]
            g = np.abs(np.gradient(prof)); k = int(np.argmax(g))
            if g[k] < self.limits['min_step'] or k in (0, len(g) - 1): continue
            a, b, cc = g[k - 1], g[k], g[k + 1]; sub = .5*(a - cc)/(a - 2*b + cc) if (a - 2*b + cc) != 0 else 0.
            pts.append(c + (offs[k] + sub*step)*n)
        return np.array(pts)

    @staticmethod
    def fit(pts):
        """Line through pts (centroid, unit direction) with one round of 2-sigma outlier rejection; RMS residual px."""
        def once(p):
            c = p.mean(0); u = np.linalg.svd(p - c)[2][0]; r = (p - c) @ np.array([-u[1], u[0]]); return c, u, r
        c, u, r = once(pts); keep = np.abs(r) <= max(2*r.std(), .5)
        c, u, r = once(pts[keep]); return c, u, float(np.sqrt((r**2).mean())), int(keep.sum())

    def measure(self, bgr):
        gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32), (3, 3), 0)
        fits = {}
        for name, line in self.lines.items():
            pts = self.edge_points(gray, line)
            need = self.limits['min_points_fraction']*np.linalg.norm(line[1] - line[0])/3
            if len(pts) < need: return dict(ok=False, reason=f'{name}: {len(pts)} edge points (< {need:.0f})')
            c, u, rms, n = self.fit(pts)
            if rms > self.limits['max_rms_px']: return dict(ok=False, reason=f'{name}: line residual {rms:.2f}px')
            fits[name] = (c, u, rms, n)
        # Table-plane rigid motion: map each fitted line and its reference onto the table, compare directions
        # (rotation) and, after removing the rotation, the line positions (translation, least squares).
        A, b, angles, w = [], [], [], []
        pivot = self.to_table(np.mean([v.mean(0) for v in self.lines.values()], axis=0))[0]   # about the pallet middle
        for name, (c, u, rms, n) in fits.items():
            ref = self.lines[name]; rp = self.to_table(ref); nu = rp[1] - rp[0]; nu /= np.linalg.norm(nu)
            q = self.to_table([c - 40*u, c + 40*u]); mu = q[1] - q[0]; mu /= np.linalg.norm(mu)
            if mu @ nu < 0: mu = -mu
            angles.append(np.arctan2(nu[0]*mu[1] - nu[1]*mu[0], nu @ mu)); w.append(n)
        rot = float(np.average(angles, weights=w)); R = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
        for name, (c, u, rms, n) in fits.items():
            rp = self.to_table(self.lines[name]); nu = rp[1] - rp[0]; nu /= np.linalg.norm(nu); normal = R @ np.array([-nu[1], nu[0]])
            moved_ref = pivot + (rp.mean(0) - pivot) @ R.T          # reference line point after the rotation about the pivot
            obs = self.to_table(c)[0]
            A.append(normal); b.append(normal @ (obs - moved_ref))
        t, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
        out = dict(ok=True, dx_mm=float(t[0]*1000), dy_mm=float(t[1]*1000), rot_deg=float(np.degrees(rot)),
                   lines={k: dict(rms_px=round(v[2], 3), points=v[3]) for k, v in fits.items()})
        if max(abs(out['dx_mm']), abs(out['dy_mm'])) > self.limits['max_offset_mm'] or abs(out['rot_deg']) > self.limits['max_rot_deg']:
            out.update(ok=False, reason=f"offset outside the supported range ({out['dx_mm']:.1f}, {out['dy_mm']:.1f}) mm, {out['rot_deg']:.1f} deg")
        return out
