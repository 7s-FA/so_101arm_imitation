"""Portable step86 inference/IO; NumPy only, no simulator or hardware imports."""
from dataclasses import dataclass
from pathlib import Path
import json
import numpy as np

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
PACKAGE = Path(__file__).resolve().parents[1]

def finite_vector(value, count, name):
    a = np.asarray(value, dtype=np.float32)
    if a.shape != (count,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must be finite shape ({count},), got {a.shape}')
    return a

def observation(image_estimated_jig_xy_mm, measured_joint_q_rad):
    xy = finite_vector(image_estimated_jig_xy_mm, 2, 'image_estimated_jig_xy_mm')
    q = finite_vector(measured_joint_q_rad, 6, 'measured_joint_q_rad')
    return np.concatenate((xy / np.float32(10), q / np.float32(np.pi))).astype(np.float32)

def residual_world_xy_m(action):
    return np.clip(finite_vector(action, 2, 'action'), -1, 1) * np.float32(.003)

def corrected_goal(nominal_goal_world_m, action):
    goal = finite_vector(nominal_goal_world_m, 3, 'nominal_goal_world_m').astype(np.float64)
    goal[:2] += residual_world_xy_m(action)
    return goal

def reward(all_three_success, third_error_mm):
    error = float(third_error_mm)
    if not np.isfinite(error) or error < 0: raise ValueError('invalid position error')
    return (5. if all_three_success else -20.) - min(error, 30.)

def raw_to_q(raw, config):
    x = np.asarray(raw, dtype=np.float64)
    if x.shape[-1:] != (6,) or not np.isfinite(x).all(): raise ValueError('invalid raw array')
    j = config['joints']
    return (x - [j[n]['raw_origin'] for n in JOINTS]) * [j[n]['radians_per_tick'] for n in JOINTS] + [j[n]['offset_rad'] for n in JOINTS]

def q_to_raw(q, config):
    x = np.asarray(q, dtype=np.float64)
    if x.shape[-1:] != (6,) or not np.isfinite(x).all(): raise ValueError('invalid q array')
    j = config['joints']
    return (x - [j[n]['offset_rad'] for n in JOINTS]) / [j[n]['radians_per_tick'] for n in JOINTS] + [j[n]['raw_origin'] for n in JOINTS]

class ResidualPolicy:
    """Deterministic mean of SB3 8->32(tanh)->32(tanh)->2, Box clipped.

    No VecNormalize, no output tanh, no stochastic sampling. Not a PPO trainer.
    """
    def __init__(self, weights=PACKAGE/'policies/actor_numpy.npz'):
        with np.load(weights, allow_pickle=False) as z:
            self.layers = [(z[f'w{i}'].copy(), z[f'b{i}'].copy()) for i in range(3)]
        for (w, b), shape in zip(self.layers, [(32,8),(32,32),(2,32)]):
            if w.shape != shape or b.shape != (shape[0],): raise ValueError('wrong actor architecture')
            if not np.isfinite(w).all() or not np.isfinite(b).all(): raise ValueError('nonfinite weights')

    def predict(self, obs):
        x = np.asarray(obs, dtype=np.float32)
        if x.ndim not in (1,2) or x.shape[-1] != 8 or not np.isfinite(x).all():
            raise ValueError('observation must be finite (8,) or (N,8)')
        for i, (w, b) in enumerate(self.layers):
            x = x @ w.T + b
            if i < 2: x = np.tanh(x)
        return np.clip(x, -1, 1)

@dataclass(frozen=True)
class ActorContext:
    image_estimated_jig_xy_mm: object
    measured_joint_q_rad: object
    image_source: str
    stage: str = 'third_stage_boundary'

    def as_observation(self):
        if self.image_source not in ('initial_synthetic_camera', 'initial_real_camera'):
            raise ValueError('Actor requires camera estimate, not simulator object truth')
        if self.stage != 'third_stage_boundary': raise ValueError('wrong policy call time')
        return observation(self.image_estimated_jig_xy_mm, self.measured_joint_q_rad)

class OneShotResidual:
    def __init__(self, policy=None):
        self.policy = policy or ResidualPolicy()
        self.used = False

    def reset(self): self.used = False

    def decide(self, context, nominal_goal_world_m):
        if self.used: raise RuntimeError('Only one residual decision per episode')
        obs = context.as_observation()
        action = self.policy.predict(obs)
        goal = corrected_goal(nominal_goal_world_m, action)
        self.used = True
        return dict(observation=obs, action=action, residual_xy_m=residual_world_xy_m(action), goal_world_m=goal)
