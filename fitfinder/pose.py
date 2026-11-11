"""Pose: rotation R and translation t that put the part on the target faces, and the DOF left free.

Rotation (Kabsch, det = +1 so mirror images never match):
    plane: query normal -> reversed target normal;  cylinder: axis -> axis (both signs tried);
    cone: axis -> axis (apex -> wide end)
Translation (least squares rows):
    plane  n.t = n.(o_t - R o_q)  1 row     cylinder  e.t = e.(p_t - R p_q)  2 rows
    sphere / cone apex  t = x_t - R x_q    3 rows
Free DOF = null space of the rows + rotations every face is invariant to:
    (#T, #R) fixed (0,0) prismatic (1,0) revolute (0,1) cylindrical (1,1) planar (2,1) spherical (0,3)
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np

from . import config as C


@dataclass
class Pose:
    R: np.ndarray
    t: np.ndarray
    free_t: list = field(default_factory=list)    # free translation directions (target-local)
    free_r: list = field(default_factory=list)    # free rotation axes: (point, direction)
    joint: str = "fixed"
    residual: float = 0.0
    gaps: list = field(default_factory=list)      # signed gap per plane pair at the pose

    def matrix(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = self.R, self.t
        return m


JOINTS = {(0, 0): "fixed", (1, 0): "prismatic", (0, 1): "revolute", (1, 1): "cylindrical",
          (2, 1): "planar", (0, 3): "spherical"}


def _perp_basis(a):
    h = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(a, h)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(a, e1)


def kabsch(us, vs):
    """Best proper rotation R with R u ~ v. Returns (R, max_angle_error, rank, mirror_only)."""
    if not us:
        return np.eye(3), 0.0, 0, False
    H = sum(np.outer(u, v) for u, v in zip(us, vs))
    U, S, Vt = np.linalg.svd(H)
    rank = int(np.sum(S > 1e-6 * max(S.max(), 1e-12)))
    d = 1.0 if np.linalg.det(Vt.T @ U.T) > 0 else -1.0
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    err = max(math.acos(max(-1, min(1, float(np.dot(R @ u, v))))) for u, v in zip(us, vs))
    mirror = False
    if d < 0 and rank == 3:
        R0 = Vt.T @ U.T
        err0 = max(math.acos(max(-1, min(1, float(np.dot(R0 @ u, v))))) for u, v in zip(us, vs))
        mirror = err0 <= C.ANGLE_TOL < err
    return R, err, rank, mirror


def rows(query: list, target: list, R: np.ndarray):
    """The equations A t = b for the translation t, once the rotation R is known.
    A plane pair fixes t along the plane's normal (1 row), a cylinder pair across its axis
    (2 rows), a sphere or cone pair in all directions (3 rows)."""
    A, b, kinds = [], [], []
    for q, t in zip(query, target):
        if q.kind == "plane":
            A.append(t.n)
            b.append(np.dot(t.n, t.o - R @ q.o))
            kinds.append(("plane", t.n))
        elif q.kind == "cylinder":
            for e in _perp_basis(t.a):
                A.append(e)
                b.append(np.dot(e, t.p - R @ q.p))
                kinds.append(("axis", e))
        elif q.kind == "sphere":
            for k in range(3):
                e = np.eye(3)[k]
                A.append(e)
                b.append(np.dot(e, t.c - R @ q.c))
                kinds.append(("point", e))
        elif q.kind == "cone":
            for k in range(3):
                e = np.eye(3)[k]
                A.append(e)
                b.append(np.dot(e, t.p - R @ q.p))
                kinds.append(("point", e))
    return np.array(A).reshape(-1, 3), np.array(b), kinds


def free_translations(A: np.ndarray) -> list:
    if len(A) == 0:
        return [np.eye(3)[k] for k in range(3)]
    _, S, Vt = np.linalg.svd(A)
    smax = max(S.max(), 1e-12)
    rank = int(np.sum(S > 1e-9 * smax))
    return [Vt[k] for k in range(rank, 3)]


def free_rotations(target: list) -> list:
    """Rotation axes (point, dir) every matched target face is invariant to."""
    axial = [t for t in target if t.kind in ("cylinder", "cone")]
    spheres = [t for t in target if t.kind == "sphere"]
    planes = [t for t in target if t.kind == "plane"]
    if not axial and spheres and not planes:
        c0 = spheres[0].c
        if all(np.linalg.norm(s.c - c0) < C.GEOM_TOL for s in spheres):
            return [(c0, np.eye(3)[k]) for k in range(3)]
        return []
    if axial:
        p0, a0 = axial[0].p, axial[0].a
    elif planes:
        p0, a0 = planes[0].o, planes[0].n
    else:
        return []
    sin_tol = math.sin(C.ANGLE_TOL)
    for t in axial:
        w = t.p - p0
        if np.linalg.norm(np.cross(t.a, a0)) > sin_tol or np.linalg.norm(w - np.dot(w, a0) * a0) > C.GEOM_TOL:
            return []
    for t in planes:
        if np.linalg.norm(np.cross(t.n, a0)) > sin_tol:
            return []
    for t in spheres:
        w = t.c - p0
        if np.linalg.norm(w - np.dot(w, a0) * a0) > C.GEOM_TOL:
            return []
    return [(p0, a0)]


def settle(A, b, kinds, t):
    """Least squares can sink one-sided contacts a little; push the part out of planes it penetrates."""
    for _ in range(10):
        worst, wk = -1e-6, None
        for k, (kind, n) in enumerate(kinds):
            if kind == "plane":
                g = float(A[k] @ t - b[k])
                if g < worst:
                    worst, wk = g, k
        if wk is None:
            break
        t = t + (-worst) * kinds[wk][1]
    return t


def solve(query: list, target: list):
    """Yield (Pose or None, reason) for each axis-sign combination."""
    cyl_idx = [i for i, q in enumerate(query) if q.kind == "cylinder"]
    out = []
    for signs in itertools.product((1, -1), repeat=min(len(cyl_idx), 3)):
        us, vs = [], []
        sgn = dict(zip(cyl_idx, signs))
        for i, (q, t) in enumerate(zip(query, target)):
            if q.kind == "plane":
                us.append(q.n)
                vs.append(-t.n)
            elif q.kind == "cylinder":
                us.append(q.a)
                vs.append(sgn.get(i, 1) * t.a)
            elif q.kind == "cone":
                us.append(q.a)
                vs.append(t.a)
        R, err, rank, mirror = kabsch(us, vs)
        if mirror:
            out.append((None, "mirror image only"))
            continue
        if err > C.ANGLE_TOL:
            out.append((None, "angles"))
            continue
        A, b, kinds = rows(query, target, R)
        t, *_ = np.linalg.lstsq(A, b, rcond=None) if len(A) else (np.zeros(3),)
        res = A @ t - b if len(A) else np.zeros(0)
        # rows from opposed planes with clearance disagree by half the clearance: allowed
        if len(res) and np.max(np.abs(res)) > C.CLEARANCE_MAX / 2 + C.GEOM_TOL:
            out.append((None, "position residual"))
            continue
        t = settle(A, b, kinds, t)
        ft = free_translations(A)
        fr = free_rotations(target)
        joint = JOINTS.get((len(ft), len(fr)), f"other({len(ft)}T,{len(fr)}R)")
        gaps = [float(A[k] @ t - b[k]) for k, (kind, _) in enumerate(kinds) if kind == "plane"]
        out.append((Pose(R, t, ft, fr, joint, float(np.max(np.abs(res))) if len(res) else 0.0, gaps), ""))
    return out


def selection_dof(query: list):
    """Free DOF of the selection itself (used to detect under-constrained selections)."""
    A, b, _ = rows(query, query, np.eye(3))
    return len(free_translations(A)), len(free_rotations(query))
