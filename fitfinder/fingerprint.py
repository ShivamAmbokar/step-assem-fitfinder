"""Relations between faces that do not change when the part moves (angles, gaps, distances).
A target group matches if it mirrors every relation; clearance of an opposed pair c = s + s'.
"""
from __future__ import annotations

import math

import numpy as np

from . import config as C
from .faces import FaceDesc


def single_complement(q: FaceDesc, t: FaceDesc):
    """Can target face t receive query face q?  -> (ok, clearance or None, reason)."""
    if q.kind != t.kind or q.kind == "other":
        return False, None, "type"
    if q.kind == "plane":
        return True, None, ""
    if q.convex == t.convex:
        return False, None, "same convexity"
    if q.kind in ("cylinder", "sphere"):
        c = 2 * (t.r - q.r) if q.convex else 2 * (q.r - t.r)     # diametral clearance
        return True, c, ""
    if q.kind == "cone":
        if abs(q.half - t.half) > C.ANGLE_TOL:
            return False, None, "cone angle"
        return True, None, ""
    return False, None, "type"


def _dir(f: FaceDesc):
    return f.n if f.kind == "plane" else f.a       # sphere: None


def _point(f: FaceDesc):
    return {"plane": f.o, "cylinder": f.p, "cone": f.p, "sphere": f.c}.get(f.kind)


def _line_dist(p1, a1, p2, a2) -> float:
    """Distance between two lines (parallel or skew)."""
    w = p2 - p1
    cr = np.cross(a1, a2)
    if np.linalg.norm(cr) < 1e-9:
        return float(np.linalg.norm(w - np.dot(w, a1) * a1))
    return float(abs(np.dot(w, cr)) / np.linalg.norm(cr))


def relation(fi: FaceDesc, fj: FaceDesc) -> dict:
    """Invariant relation of a face pair (both in the same frame)."""
    ki, kj = fi.kind, fj.kind
    rel = {"kinds": (ki, kj)}
    di, dj = _dir(fi), _dir(fj)
    sin_tol = math.sin(C.ANGLE_TOL)
    if di is not None and dj is not None:
        cos = float(np.dot(di, dj))
        signed = (ki in ("plane", "cone")) and (kj in ("plane", "cone"))
        rel["cos"] = cos if signed else abs(cos)
        rel["cos_flip"] = signed and (ki == "plane") != (kj == "plane")   # plane-cone: sign flips
        if ki == "plane" and kj == "plane" and abs(cos) > 1 - 1e-6:
            rel["s"] = float(np.dot(fj.o - fi.o, fi.n))                 # signed gap
            rel["opposed"] = cos < 0
        elif ki == "plane" and kj in ("cylinder", "cone") and abs(np.dot(fi.n, fj.a)) < sin_tol:
            rel["h"] = float(np.dot(fj.p - fi.o, fi.n))                 # axis-to-plane, signed
        elif kj == "plane" and ki in ("cylinder", "cone") and abs(np.dot(fj.n, fi.a)) < sin_tol:
            rel["h"] = float(np.dot(fi.p - fj.o, fj.n))
        elif ki in ("cylinder", "cone") and kj in ("cylinder", "cone"):
            rel["d"] = _line_dist(fi.p, fi.a, fj.p, fj.a)
    else:
        pi, pj = _point(fi), _point(fj)
        if ki == "sphere" and kj == "sphere":
            rel["d"] = float(np.linalg.norm(pj - pi))
        elif "plane" in (ki, kj):
            pl, sp = (fi, fj) if ki == "plane" else (fj, fi)
            rel["h"] = float(np.dot(_point(sp) - pl.o, pl.n))
        else:
            ax, sp = (fi, fj) if ki != "sphere" else (fj, fi)
            w = sp.c - ax.p                                              # sphere centre to axis
            rel["d"] = float(np.linalg.norm(w - np.dot(w, ax.a) * ax.a))
    return rel


def compare(rq: dict, rt: dict, strict: bool):
    """Does target relation rt match query relation rq?  -> (ok, clearance or None, reason)."""
    lo, hi = (-C.INTERFERENCE_MAX, C.CLEARANCE_MAX) if strict else \
             (-C.INTERFERENCE_MAX - C.RELAX, C.CLEARANCE_MAX + C.RELAX)
    eps = 1e-6
    if ("cos" in rq) != ("cos" in rt):
        return False, None, "geometry"
    if "cos" in rq:
        want = -rq["cos"] if rq.get("cos_flip") else rq["cos"]
        a1 = math.acos(max(-1, min(1, want)))
        a2 = math.acos(max(-1, min(1, rt["cos"])))
        if abs(a1 - a2) > C.ANGLE_TOL:
            return False, None, "angle"
    if "s" in rq:
        if "s" not in rt:
            return False, None, "geometry"
        s, s2 = rq["s"], rt["s"]
        c = s + s2
        if rq["opposed"]:
            if abs(s) > C.GEOM_TOL and np.sign(s2) != -np.sign(s):
                return False, None, "faces point the wrong way"
            if not (lo - eps <= c <= hi + eps):
                return False, c, "too loose" if c > hi else "too small"
            return True, c, ""
        if abs(c) > hi + eps:                       # same-direction planes: step heights must match
            return False, c, "step mismatch"
        return True, None, ""
    if "h" in rq:
        if "h" not in rt:
            return False, None, "geometry"
        h, h2 = rq["h"], rt["h"]
        if abs(h) > C.GEOM_TOL and np.sign(h2) != -np.sign(h):
            return False, None, "faces point the wrong way"
        c = h + h2
        if not (lo - eps <= c <= hi + eps):
            return False, c, "too loose" if c > hi else "too small"
        return True, c, ""
    if "d" in rq:
        if "d" not in rt:
            return False, None, "geometry"
        if abs(rq["d"] - rt["d"]) > (hi if strict else hi):
            return False, None, "distance"
        return True, None, ""
    return True, None, ""


def fingerprint(faces: list) -> dict:
    """All pair relations of the selection: {(i, j): relation} with i < j (indices into faces)."""
    return {(i, j): relation(faces[i], faces[j])
            for i in range(len(faces)) for j in range(i + 1, len(faces))}
