"""Backtracking search: give every selected face its own target face so that all pairwise
relations hold, including proximity (faces close together on the part need partners close together).
"""
from __future__ import annotations

import time

import math

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location

from . import config as C
from .fingerprint import _dir, compare, fingerprint, relation, single_complement

_BOX: dict = {}


class Assignments(list):
    """search() result; .stopped = True if the time or count limit cut the search short."""
    stopped = False


def face_box(d) -> np.ndarray:
    """Axis-aligned box of a face in its own part frame: [xmin, ymin, zmin, xmax, ymax, zmax]."""
    key = id(d)
    if key not in _BOX:
        b = Bnd_Box()
        BRepBndLib.Add_s(d.face, b, False)
        _BOX[key] = (d, np.array(b.Get()))
    return _BOX[key][1]


def face_points(d) -> np.ndarray:
    """Corner points of a face's coarse mesh (part frame): enough to know its extent in any direction."""
    key = ("pts", id(d))
    if key not in _BOX:
        bb = face_box(d)
        BRepMesh_IncrementalMesh(d.face, float(np.clip(0.05 * np.linalg.norm(bb[3:] - bb[:3]), 0.05, 2.0)), False, 0.5)
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(d.face, loc)
        if tri is None:
            pts = np.array([bb[:3], bb[3:]])
        else:
            t = loc.Transformation()
            pts = np.array([[(p := tri.Node(i).Transformed(t)).X(), p.Y(), p.Z()] for i in range(1, tri.NbNodes() + 1)])
        _BOX[key] = (d, pts)
    return _BOX[key][1]


def _unit_cross(a, b):
    c = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    n = math.sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2])
    return n, (np.array(c) / n if n > 0 else None)


def band_ok(qa, qb, ta, tb, slack: float) -> bool:
    """Two flats at an angle share one line direction u = n_a x n_b. Moving the part shifts both
    selected faces along u by the same amount, so their partner faces must overlap them for ONE
    common shift: the two allowed shift intervals must intersect."""
    nq, u_q = _unit_cross(qa.n, qb.n)
    if nq < 0.05:
        return True
    nt, u_t = _unit_cross(ta.n, tb.n)
    if u_t is None:
        return True

    def shifts(q, t):
        pq, pt = face_points(q) @ u_q, face_points(t) @ u_t
        return pt.min() - pq.max() - slack, pt.max() - pq.min() + slack
    lo1, hi1 = shifts(qa, ta)
    lo2, hi2 = shifts(qb, tb)
    return max(lo1, lo2) <= min(hi1, hi2)


def box_max_dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.maximum(np.abs(a[3:] - b[:3]), np.abs(b[3:] - a[:3]))))


def proximity_limits(query: list, strict: bool) -> dict:
    """Two selected faces at most D apart touch partners that are at most D + 2 x clearance apart,
    wherever the part is: a rigid-motion invariant that rules out far-apart target faces."""
    slack = 2 * (C.CLEARANCE_MAX + (0.0 if strict else C.RELAX)) + 0.1
    boxes = [face_box(q) for q in query]
    return {(i, j): box_max_dist(boxes[i], boxes[j]) + slack
            for i in range(len(query)) for j in range(i + 1, len(query))}


def candidates(query: list, target: list, strict: bool = False) -> list:
    """For each query face: list of (target index, single clearance)."""
    lo, hi = (-C.INTERFERENCE_MAX, C.CLEARANCE_MAX) if strict else \
             (-C.INTERFERENCE_MAX - C.RELAX, C.CLEARANCE_MAX + C.RELAX)
    out = []
    for q in query:
        cands = []
        for k, t in enumerate(target):
            if t.area < C.MIN_AREA_RATIO * q.area:
                continue
            ok, c, _ = single_complement(q, t)
            if ok and (c is None or lo <= c <= hi):
                cands.append((k, c))
        out.append(cands)
    return out


def search(query: list, target: list, timeout: float = 20.0, strict: bool = False, limit: int = 0) -> list:
    """All assignments: list of tuples (target index per query face). strict = real size windows
    (else the wider window used to explain near-misses); limit > 0 stops after that many.

    For speed, the necessary conditions between every two candidate lists (angle, gap between
    parallel flats, distance) are computed up front as boolean tables with numpy; while building
    a group, the candidates for the next face are one AND of table rows, and the exact checks
    then run only on the survivors."""
    n = len(query)
    fp = fingerprint(query)
    cands = candidates(query, target, strict)
    if any(not c for c in cands):
        return Assignments()
    near = proximity_limits(query, strict)
    lo, hi = (-C.INTERFERENCE_MAX, C.CLEARANCE_MAX) if strict else \
             (-C.INTERFERENCE_MAX - C.RELAX, C.CLEARANCE_MAX + C.RELAX)
    nan3 = [np.nan] * 3
    D = np.array([_dir(t) if _dir(t) is not None else nan3 for t in target], dtype=float)
    O = np.array([t.o if t.kind == "plane" else nan3 for t in target], dtype=float)
    B = np.array([face_box(t) for t in target])
    K = [np.array([k for k, _ in c]) for c in cands]
    order = sorted(range(n), key=lambda i: len(cands[i]))
    rel_cache: dict = {}
    slack = C.CLEARANCE_MAX + (0.0 if strict else C.RELAX) + 0.1

    def trel(a, b):
        key = (a, b)
        if key not in rel_cache:
            rel_cache[key] = relation(target[a], target[b])
        return rel_cache[key]

    def pair_table(i, j):
        """Necessary conditions for every (candidate of query i, candidate of query j), i < j, as one
        boolean table: right angle, right gap between parallel flats, close enough together."""
        rq, A, Bk = fp[(i, j)], K[i], K[j]
        out = np.empty((len(A), len(Bk)), bool)
        signed = all(query[x].kind in ("plane", "cone") for x in (i, j))
        for s0 in range(0, len(A), 256):
            a = A[s0:s0 + 256]
            gap = np.maximum(0.0, np.maximum(B[None, Bk, :3] - B[a, None, 3:], B[a, None, :3] - B[None, Bk, 3:]))
            keep = np.sqrt((gap ** 2).sum(-1)) <= near[(i, j)]
            if "cos" in rq:
                want = -rq["cos"] if rq.get("cos_flip") else rq["cos"]
                cos = D[a] @ D[Bk].T
                cos = cos if signed else np.abs(cos)
                with np.errstate(invalid="ignore"):
                    ang = np.abs(np.arccos(np.clip(cos, -1, 1)) - np.arccos(np.clip(want, -1, 1)))
                    keep &= ang <= C.ANGLE_TOL + 1e-9
                    if "s" in rq:                               # parallel flats: the gap must fit
                        st = np.einsum("bk,ak->ab", O[Bk], D[a]) - np.einsum("ak,ak->a", O[a], D[a])[:, None]
                        c = rq["s"] + st
                        keep &= (c >= lo - 1e-6) & (c <= hi + 1e-6) if rq["opposed"] else (np.abs(c) <= hi + 1e-6)
            out[s0:s0 + 256] = keep
        return out

    tables = {pair: pair_table(*pair) for pair in fp}
    results, assign, used = Assignments(), [None] * n, set()
    row = [None] * n                     # position of each assigned face in its candidate list
    t0 = time.time()

    def rec(depth):
        """Give the next selected face a partner, then go one level deeper (backtracking)."""
        if time.time() - t0 > timeout or (limit and len(results) >= limit):
            results.stopped = True
            return
        if depth == n:
            results.append(tuple(assign))
            return
        qi = order[depth]
        keep = np.ones(len(K[qi]), bool)
        for prev in order[:depth]:
            keep &= tables[(prev, qi)][row[prev]] if prev < qi else tables[(qi, prev)][:, row[prev]]
        for r in np.nonzero(keep)[0]:
            k = int(K[qi][r])
            if k in used:
                continue
            ok = True
            for prev in order[:depth]:
                i, j = (prev, qi) if prev < qi else (qi, prev)
                ti, tj = (assign[i], k) if prev < qi else (k, assign[j])
                if query[i].kind == "plane" and query[j].kind == "plane" and \
                        not band_ok(query[i], query[j], target[ti], target[tj], slack):
                    ok = False
                    break
                good, _, _ = compare(fp[(i, j)], trel(ti, tj), strict=strict)
                if not good:
                    ok = False
                    break
            if ok:
                assign[qi], row[qi] = k, r
                used.add(k)
                rec(depth + 1)
                used.discard(k)
                assign[qi] = None

    rec(0)
    return results


def strict_check(query: list, target: list, assignment: tuple):
    """Re-check an assignment with the STRICT windows.
    Returns (ok, clearances, reason, worst_value)."""
    fp = fingerprint(query)
    clear = []
    for i, q in enumerate(query):
        ok, c, why = single_complement(q, target[assignment[i]])
        if c is not None:
            if not (-C.INTERFERENCE_MAX - 1e-6 <= c <= C.CLEARANCE_MAX + 1e-6):
                return False, clear, ("too loose" if c > 0 else "too small"), c
            clear.append(c)
    for (i, j), rq in fp.items():
        rt = relation(target[assignment[i]], target[assignment[j]])
        ok, c, why = compare(rq, rt, strict=True)
        if not ok:
            return False, clear, why, c
        if c is not None:
            clear.append(c)
    return True, clear, "", None
