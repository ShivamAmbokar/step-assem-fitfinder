"""Find the source part inside the assembly, and in-place mode (what it touches right now).

find_sources: same face count and volume, plus a rigid registration (identity, then principal
axes, then face centroids) that maps every face. in_place: at the current position, each selected face looks for an
opposed/coaxial face on another part within INPLACE_GAP that really overlaps.
"""
from __future__ import annotations

import itertools

import numpy as np
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from . import config as C
from .fingerprint import compare, fingerprint, relation, single_complement
from .fit import behaviour, classify
from .pose import JOINTS, free_rotations, free_translations, rows
from .verify import pair_overlap


def _frame(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    c = props.CentreOfMass()
    pp = props.PrincipalProperties()
    axes = [pp.FirstAxisOfInertia(), pp.SecondAxisOfInertia(), pp.ThirdAxisOfInertia()]
    F = np.array([[a.X(), a.Y(), a.Z()] for a in axes]).T      # columns = principal axes
    return np.array([c.X(), c.Y(), c.Z()]), F


def _same_area(a: float, b: float) -> bool:
    # 1 %: a STEP round trip changes the computed area of tiny faces by a few tenths of a percent
    return abs(a - b) <= 1e-2 * max(a, b) + 1e-3


def _map_faces(pdescs, ddescs, m) -> dict | None:
    """Map every part face (moved by m) onto a definition face of the same kind and place."""
    mapping = {}
    for p in pdescs:
        c = m[:3, :3] @ p.center + m[:3, 3]
        best = None
        for d in ddescs:
            if d.kind == p.kind and _same_area(d.area, p.area) and np.linalg.norm(d.center - c) <= 1e-2:
                best = d.fid
                break
        if best is None:
            return None
        mapping[p.fid] = best
    return mapping


def register(part, pdescs, definition, ddescs):
    """Rigid transform part-frame -> definition-frame, or None."""
    if len(pdescs) != len(ddescs) or abs(part.volume - definition.volume) > 1e-4 * max(part.volume, 1):
        return None
    eye = np.eye(4)
    mp = _map_faces(pdescs, ddescs, eye)
    if mp:
        return eye, mp
    cp, Fp = _frame(part.shape)
    cd, Fd = _frame(definition.shape)
    for s in itertools.product((1, -1), repeat=3):
        S = np.diag(s)
        R = Fd @ S @ Fp.T
        if np.linalg.det(R) < 0:
            continue
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = R, cd - R @ cp
        mp = _map_faces(pdescs, ddescs, m)
        if mp:
            return m, mp
    return _by_centroids(pdescs, ddescs, cp, cd)


def _by_centroids(pdescs, ddescs, cp, cd):
    """Fallback for round parts (a pin), whose principal axes are not unique: pair three faces by
    kind and area, and fit a rotation to their centroids."""
    same = {p.fid: [d for d in ddescs if d.kind == p.kind and _same_area(d.area, p.area)] for p in pdescs}
    anchors = []
    for p in sorted(pdescs, key=lambda p: (len(same[p.fid]), -p.area)):
        pts = [a.center - cp for a in anchors] + [p.center - cp]
        if not same[p.fid] or (len(pts) == 3 and np.linalg.matrix_rank(np.array(pts), tol=1e-3) < 2):
            continue
        anchors.append(p)
        if len(anchors) == 3:
            break
    if len(anchors) < 3:
        return None
    P = np.array([a.center - cp for a in anchors])
    for picks in itertools.islice(itertools.product(*[same[a.fid] for a in anchors]), 64):
        Q = np.array([d.center - cd for d in picks])
        U, _, Vt = np.linalg.svd(P.T @ Q)
        R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        if np.abs(P @ R.T - Q).max() > 1e-2:
            continue
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = R, cd - R @ cp
        mp = _map_faces(pdescs, ddescs, m)
        if mp:
            return m, mp
    return None


def find_sources(part, pdescs, asm, desc_of) -> list:
    """[(occurrence, M part->definition, face-id mapping)] for every copy of the part."""
    out = []
    for occ in asm.occurrences:
        d = occ.definition
        if len(d.faces) != len(pdescs):
            continue
        reg = register(part, pdescs, d, desc_of(d))
        if reg:
            out.append((occ, reg[0], reg[1]))
    return out


def _touch(qw, tw):
    """Do world-frame faces qw (selected) and tw (other part) face each other? -> (ok, gap/clear)."""
    ok, c, _ = single_complement(qw, tw)
    if not ok:
        return False, None
    if qw.kind == "plane":
        if np.dot(qw.n, tw.n) > -np.cos(C.ANGLE_TOL):
            return False, None
        gap = float(np.dot(qw.o - tw.o, tw.n))            # > 0: air between the faces
        return (-C.INTERFERENCE_MAX <= gap <= C.INPLACE_GAP), gap
    if qw.kind in ("cylinder", "cone"):
        if np.linalg.norm(np.cross(qw.a, tw.a)) > np.sin(C.ANGLE_TOL):
            return False, None
        w = tw.p - qw.p
        if np.linalg.norm(w - np.dot(w, qw.a) * qw.a) > C.INPLACE_GAP:
            return False, None
        return (c is None or -C.INTERFERENCE_MAX <= c <= C.INPLACE_GAP * 2), c
    if qw.kind == "sphere":
        if np.linalg.norm(qw.c - tw.c) > C.INPLACE_GAP:
            return False, None
        return (-C.INTERFERENCE_MAX <= c <= C.INPLACE_GAP * 2), c
    return False, None


def in_place(src_occ, m_reg, mapping, sel_ids, asm, desc_of, source_paths) -> dict | None:
    """What the selected faces of the copy src_occ touch right now, without moving it.
    Each selected face looks for a facing (or coaxial) face on another part within INPLACE_GAP
    that really overlaps it. Returns the hits per selected face with the fit and the joint,
    or None if no selected face touches anything."""
    m_world = src_occ.world @ m_reg
    ddescs = desc_of(src_occ.definition)
    q_local = [ddescs[mapping[f] - 1] for f in sel_ids]
    for q in q_local:
        q.samples()
    q_world = [q.moved(src_occ.world) for q in q_local]
    src_box = src_occ.world_bbox(C.INPLACE_GAP)
    hits = {i: [] for i in range(len(q_local))}
    for occ in asm.occurrences:
        if occ.path in source_paths or not _box_ok(src_box, occ):
            continue
        tdescs = desc_of(occ.definition)
        m_q2t = np.linalg.inv(occ.world) @ src_occ.world
        for t in tdescs:                       # samples are made on demand, only for faces that touch
            tw = t.moved(occ.world)
            for i, (ql, qw) in enumerate(zip(q_local, q_world)):
                ok, val = _touch(qw, tw)
                if not ok:
                    continue
                ov = pair_overlap(ql, t, m_q2t, C.INPLACE_GAP + C.GEOM_TOL)
                if ov >= C.MIN_PAIR_OVERLAP:
                    hits[i].append({"occ": occ, "t_local": t, "t_world": tw, "overlap": ov, "value": val})
    covered = sum(1 for v in hits.values() if v)
    if covered == 0:
        return None
    result = {"found_by": "in-place", "coverage": f"{covered}/{len(q_local)}", "hits": hits,
              "m_part_world": m_world, "complete": covered == len(q_local)}
    if not result["complete"]:
        return result
    best = [max(hits[i], key=lambda h: h["overlap"]) for i in range(len(q_local))]
    t_world = [b["t_world"] for b in best]
    values = [b["value"] for b in best if b["value"] is not None]
    fp = fingerprint(q_world)
    for (i, j), rq in fp.items():
        ok, c, _ = compare(rq, relation(t_world[i], t_world[j]), strict=True)
        if c is not None:
            values.append(c)
    fit, val = classify(values)
    A, _, _ = rows(q_world, t_world, np.eye(3))
    ft, fr = free_translations(A), free_rotations(t_world)
    joint = JOINTS.get((len(ft), len(fr)), f"other({len(ft)}T,{len(fr)}R)")
    result.update(fit=fit, clearance=val, joint=joint, behaviour=behaviour(fit, joint),
                  free_t=[list(map(float, f)) for f in ft])
    return result


def _box_ok(src_box, occ):
    from .loader import bbox_overlap
    return bbox_overlap(src_box, occ.world_bbox())
