"""Step 4 of the method: check a position on the real shapes.

place() is the entry point. It tries the part at up to SLIDE_STEPS positions along the direction
it can still slide in, keeps the positions where the selected faces really lie on their
partner faces (overlap_spots), takes the best one that runs into nothing (collides), and
finally pushes the part in as far as it goes (seat).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAPI import GeomAPI_ProjectPointOnSurf
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Pnt, gp_Pnt2d
from OCP.TopAbs import TopAbs_IN, TopAbs_ON
from OCP.TopAbs import TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopTools import TopTools_ListOfShape
from OCP.TopoDS import TopoDS

from . import config as C
from . import meshcheck
from .meshcheck import is_solid
from .faces import _mid_point_normal
from .loader import bbox_overlap, face_list, matrix_to_trsf, shape_bbox

_SURF_CACHE: dict = {}


def _surface(face):
    key = face.__hash__()          # = memory address of the face: keep the face alive with it
    if key not in _SURF_CACHE:
        _SURF_CACHE[key] = (face, BRep_Tool.Surface_s(face))
    return _SURF_CACHE[key][1]


def clear_caches():
    """Called at the start of every run (the browser server runs many searches in one process)."""
    _SURF_CACHE.clear()
    _SURFACE_POINTS.clear()
    _SAME.clear()
    meshcheck.clear_caches()


def apply(m: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ m[:3, :3].T + m[:3, 3]


def on_face(pts: np.ndarray, face, max_dist: float) -> np.ndarray:
    """Per point (already in the face's frame): does it project inside the trimmed face?"""
    surf, out = _surface(face), np.zeros(len(pts), dtype=bool)
    for i, p in enumerate(pts):
        proj = GeomAPI_ProjectPointOnSurf(gp_Pnt(*p), surf)
        if proj.NbPoints() == 0 or proj.LowerDistance() > max_dist:
            continue
        u, v = proj.LowerDistanceParameters()
        out[i] = BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), 1e-6).State() in (TopAbs_IN, TopAbs_ON)
    return out


def _pieces(t) -> list:
    return t if isinstance(t, list) else [t]


def _area(t) -> float:
    return sum(x.area for x in _pieces(t))


def pair_overlap(q, t, m_q2t: np.ndarray, max_dist: float, coarse: bool = False) -> float:
    """Overlap of query face q (part frame) and target t (target frame), m maps q -> t frame.
    t is one face or a list of pieces of one surface (a wall split by a recess): a point counts
    when it lands on any piece. coarse=True uses every 4th sample point while scanning positions."""
    step, ts = (4 if coarse else 1), _pieces(t)
    if q.area <= _area(ts):
        pts = apply(m_q2t, q.samples()[0][::step])
        if len(pts) == 0:
            return 0.0
        hit = np.zeros(len(pts), dtype=bool)
        for x in ts:
            hit[~hit] = on_face(pts[~hit], x.face, max_dist)
        return float(hit.mean())
    pts = np.concatenate([x.samples()[0][::step] for x in ts])
    if len(pts) == 0:
        return 0.0
    return float(on_face(apply(np.linalg.inv(m_q2t), pts), q.face, max_dist).mean())


def overlaps(query, target, m: np.ndarray, max_dist: float):
    vals = [pair_overlap(q, t, m, max_dist) for q, t in zip(query, target)]
    w = [min(q.area, _area(t)) for q, t in zip(query, target)]
    return vals, float(np.dot(vals, w) / max(sum(w), 1e-12))


def common_volume(a, b) -> float:
    """Exact overlap volume of two shapes (mm^3), by OpenCascade's boolean 'common'.
    Slow, and not fully reliable on complex parts, so meshcheck.py runs in front of it."""
    args, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    args.Append(a)
    tools.Append(b)
    op = BRepAlgoAPI_Common()
    op.SetArguments(args)
    op.SetTools(tools)
    op.SetRunParallel(True)      # use all CPU cores
    op.SetUseOBB(True)           # oriented boxes: skip face pairs that cannot meet
    op.Build()
    if not op.IsDone() or getattr(op, "HasErrors", lambda: False)():
        d = BRepExtrema_DistShapeShape(a, b)
        return 0.0 if (d.IsDone() and d.Value() > 1e-6) else float("nan")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(op.Shape(), props)
    return abs(props.Mass())


def collides(src_shape, m_world: np.ndarray, others: list, allowance: float):
    """Does the part, placed at m_world, run into any other part?
    Returns (name of the first part hit or None, rough size of the overlap in mm^3).
    others = [(placed part, its shape in world coordinates, its bounding box)].
    1. Parts whose bounding boxes are apart are skipped.
    2. The fast mesh check (meshcheck.py) catches every clear collision.
    3. Only if it finds none: the exact OpenCascade overlap volume, for solid parts."""
    moved = src_shape.Moved(TopLoc_Location(matrix_to_trsf(m_world)))
    box = shape_bbox(moved)
    near = [(occ, wshape) for occ, wshape, obox in others if bbox_overlap(box, obox)]
    for occ, _ in near:
        size = meshcheck.clash(src_shape, m_world, occ, allowance)
        if size is not None:
            return occ.name, size
    for occ, wshape in near:
        if not is_solid(occ.definition.shape):
            continue            # a surface model has no volume: the mesh check was its only test
        vol = common_volume(moved, wshape)
        if math.isnan(vol):
            continue            # boolean failed and the shapes touch: contact, not collision
        if vol > C.COLLISION_VOL + allowance:
            return occ.name, vol
    return None, 0.0


_SAME: dict = {}
_SURFACE_POINTS: dict = {}


def surface_points(shape) -> np.ndarray:
    """Points on the part's surface (part frame): its corners, one point per face, and the
    nodes of a coarse mesh."""
    key = id(shape)
    if key not in _SURFACE_POINTS:
        pts = [np.array(_mid_point_normal(f)[0], dtype=float).reshape(1, 3) for f in face_list(shape)]
        ex = TopExp_Explorer(shape, TopAbs_VERTEX)
        while ex.More():
            v = BRep_Tool.Pnt_s(TopoDS.Vertex_s(ex.Current()))
            pts.append(np.array([[v.X(), v.Y(), v.Z()]]))
            ex.Next()
        bb = shape_bbox(shape)
        BRepMesh_IncrementalMesh(shape, float(np.clip(0.02 * np.linalg.norm(bb[3:] - bb[:3]), 0.05, 2.0)), False, 0.5)
        for f in face_list(shape):
            loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(f, loc)
            if tri is not None:
                t = loc.Transformation()
                pts.append(np.array([[(q := tri.Node(i).Transformed(t)).X(), q.Y(), q.Z()]
                                     for i in range(1, tri.NbNodes() + 1)]).reshape(-1, 3))
        _SURFACE_POINTS[key] = (shape, np.vstack(pts))     # the shape is kept so its id() stays unique
    return _SURFACE_POINTS[key][1]


def looks_same(shape, m: np.ndarray) -> bool:
    """True if moving the part by m (part frame) leaves it looking the same, e.g. turning a plain
    pin about its own axis. Then that turn cannot cure a collision and is not worth a boolean.
    Test: every surface point, moved, still lies on the part's surface."""
    key = (id(shape), tuple(np.round(m[:3, :4], 3).ravel()))
    if key not in _SAME:
        c = BRepClass3d_SolidClassifier()
        c.Load(shape)
        same = True
        for p in apply(m, surface_points(shape)):
            c.Perform(gp_Pnt(*p), 10 * C.GEOM_TOL)
            if c.State() != TopAbs_ON:
                same = False
                break
        _SAME[key] = (shape, same)
    return _SAME[key][1]


def _rot_about(point, axis, ang) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, point - R @ point
    return m


def slide_range(query, target, base: np.ndarray, f: np.ndarray):
    """Shifts u along f at which every selected face still lies over its target face:
    intersection over pairs of [ta - qb, tb - qa] (face samples projected on f). (lo, hi) or None."""
    lo, hi = -np.inf, np.inf
    for q, t in zip(query, target):
        qp = apply(base, q.samples()[0]) @ f
        tp = np.concatenate([x.samples()[0] for x in _pieces(t)]) @ f
        if len(qp) == 0 or len(tp) == 0:
            return None
        lo, hi = max(lo, tp.min() - qp.max()), min(hi, tp.max() - qp.min())
    return (lo, hi) if hi >= lo else None


@dataclass
class Spot:
    """One trial position of the part along its slide."""
    shift: float        # mm moved along the free direction from the solved position
    m: np.ndarray       # 4x4 transform: part frame -> target part frame
    overlap: float      # share of the selected faces lying on their partners (0..1, area-weighted)


def overlap_spots(query, target, pose, max_dist: float):
    """Trial positions where every selected face really lies on its partner face.
    A part that can slide is tried at SLIDE_STEPS evenly spaced shifts; a part that cannot is
    tried where it is. Returns (spots, slide direction, distance between two shifts)."""
    base = pose.matrix()
    slide = pose.free_t[0] if len(pose.free_t) == 1 else np.zeros(3)
    shifts = [0.0]
    if len(pose.free_t) == 1:
        rng = slide_range(query, target, base, slide)
        if rng is None:
            return [], slide, 0.0
        if rng[1] > rng[0]:
            shifts = list(np.linspace(rng[0], rng[1], C.SLIDE_STEPS))
        else:
            shifts = [rng[0]]
    weights = [min(q.area, _area(t)) for q, t in zip(query, target)]
    spots = []
    for u in shifts:
        m = base.copy()
        m[:3, 3] += u * slide
        vals = []
        for q, t in zip(query, target):
            vals.append(pair_overlap(q, t, m, max_dist, coarse=True))
            if vals[-1] < C.MIN_PAIR_OVERLAP:
                break                                   # every face pair must touch
        if len(vals) == len(query) and vals[-1] >= C.MIN_PAIR_OVERLAP:
            mean = float(np.dot(vals, weights) / max(sum(weights), 1e-12))
            if mean >= C.MIN_MEAN_OVERLAP:
                spots.append(Spot(float(u), m, mean))
    step = shifts[1] - shifts[0] if len(shifts) > 1 else 0.0
    return spots, slide, float(step)


def place(query, target, pose, occ_world, src_shape, others, allowance, max_dist) -> dict:
    """Find a position near the solved pose where the part really sits.
    target[i] = the partner of query[i]: one face, or the pieces of one split face.
    Returns {"ok": True, "m_local": part -> target part transform, "overlaps", "mean",
    "engaged_length"} or {"ok": False, "reason": ...}."""
    spots, slide, step = overlap_spots(query, target, pose, max_dist)
    if not spots:
        return {"ok": False, "reason": "faces do not really overlap"}
    engaged = round((len(spots) - 1) * step, 2)        # length over which the faces stay engaged
    by_overlap = sorted(spots, key=lambda s: -s.overlap)

    def result(m):
        m = seat(query, target, m, slide, step, src_shape, occ_world, others, allowance, max_dist)
        vals, mean = overlaps(query, target, m, max_dist)
        return {"ok": True, "m_local": m, "overlaps": vals, "mean": mean, "engaged_length": engaged}

    # 1. Along the slide, best overlap first: the first position that runs into nothing wins.
    blocker = None
    for s in by_overlap:
        blocker, _ = collides(src_shape, occ_world @ s.m, others, allowance)
        if blocker is None:
            return result(s.m)

    # 2. Everything collides. If the part can turn (a pin, a T-handle that hits a wall), try
    #    turning it, for the 3 best positions. Turns that leave the part looking the same are
    #    skipped. If three turns clash by the same amount, the clash is round (a pin bottoming
    #    out in a blind hole) and turning further cannot cure it.
    axes = pose.free_r[:1] if len(pose.free_r) < 3 else pose.free_r
    angles = [2 * math.pi * k / C.ROT_STEPS for k in range(1, C.ROT_STEPS)] if pose.free_r else []
    for s in by_overlap[:3]:
        back = np.linalg.inv(s.m)
        for point, direction in axes:
            sizes = []
            for ang in angles:
                turned = _rot_about(point, direction, ang) @ s.m
                if looks_same(src_shape, back @ turned):
                    continue
                blocker, size = collides(src_shape, occ_world @ turned, others, allowance)
                if blocker is None:
                    return result(turned)
                sizes.append(size)
                if len(sizes) >= 3 and max(sizes) - min(sizes) <= 0.02 * max(sizes):
                    break
    return {"ok": False, "reason": f"collides with {blocker}"}


def seat(query, target, m, slide, step, src_shape, occ_world, others, allowance, max_dist) -> np.ndarray:
    """Push the part further along its slide, towards more overlap, until it touches something
    (a key goes down until its tip is on the socket floor). The contact point is found by
    halving the last step 7 times. Returns the seated transform (or m unchanged)."""
    if step == 0.0:
        return m
    best_m, best_mean = m, overlaps(query, target, m, max_dist)[1]
    for direction in (1, -1):
        free, blocked = 0.0, step * direction          # shifts from m: known free / to be tested
        further = m.copy()
        further[:3, 3] += blocked * slide
        if overlaps(query, target, further, max_dist)[1] <= best_mean + 1e-6:
            continue                                    # no more overlap this way
        blocker = collides(src_shape, occ_world @ further, others, allowance)[0]
        if blocker is None:
            free = blocked                              # a whole step further is still free
        else:
            # halve against the blocking part only (one check per halving, not one per
            # neighbour); the final position is checked against everything below
            only = [x for x in others if x[0].name == blocker]
            for _ in range(7):
                mid = (free + blocked) / 2
                trial = m.copy()
                trial[:3, 3] += mid * slide
                if collides(src_shape, occ_world @ trial, only, allowance)[0] is None:
                    free = mid
                else:
                    blocked = mid
        if free == 0.0:
            continue
        trial = m.copy()
        trial[:3, 3] += free * slide
        if collides(src_shape, occ_world @ trial, others, allowance)[0] is not None:
            continue                                    # something else is in the way: stay put
        mean = overlaps(query, target, trial, max_dist)[1]
        if mean > best_mean:
            best_m, best_mean = trial, mean
    return best_m
