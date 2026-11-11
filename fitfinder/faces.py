"""Face records: type, size, outward normal / axis, convexity.

    plane    n = outward normal, o = point on the face
    cylinder a = axis, p = point on axis, r, convex (True = shaft, False = bore)
    cone     a = axis apex -> wide end, p = apex, half = half-angle, convex
    sphere   c = centre, r, convex (True = ball, False = socket)
    other    freeform / torus: not matched
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepClass import BRepClass_FaceClassifier
from OCP.BRepGProp import BRepGProp, BRepGProp_Face
from OCP.BRepTools import BRepTools
from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Sphere
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Cone, gp_Cylinder, gp_Pln, gp_Pnt, gp_Pnt2d, gp_Sphere, gp_Vec
from OCP.ShapeAnalysis import ShapeAnalysis_CanonicalRecognition
from OCP.TopAbs import TopAbs_IN, TopAbs_ON

CANON_TOL = 1e-4   # mm, tolerance to recognise a B-spline face as plane/cylinder/cone/sphere


@dataclass
class FaceDesc:
    fid: int                       # 1-based face ID in its definition
    kind: str                      # plane | cylinder | cone | sphere | other
    area: float
    center: np.ndarray             # area centroid
    n: np.ndarray | None = None    # plane outward normal
    o: np.ndarray | None = None    # point on plane
    a: np.ndarray | None = None    # axis direction (cyl: canonical sign, cone: apex->wide)
    p: np.ndarray | None = None    # point on axis (cyl), apex (cone)
    c: np.ndarray | None = None    # sphere centre
    r: float = 0.0                 # radius (cyl, sphere)
    half: float = 0.0              # cone half-angle (rad)
    convex: bool | None = None
    face: object = None            # TopoDS_Face (local frame)
    _samples: tuple | None = field(default=None, repr=False)

    def moved(self, m: np.ndarray) -> "FaceDesc":
        """Copy of this descriptor in another frame (m = 4x4 rigid transform)."""
        R, t = m[:3, :3], m[:3, 3]
        P = lambda x: None if x is None else R @ x + t
        D = lambda x: None if x is None else R @ x
        d = FaceDesc(self.fid, self.kind, self.area, P(self.center), D(self.n), P(self.o), D(self.a),
                     P(self.p), P(self.c), self.r, self.half, self.convex, self.face)
        if self._samples is not None:
            pts, nrm = self._samples
            d._samples = (pts @ R.T + t, nrm @ R.T)
        return d

    def samples(self, n: int = 12):
        """Interior points (k,3) + outward normals (k,3) of the trimmed face, local frame."""
        if self._samples is None:
            self._samples = sample_face(self.face, n)
        return self._samples

    def short(self) -> str:
        f = lambda v: "(" + ", ".join(f"{x:.3g}" for x in v) + ")"
        if self.kind == "plane":
            return f"#{self.fid} plane n={f(self.n)} area={self.area:.1f}"
        if self.kind == "cylinder":
            return f"#{self.fid} cylinder r={self.r:.4g} {'shaft' if self.convex else 'bore'} axis={f(self.a)}"
        if self.kind == "cone":
            return f"#{self.fid} cone half={math.degrees(self.half):.3f}deg {'convex' if self.convex else 'concave'}"
        if self.kind == "sphere":
            return f"#{self.fid} sphere r={self.r:.4g} {'ball' if self.convex else 'socket'}"
        return f"#{self.fid} other area={self.area:.1f}"


def _v(x) -> np.ndarray:
    return np.array([x.X(), x.Y(), x.Z()], dtype=float)


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def canonical_axis(a: np.ndarray) -> np.ndarray:
    """Make an undirected axis direction unique: first non-zero component positive."""
    for x in a:
        if abs(x) > 1e-9:
            return a if x > 0 else -a
    return a


def point_and_normal(face, u: float, v: float):
    """Point and OUTWARD normal (face orientation respected) at (u, v)."""
    pnt, vec = gp_Pnt(), gp_Vec()
    BRepGProp_Face(face).Normal(u, v, pnt, vec)
    n = _v(vec)
    nn = np.linalg.norm(n)
    return _v(pnt), (n / nn if nn > 1e-12 else n)


def _mid_point_normal(face):
    u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
    # try the middle first, then a few other interior points (middle may be in a hole)
    for fu, fv in ((0.5, 0.5), (0.25, 0.25), (0.75, 0.75), (0.25, 0.75), (0.75, 0.25)):
        u, v = u0 + fu * (u1 - u0), v0 + fv * (v1 - v0)
        if BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), 1e-7).State() in (TopAbs_IN, TopAbs_ON):
            return point_and_normal(face, u, v)
    return point_and_normal(face, u0 + 0.5 * (u1 - u0), v0 + 0.5 * (v1 - v0))


def sample_face(face, n: int = 12):
    """n x n UV grid, keep points inside the trimmed face. Refines once if too few."""
    for k in (n, 3 * n):
        u0, u1, v0, v1 = BRepTools.UVBounds_s(face)
        pts, nrm = [], []
        for i in range(k):
            for j in range(k):
                u = u0 + (i + 0.5) / k * (u1 - u0)
                v = v0 + (j + 0.5) / k * (v1 - v0)
                if BRepClass_FaceClassifier(face, gp_Pnt2d(u, v), 1e-7).State() == TopAbs_IN:
                    p, q = point_and_normal(face, u, v)
                    pts.append(p)
                    nrm.append(q)
        if len(pts) >= 12:
            break
    return np.array(pts).reshape(-1, 3), np.array(nrm).reshape(-1, 3)


def _surface_kind(face):
    """Return (kind, gp_object) using the exact surface type, else canonical recognition."""
    ad = BRepAdaptor_Surface(face, True)
    t = ad.GetType()
    if t == GeomAbs_Plane:
        return "plane", ad.Plane()
    if t == GeomAbs_Cylinder:
        return "cylinder", ad.Cylinder()
    if t == GeomAbs_Cone:
        return "cone", ad.Cone()
    if t == GeomAbs_Sphere:
        return "sphere", ad.Sphere()
    try:
        rec = ShapeAnalysis_CanonicalRecognition(face)
        g = gp_Pln()
        if rec.IsPlane(CANON_TOL, g):
            return "plane", g
        g = gp_Cylinder()
        if rec.IsCylinder(CANON_TOL, g):
            return "cylinder", g
        g = gp_Cone()
        if rec.IsCone(CANON_TOL, g):
            return "cone", g
        g = gp_Sphere()
        if rec.IsSphere(CANON_TOL, g):
            return "sphere", g
    except Exception:
        pass
    return "other", None


def describe(face, fid: int) -> FaceDesc:
    """The face record of one face: its type (plane, cylinder, cone, sphere or other), area,
    outward normal or axis, radius, and whether it is convex (a shaft) or concave (a hole)."""
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    area, center = props.Mass(), _v(props.CentreOfMass())
    kind, g = _surface_kind(face)
    d = FaceDesc(fid, kind, area, center, face=face)
    if kind == "other":
        return d
    pt, nrm = _mid_point_normal(face)
    if kind == "plane":
        d.n = nrm                       # outward, orientation already respected
        d.o = center                    # centroid of a planar face lies in its plane
    elif kind == "cylinder":
        ax = g.Axis()
        a, p0 = _unit(_v(ax.Direction())), _v(ax.Location())
        foot = p0 + np.dot(pt - p0, a) * a
        d.a, d.r = canonical_axis(a), g.Radius()
        d.p = p0 + np.dot(center - p0, a) * a     # axis point nearest the face centre
        d.convex = bool(np.dot(pt - foot, nrm) > 0)
    elif kind == "cone":
        ax = g.Axis()
        a, apex = _unit(_v(ax.Direction())), _v(g.Apex())
        if np.dot(center - apex, a) < 0:          # orient axis apex -> wide end
            a = -a
        foot = apex + np.dot(pt - apex, a) * a
        d.a, d.p, d.half = a, apex, abs(g.SemiAngle())
        d.convex = bool(np.dot(pt - foot, nrm) > 0)
    elif kind == "sphere":
        d.c, d.r = _v(g.Location()), g.Radius()
        d.convex = bool(np.dot(pt - d.c, nrm) > 0)
    return d


def describe_all(faces: list) -> list:
    return [describe(f, i) for i, f in enumerate(faces, start=1)]


def same_surface(a: FaceDesc, b: FaceDesc, tol: float = 1e-3) -> bool:
    """Do two face records lie on the same plane / cylinder / cone / sphere (same side)?"""
    if a.kind != b.kind or a.kind == "other":
        return False
    if a.kind == "plane":
        return float(np.dot(a.n, b.n)) > 1 - 1e-6 and abs(float(np.dot(b.o - a.o, a.n))) < tol
    if a.convex != b.convex:
        return False
    if a.kind == "sphere":
        return np.linalg.norm(a.c - b.c) < tol and abs(a.r - b.r) < tol
    if abs(abs(float(np.dot(a.a, b.a))) - 1) > 1e-6:
        return False
    if a.kind == "cylinder":
        d = b.p - a.p
        return np.linalg.norm(d - np.dot(d, a.a) * a.a) < tol and abs(a.r - b.r) < tol
    if a.kind == "cone":
        return np.linalg.norm(a.p - b.p) < tol and abs(a.half - b.half) < 1e-6
    return False


def surface_groups(shape, descs: list) -> dict:
    """Faces that are really pieces of ONE surface - a bore stored as two half-cylinders, a flat
    split by an edge. Two faces are merged only if they share an edge AND lie on the same surface,
    so two separate coaxial bores stay separate. Returns {face id: id of the group's first face}."""
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape, TopTools_IndexedMapOfShape
    parent = {d.fid: d.fid for d in descs}

    def root(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    fmap = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_FACE, fmap)
    edges = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, edges)
    for i in range(1, edges.Extent() + 1):
        ids = sorted({fmap.FindIndex(f) for f in edges.FindFromIndex(i)})
        for k in range(len(ids)):
            for j in range(k + 1, len(ids)):
                a, b = ids[k], ids[j]
                if a in parent and b in parent and same_surface(descs[a - 1], descs[b - 1]):
                    ra, rb = root(a), root(b)
                    parent[max(ra, rb)] = min(ra, rb)
    return {f: root(f) for f in parent}
