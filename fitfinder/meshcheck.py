"""Fast collision check on triangle meshes, run before the exact (slow) OpenCascade boolean.

The moving part is filled once with a grid of "inner points" that lie at least MESH_DEPTH
(0.25 mm) below its surface. For a trial position, a straight line is drawn through every
inner point and its crossings with the other part's triangles are counted. Two questions are
asked, and a "yes" to either one means the parts collide:

1. Is the inner point inside the other part?
   Yes if the line crosses the other part's surface an odd number of times on both sides of
   the point. Then the two solids overlap.
2. Does the other part's surface pass right next to the inner point (within MESH_REACH)?
   Then that surface runs through the inside of my part. This also works for a part that is
   only a skin of faces with no inside of its own (a "surface model", like the lathe's motor).

Why a good fit is never reported as a collision: in a real fit the two parts only touch, so
the other part's surface lies ON my part's surface, 0.25 mm away from every inner point. A
press fit overlaps by at most 0.05 mm and the meshes are accurate to 0.05 mm, which still
leaves a margin. The check only ever says "collides". When it finds nothing, the exact
boolean in verify.py still decides.
"""
from __future__ import annotations

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

from . import config as C
from .loader import face_list, shape_bbox

# rays are cast along the z axis of this fixed, "crooked" frame, so they are never parallel
# to the flat faces that real CAD is full of
_Q = np.linalg.qr(np.array([[0.83, 0.31, 0.47], [-0.29, 0.95, -0.11], [-0.48, -0.05, 0.88]]))[0]

_MESHES: dict = {}     # shape -> Mesh
_INNER: dict = {}      # shape -> (inner points, grid spacing)
_SOLID: dict = {}      # shape -> bool


def clear_caches():
    _MESHES.clear()
    _INNER.clear()
    _SOLID.clear()


def is_solid(shape) -> bool:
    """At least one solid and every shell closed (each edge shared by two faces). Only then
    does the part have an inside. A surface model (loose faces, open shells) has none."""
    key = id(shape)
    if key not in _SOLID:
        ok = TopExp_Explorer(shape, TopAbs_SOLID).More()
        ex = TopExp_Explorer(shape, TopAbs_SHELL)
        n = 0
        while ex.More() and ok:
            n += 1
            ok = BRep_Tool.IsClosed_s(TopoDS.Shell_s(ex.Current()))
            ex.Next()
        _SOLID[key] = (shape, ok and n > 0)     # the shape is kept so its id() stays unique
    return _SOLID[key][1]


def _triangles(shape) -> np.ndarray:
    """(m, 3, 3) triangle corners of a fine mesh of the shape (its own frame)."""
    BRepMesh_IncrementalMesh(shape, C.MESH_DEFLECTION, False, 0.3, True)
    out = []
    for f in face_list(shape):
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is None:
            continue
        t = loc.Transformation()
        nodes = np.array([[(p := tri.Node(i).Transformed(t)).X(), p.Y(), p.Z()]
                          for i in range(1, tri.NbNodes() + 1)])
        idx = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
        out.append(nodes[idx])
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


class Mesh:
    """The triangles of one part, with a fast "which of these points are inside?" test.

    The test draws a line through each point (along the z axis of the tilted frame _Q) and
    counts the triangles it crosses above and below the point. To avoid testing every point
    against every triangle, the triangles are sorted into square columns seen along that
    line; a point is only tested against the triangles in its own column."""

    def __init__(self, shape):
        self.tris = _triangles(shape)                       # (m, 3, 3), part frame
        self.box = shape_bbox(shape)
        self.closed = is_solid(shape)
        t = self.tris @ _Q.T                                # the same triangles in the line frame
        v0, e1, e2 = t[:, 0], t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]
        det = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        keep = np.abs(det) > 1e-12                          # triangles seen edge-on are never crossed
        self.v0, self.e1, self.e2, self.det = v0[keep], e1[keep], e2[keep], det[keep]
        lo, hi = t[keep, :, :2].min(1), t[keep, :, :2].max(1)
        self.origin = lo.min(0) if len(lo) else np.zeros(2)
        # column width: about half a typical triangle; doubled until the table is small enough
        self.cell = max(0.5 * float(np.median((hi - lo).max(1))) if len(lo) else 1.0, 0.5)
        while True:
            i0 = np.floor((lo - self.origin) / self.cell).astype(np.int64)
            i1 = np.floor((hi - self.origin) / self.cell).astype(np.int64)
            w, h = i1[:, 0] - i0[:, 0] + 1, i1[:, 1] - i0[:, 1] + 1
            n = w * h                                       # columns covered by each triangle
            if n.sum() <= 4_000_000:
                break
            self.cell *= 2
        # one table row per (column, triangle in it), sorted by column number
        self.ny = int(i1[:, 1].max()) + 1 if len(lo) else 1
        tri = np.repeat(np.arange(len(n)), n)
        k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        column = (np.repeat(i0[:, 0], n) + k % np.repeat(w, n)) * self.ny + np.repeat(i0[:, 1], n) + k // np.repeat(w, n)
        order = np.argsort(column, kind="stable")
        self.column, self.tri = column[order], tri[order]

    def crossings(self, pts: np.ndarray, reach: float):
        """For each point (part frame), look along the line through it and count the triangles
        it crosses: (above the point, below the point, within `reach` of the point)."""
        above, below, close = np.zeros(len(pts)), np.zeros(len(pts)), np.zeros(len(pts))
        in_box = np.nonzero(np.all((pts >= self.box[:3] - reach) & (pts <= self.box[3:] + reach), axis=1))[0]
        p = pts[in_box] @ _Q.T
        c = np.floor((p[:, :2] - self.origin) / self.cell).astype(np.int64)
        column = c[:, 0] * self.ny + c[:, 1]
        first = np.searchsorted(self.column, column, "left")            # each point's rows in the table
        count = np.searchsorted(self.column, column, "right") - first
        start = 0
        while start < len(p):                               # in chunks of about 2 million tests
            stop = start + max(1, int(np.searchsorted(np.cumsum(count[start:]), 2_000_000)))
            n = count[start:stop]
            pi = np.repeat(np.arange(stop - start), n)      # point number of every (point, triangle) test
            t = self.tri[np.repeat(first[start:stop], n) + np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)]
            q = p[start:stop][pi]
            # where the line through q meets the triangle's plane (u, v), and at what height z
            d = q[:, :2] - self.v0[t, :2]
            u = (d[:, 0] * self.e2[t, 1] - d[:, 1] * self.e2[t, 0]) / self.det[t]
            v = (self.e1[t, 0] * d[:, 1] - self.e1[t, 1] * d[:, 0]) / self.det[t]
            crossed = (u >= 0) & (v >= 0) & (u + v <= 1)
            dz = self.v0[t, 2] + u * self.e1[t, 2] + v * self.e2[t, 2] - q[:, 2]
            idx = in_box[start:stop]
            above[idx] = np.bincount(pi, weights=crossed & (dz > 0), minlength=stop - start)
            below[idx] = np.bincount(pi, weights=crossed & (dz < 0), minlength=stop - start)
            close[idx] = np.bincount(pi, weights=crossed & (np.abs(dz) < reach), minlength=stop - start)
            start = stop
        return above, below, close

    def inside(self, pts: np.ndarray) -> np.ndarray:
        """True/False per point: inside the part. Inside = an odd number of crossings above the
        point AND below it (asking for both makes a small gap in the mesh harmless).
        A surface model has no inside, so every point is False."""
        if not self.closed:
            return np.zeros(len(pts), dtype=bool)
        above, below, _ = self.crossings(pts, 0.0)
        return (above % 2 == 1) & (below % 2 == 1)

    def deep_inside(self, pts: np.ndarray) -> np.ndarray:
        """True/False per point: at least MESH_DEPTH below the surface. Test: the point and the
        six points 2 * MESH_DEPTH away from it along x, y and z are all inside."""
        keep = self.inside(pts)
        for d in np.vstack([np.eye(3), -np.eye(3)]) * 2 * C.MESH_DEPTH:
            idx = np.nonzero(keep)[0]
            if len(idx) == 0:
                break
            keep[idx] = self.inside(pts[idx] + d)
        return keep


def mesh_of(shape) -> Mesh:
    """The mesh of a part shape, built once per run."""
    key = id(shape)
    if key not in _MESHES:
        _MESHES[key] = (shape, Mesh(shape))
    return _MESHES[key][1]


def inner_points(shape):
    """Question 1's points: a grid filling the part, keeping the points deep inside it.
    Returns (points in the part frame, grid spacing h)."""
    key = id(shape)
    if key not in _INNER:
        bb = shape_bbox(shape)
        size = np.maximum(bb[3:] - bb[:3], 1e-6)
        h = max((float(np.prod(size)) / C.MESH_GRID_POINTS) ** (1 / 3), 0.1)
        axes = [np.arange(lo + h / 2, hi, h) for lo, hi in zip(bb[:3], bb[3:])]
        grid = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        _INNER[key] = (shape, grid[mesh_of(shape).deep_inside(grid)], h)
    return _INNER[key][1], _INNER[key][2]


def clash(src_shape, m_world: np.ndarray, occ, allowance: float):
    """Does the part placed at m_world surely run into the placed part occ?
    Returns a rough size of the overlap (mm^3), or None when no clash is seen."""
    if not is_solid(src_shape):
        return None                                         # my part has no inside: nothing to test
    pts, h = inner_points(src_shape)
    if len(pts) == 0:
        return None
    theirs = mesh_of(occ.definition.shape)
    # a spread-out subset of the points is enough against a solid; a surface model can only be
    # caught by question 2, which needs a point right next to its skin, so it gets all of them
    step = -(-len(pts) // C.MESH_TEST_POINTS) if theirs.closed else 1
    to_theirs = np.linalg.inv(occ.world) @ m_world          # my frame -> their frame
    above, below, close = theirs.crossings(pts[::step] @ to_theirs[:3, :3].T + to_theirs[:3, 3], C.MESH_REACH)
    size = step * h ** 3                                    # volume that one tested point stands for
    # 1. my inner points inside their part
    n = int(((above % 2 == 1) & (below % 2 == 1)).sum()) if theirs.closed else 0
    if n >= C.MESH_MIN_POINTS and n * size > C.COLLISION_VOL + allowance:
        return n * size
    # 2. their surface right next to my inner points
    n = int((close > 0).sum())
    return n * size if n >= C.MESH_MIN_POINTS else None
