"""Outputs: matches.json, matched_faces.csv, overview.png, match_<n>.png (VTK, offscreen).

Colours: selected faces blue, matched target faces orange, ghost of the part at each fit green,
other parts light grey (transparent in the overview so hidden sockets stay visible).
"""
from __future__ import annotations

import csv
import json
import os

import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location

from . import config as C
from .loader import face_list, load_assembly, load_part, shape_bbox, trsf_to_matrix

BLUE, ORANGE, GREY, GREEN = (40, 110, 255), (255, 120, 30), (205, 205, 205), (60, 200, 90)


def _jsonable(x):
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (set, frozenset, tuple)):
        return list(x)
    return str(x)


def write_json(result: dict, out_dir: str):
    with open(os.path.join(out_dir, "matches.json"), "w") as f:
        json.dump(result, f, indent=1, default=_jsonable)


def write_csv(result: dict, out_dir: str):
    with open(os.path.join(out_dir, "matched_faces.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["match_id", "part", "occurrence", "target_face_id", "selected_face_id", "overlap",
                    "fit", "clearance_mm", "joint", "found_by"])
        for m in result["matches"]:
            for p in m["pairs"]:
                w.writerow([m["id"], p["target"].split("/")[-1], p["target"], p["target_face"],
                            p["selected_face"], p["overlap"], m["fit"], m["clearance"], m["joint"],
                            m["found_by"]])


def _short(path: str) -> str:
    """Last name in a part path; a part the file gives no name is shown by its parent's name."""
    segs = path.split("/")
    return segs[-2] if segs[-1].startswith("=>[") and len(segs) > 1 else segs[-1]


def summary_lines(result: dict) -> list:
    """Human readable result: one block per placed part (group), then near-misses and warnings."""
    groups = result.get("groups") or [{"group": m["id"], "parts": m["parts"], "occurrences": m["occurrences"],
                                       "positions": 1, "match_ids": [m["id"]]} for m in result["matches"]]
    by_id = {m["id"]: m for m in result["matches"]}
    lines = [f"{len(groups)} part(s) / {len(result['matches'])} position(s) fit the selected faces "
             f"{result['selected_faces']}  ({result['seconds']} s)"]
    limit = result.get("max_clearance_mm")
    if limit is not None and abs(limit - C.CLEARANCE_MAX) > 1e-9:
        lines.append(f"  clearance limit for this run: {limit:g} mm")
    for g in groups:
        ms = [by_id[i] for i in g["match_ids"]]
        m0 = ms[0]
        same = len({(m["fit"], m["clearance"]) for m in ms}) == 1
        fit = f"{m0['fit']} (clearance {m0['clearance']} mm), {m0['joint']} -> {m0['behaviour']}" if same \
            else f"different fits per position, {m0['joint']}"
        head = f"  [{g['group']}] {', '.join(g['parts'])}: {fit}"
        if len(ms) > 1:
            head += f"  - {len(ms)} positions"
        lines.append(head)
        for m in ms:
            if len(m["faces"]) == 1:        # one part: its name is in the line above
                faces = f"faces {next(iter(m['faces'].values()))}"
            else:
                faces = "; ".join(f"{_short(k)} faces {v}" for k, v in m["faces"].items())
            extra = "" if same else f"  ({m['fit']}, {m['clearance']} mm, {m['behaviour']})"
            lines.append(f"        match #{m['id']}: {faces}{extra}  [{m['found_by']}]")
    for n in result["near_misses"]:
        v = "" if n["value"] is None else f" ({n['value']:+.3f} mm)"
        lines.append(f"  near-miss {n['part']}: {n['reason']}{v}")
    for w in result["warnings"]:
        lines.append(f"  warning {w}")
    return lines


_MESH: dict = {}


def mesh(shape, key, deflection=0.3):
    """{face_id: (points (k,3) local, triangles (m,3))}"""
    if key in _MESH:
        return _MESH[key]
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    out = {}
    for fid, face in enumerate(face_list(shape), start=1):
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        m = trsf_to_matrix(loc.Transformation())
        pts = np.array([[tri.Node(i).X(), tri.Node(i).Y(), tri.Node(i).Z()] for i in range(1, tri.NbNodes() + 1)])
        pts = pts @ m[:3, :3].T + m[:3, 3]
        tris = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
        if face.Orientation() == TopAbs_REVERSED:
            tris = tris[:, [0, 2, 1]]
        out[fid] = (pts, tris)
    _MESH[key] = out
    return out


def _polydata(faces_mesh, world, colour_of):
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray
    pts_all, tris_all, cols = [], [], []
    base = 0
    for fid, (pts, tris) in faces_mesh.items():
        pts_all.append(pts @ world[:3, :3].T + world[:3, 3])
        tris_all.append(tris + base)
        cols += [colour_of(fid)] * len(tris)
        base += len(pts)
    if not pts_all:
        return None
    P = np.vstack(pts_all)
    T = np.vstack(tris_all)
    poly = vtk.vtkPolyData()
    vp = vtk.vtkPoints()
    vp.SetData(numpy_to_vtk(P.astype(np.float64), deep=True))
    poly.SetPoints(vp)
    cells = np.hstack([np.full((len(T), 1), 3), T]).astype(np.int64).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(T), numpy_to_vtkIdTypeArray(cells, deep=True))
    poly.SetPolys(ca)
    c = numpy_to_vtk(np.array(cols, dtype=np.uint8), deep=True)
    c.SetName("colors")
    poly.GetCellData().SetScalars(c)
    return poly


def _edges_actor(poly, opacity):
    """CAD-style outline: sharp edges (> 30 deg) and open boundaries as thin dark lines."""
    import vtk
    fe = vtk.vtkFeatureEdges()
    fe.SetInputData(poly)
    fe.SetFeatureAngle(30)
    fe.BoundaryEdgesOn()
    fe.FeatureEdgesOn()
    fe.ManifoldEdgesOff()
    fe.NonManifoldEdgesOff()
    fe.ColoringOff()
    mp = vtk.vtkPolyDataMapper()
    mp.SetInputConnection(fe.GetOutputPort())
    mp.ScalarVisibilityOff()
    a = vtk.vtkActor()
    a.SetMapper(mp)
    a.GetProperty().SetColor(0.2, 0.22, 0.26)
    a.GetProperty().SetOpacity(opacity)
    a.GetProperty().SetLineWidth(1.2)
    return a


def _actor(poly, opacity=1.0):
    import vtk
    mp = vtk.vtkPolyDataMapper()
    mp.SetInputData(poly)
    mp.SetScalarModeToUseCellData()
    a = vtk.vtkActor()
    a.SetMapper(mp)
    a.GetProperty().SetOpacity(opacity)
    return a


def render(result: dict, out_dir: str, size=(1400, 1000)):
    """overview.png + match_<n>.png. Needs a display, or VTK offscreen support (on headless Linux
    run under `xvfb-run`)."""
    import vtk
    _MESH.clear()   # definition keys (XCAF label entries) repeat between files
    asm = load_assembly(result["assembly"])
    part = load_part(result["part"])
    targets = {}            # occurrence path -> set(face ids)
    for m in result["matches"]:
        for path, fids in m["faces"].items():
            targets.setdefault(path, set()).update(fids)
    sources = set(result["source_occurrences"])
    sel = set(result["selected_faces"])

    def scene(focus_paths=None, ghosts=()):
        ren = vtk.vtkRenderer()
        ren.SetBackground(1, 1, 1)
        for o in asm.occurrences:
            fm = mesh(o.definition.shape, o.definition.key)
            tf = targets.get(o.path, set()) if focus_paths is None or o.path in focus_paths else set()
            is_src = o.path in sources
            colour = lambda fid, tf=tf, s=is_src: (ORANGE if fid in tf else BLUE if (s and fid in sel) else GREY)
            poly = _polydata(fm, o.world, colour)
            if poly is None:
                continue
            involved = o.path in targets or is_src
            if focus_paths is not None:
                op = 0.55 if o.path in focus_paths else 0.2
            else:
                op = 1.0 if involved else 0.3
            ren.AddActor(_actor(poly, op))
            ren.AddActor(_edges_actor(poly, 0.9 if op >= 0.5 else 0.55))
        pm = mesh(part.shape, "__part__")
        for T in ghosts:
            poly = _polydata(pm, np.array(T), lambda fid: BLUE if fid in sel else GREEN)
            if poly is not None:
                ren.AddActor(_actor(poly, 0.45))
        return ren

    def shoot(ren, path, focus_box=None):
        win = vtk.vtkRenderWindow()
        win.SetOffScreenRendering(1)
        win.AddRenderer(ren)
        win.SetSize(*size)
        cam = ren.GetActiveCamera()
        cam.SetPosition(1, -1.3, 0.9)
        cam.SetFocalPoint(0, 0, 0)
        cam.SetViewUp(0, 0, 1)
        if focus_box is not None:
            ren.ResetCamera(*focus_box)
        else:
            ren.ResetCamera()
        cam.Zoom(1.1)
        win.Render()
        f = vtk.vtkWindowToImageFilter()
        f.SetInput(win)
        f.Update()
        w = vtk.vtkPNGWriter()
        w.SetFileName(path)
        w.SetInputConnection(f.GetOutputPort())
        w.Write()

    shoot(scene(), os.path.join(out_dir, "overview.png"))
    for m in result["matches"]:
        focus = set(m["occurrences"])
        boxes = [o.world_bbox() for o in asm.occurrences if o.path in focus]
        pb = shape_bbox(part.shape)
        if pb is not None:
            T = np.array(m["transform"])
            corners = np.array([[pb[i], pb[1 + j], pb[2 + k]] for i in (0, 3) for j in (0, 3) for k in (0, 3)])
            cw = corners @ T[:3, :3].T + T[:3, 3]
            boxes.append(np.concatenate([cw.min(0), cw.max(0)]))
        boxes = [b for b in boxes if b is not None]
        fb = None
        if boxes:
            b = np.array(boxes)
            lo, hi = b[:, :3].min(0), b[:, 3:].max(0)
            pad = 0.3 * (hi - lo).max()
            fb = [lo[0] - pad, hi[0] + pad, lo[1] - pad, hi[1] + pad, lo[2] - pad, hi[2] + pad]
        shoot(scene(focus, [m["transform"]]), os.path.join(out_dir, f"match_{m['id']}.png"), fb)
