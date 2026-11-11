"""run_match: the whole method from start to finish. Read this file first.

    Step 1  describe the selected faces                      describe_selection   (faces.py)
    Step 2  find groups of faces arranged the same way       search               (search.py)
    Step 3  work out where each group puts the part          solve_positions      (pose.py)
    Step 4  check each position on the real shapes           check_position       (verify.py)
    then    if the part is already in the assembly, add what it touches now (inplace.py)

Steps 2 and 3 run once per part SHAPE (three copies of a pinion share one search). Step 4 runs
once per placed COPY, because each copy has different neighbours to run into.
"""
from __future__ import annotations

import dataclasses
import time

import numpy as np

from . import config as C
from .faces import describe_all, surface_groups
from .fit import behaviour, classify
from .inplace import find_sources, in_place
from .loader import bbox_overlap, load_assembly, load_part, shape_bbox
from .meshcheck import is_solid
from .pose import selection_dof, solve
from .search import _BOX, search, strict_check
from .verify import clear_caches, pair_overlap, place

STAGE = {"size": 1, "pose": 2, "overlap": 3, "collision": 4}     # how far a rejected attempt got


def max_dist() -> float:
    """Faces this far apart can still be partners (the clearance limit plus a hair)."""
    return C.CLEARANCE_MAX + C.GEOM_TOL


class Results:
    """Fits found so far, and the near-misses: attempts that failed a check, with the reason."""

    def __init__(self):
        self.matches: dict = {}     # (placed part path, surfaces) -> match
        self.near: dict = {}        # (placed part path, surfaces) -> (stage, reason, value, part name)

    def reject(self, key, stage: str, reason: str, value, name: str):
        """Remember why an attempt failed. Per key, the attempt that got furthest is kept."""
        if key in self.matches:
            return
        if key not in self.near or STAGE[stage] > self.near[key][0]:
            self.near[key] = (STAGE[stage], reason, value, [name])


def face_records(definition) -> list:
    """Face records of a part shape. Kept on the shape, so a loaded assembly is described once."""
    if getattr(definition, "_descs", None) is None:
        definition._descs = describe_all(definition.faces)
    return definition._descs


def size_constrained(query: list) -> bool:
    """True if the selection fixes a size: a round face, or more flats than independent normal
    directions (opposite flats, a dovetail). An L corner or a V fixes none."""
    if any(q.kind in ("cylinder", "cone", "sphere") for q in query):
        return True
    normals = [q.n for q in query if q.kind == "plane"]
    return len(normals) > np.linalg.matrix_rank(np.array(normals), tol=1e-3) if normals else False


def position_key(pose) -> tuple:
    """Where a pose puts the part, ignoring the directions it can still slide in."""
    t = pose.t.copy()
    for f in pose.free_t:
        t -= np.dot(t, f) * f
    return pose.joint, tuple(np.round(pose.R, 3).ravel()), tuple(np.round(t, 2))


def group_matches(matches: list) -> list:
    """One group per placed part, listing all its positions."""
    groups: dict = {}
    for m in matches:
        groups.setdefault(tuple(m["occurrences"]), []).append(m)
    out = []
    for gid, (occs, ms) in enumerate(groups.items(), start=1):
        for m in ms:
            m["group"] = gid
        out.append({"group": gid, "parts": ms[0]["parts"], "occurrences": list(occs),
                    "positions": len(ms), "match_ids": [m["id"] for m in ms],
                    "fits": sorted({m["fit"] for m in ms}), "joints": sorted({m["joint"] for m in ms})})
    return out


def collapse_near_misses(near: dict, matches: dict) -> list:
    """At most one near-miss per part (the attempt that got furthest); none for matched parts."""
    matched_occs = {p for m in matches.values() for p in m["occurrences"]}
    best: dict = {}
    for (path, fset), (stage, reason, value, names) in near.items():
        if path in matched_occs:
            continue
        rank = (stage, -abs(value) if value is not None else 0.0)
        if path not in best or rank > best[path][0]:
            best[path] = (rank, {"part": names[0], "occurrence": path, "faces": sorted(fset), "reason": reason,
                                 "value": None if value is None else round(float(value), 4)})
    return [v for _, v in sorted(best.values(), key=lambda x: x[1]["occurrence"])]


def load_inputs(part_path, assembly_path, faces, use_green, near, margin, part, assembly, log):
    """Read the two files (unless already loaded), pick the selected face IDs, apply --near."""
    part = part or load_part(part_path)
    sel = list(faces) if faces else (part.green_ids if use_green else [])
    if not sel:
        raise ValueError("no faces selected (give --faces or a part with green faces)")
    bad = [f for f in sel if f < 1 or f > len(part.faces)]
    if bad:
        raise ValueError(f"face IDs out of range 1..{len(part.faces)}: {bad}")
    asm = assembly or load_assembly(assembly_path)
    if near:
        boxes = [o.world_bbox(margin) for o in asm.occurrences if near.lower() in o.path.lower()]
        if not boxes:
            raise ValueError(f"--near: no part name contains '{near}'")
        asm = dataclasses.replace(asm, occurrences=[o for o in asm.occurrences
                                                   if any(bbox_overlap(b, o.world_bbox()) for b in boxes)])
        log(f"--near {near}: kept {len(asm.occurrences)} parts within {margin} mm")
    return part, sel, asm


def describe_selection(part, pdescs: list, sel: list):
    """Step 1. The face records of the selected faces, and warnings about the selection.
    Freeform faces are dropped. Pieces of one surface (a cylinder stored as two halves) count
    as one face."""
    query = [pdescs[f - 1] for f in sel]
    warnings = [f"UNSUPPORTED_FACE:{q.fid}" for q in query if q.kind == "other"]
    query = [q for q in query if q.kind != "other"]
    same_surface = surface_groups(part.shape, pdescs)
    one_each: dict = {}
    for q in query:
        one_each.setdefault(same_surface[q.fid], q)
    if len(one_each) < len(query):
        warnings.append("MERGED_FACES: some selected faces are pieces of one surface (e.g. a cylinder stored "
                        "as two halves) and are treated as one face")
        query = list(one_each.values())
    if not query:
        raise ValueError("only unsupported (freeform/torus) faces selected")
    for q in query:
        q.samples()
    free_slides, _ = selection_dof(query)
    if free_slides >= 2:
        warnings.append("UNDER_CONSTRAINED_SELECTION: global search skipped (selection could slide in 2 directions)")
    if not size_constrained(query):
        warnings.append("SIZE_NOT_CONSTRAINED: the selected faces fix no width or radius (each flat can "
                        "touch its partner independently), so a bigger pocket of the same shape also fits. "
                        "Add an opposite face if the width matters.")
    return query, warnings


def obstacles(part, asm, source_paths: set):
    """Every placed part the moving part could run into: [(placed part, world shape, bounding
    box)]. Copies of the part itself are left out. Also returns warnings about surface models."""
    others = []
    for o in asm.occurrences:
        if o.path not in source_paths:
            ws = o.world_shape()
            others.append((o, ws, shape_bbox(ws)))
    warnings = []
    surface = sorted({o.name for o, _, _ in others if o.definition.faces and not is_solid(o.definition.shape)})
    if surface:
        warnings.append("SURFACE_MODEL: these parts are only a skin of faces, not a closed solid. A collision is "
                        "seen where their skin cuts through your part, but not if your part lies "
                        "completely inside them: " + ", ".join(surface))
    if not is_solid(part.shape):
        warnings.append("SURFACE_MODEL: the selected part is not a closed solid, so collisions cannot be checked")
    return others, warnings


def solve_positions(query, tfaces, groups, same_surface, copies, found: Results) -> dict:
    """Step 3. For each face group: check the sizes with the real tolerance windows, then work
    out the position(s) it puts the part in.

    Several groups often give the same position: a wall split into pieces by a recess gives
    one group per piece. They are collected per position, so that each position is checked
    only once. Returns {position: [(group, pose, clearances, surfaces), ...]}."""
    positions: dict = {}
    for group in groups:
        surfaces = frozenset(same_surface[tfaces[k].fid] for k in group)
        ok, clear, why, value = strict_check(query, tfaces, group)
        if not ok:
            for o in copies:
                found.reject((o.path, surfaces), "size", why, value, o.name)
            continue
        for pose, why in solve(query, [tfaces[k] for k in group]):
            if pose is None:
                for o in copies:
                    found.reject((o.path, surfaces), "pose", why, None, o.name)
            else:
                positions.setdefault(position_key(pose), []).append((group, pose, clear, surfaces))
    return positions


def check_position(query, tfaces, candidates, same_surface, copies, part, others, found: Results):
    """Step 4. Check one position on the real shapes, once for every placed copy of the target
    part, and record a match or the reason for rejecting it."""
    _, pose, clear, _ = candidates[0]
    # the partner(s) of each selected face: all pieces that the groups at this position name
    partners = [list({tfaces[c[0][i]].fid: tfaces[c[0][i]] for c in candidates}.values())
                for i in range(len(query))]
    surfaces = frozenset().union(*(c[3] for c in candidates))
    for pieces in partners:
        for t in pieces:
            t.samples()
    tightest = min(clear) if clear else 0.0
    allowance = 0.0
    if tightest < 0:        # press fit: the intended overlap volume is not a collision
        allowance = 1.5 * sum(min(q.area, ps[0].area) for q, ps in zip(query, partners)) * (-tightest) / 2
    for o in copies:
        key = (o.path, surfaces)
        if any(k[0] == o.path and surfaces <= k[1] for k in found.matches):
            continue        # these surfaces already fit (the same socket, the key turned 90 deg)
        target_first = sorted(others, key=lambda x: x[0].path != o.path)    # pushed in too deep, it hits the target
        res = place(query, partners, pose, o.world, part.shape, target_first, allowance, max_dist())
        if not res["ok"]:
            stage = "collision" if res["reason"].startswith("collides") else "overlap"
            found.reject(key, stage, res["reason"], None, o.name)
            continue
        fit, value = classify(clear)
        pairs, touched = [], set()
        for q, pieces in zip(query, partners):
            for t in pieces:
                ov = pair_overlap(q, t, res["m_local"], max_dist())
                if ov > 0 or len(pieces) == 1:
                    pairs.append({"selected_face": q.fid, "target": o.path, "target_face": t.fid,
                                  "overlap": round(ov, 3)})
                    touched.add(same_surface[t.fid])
        found.matches[key] = {
            "found_by": "search", "parts": [o.name], "occurrences": [o.path],
            "faces": {o.path: sorted(f for f, g in same_surface.items() if g in touched)},
            "pairs": pairs,
            "fit": fit, "clearance": round(float(value), 4), "joint": pose.joint,
            "behaviour": behaviour(fit, pose.joint),
            "free_directions": [list(np.round(o.world[:3, :3] @ f, 4)) for f in pose.free_t],
            "engaged_length_mm": res["engaged_length"],
            "transform": (o.world @ res["m_local"]).round(6).tolist(),
            "mean_overlap": round(res["mean"], 3)}
        found.near.pop(key, None)


def search_shape(query, copies, part, others, found: Results, warnings: list):
    """Steps 2 to 4 for one part shape; copies = its placed copies in the assembly."""
    definition = copies[0].definition
    tfaces = face_records(definition)
    groups = search(query, tfaces, strict=True)                     # Step 2: real fits only
    if groups.stopped:
        warnings.append(f"SEARCH_LIMIT: {definition.name}: stopped after {len(groups)} face groups")
    if groups:
        same_surface = surface_groups(definition.shape, tfaces)     # split faces count as one surface
    else:
        # nothing fits: look at a few loosely matching groups, only to explain why not
        groups = search(query, tfaces, strict=False, limit=C.NEAR_MISS_GROUPS)
        same_surface = {t.fid: t.fid for t in tfaces}
    positions = solve_positions(query, tfaces, groups, same_surface, copies, found)     # Step 3
    for candidates in positions.values():                                               # Step 4
        check_position(query, tfaces, candidates, same_surface, copies, part, others, found)


def add_in_place(query, sources, asm, source_paths, found: Results) -> list:
    """Cross-check: where the part already sits in the assembly, report what its selected faces
    touch right now. If the search found the same faces, the two are merged into one match.
    Returns the copies whose selected faces only partly touch something."""
    partial = []
    for occ, m_reg, mapping in sources:
        r = in_place(occ, m_reg, mapping, [q.fid for q in query], asm, face_records, source_paths)
        if r is None:
            continue
        if not r["complete"]:
            partial.append({"source": occ.path, "coverage": r["coverage"]})
            continue
        faces: dict = {}
        pairs = []
        for i, hits in r["hits"].items():
            for h in hits:
                faces.setdefault(h["occ"].path, set()).add(h["t_local"].fid)
                pairs.append({"selected_face": query[i].fid, "target": h["occ"].path,
                              "target_face": h["t_local"].fid, "overlap": round(h["overlap"], 3)})
        m = {"found_by": "in-place", "parts": sorted({h["occ"].name for hits in r["hits"].values() for h in hits}),
             "occurrences": sorted(faces), "faces": {k: sorted(v) for k, v in faces.items()},
             "pairs": pairs, "fit": r["fit"], "clearance": round(float(r["clearance"]), 4),
             "joint": r["joint"], "behaviour": r["behaviour"],
             "free_directions": [list(np.round(f, 4)) for f in r["free_t"]],
             "transform": np.asarray(r["m_part_world"]).round(6).tolist(), "source": occ.path}
        merged = False
        for mb in list(found.matches.values()):     # the same faces found by the search -> one match
            if set(mb["occurrences"]) <= set(m["occurrences"]) and all(
                    set(mb["faces"][p]) <= set(m["faces"].get(p, [])) for p in mb["occurrences"]):
                mb["found_by"] = "in-place + search"
                mb["in_place"] = {"fit": m["fit"], "clearance": m["clearance"], "joint": m["joint"]}
                merged = True
        if not merged:
            found.matches[("inplace", occ.path)] = m
    return partial


def run_match(part_path: str, assembly_path: str, faces=None, use_green=False, log=print,
              near: str | None = None, margin: float = 150.0, part=None, assembly=None,
              max_clearance: float | None = None) -> dict:
    """Find every place in the assembly where the selected faces of the part fit.
    faces = face IDs (or use_green=True for the faces coloured green in the part file).
    part / assembly: already-loaded objects (the viewer passes them, to skip re-reading files).
    max_clearance: the loosest gap (mm) still reported as a fit, for this run only; the default
    is CLEARANCE_MAX in config.py. A dovetail with room for a gib strip needs 3 mm or so.
    Returns the result as a dict; output.py writes it to files."""
    if max_clearance is None:
        return _run(part_path, assembly_path, faces, use_green, log, near, margin, part, assembly)
    if not 0 < max_clearance <= 20:
        raise ValueError("max clearance must be between 0 and 20 mm")
    saved = C.CLEARANCE_MAX, C.INPLACE_GAP
    C.CLEARANCE_MAX, C.INPLACE_GAP = float(max_clearance), max(C.INPLACE_GAP, float(max_clearance))
    try:
        return _run(part_path, assembly_path, faces, use_green, log, near, margin, part, assembly)
    finally:
        C.CLEARANCE_MAX, C.INPLACE_GAP = saved


def _run(part_path, assembly_path, faces, use_green, log, near, margin, part, assembly) -> dict:
    t0 = time.time()
    clear_caches()
    _BOX.clear()
    part, sel, asm = load_inputs(part_path, assembly_path, faces, use_green, near, margin, part, assembly, log)
    log(f"loaded: part {len(part.faces)} faces, selection {sel}; assembly {len(asm.occurrences)} parts "
        f"({time.time() - t0:.1f}s)")

    # Step 1: describe the selected faces
    pdescs = describe_all(part.faces)
    query, warnings = describe_selection(part, pdescs, sel)
    free_slides, free_turns = selection_dof(query)
    log("selected faces: " + "; ".join(q.short() for q in query))

    # copies of the part that already sit in the assembly are not searched and are not obstacles
    sources = find_sources(part, pdescs, asm, face_records)
    source_paths = {o.path for o, _, _ in sources}
    source_shapes = {o.definition.key for o, _, _ in sources}
    log(f"source part found in assembly: {sorted(source_paths) or 'no'}")
    others, more = obstacles(part, asm, source_paths)
    warnings += more

    # Steps 2 to 4, once per part shape
    found = Results()
    if free_slides < 2:
        by_shape: dict = {}
        for o in asm.occurrences:
            if o.definition.key not in source_shapes:
                by_shape.setdefault(o.definition.key, []).append(o)
        for copies in by_shape.values():
            search_shape(query, copies, part, others, found, warnings)
        log(f"search done: {len(found.matches)} matches ({time.time() - t0:.1f}s)")

    # cross-check with where the part sits now
    partial = add_in_place(query, sources, asm, source_paths, found)

    matches = sorted(found.matches.values(), key=lambda m: (m["parts"], m["occurrences"]))
    for i, m in enumerate(matches, start=1):
        m["id"] = i
    return {"part": part_path, "assembly": assembly_path, "selected_faces": sel,
            "max_clearance_mm": C.CLEARANCE_MAX,
            "selection_dof": {"translations": free_slides, "rotations": free_turns},
            "source_occurrences": sorted(source_paths), "status": "OK" if matches else "NO_FIT",
            "matches": matches, "groups": group_matches(matches),
            "near_misses": collapse_near_misses(found.near, found.matches), "partial_in_place": partial,
            "warnings": warnings + asm.warnings, "seconds": round(time.time() - t0, 1)}
