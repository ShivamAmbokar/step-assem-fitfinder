"""Step-by-step walkthrough of ONE search, with explanations. Read this file top to bottom
alongside its printout: each STEP calls exactly one function from fitfinder.

  python tools/walkthrough.py                                   # bolted cover example
  python tools/walkthrough.py --case A_chuck_key --target pinion_1
  python tools/walkthrough.py --part my_key.step --assembly my_lathe.step --faces 7,8,9,10 --target chuck

This is the same work run_match() does for EVERY part; here it is done for one target part only,
and every intermediate result is printed.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fitfinder import config as C  # noqa: E402
from fitfinder.faces import describe_all  # noqa: E402
from fitfinder.fit import behaviour, classify  # noqa: E402
from fitfinder.inplace import find_sources  # noqa: E402
from fitfinder.loader import load_assembly, load_part, shape_bbox  # noqa: E402
from fitfinder.match import size_constrained  # noqa: E402
from fitfinder.pose import selection_dof, solve  # noqa: E402
from fitfinder.search import candidates, search, strict_check  # noqa: E402
from fitfinder.verify import place  # noqa: E402

np.set_printoptions(precision=3, suppress=True)


def step(n, title, explain):
    print(f"\n{'=' * 78}\nSTEP {n}: {title}\n{'-' * 78}")
    for line in explain.strip().splitlines():
        print("  " + line.strip())
    print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="I_cover_bolts", help="a folder in tests_out/ or tests_stress/")
    ap.add_argument("--part")
    ap.add_argument("--assembly")
    ap.add_argument("--faces", help="e.g. 7,8,9,10 (default: green faces / selection.json)")
    ap.add_argument("--target", help="name (or part of it) of the assembly part to try (default: first match)")
    a = ap.parse_args()
    case_dir = next((os.path.join(t, a.case) for t in ("tests_out", "tests_stress")
                     if os.path.isdir(os.path.join(t, a.case))), os.path.join("tests_out", a.case))
    part_path = a.part or os.path.join(case_dir, "part.step")
    asm_path = a.assembly or os.path.join(case_dir, "assembly.step")

    # ---------------------------------------------------------------------------------------------
    step(1, "read the part and pick the selected faces  (loader.load_part, faces.describe_all)", """
        Every face gets a number (1, 2, 3 ... same as FreeCAD Face1, Face2 ...).
        describe_all() turns each face into a small record: its type (plane / cylinder / cone /
        sphere) and the numbers that define it: normal pointing OUT of the material, radius,
        axis, and whether a round face is convex (a shaft) or concave (a bore).""")
    part = load_part(part_path)
    pdescs = describe_all(part.faces)
    if a.faces:
        sel = [int(x) for x in a.faces.split(",")]
    elif part.green_ids:
        sel = sorted(part.green_ids)
    else:
        sel = json.load(open(os.path.join(case_dir, "selection.json")))["faces"]
    for d in pdescs:
        print(("  >> " if d.fid in sel else "     ") + d.short())
    query = [pdescs[f - 1] for f in sel]
    print(f"\n  selected (>>): {sel}")

    # ---------------------------------------------------------------------------------------------
    step(2, "read the assembly  (loader.load_assembly)", """
        A STEP assembly stores each part shape ONCE (a 'definition') and places it one or more
        times (an 'occurrence' = definition + 4x4 position matrix). So 4 identical pinions are one
        definition, searched once, and 4 occurrences, each checked in its own place.""")
    asm = load_assembly(asm_path)
    for o in asm.occurrences:
        print(f"     {o.path:<28} definition={o.definition.name:<14} faces={len(o.definition.faces):<4}"
              f" position={o.world[:3, 3]}")

    # ---------------------------------------------------------------------------------------------
    step(3, "what does the selection itself allow?  (pose.selection_dof, match.size_constrained)", """
        Count how the selected faces could move if they were glued to a matching set:
          translations: a flat fixes 1 direction, a cylinder 2, a sphere or cone 3.
          rotations:    a cylinder alone can turn about its axis, a sphere about anything.
        2+ free translations (e.g. one flat face) = the selection fits almost anywhere: the
        global search is skipped. No round face and no pair of opposite flats = no WIDTH is fixed
        (a 10 mm key corner also fits a 12 mm socket corner) -> warning.""")
    nT, nR = selection_dof(query)
    print(f"  free translations: {nT}   free rotations: {nR}")
    print(f"  selection fixes a size: {size_constrained(query)}")

    # ---------------------------------------------------------------------------------------------
    step(4, "is the part already in the assembly?  (inplace.find_sources)", """
        Look for a part in the assembly with the same face count and volume as our part, and work
        out the transform between them. If found, it is the 'source': it is skipped as a target,
        and its current neighbours are reported too (found_by = in-place).""")
    cache = {}

    def desc_of(d):
        if d.key not in cache:
            cache[d.key] = describe_all(d.faces)
        return cache[d.key]
    sources = find_sources(part, pdescs, asm, desc_of)
    src_paths = {o.path for o, _, _ in sources}
    print(f"  found in assembly as: {sorted(src_paths) or 'no'}")

    # ---------------------------------------------------------------------------------------------
    want = a.target
    exp_file = os.path.join(case_dir, "expected.json")
    if not want and not a.part and os.path.exists(exp_file):     # test case: follow the expected answer
        exp = json.load(open(exp_file))["matches"]
        want = exp[0]["parts"][0] if exp else None
    targets = [o for o in asm.occurrences if o.path not in src_paths
               and (not want or want.lower() in o.path.lower())]
    if not targets:
        print("no target part with that name"); return
    occ = targets[0]
    tdescs = desc_of(occ.definition)
    step(5, f"which faces of '{occ.path}' could mate with each selected face?  (search.candidates)", """
        A face can only mate with a face of the same type that 'faces it': a shaft with a bore of
        (almost) the same radius, a flat with a flat. The number after each candidate is the
        clearance in mm (None for flats: their gap is only known once two faces are paired).""")
    for q, cs in zip(query, candidates(query, tdescs)):
        print(f"  selected #{q.fid} {q.kind:<9} -> " + (", ".join(
            f"#{tdescs[k].fid}" + ("" if c is None else f" ({c:+.3f})") for k, c in cs) or "nothing"))

    # ---------------------------------------------------------------------------------------------
    step(6, "match the faces AS A GROUP  (search.search)", """
        Try every way of giving each selected face its own target face, keeping only those where
        the RELATIONS between faces agree too: same angles between normals/axes, same distances
        between parallel flats, same axis-to-flat heights. This is what stops single-face false
        positives. These relations do not change when a part is moved or turned, so the target
        can sit anywhere in the assembly.""")
    assigns = search(query, tdescs)
    print(f"  {len(assigns)} group assignment(s)")
    for asg in assigns[:8]:
        print("    " + ", ".join(f"#{q.fid}->#{tdescs[k].fid}" for q, k in zip(query, asg)))
    if not assigns:
        print("  -> no group match on this part. Try another --target."); return

    # ---------------------------------------------------------------------------------------------
    step(7, "strict size check  (search.strict_check)", f"""
        Now with the real limits: every clearance must be between -{C.INTERFERENCE_MAX} mm
        (press fit) and +{C.CLEARANCE_MAX} mm (loose running fit).""")
    ok = []
    for asg in assigns:
        good, clear, why, worst = strict_check(query, tdescs, asg)
        print(f"    {[tdescs[k].fid for k in asg]}: {'OK' if good else 'REJECT (' + why + ')'}  "
              f"clearances={np.round(clear, 3) if clear else []}")
        if good:
            ok.append((asg, clear))
    if not ok:
        return

    # ---------------------------------------------------------------------------------------------
    step(8, "where must the part go?  (pose.solve)  +  STEP 9: is it real in 3-D?  (verify.place)", """
        pose.solve - Rotation: turn the part so each selected normal points straight AT its target
        face (Kabsch / SVD, mirror images refused). Translation: least squares, each flat gives 1
        equation, each cylinder 2. Directions left free = how the fitted part can still move:
        that names the joint (prismatic = slides, revolute = turns, cylindrical = both ...).
        verify.place - the equations use endless planes and cylinders; now use the REAL faces:
          overlap  - do the selected faces actually lie ON their target faces (not beside them)?
          slide    - along a free direction, only where the faces overlap;
          collide  - does the placed part cut into ANY other part? (quick corner test, then an
                     exact boolean intersection); turn it about free axes if it does;
          refine   - push it as deep as it goes without colliding (key tip on the socket floor).
        Each group assignment from step 7 is tried until one survives.""")
    others = []
    for o in asm.occurrences:
        if o.path in src_paths:
            continue
        ws = o.world_shape()
        others.append((o, ws, shape_bbox(ws)))
    diag = lambda b: float(np.linalg.norm(b[3:] - b[:3]))  # noqa: E731
    res = None
    for asg, clear in ok:
        target = [tdescs[k] for k in asg]
        for pose, why in solve(query, target):
            label = f"    {[t.fid for t in target]}: "
            if pose is None:
                print(label + f"no pose ({why})")
                continue
            lo = min(clear) if clear else 0.0
            allowance = 1.5 * sum(min(q.area, t.area) for q, t in zip(query, target)) * (-lo) / 2 if lo < 0 else 0.0
            r = place(query, target, pose, occ.world, part.shape, diag(shape_bbox(part.shape)),
                      diag(shape_bbox(occ.definition.shape)), others, allowance, C.CLEARANCE_MAX + C.GEOM_TOL)
            print(label + ("PLACED" if r["ok"] else f"rejected - {r['reason']}"))
            if r["ok"]:
                res, ok_clear = r, clear
                break
        if res:
            break
    if res is None:
        print("\n  nothing survived on this part -> it would be listed as a near-miss"); return
    print(f"\n  rotation R=\n{pose.R}\n  joint={pose.joint}  free slide directions={pose.free_t}  "
          f"free turn axes={len(pose.free_r)}")
    print(f"  overlap per face pair: {np.round(res['overlaps'], 2)}  (1.0 = face fully covered)")
    print(f"  part -> world transform:\n{occ.world @ res['m_local']}")

    # ---------------------------------------------------------------------------------------------
    step(10, "what kind of fit?  (fit.classify, fit.behaviour)", f"""
        exact: every clearance within +/-{C.EXACT_GAP} mm;  clearance: up to +{C.CLEARANCE_MAX} mm;
        interference: down to -{C.INTERFERENCE_MAX} mm (press fit).""")
    fit, val = classify(ok_clear)
    print(f"  {fit} ({val:+.3f} mm), {pose.joint} -> {behaviour(fit, pose.joint)}")
    print("\nDone. run_match() repeats steps 5-10 for every part definition and every placement,")
    print("then groups the results per part. Compare: python -m fitfinder match ... (same files).")


if __name__ == "__main__":
    main()
