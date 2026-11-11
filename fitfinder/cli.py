"""Command line.

  python -m fitfinder match --part P.step --assembly A.step (--faces 7,8,9,10 | --green) --out out/x
  python -m fitfinder serve --root ~/cad        # browser viewer at http://127.0.0.1:8765
  python -m fitfinder extract ASM.step --name key --out key.step   # one part out of an assembly
  python -m fitfinder inspect FILE.step          # parts, placements, face counts
  python -m fitfinder faces FILE.step            # describe every face of a part
"""
from __future__ import annotations

import argparse
import os
import sys


def _vec(v, nd=3):
    return "(" + ", ".join(f"{0.0 if abs(x) < 5e-4 else x:.{nd}g}" for x in v) + ")"


def print_faces(path: str, kind: str | None = None, ats: list | None = None):
    """The part's faces as a table you can match against your CAD model:
    ID (= FreeCAD FaceN), green?, type, size, direction, centre point, area."""
    import numpy as np
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.gp import gp_Pnt

    from .faces import describe_all
    from .loader import load_part, shape_bbox
    part = load_part(path)
    descs = describe_all(part.faces)
    bb = shape_bbox(part.shape)
    print(f"{path}: {len(descs)} faces; part spans x {bb[0]:.1f}..{bb[3]:.1f}, y {bb[1]:.1f}..{bb[4]:.1f}, "
          f"z {bb[2]:.1f}..{bb[5]:.1f} mm (its own coordinates)")
    if ats:
        hits = []
        for at in ats:
            p = [float(x) for x in at.split(",")]
            v = BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex()
            dist = []
            for d in descs:
                e = BRepExtrema_DistShapeShape(v, d.face)
                dist.append(e.Value() if e.IsDone() else 1e9)
            k = int(np.argmin(dist))
            hits.append(descs[k].fid)
            note = "" if dist[k] < 0.01 else f"   (nearest face, {dist[k]:.3f} mm away: check the point)"
            print(f"  point ({at}) -> face {descs[k].fid}: {descs[k].short()}{note}")
        print(f"\n  use:  --faces {','.join(str(h) for h in dict.fromkeys(hits))}")
        return
    print(f"\n  {'ID':>4} {'green':5} {'type':9} {'size':22} {'direction':24} {'centre (x, y, z)':28} {'area mm2':>9}")
    for d in descs:
        if kind and d.kind != kind:
            continue
        if d.kind == "plane":
            size, direc = "-", "normal " + _vec(d.n)
        elif d.kind == "cylinder":
            size, direc = f"r={d.r:.4g} (dia {2 * d.r:.4g}) {'shaft' if d.convex else 'bore'}", "axis " + _vec(d.a)
        elif d.kind == "cone":
            size = f"half-angle {np.degrees(d.half):.3g} deg {'outer' if d.convex else 'inner'}"
            direc = "axis " + _vec(d.a)
        elif d.kind == "sphere":
            size, direc = f"r={d.r:.4g} {'ball' if d.convex else 'socket'}", "centre " + _vec(d.c, 4)
        else:
            size, direc = "freeform (not matched)", "-"
        print(f"  {d.fid:>4} {'yes' if d.fid in part.green_ids else '':5} {d.kind:9} {size:22} {direc:24} "
              f"{_vec(d.center, 4):28} {d.area:9.1f}")
    if part.green_ids:
        print(f"\n  green faces: {sorted(part.green_ids)}   (use --green, or --faces {','.join(map(str, sorted(part.green_ids)))})")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="fitfinder")
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("match", help="find every place the selected faces fit")
    m.add_argument("--part", required=True)
    m.add_argument("--assembly", required=True)
    g = m.add_mutually_exclusive_group(required=True)
    g.add_argument("--faces", help="comma separated face IDs on the part, e.g. 7,8,9,10")
    g.add_argument("--green", action="store_true", help="use the part's green faces")
    m.add_argument("--out", default="out/run")
    m.add_argument("--no-render", action="store_true")
    m.add_argument("--near", help="only search parts near the part(s) whose name contains this text")
    m.add_argument("--margin", type=float, default=150.0)
    m.add_argument("--max-clearance", type=float, metavar="MM",
                   help="loosest gap still reported as a fit (default 1.0 mm)")
    s = sub.add_parser("serve", help="browser viewer: pick faces, run, inspect results")
    s.add_argument("--root", default=".", help="folder with your STEP files (only files under it are served)")
    s.add_argument("--port", type=int, default=8765)
    s.add_argument("--part", help="pre-fill: part path relative to --root")
    s.add_argument("--assembly", help="pre-fill: assembly path relative to --root")
    s.add_argument("--no-browser", action="store_true", help="do not try to open a browser tab")
    e = sub.add_parser("extract", help="save one part of an assembly as its own STEP file")
    e.add_argument("assembly")
    e.add_argument("--name", required=True, help="part name (or part of it), as shown by 'inspect'")
    e.add_argument("--out", required=True, help="output .step file")
    i = sub.add_parser("inspect")
    i.add_argument("file")
    f = sub.add_parser("faces", help="table of the part's faces: ID, type, size, direction, position")
    f.add_argument("file")
    f.add_argument("--kind", help="only this type: plane, cylinder, cone, sphere, other")
    f.add_argument("--at", action="append", default=[], metavar="X,Y,Z",
                   help="which face passes through this point (part coordinates, mm); repeatable. Write --at=-5,0,20 when the first number is negative")
    a = ap.parse_args(argv)

    if a.cmd == "serve":
        from .web.server import serve
        serve(a.root, a.port, a.part, a.assembly, open_browser=not a.no_browser)
        return 0
    if a.cmd == "extract":
        from .loader import extract_part
        path = extract_part(a.assembly, a.name, a.out)
        print(f"saved {path} -> {a.out}")
        return 0
    if a.cmd == "inspect":
        from .loader import _print_assembly
        _print_assembly(a.file)
        return 0
    if a.cmd == "faces":
        print_faces(a.file, a.kind, a.at)
        return 0

    from .match import run_match
    from .output import render, summary_lines, write_csv, write_json
    os.makedirs(a.out, exist_ok=True)
    faces = [int(x) for x in a.faces.split(",")] if a.faces else None
    try:
        res = run_match(a.part, a.assembly, faces=faces, use_green=a.green, near=a.near, margin=a.margin,
                        max_clearance=a.max_clearance)
    except (ValueError, FileNotFoundError) as e:
        print("ERROR:", e)
        return 3
    write_json(res, a.out)
    write_csv(res, a.out)
    lines = summary_lines(res)
    open(os.path.join(a.out, "summary.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    if not a.no_render:
        try:
            render(res, a.out)
            print(f"renders written to {a.out}/")
        except Exception as e:
            print(f"render skipped ({type(e).__name__}: {e}). On headless Linux run under xvfb-run.")
    return 0 if res["matches"] else 1          # exit code: 0 = fits found, 1 = no fit, 3 = input error


if __name__ == "__main__":
    sys.exit(main())
