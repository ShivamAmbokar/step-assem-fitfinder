"""Second, independent test set: things real CAD exports do that the basic cases (A-I) do not.

  python tools/make_stress.py              # -> tests_stress/S*/  (same layout as tests_out/)
  python tools/run_tests.py --tests tests_stress

Expected answers are the physical truth, written before running the matcher.
"""
from __future__ import annotations

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cadquery as cq
import make_tests as mt
from make_tests import KEY_POINTS, W, loc, make_body, make_key, make_pinion, one_solid, pinion_loc, solid
from OCP.BRepBuilderAPI import BRepBuilderAPI_NurbsConvert

mt.OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests_stress")


def match(part, fit, joint):
    return {"parts": [part], "fit": fit, "joint": joint}


def s1_nested():
    """Two chucks, each a sub-assembly (body + 4 pinions), inside a tilted 'headstock'
    sub-assembly: placements multiply through 3 levels."""
    def chuck(name):
        a = cq.Assembly(name=name)
        a.add(make_body([0, 90, 180, 270]), name="body")
        for i, ang in enumerate([0, 90, 180, 270]):
            a.add(make_pinion(10.0), name=f"pinion_{i + 1}", loc=pinion_loc(ang))
        return a
    head = cq.Assembly(name="headstock")
    head.add(chuck("chuck_A"), loc=loc((0, 0, 0)))
    head.add(chuck("chuck_B"), loc=loc((300, 0, 0), ((0, 1, 0), 90)))
    root = cq.Assembly(name="root")
    root.add(head, loc=loc((50, -80, 20), ((1, 0, 0), 30), ((0, 0, 1), 20)))
    key = make_key()
    root.add(key, name="key", loc=loc((0, 400, 0), ((0, 0, 1), 63)))
    d = os.path.join(mt.OUT, "S1_nested_chucks")
    os.makedirs(d, exist_ok=True)
    root.export(os.path.join(d, "assembly.step"))
    exp = {"count": 8, "matches": [match(f"pinion_{i}", "exact", "prismatic") for i in (1, 2, 3, 4)] * 2,
           "near_misses": []}
    _save_part(d, key, KEY_POINTS, exp, "S1_nested_chucks: 2 chucks in a tilted 3-level assembly -> 8 pinions")


def _save_part(d, shape, points, exp, note):
    import json
    ids = mt.write_part_green(os.path.join(d, "part.step"), shape, points)
    json.dump({"faces": ids}, open(os.path.join(d, "selection.json"), "w"), indent=1)
    json.dump(exp, open(os.path.join(d, "expected.json"), "w"), indent=1)
    print(f"{note}  selected {ids}")


def s2_chamfered():
    """Chamfered key tip + chamfered socket mouths: the mating flats are trimmed, not full rectangles."""
    drive = W().box(10, 10, 40, centered=(True, True, False)).faces("<Z").edges().chamfer(0.5)
    handle = W().box(80, 12, 12, centered=(True, True, False)).translate((0, 0, 40))
    key = solid(drive.union(handle))

    def pinion():
        p = W().circle(10).extrude(30).faces(">Z").workplane().rect(10, 10).cutBlind(-15)
        return solid(p.faces(">Z").edges("%LINE").chamfer(1.0))
    parts = [("body", make_body([0, 120, 240]), loc())]
    for i, a in enumerate([0, 120, 240]):
        parts.append((f"pinion_{i + 1}", pinion(), pinion_loc(a)))
    parts.append(("key", key, loc((0, 0, 200), ((1, 0, 0), 180))))
    exp = {"count": 3, "matches": [match(f"pinion_{i}", "exact", "prismatic") for i in (1, 2, 3)], "near_misses": []}
    mt.save_case("S2_chamfered", parts, key, KEY_POINTS, exp, "chamfered key tip and socket mouths")


def s3_split_bore():
    """A bore whose cylindrical surface is stored as TWO half-cylinder faces (common in exports
    from some CAD systems). Engineering truth: the pin fits both plates."""
    pin = one_solid(cq.Solid.makeCylinder(4, 30).fuse(cq.Solid.makeCylinder(7, 3, cq.Vector(0, 0, 30))).clean())
    plate = cq.Solid.makeBox(40, 40, 10, cq.Vector(-20, -20, 0))
    h1 = cq.Solid.makeCylinder(4, 12, cq.Vector(0, 0, -1), cq.Vector(0, 0, 1), 180)
    h2 = h1.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180)
    tool = h1.fuse(h2)                       # no clean(): the hole keeps two half faces
    split = one_solid(plate.cut(tool))
    whole = one_solid(plate.cut(cq.Solid.makeCylinder(4, 12, cq.Vector(0, 0, -1))))
    print(f"  S3: split plate has {len(split.Faces())} faces, whole plate {len(whole.Faces())}")
    parts = [("plate_split", split, loc((0, 0, 0))), ("plate_whole", whole, loc((80, 0, 0), ((1, 0, 0), 90))),
             ("pin", pin, loc((0, 100, 0)))]
    exp = {"count": 2, "matches": [match("plate_split", "exact", "cylindrical"),
                                   match("plate_whole", "exact", "cylindrical")], "near_misses": []}
    mt.save_case("S3_split_bore", parts, pin, [(4, 0, 15)], exp, "bore split into two half faces")


N_TIP = (0.3, 0.4, 0.866)        # compound-angle tip: makes the key chiral (mirror image != rotation)


def _sloped_drive():
    drive = cq.Solid.makeBox(10, 10, 40, cq.Vector(-5, -5, 0))
    n = cq.Vector(*N_TIP).normalized()
    z = cq.Vector(0, 0, 1)
    axis, ang = z.cross(n), math.degrees(math.acos(z.dot(n)))
    half = cq.Solid.makeBox(200, 200, 200, cq.Vector(-100, -100, 0)).rotate(cq.Vector(), axis, ang)
    return one_solid(drive.intersect(half.translate(cq.Vector(0, 0, 6))))


def s4_mirror():
    drive = _sloped_drive()
    key = one_solid(drive.fuse(cq.Solid.makeBox(80, 12, 12, cq.Vector(-40, -6, 40))).clean())

    def holder(mirror):
        tool = drive.translate(cq.Vector(0, 0, 14))
        if mirror:
            tool = tool.mirror("YZ")
        return one_solid(cq.Solid.makeCylinder(12, 30).cut(tool))
    parts = [("holder_1", holder(False), loc((0, 0, 0))),
             ("holder_2", holder(False), loc((100, 0, 0), ((1, 1, 0), 50), ((0, 0, 1), 15))),
             ("holder_mirror", holder(True), loc((-100, 0, 0))),
             ("key", key, loc((0, 150, 0)))]
    pts = [(5, 0, 25), (-5, 0, 25), (0, 5, 25), (0, -5, 25), (0, 0, 6)]
    exp = {"count": 2, "matches": [match("holder_1", "exact", "fixed"), match("holder_2", "exact", "fixed")],
           "near_misses": ["holder_mirror"]}
    mt.save_case("S4_mirror", parts, key, pts, exp, "chiral key: 2 real sockets + 1 mirror-image socket")


def s5_tolerances():
    """Pin d=10 in holes right at the classification limits (exact <=0.02, clearance <=1.0,
    press fit >= -0.05)."""
    pin = one_solid(cq.Solid.makeCylinder(5, 30).fuse(cq.Solid.makeCylinder(8, 3, cq.Vector(0, 0, 30))).clean())
    holes = [10.015, 10.6, 10.98, 11.04, 9.96, 9.93]
    p = W().box(200, 40, 10, centered=(True, True, False))
    for i, d in enumerate(holes):
        p = p.faces(">Z").workplane().pushPoints([(-80 + 32 * i, 0)]).hole(d)
    parts = [("plate", solid(p), loc((0, 0, 0), ((0, 0, 1), 10))), ("pin", pin, loc((0, 100, 0)))]
    exp = {"count": 4, "matches": [match("plate", "exact", "cylindrical"), match("plate", "clearance", "cylindrical"),
                                   match("plate", "clearance", "cylindrical"),
                                   match("plate", "interference", "cylindrical")], "near_misses": []}
    mt.save_case("S5_tolerance_edges", parts, pin, [(5, 0, 15)], exp,
                 "holes +0.015 exact, +0.6/+0.98 clearance, +1.04 too loose, -0.04 press, -0.07 too small")


def s6_long_bed():
    """1500 mm bed with two keyway pockets 6 wide x 3.5 deep: one exactly the key length (20),
    one 30 long. Parallel key 6x6x20, selected: both sides + bottom (ends not selected)."""
    bed = W().box(1500, 100, 80, centered=(True, True, False))
    bed = bed.faces(">Z").workplane().pushPoints([(620, 0)]).rect(20, 6).cutBlind(-3.5)
    bed = bed.faces(">Z").workplane().pushPoints([(-300, 20)]).rect(30, 6).cutBlind(-3.5)
    key = solid(W().box(20, 6, 6, centered=(True, True, False)))
    parts = [("bed", solid(bed), loc((0, 0, 0), ((0, 0, 1), 5))), ("key", key, loc((0, 300, 0)))]
    exp = {"count": 2, "matches": [match("bed", "exact", "prismatic")] * 2, "near_misses": []}
    mt.save_case("S6_long_bed", parts, key, [(0, 3, 3), (0, -3, 3), (0, 0, 0)], exp,
                 "1.5 m bed: 20 mm pocket (exact length) + 30 mm pocket")


def s7_many():
    """36 bushings (bore 8.0) in random orientations: every instance must be found."""
    rnd = random.Random(7)
    pin = one_solid(cq.Solid.makeCylinder(4, 30).fuse(cq.Solid.makeCylinder(7, 3, cq.Vector(0, 0, 30))).clean())
    bush = one_solid(cq.Solid.makeCylinder(8, 10).cut(cq.Solid.makeCylinder(4, 12, cq.Vector(0, 0, -1))))
    parts = []
    for i in range(36):
        ax = (rnd.uniform(-1, 1), rnd.uniform(-1, 1), rnd.uniform(-1, 1))
        parts.append((f"bush_{i + 1}", bush, loc((60 * (i % 6), 60 * (i // 6), 0), (ax, rnd.uniform(0, 360)))))
    parts.append(("pin", pin, loc((-100, 0, 0))))
    exp = {"count": 36, "matches": [match(f"bush_{i + 1}", "exact", "cylindrical") for i in range(36)],
           "near_misses": []}
    mt.save_case("S7_many_instances", parts, pin, [(4, 0, 15)], exp, "36 bushings, random orientations")


def _nurbs(shape):
    return cq.Shape.cast(BRepBuilderAPI_NurbsConvert(shape.wrapped, True).Shape())


def s8_bspline():
    """Bushings whose faces are B-spline surfaces (as some exporters write them)."""
    pin = one_solid(cq.Solid.makeCylinder(4, 30).fuse(cq.Solid.makeCylinder(7, 3, cq.Vector(0, 0, 30))).clean())

    def bush(r):
        return one_solid(cq.Solid.makeCylinder(8, 10).cut(cq.Solid.makeCylinder(r, 12, cq.Vector(0, 0, -1))))
    parts = [("bush_nurbs", _nurbs(bush(4.0)), loc((0, 0, 0), ((1, 0, 0), 40))),
             ("bush_plain", bush(4.0), loc((60, 0, 0))),
             ("bush_nurbs_loose", _nurbs(bush(4.3)), loc((120, 0, 0), ((0, 1, 0), 70))),
             ("pin", pin, loc((0, 100, 0)))]
    exp = {"count": 3, "matches": [match("bush_nurbs", "exact", "cylindrical"), match("bush_plain", "exact", "cylindrical"),
                                   match("bush_nurbs_loose", "clearance", "cylindrical")], "near_misses": []}
    mt.save_case("S8_bspline", parts, pin, [(4, 0, 15)], exp, "B-spline bushings (exact, exact, +0.6)")


def _trap(w0, w1, h, length, x0=0.0):
    """Trapezoid (width w0 at z=0, w1 at z=h) extruded along +X from x0."""
    pts = [(-w0 / 2, 0), (w0 / 2, 0), (w1 / 2, h), (-w1 / 2, h)]
    return solid(W("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(length))


def s9_dovetail():
    """60 deg dovetail: slide block (groove) on rails in odd orientations; a 45 deg rail as decoy."""
    h, w0 = 8.0, 20.0
    w60 = w0 + 2 * h / math.tan(math.radians(60))
    w45 = w0 + 2 * h / math.tan(math.radians(45))
    block = one_solid(cq.Solid.makeBox(60, 50, 25, cq.Vector(0, -25, 0)).cut(_trap(w0, w60, h, 62, -1)))

    def rail(w1):
        base = cq.Solid.makeBox(200, 60, 20, cq.Vector(0, -30, -20))
        return one_solid(base.fuse(_trap(w0, w1, h, 200)).clean())
    parts = [("rail_1", rail(w60), loc((0, 0, 0))),
             ("rail_2", rail(w60), loc((0, 200, 50), ((1, 0, 0), 33), ((0, 0, 1), 71))),
             ("rail_45", rail(w45), loc((0, -200, 0))),
             ("block", block, loc((300, 0, 100)))]
    yf = (w0 / 2 + w60 / 2) / 2
    exp = {"count": 2, "matches": [match("rail_1", "exact", "prismatic"), match("rail_2", "exact", "prismatic")],
           "near_misses": []}
    mt.save_case("S9_dovetail60", parts, block, [(30, yf, h / 2), (30, -yf, h / 2), (30, 0, h)], exp,
                 "60 deg dovetail on 2 rails (one at a compound angle), 45 deg decoy")


def s10_pinion_seat():
    """Pinion cylinder + bottom: all pocket floors are coplanar, so only the real-face overlap
    check can tell each bore's own floor from the others. Expected: the 4 free pockets."""
    parts = [("body", make_body([0, 90, 180, 270, 45]), loc())]
    for i, ang in enumerate([0, 90, 180, 270]):
        parts.append((f"pinion_{i + 1}", make_pinion(10.0), pinion_loc(ang)))
    parts.append(("pinion_wide", make_pinion(12.0), pinion_loc(45)))
    pin = make_pinion(10.0)
    exp = {"count": 4, "matches": [match("body", "exact", "revolute")] * 4, "near_misses": []}
    mt.save_case("S10_pinion_seat", parts, pin, [(10, 0, 15), (5, 0, 0)], exp,
                 "pinion cylinder + bottom face: 4 pockets, not 4 x 5 bore/floor combinations")


if __name__ == "__main__":
    only = sys.argv[1:]
    for fn in (s1_nested, s2_chamfered, s3_split_bore, s4_mirror, s5_tolerances, s6_long_bed, s7_many,
               s8_bspline, s9_dovetail, s10_pinion_seat):
        if not only or any(fn.__name__.startswith(o.lower()) for o in only):
            fn()
