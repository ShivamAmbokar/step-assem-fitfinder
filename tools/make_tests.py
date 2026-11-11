"""Build synthetic test assemblies with KNOWN answers (cadquery).

Writes tests_out/<case>/{assembly.step, part.step (selected faces green), selection.json,
expected.json}. Only run_tests.py reads expected.json.
"""
from __future__ import annotations

import json
import math
import os
import sys

import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_DocumentTool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from fitfinder.loader import face_list

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests_out")
W = cq.Workplane


def loc(t=(0, 0, 0), *rots):
    """Translate(t) * R1 * R2 ... ; each rot = (axis, degrees). Rightmost applied first."""
    out = cq.Location(cq.Vector(*t))
    for axis, deg in rots:
        out = out * cq.Location(cq.Vector(0, 0, 0), cq.Vector(*axis), deg)
    return out


def write_assembly(path: str, parts: list):
    """parts = [(name, workplane_or_shape, cq.Location)]"""
    a = cq.Assembly(name="root")
    for name, shape, lc in parts:
        a.add(shape, name=name, loc=lc)
    a.export(path)


def write_part_green(path: str, shape: cq.Shape, green_points: list) -> list:
    """Write a single part; colour the faces containing green_points green. Returns face IDs."""
    if not shape.wrapped.Location().IsIdentity():
        # bake a placement stored ON the solid (e.g. cadquery box(centered=...)) into the geometry:
        # XCAF cannot find faces of a located shape to colour them
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.TopLoc import TopLoc_Location
        trsf = shape.wrapped.Location().Transformation()
        shape = cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped.Located(TopLoc_Location()), trsf, True).Shape())
    ids = face_ids_at(shape, green_points)
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("XmlXCAF"))
    app.InitDocument(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    label = st.AddShape(shape.wrapped, False)
    TDataStd_Name.Set_s(label, TCollection_ExtendedString("part"))
    faces = face_list(shape.wrapped)
    green = Quantity_Color(0.0, 1.0, 0.0, Quantity_TOC_RGB)
    for fid in ids:
        sub = st.AddSubShape(label, faces[fid - 1])
        ct.SetColor(sub, green, XCAFDoc_ColorSurf)
    w = STEPCAFControl_Writer()
    w.SetColorMode(True)
    w.SetNameMode(True)
    w.Transfer(doc, STEPControl_AsIs)
    w.Write(path)
    return ids


def face_ids_at(shape: cq.Shape, points: list) -> list:
    """Face ID (1-based, MapShapes order) of the face passing through each point."""
    faces = face_list(shape.wrapped)
    ids = []
    for p in points:
        v = BRepBuilderAPI_MakeVertex(gp_Pnt(*p)).Vertex()
        best, best_d = None, 1e9
        for i, f in enumerate(faces, start=1):
            d = BRepExtrema_DistShapeShape(v, f)
            if d.IsDone() and d.Value() < best_d:
                best, best_d = i, d.Value()
        if best_d > 1e-4:
            raise RuntimeError(f"no face at {p} (closest {best_d:.4f})")
        ids.append(best)
    return ids


def save_case(name: str, parts: list, source_shape: cq.Shape, sel_points: list, expected: dict,
              note: str):
    d = os.path.join(OUT, name)
    os.makedirs(d, exist_ok=True)
    write_assembly(os.path.join(d, "assembly.step"), parts)
    ids = write_part_green(os.path.join(d, "part.step"), source_shape, sel_points)
    json.dump({"faces": ids}, open(os.path.join(d, "selection.json"), "w"), indent=1)
    json.dump(expected, open(os.path.join(d, "expected.json"), "w"), indent=1)
    print(f"{name}: assembly {len(parts)} parts, source part selected faces {ids}  - {note}")


def solid(wp) -> cq.Shape:
    return wp.val() if isinstance(wp, cq.Workplane) else wp


def make_key():
    drive = W().box(10, 10, 40, centered=(True, True, False))                 # z 0..40
    handle = W().box(80, 12, 12, centered=(True, True, False)).translate((0, 0, 40))
    return solid(drive.union(handle))


KEY_POINTS = [(5, 0, 20), (-5, 0, 20), (0, 5, 20), (0, -5, 20)]


def make_pinion(w):
    p = W().circle(10).extrude(30)
    p = p.faces(">Z").workplane().rect(w, w).cutBlind(-15)                    # socket z 15..30
    return solid(p)


def make_body(angles):
    pts = [(50 * math.cos(math.radians(a)), 50 * math.sin(math.radians(a))) for a in angles]
    b = W().circle(80).extrude(60)
    b = b.faces(">Z").workplane().pushPoints(pts).hole(20, 30)                # pockets z 30..60
    return solid(b)


def pinion_loc(angle):
    a = math.radians(angle)
    return loc((50 * math.cos(a), 50 * math.sin(a), 30), ((0, 0, 1), angle))


def case_a():
    key = make_key()
    angles = [0, 90, 180, 270]
    parts = [("body", make_body(angles + [45]), loc())]
    for i, a in enumerate(angles):
        parts.append((f"pinion_{i + 1}", make_pinion(10.0), pinion_loc(a)))
    parts.append(("pinion_wide", make_pinion(12.0), pinion_loc(45)))
    parts.append(("key", key, loc((0, 0, 150), ((0, 0, 1), 17))))
    exp = {"count": 4,
           "matches": [{"parts": [f"pinion_{i + 1}"], "fit": "exact", "joint": "prismatic"} for i in range(4)],
           "near_misses": ["pinion_wide"]}
    save_case("A_chuck_key", parts, key, KEY_POINTS, exp, "4 sockets 10.0 + one 12x12 decoy")


def case_b():
    shaft = solid(W().circle(10).extrude(100).union(
        W().box(6, 5.5, 100, centered=(True, False, False)).translate((0, 8, 0))))   # key y 8..13.5

    def hub(r_bore, key_w, keyway=True):
        h = W().circle(25).extrude(30)
        cut = W().circle(r_bore).extrude(32).translate((0, 0, -1))
        if keyway:
            cut = cut.union(W().box(key_w, 13.5, 32, centered=(True, False, False)).translate((0, 0, -1)))
        return solid(h.cut(cut))

    parts = [
        ("shaft", shaft, loc((0, -200, 0), ((0, 1, 0), 90))),
        ("hub_1", hub(10, 6), loc((200, 0, 0), ((1, 0, 0), 90))),
        ("hub_2", hub(10, 6), loc((-200, 50, 0), ((0, 0, 1), 70), ((0, 1, 0), 30))),
        ("hub_nokey", hub(10, 6, keyway=False), loc((0, 200, 0))),
        ("hub_wide", hub(10, 8), loc((200, 200, 0))),
    ]
    sel = [(0, -10, 50), (3, 11.5, 50), (-3, 11.5, 50), (0, 13.5, 50)]
    exp = {"count": 2,
           "matches": [{"parts": ["hub_1"], "fit": "exact", "joint": "prismatic"},
                       {"parts": ["hub_2"], "fit": "exact", "joint": "prismatic"}],
           "near_misses": ["hub_wide"]}
    save_case("B_keyed_shaft", parts, shaft, sel, exp, "2 keyed hubs + no-key and wide-keyway decoys")


def case_c():
    body = solid(W().box(200, 80, 30, centered=(False, True, False)).union(
        W().box(200, 30, 10, centered=(False, True, False)).translate((0, 0, 30))))
    jaw = W().box(60, 80, 40, centered=(False, True, False)).translate((20, 0, 30))
    jaw = solid(jaw.cut(W().box(62, 30.2, 11.1, centered=(False, True, False)).translate((19, 0, 29))))
    parts = [("vice_body", body, loc()), ("jaw", jaw, loc())]
    sel = [(50, -30, 30), (50, 30, 30), (50, 0, 40.1), (50, -15.1, 35), (50, 15.1, 35)]
    exp = {"count": 1,
           "matches": [{"parts": ["vice_body"], "fit": "clearance", "joint": "prismatic"}],
           "near_misses": []}
    save_case("C_vice", parts, jaw, sel, exp, "jaw sits on rail with 0.1 gaps (in place)")


def case_d():
    def motor(r, flat):
        body = W().box(42, 42, 40, centered=(True, True, False)).translate((0, 0, -40))
        shaft = W().circle(r).extrude(25).translate((0, 0, -1))
        if flat is not None:
            shaft = shaft.intersect(W().box(20, 20, 30, centered=(True, False, False))
                                    .translate((0, flat - 20, -2)))
        return solid(body.union(shaft))

    bore = W().circle(4.025).extrude(22).translate((0, 0, -1)).intersect(
        W().box(20, 20, 30, centered=(True, False, False)).translate((0, 3.55 - 20, -2)))
    pulley = solid(W().circle(20).extrude(20).cut(bore))
    parts = [
        ("motor_1", motor(4, 3.5), loc()),
        ("motor_2", motor(4, 3.5), loc((150, 0, 0), ((1, 0, 0), 90))),
        ("motor_round", motor(4, None), loc((0, 150, 0))),
        ("pulley", pulley, loc((-150, 0, 50), ((0, 1, 0), 60))),
    ]
    sel = [(0, -4.025, 10), (0, 3.55, 10)]
    exp = {"count": 2,
           "matches": [{"parts": ["motor_1"], "fit": "clearance", "joint": "prismatic"},
                       {"parts": ["motor_2"], "fit": "clearance", "joint": "prismatic"}],
           "near_misses": []}
    save_case("D_pulley_motor", parts, pulley, sel, exp, "D-bore 8.05/3.55 on D-shafts 8/3.5, round-shaft decoy")


def case_e():
    key = make_key()
    base = lambda: W().box(40, 40, 20, centered=(True, True, False))
    rect = solid(base().faces(">Z").workplane().rect(10, 12).cutBlind(-15))
    uslot = solid(base().cut(W().box(42, 10, 16, centered=(True, True, False)).translate((0, 0, 5))))
    hole = solid(base().faces(">Z").workplane().hole(10, 15))
    boss = solid(W().box(40, 40, 10, centered=(True, True, False)).union(
        W().box(10, 10, 15, centered=(True, True, False)).translate((0, 0, 10))))
    parts = [("block_rect", rect, loc((0, 0, 0))), ("block_uslot", uslot, loc((100, 0, 0), ((0, 0, 1), 15))),
             ("block_hole", hole, loc((200, 0, 0), ((0, 0, 1), 30))),
             ("block_boss", boss, loc((300, 0, 0), ((0, 0, 1), 45))),
             ("key", key, loc((0, 0, 150), ((0, 0, 1), 17)))]
    exp = {"count": 0, "matches": [], "near_misses": ["block_rect"]}
    save_case("E_clutter", parts, key, KEY_POINTS, exp, "look-alikes only: must find nothing")


def one_solid(shape) -> cq.Shape:
    sols = shape.Solids()
    return sols[0] if len(sols) == 1 else shape


def case_f():
    """Pin (shank r=3) into round holes: exact, clearance and press fit on ONE plate set at an odd
    angle, plus a plate whose hole is far too big."""
    pin = one_solid(cq.Solid.makeCylinder(3, 20).fuse(cq.Solid.makeCylinder(5, 3, cq.Vector(0, 0, 20))).clean())
    plate = solid(W().box(100, 40, 10, centered=(True, True, False)).faces(">Z").workplane()
                  .pushPoints([(-36, 0), (-12, 0)]).hole(6.0)
                  .faces(">Z").workplane().pushPoints([(12, 0)]).hole(6.5)
                  .faces(">Z").workplane().pushPoints([(36, 0)]).hole(5.97))
    plate_big = solid(W().box(30, 30, 10, centered=(True, True, False)).faces(">Z").workplane().hole(8.0))
    parts = [("plate", plate, loc((0, 0, 0), ((0, 0, 1), 20), ((1, 0, 0), 30))),
             ("plate_big", plate_big, loc((0, 120, 0))),
             ("pin", pin, loc((0, -120, 40), ((0, 1, 0), 90)))]
    exp = {"count": 4, "matches": [
        {"parts": ["plate"], "fit": "exact", "joint": "cylindrical"},
        {"parts": ["plate"], "fit": "exact", "joint": "cylindrical"},
        {"parts": ["plate"], "fit": "clearance", "joint": "cylindrical"},
        {"parts": ["plate"], "fit": "interference", "joint": "cylindrical"}],
        "near_misses": ["plate_big"]}
    save_case("F_pin_plate", parts, pin, [(3, 0, 10)], exp,
              "one plate: 2x exact, 1x clearance 0.5, 1x press -0.03; plate_big too loose")


def case_g():
    """Ball stud (sphere r=5) into spherical cups."""
    stud = one_solid(cq.Solid.makeSphere(5, angleDegrees1=-90, angleDegrees2=90)
                     .fuse(cq.Solid.makeCylinder(2, 17, cq.Vector(0, 0, 3))).clean())

    def cup(r):
        return solid(W().box(30, 30, 15, centered=(True, True, False))
                     .cut(W().sphere(r).translate((0, 0, 15))))
    parts = [("cup_1", cup(5.0), loc((0, 0, 0))), ("cup_2", cup(5.0), loc((60, 0, 0), ((0, 0, 1), 40))),
             ("cup_loose", cup(5.1), loc((120, 0, 0))), ("cup_big", cup(7.0), loc((180, 0, 0))),
             ("stud", stud, loc((0, 80, 30)))]
    exp = {"count": 3, "matches": [
        {"parts": ["cup_1"], "fit": "exact", "joint": "spherical"},
        {"parts": ["cup_2"], "fit": "exact", "joint": "spherical"},
        {"parts": ["cup_loose"], "fit": "clearance", "joint": "spherical"}],
        "near_misses": []}
    save_case("G_ball_socket", parts, stud, [(0, 0, -5)], exp, "2 exact cups, 1 cup +0.1 radial, cup_big too loose")


def case_h():
    """Taper shank (cone r 5->6 over 20) into taper sleeves; a steeper taper must not match."""
    shank = one_solid(cq.Solid.makeCone(5, 6, 20).fuse(cq.Solid.makeCylinder(10, 15, cq.Vector(0, 0, 20))).clean())

    def sleeve(r_top, slope):
        block = cq.Solid.makeBox(30, 30, 30, cq.Vector(-15, -15, 0))
        h = 25
        return one_solid(block.cut(cq.Solid.makeCone(r_top + 0.05 - h * slope, r_top + 0.05, h, cq.Vector(0, 0, 6))))
    parts = [("sleeve_1", sleeve(5.95, 1 / 20), loc((0, 0, 0))),
             ("sleeve_2", sleeve(5.95, 1 / 20), loc((80, 0, 15), ((1, 0, 0), 90), ((0, 0, 1), 10))),
             ("sleeve_steep", sleeve(5.95, 1 / 10), loc((160, 0, 0))),
             ("shank", shank, loc((0, 100, 0)))]
    exp = {"count": 2, "matches": [
        {"parts": ["sleeve_1"], "fit": "exact", "joint": "revolute"},
        {"parts": ["sleeve_2"], "fit": "exact", "joint": "revolute"}],
        "near_misses": []}
    save_case("H_taper", parts, shank, [(5.5, 0, 10)], exp, "2 matching tapers (one on its side), steeper taper decoy")


def case_i():
    """Bolted cover - the part (a bolt) is ALREADY in the assembly, in one of the 4 holes.
    Selected: bolt shank (cylinder) + underside of the head (plane).
      cover:     4 holes 10.5 -> clearance 0.5, revolute (one is where the bolt already sits)
      housing:   4 blind holes 10.0 - right size, but the head would have to sit inside the cover
      lid_small: holes 12.0 -> too loose"""
    pts = [(45, 45), (-45, 45), (-45, -45), (45, -45)]
    bolt = one_solid(cq.Solid.makeCylinder(5, 25, cq.Vector(0, 0, -25))
                     .fuse(cq.Solid.makeCylinder(8, 6)).clean())
    cover = solid(W().box(120, 120, 10, centered=(True, True, False)).faces(">Z").workplane()
                  .pushPoints(pts).hole(10.5))
    housing = solid(W().box(120, 120, 40, centered=(True, True, False)).faces(">Z").workplane()
                    .pushPoints(pts).hole(10.0, depth=20))
    lid_small = solid(W().box(40, 40, 8, centered=(True, True, False)).faces(">Z").workplane().hole(12.0))
    parts = [("housing", housing, loc((0, 0, 0))),
             ("cover", cover, loc((0, 0, 40))),
             ("bolt", bolt, loc((45, 45, 50))),            # in hole 1, head on the cover
             ("lid_small", lid_small, loc((200, 0, 0)))]
    exp = {"count": 4, "matches": [{"parts": ["cover"], "fit": "clearance", "joint": "revolute"}] * 4,
           "near_misses": ["housing", "lid_small"]}
    save_case("I_cover_bolts", parts, bolt, [(5, 0, -12), (6.5, 0, 0)], exp,
              "bolt already in hole 1: cover x4 holes (clearance 0.5, revolute); housing collides; lid_small too loose")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    only = sys.argv[1:]
    for fn in (case_a, case_b, case_c, case_d, case_e, case_f, case_g, case_h, case_i):
        if not only or fn.__name__[-1].upper() in "".join(only).upper():
            fn()
