"""Read STEP files: parts, placements (occurrences), face IDs, green faces.

Definitions are part shapes in their own frame; occurrences are placed copies (4x4 world
transform), so geometry is analysed once per definition. Face IDs are 1-based in
TopExp.MapShapes order (= FreeCAD FaceN).
"""
from __future__ import annotations

import gzip
import os
import shutil
import sys
import tempfile
from dataclasses import dataclass, field

import numpy as np
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence, TDF_Tool
from OCP.TCollection import TCollection_AsciiString
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_DocumentTool, XCAFDoc_ShapeTool


@dataclass
class Definition:
    """A part shape stored once, in its own local frame."""
    key: str                 # unique id of the definition (XCAF label entry, or file+index)
    name: str
    shape: TopoDS_Shape
    faces: list = field(default_factory=list)   # faces[i] = TopoDS_Face with ID i+1
    volume: float = 0.0


@dataclass
class Occurrence:
    """One placed copy of a definition."""
    path: str                # e.g. "chuck/pinion_2"
    name: str                # leaf name, e.g. "pinion_2"
    definition: Definition
    world: np.ndarray        # 4x4 transform: local -> world

    def world_shape(self) -> TopoDS_Shape:
        """The definition shape moved into world coordinates."""
        return self.definition.shape.Moved(TopLoc_Location(matrix_to_trsf(self.world)))

    def world_bbox(self, gap: float = 0.0):
        return shape_bbox(self.world_shape(), gap)


@dataclass
class Assembly:
    path: str
    definitions: dict        # key -> Definition
    occurrences: list        # [Occurrence]
    warnings: list = field(default_factory=list)


@dataclass
class Part:
    path: str
    shape: TopoDS_Shape
    faces: list
    green_ids: list
    volume: float


def trsf_to_matrix(trsf) -> np.ndarray:
    m = np.eye(4)
    for i in range(3):
        for j in range(4):
            m[i, j] = trsf.Value(i + 1, j + 1)
    return m


def matrix_to_trsf(m: np.ndarray):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*[float(m[i, j]) for i in range(3) for j in range(4)])
    return t


def face_list(shape: TopoDS_Shape) -> list:
    """Faces in TopExp.MapShapes order. faces[i] has face ID i+1."""
    imap = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_FACE, imap)
    return [TopoDS.Face_s(imap.FindKey(i)) for i in range(1, imap.Extent() + 1)]


def shape_volume(shape: TopoDS_Shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return abs(props.Mass())


def shape_bbox(shape: TopoDS_Shape, gap: float = 0.0):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    if box.IsVoid():
        return None
    x0, y0, z0, x1, y1, z1 = box.Get()
    return np.array([x0 - gap, y0 - gap, z0 - gap, x1 + gap, y1 + gap, z1 + gap])


def bbox_overlap(a, b) -> bool:
    if a is None or b is None:
        return False
    return bool(np.all(a[:3] <= b[3:]) and np.all(b[:3] <= a[3:]))


def _ungzip(path: str) -> str:
    if not path.lower().endswith(".gz"):
        return path
    tmp = tempfile.NamedTemporaryFile(suffix=".step", delete=False)
    with gzip.open(path, "rb") as src, open(tmp.name, "wb") as dst:
        shutil.copyfileobj(src, dst)
    return tmp.name


def _read_xcaf(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    doc = TDocStd_Document(TCollection_ExtendedString("fitfinder"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(_ungzip(path)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {path}")
    if not reader.Transfer(doc):
        raise RuntimeError(f"STEP transfer failed: {path}")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    return doc, shape_tool, color_tool


def _label_name(label: TDF_Label) -> str:
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def _nice(*names: str) -> str:
    """First name that is not empty and not purely numeric; drop cadquery's '_part' suffix."""
    for n in names:
        if n and not n.replace("/", "").isdigit():
            return n[:-5] if n.endswith("_part") else n
    return ""


def _label_entry(label: TDF_Label) -> str:
    s = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, s)
    return s.ToCString()


def _solids(shape: TopoDS_Shape) -> list:
    out, exp = [], TopExp_Explorer(shape, TopAbs_SOLID)
    while exp.More():
        out.append(exp.Current())
        exp.Next()
    return out


CACHE_VERSION = 2


def _cache_files(path: str, split_solids: bool):
    """Cache file names for this STEP file: the key changes when the file changes."""
    if os.environ.get("FITFINDER_NO_CACHE"):
        return None
    import hashlib
    st = os.stat(path)
    key = f"{os.path.abspath(path)}|{st.st_size}|{st.st_mtime_ns}|{split_solids}|{CACHE_VERSION}"
    folder = os.environ.get("FITFINDER_CACHE") or os.path.join(os.path.expanduser("~"), ".cache", "fitfinder")
    base = os.path.join(folder, hashlib.sha1(key.encode()).hexdigest())
    return folder, base + ".brep", base + ".json"


def _save_cache(asm: Assembly, files) -> None:
    import json
    from OCP.BinTools import BinTools
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound
    folder, brep, meta = files
    os.makedirs(folder, exist_ok=True)
    defs = list(asm.definitions.values())
    comp, b = TopoDS_Compound(), BRep_Builder()
    b.MakeCompound(comp)
    for d in defs:
        b.Add(comp, d.shape)
    BinTools.Write_s(comp, brep)
    json.dump({"version": CACHE_VERSION, "warnings": asm.warnings,
               "definitions": [{"key": d.key, "name": d.name, "volume": d.volume, "faces": len(d.faces)}
                               for d in defs],
               "occurrences": [{"path": o.path, "name": o.name, "def": o.definition.key,
                                "world": o.world.tolist()} for o in asm.occurrences]}, open(meta, "w"))


def _load_cache(path: str, files):
    import json
    from OCP.BinTools import BinTools
    from OCP.TopoDS import TopoDS_Iterator
    _, brep, meta = files
    info = json.load(open(meta))
    comp = TopoDS_Shape()
    BinTools.Read_s(comp, brep)
    it, shapes = TopoDS_Iterator(comp), []
    while it.More():
        shapes.append(it.Value())
        it.Next()
    if len(shapes) != len(info["definitions"]):
        return None
    defs = {}
    for d, shape in zip(info["definitions"], shapes):
        defs[d["key"]] = Definition(d["key"], d["name"], shape, face_list(shape), d["volume"])
        if len(defs[d["key"]].faces) != d["faces"]:
            return None
    occs = [Occurrence(o["path"], o["name"], defs[o["def"]], np.array(o["world"])) for o in info["occurrences"]]
    return Assembly(path, defs, occs, info["warnings"])


def load_assembly(path: str, split_solids: bool = True) -> Assembly:
    """Read a STEP assembly into definitions + occurrences (with world transforms).
    Reading a big STEP file is slow (the WM290V lathe: 15 s), so the result is also saved in a
    cache folder (~/.cache/fitfinder, or $FITFINDER_CACHE) and read from there next time, as long
    as the STEP file is unchanged. FITFINDER_NO_CACHE=1 turns this off."""
    files = _cache_files(path, split_solids) if os.path.exists(path) else None
    if files and os.path.exists(files[1]) and os.path.exists(files[2]):
        try:
            asm = _load_cache(path, files)
            if asm is not None:
                return asm
        except Exception:
            pass
    asm = _load_step(path, split_solids)
    if files:
        try:
            _save_cache(asm, files)
        except Exception:
            pass
    return asm


def _load_step(path: str, split_solids: bool) -> Assembly:
    """Read the STEP file through XCAF and walk its assembly tree: every leaf becomes an
    Occurrence (a placed copy), and every distinct shape a Definition."""
    doc, st, _ = _read_xcaf(path)
    defs: dict = {}
    occs: list = []
    warnings: list = []

    def get_def(label: TDF_Label, fallback_name: str) -> Definition:
        key = _label_entry(label)
        if key not in defs:
            shape = XCAFDoc_ShapeTool.GetShape_s(label)
            name = _nice(_label_name(label)) or fallback_name
            defs[key] = Definition(key, name, shape)
        return defs[key]

    used_paths: dict = {}

    def unique(path: str) -> str:
        """Only the full PATH must be unique; the part keeps its own name (pinion_1 can exist in
        chuck_A and in chuck_B). A true duplicate path gets #1, #2 ..."""
        n = used_paths.get(path, 0)
        used_paths[path] = n + 1
        return path if n == 0 else f"{path}#{n}"

    def single_leaf(label: TDF_Label):
        """If an assembly is just a wrapper around ONE part (how many exporters store each
        instance), return (part label, its location) so the wrapper's name names the instance."""
        comps = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
        if comps.Length() != 1:
            return None
        ref = TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(comps.Value(1), ref)
        if XCAFDoc_ShapeTool.IsAssembly_s(ref):
            return None
        return ref, XCAFDoc_ShapeTool.GetLocation_s(comps.Value(1))

    def walk(label: TDF_Label, world: np.ndarray, prefix: str):
        if XCAFDoc_ShapeTool.IsAssembly_s(label):
            comps = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(label, comps, False)
            for i in range(1, comps.Length() + 1):
                comp = comps.Value(i)
                ref = TDF_Label()
                XCAFDoc_ShapeTool.GetReferredShape_s(comp, ref)
                loc = XCAFDoc_ShapeTool.GetLocation_s(comp)
                child_world = world @ trsf_to_matrix(loc.Transformation())
                name = _nice(_label_name(comp), _label_name(ref)) or f"part{i}"
                if XCAFDoc_ShapeTool.IsAssembly_s(ref):
                    sub = _nice(_label_name(comp), _label_name(ref))
                    leaf = single_leaf(ref)
                    if leaf is not None:      # wrapper around one part: the wrapper IS the instance
                        add_leaf(leaf[0], child_world @ trsf_to_matrix(leaf[1].Transformation()), prefix,
                                 sub or name)
                    else:
                        walk(ref, child_world, f"{prefix}{sub}/" if sub else prefix)
                else:
                    add_leaf(ref, child_world, prefix, name)
        else:
            add_leaf(label, world, prefix, _label_name(label) or "part")

    def add_leaf(ref: TDF_Label, world: np.ndarray, prefix: str, name: str):
        d = get_def(ref, name)
        # the definition shape may carry its own location: fold it into the world matrix
        own = d.shape.Location()
        if not own.IsIdentity():
            world = world @ trsf_to_matrix(own.Transformation())
            d.shape = d.shape.Located(TopLoc_Location())
        solids = _solids(d.shape)
        if split_solids and len(solids) > 1:
            # a "part" that is really several bodies: treat each solid as its own definition
            for k, s in enumerate(solids):
                skey = f"{d.key}:{k}"
                if skey not in defs:
                    defs[skey] = Definition(skey, f"{d.name}.solid{k + 1}", s.Located(TopLoc_Location()))
                sd = defs[skey]
                w = world
                if not s.Location().IsIdentity():
                    w = world @ trsf_to_matrix(s.Location().Transformation())
                nm = f"{name}.solid{k + 1}"
                occs.append(Occurrence(unique(prefix + nm), nm, sd, w))
        else:
            occs.append(Occurrence(unique(prefix + name), name, d, world))

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), np.eye(4), "")
    # a single-level root assembly name ("root") adds nothing: drop a common first segment
    if occs and all("/" in o.path for o in occs):
        first = {o.path.split("/", 1)[0] for o in occs}
        if len(first) == 1:
            for o in occs:
                o.path = o.path.split("/", 1)[1]
    for o in occs:  # "body/body" -> "body"
        segs = o.path.split("/")
        o.path = "/".join(s for k, s in enumerate(segs) if k == 0 or s != segs[k - 1])

    for o in occs:  # a part the STEP file gives no name ("=>[0:1:1:13]") is shown by its parent's name
        if o.name.startswith("=>[") and "/" in o.path:
            o.name = o.path.split("/")[-2] + o.name[o.name.index("]") + 1:]

    # finalise definitions actually used
    used = {o.definition.key: o.definition for o in occs}
    for d in used.values():
        d.faces = face_list(d.shape)
        d.volume = shape_volume(d.shape)
    empty = sum(1 for d in used.values() if not d.faces)
    if empty:
        warnings.append(f"EMPTY_PART: {empty} entries in the file have no faces and are ignored")
    for o in occs:
        det = np.linalg.det(o.world[:3, :3])
        if det < 0:
            warnings.append(f"LEFT_HANDED_PLACEMENT:{o.path}")
    return Assembly(path, used, occs, warnings)


def export_part(shape, path: str):
    """Write one part shape to its own STEP file (in the part's own coordinates)."""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    try:   # silence OpenCascade's "Statistics on Transfer" printout
        from OCP.Message import Message, Message_Gravity
        Message.DefaultMessenger_s().Printers().First().SetTraceLevel(Message_Gravity.Message_Fail)
    except Exception:
        pass
    w = STEPControl_Writer()
    w.Transfer(shape, STEPControl_AsIs)
    if w.Write(path) != IFSelect_RetDone:
        raise OSError(f"could not write {path}")
    return path


def extract_part(assembly_path: str, name: str, out_path: str) -> str:
    """Save the part whose path/name contains `name` from an assembly as its own STEP file.
    Its face IDs are the same as in the assembly, and fitfinder will find it back in the
    assembly (in-place mode), exactly like a separately exported part."""
    asm = load_assembly(assembly_path)
    hits = [o for o in asm.occurrences if name.lower() in o.path.lower()]
    if not hits:
        raise ValueError(f"no part name contains '{name}'. Names: " + ", ".join(o.path for o in asm.occurrences[:40]))
    exact = [o for o in hits if o.path.lower().split("/")[-1] == name.lower()]
    occ = (exact or hits)[0]
    export_part(occ.definition.shape, out_path)
    return occ.path


def _is_green(c: Quantity_Color) -> bool:
    return c.Green() >= 0.6 and c.Red() <= 0.4 and c.Blue() <= 0.4


def load_part(path: str) -> Part:
    """Read a single-part STEP. Returns its shape (local frame), faces, and green face IDs."""
    doc, st, ct = _read_xcaf(path)
    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    if roots.Length() == 0:
        raise RuntimeError(f"no shapes in {path}")
    if roots.Length() == 1:
        shape = XCAFDoc_ShapeTool.GetShape_s(roots.Value(1))
    else:
        from OCP.BRep import BRep_Builder
        from OCP.TopoDS import TopoDS_Compound
        comp, b = TopoDS_Compound(), BRep_Builder()
        b.MakeCompound(comp)
        for i in range(1, roots.Length() + 1):
            b.Add(comp, XCAFDoc_ShapeTool.GetShape_s(roots.Value(i)))
        shape = comp
    faces = face_list(shape)
    green = []
    for i, f in enumerate(faces, start=1):
        for ctype in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            q = Quantity_Color()
            if ct.GetColor(f, ctype, q) and _is_green(q):
                green.append(i)
                break
    return Part(path, shape, faces, green, shape_volume(shape))


def _print_assembly(path: str):
    import time
    t0 = time.time()
    asm = load_assembly(path)
    print(f"{path}: {len(asm.occurrences)} occurrences, {len(asm.definitions)} definitions "
          f"({time.time() - t0:.1f}s)")
    for o in asm.occurrences:
        t = o.world[:3, 3]
        print(f"  {o.path:40s} def={o.definition.name:25s} faces={len(o.definition.faces):4d} "
              f"vol={o.definition.volume:12.1f}  at ({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f})")
    for w in asm.warnings:
        print("  WARNING", w)


def _print_part(path: str):
    p = load_part(path)
    print(f"{path}: {len(p.faces)} faces, volume {p.volume:.1f}, green faces: {p.green_ids}")


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        try:
            asm_like = load_assembly(arg)
            if len(asm_like.occurrences) > 1:
                _print_assembly(arg)
            else:
                _print_part(arg)
        except Exception as e:
            print(f"{arg}: ERROR {type(e).__name__}: {e}")
