"""Local browser viewer: python -m fitfinder serve --root DIR  ->  http://127.0.0.1:8765

Standard library only; files are read only under --root. GET serves the page files,
POST /api/<name> calls State.api_<name>(json).
"""
from __future__ import annotations

import base64
import json
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from ..faces import describe, describe_all
from ..loader import export_part, face_list, load_assembly, load_part, shape_bbox
from ..output import _jsonable, mesh, summary_lines, write_csv, write_json

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
STATIC_FILES = {
    "index.html": "text/html; charset=utf-8",
    "style.css": "text/css; charset=utf-8",
    "viewer3d.js": "text/javascript; charset=utf-8",
    "app.js": "text/javascript; charset=utf-8",
}
EXTS = (".step", ".stp", ".step.gz", ".stp.gz")
SKIP_DIRS = {".git", ".venv", "venv", "__pycache__", "node_modules", "out"}


def _b64(a, dtype):
    return base64.b64encode(np.ascontiguousarray(a, dtype=dtype).tobytes()).decode("ascii")


def edge_segments(shape, deflection: float) -> np.ndarray:
    """The part's real B-rep edges as line segments (n, 2, 3), for CAD-style outlines.
    Seam lines (where a cylinder's surface closes on itself) are skipped, as CAD viewers do."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
    from OCP.TopExp import TopExp
    from OCP.TopoDS import TopoDS
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape
    m = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, m)
    segs = []
    for i in range(1, m.Extent() + 1):
        edge = TopoDS.Edge_s(m.FindKey(i))
        faces = list(m.FindFromIndex(i))
        if BRep_Tool.Degenerated_s(edge):
            continue
        if len(faces) == 2 and faces[0].IsSame(faces[1]):     # seam of a cylinder / cone / sphere
            continue
        try:
            c = BRepAdaptor_Curve(edge)
            d = GCPnts_QuasiUniformDeflection(c, deflection)
            if not d.IsDone() or d.NbPoints() < 2:
                continue
            pts = [d.Value(k) for k in range(1, d.NbPoints() + 1)]
        except Exception:
            continue
        xyz = np.array([[p.X(), p.Y(), p.Z()] for p in pts])
        segs.append(np.stack([xyz[:-1], xyz[1:]], axis=1))
    return np.concatenate(segs) if segs else np.zeros((0, 2, 3))


def mesh_payload(shape, key: str) -> dict:
    """Triangles of one shape, packed for the browser: positions, indices, face id per triangle."""
    bb = shape_bbox(shape)
    diag = float(np.linalg.norm(bb[3:] - bb[:3])) if bb is not None else 100.0
    defl = min(1.0, max(0.005, diag * 0.001))
    fm = mesh(shape, key, deflection=defl)
    pos, idx, fid = [], [], []
    base = 0
    for f, (pts, tris) in fm.items():
        pos.append(pts)
        idx.append(tris + base)
        fid.append(np.full(len(tris), f))
        base += len(pts)
    if not pos:
        return {"pos": "", "idx": "", "fid": "", "nfaces": 0, "edges": ""}
    return {"pos": _b64(np.vstack(pos), np.float32), "idx": _b64(np.vstack(idx), np.uint32),
            "fid": _b64(np.concatenate(fid), np.uint32), "nfaces": len(face_list(shape)),
            "edges": _b64(edge_segments(shape, defl), np.float32)}


def map_faces(a_descs: list, b_descs: list) -> dict:
    """Pair up the faces of two copies of the same shape (same frame): same type, same area,
    same centre point. Returns {face id in a: face id in b}."""
    out = {}
    for a in a_descs:
        for b in b_descs:
            if a.kind == b.kind and abs(a.area - b.area) <= 1e-3 * max(a.area, 1.0) \
                    and np.linalg.norm(a.center - b.center) < 1e-3 * (1 + np.linalg.norm(a.center)):
                out[a.fid] = b.fid
                break
    return out


class State:
    def __init__(self, root, part=None, assembly=None):
        self.root = os.path.realpath(os.path.expanduser(root))
        self.default_part, self.default_assembly = part, assembly
        self.lock = threading.Lock()          # OCCT is not thread safe: one request at a time
        self.assemblies: dict = {}
        self.parts: dict = {}

    def resolve(self, rel: str) -> str:
        full = os.path.realpath(os.path.join(self.root, os.path.expanduser(rel or "")))
        if full != self.root and not full.startswith(self.root + os.sep):
            raise ValueError(f"{rel}: outside --root {self.root}")
        if not os.path.isfile(full):
            raise FileNotFoundError(f"{rel}: no such file under {self.root}")
        return full

    def files(self) -> list:
        out = []
        for dp, dns, fns in os.walk(self.root):
            dns[:] = [d for d in dns if d not in SKIP_DIRS and not d.startswith(".")]
            if dp[len(self.root):].count(os.sep) > 4:
                dns[:] = []
            for f in fns:
                if f.lower().endswith(EXTS):
                    out.append(os.path.relpath(os.path.join(dp, f), self.root))
            if len(out) > 2000:
                break
        return sorted(out)

    def assembly(self, rel):
        full = self.resolve(rel)
        key = (full, os.path.getmtime(full))       # re-read if the file changed on disk
        if key not in self.assemblies:
            self.assemblies = {key: load_assembly(full)}   # keep only the latest one in memory
        return full, self.assemblies[key]

    def part(self, rel):
        full = self.resolve(rel)
        key = (full, os.path.getmtime(full))
        if key not in self.parts:
            self.parts[key] = load_part(full)
        return full, self.parts[key]

    def api_files(self, _):
        return {"root": self.root, "files": self.files(), "part": self.default_part,
                "assembly": self.default_assembly}

    def api_part(self, q):
        full, p = self.part(q["part"])
        descs = describe_all(p.faces)
        return {"mesh": mesh_payload(p.shape, f"part|{full}|{os.path.getmtime(full)}"),
                "faces": [{"id": d.fid, "kind": d.kind, "desc": d.short(), "area": round(d.area, 3)}
                          for d in descs],
                "green": sorted(p.green_ids)}

    def api_assembly(self, q):
        t0 = time.time()
        full, asm = self.assembly(q["assembly"])
        defs = {}
        for o in asm.occurrences:
            d = o.definition
            if d.key not in defs:
                defs[d.key] = dict(mesh_payload(d.shape, f"{full}|{os.path.getmtime(full)}|{d.key}"), name=d.name)
        occs = [{"path": o.path, "name": o.name, "def": o.definition.key,
                 "world": np.asarray(o.world).round(6).tolist()} for o in asm.occurrences]
        return {"definitions": defs, "occurrences": occs, "warnings": asm.warnings,
                "seconds": round(time.time() - t0, 2)}

    def api_face(self, q):
        _, asm = self.assembly(q["assembly"])
        d = next(o.definition for o in asm.occurrences if o.definition.key == q["def"])
        fid = int(q["fid"])
        return {"desc": describe(d.faces[fid - 1], fid).short()}

    def save_upload(self, name: str, data: bytes) -> dict:
        import re
        base = os.path.basename(name or "")
        if not base.lower().endswith(EXTS):
            raise ValueError("only .step / .stp files (optionally .gz) can be uploaded")
        base = re.sub(r"[^A-Za-z0-9._-]+", "_", base)
        folder = os.path.join(self.root, "uploads")
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, base), "wb") as f:
            f.write(data)
        return {"path": os.path.relpath(os.path.join(folder, base), self.root), "bytes": len(data)}

    def api_extract(self, q):
        import re
        full, asm = self.assembly(q["assembly"])
        occ = next(o for o in asm.occurrences if o.path == q["path"])
        clean = lambda x: re.sub(r"[^A-Za-z0-9_-]+", "_", x)
        folder = os.path.join(self.root, "uploads")
        os.makedirs(folder, exist_ok=True)
        asm_name = clean(os.path.relpath(full, self.root).split(".")[0])   # folder/name: unique per assembly
        out = os.path.join(folder, f"{asm_name}__{clean(occ.path)}.step")
        export_part(occ.definition.shape, out)
        return {"path": os.path.relpath(out, self.root), "part": occ.path}

    def api_use_occurrence(self, q):
        """Make a placed part of the assembly the part to mate: saved to uploads/, returned like
        api_part plus def_to_part (assembly face ID -> part file face ID)."""
        ex = self.api_extract(q)
        out = self.api_part({"part": ex["path"]})
        _, asm = self.assembly(q["assembly"])
        occ = next(o for o in asm.occurrences if o.path == q["path"])
        out["def_to_part"] = map_faces(describe_all(occ.definition.faces),
                                       describe_all(self.part(ex["path"])[1].faces))
        out.update(part_path=ex["path"], occurrence=occ.path, definition=occ.definition.key)
        return out

    def api_match(self, q):
        from ..match import run_match
        part_full = self.resolve(q["part"])
        asm_full = self.resolve(q["assembly"])
        faces = [int(f) for f in q.get("faces") or []]
        if not faces:
            raise ValueError("no faces selected")
        log = []
        res = run_match(part_full, asm_full, faces=faces, log=lambda *a: log.append(" ".join(map(str, a))),
                        near=(q.get("near") or None), margin=float(q.get("margin") or 150),
                        max_clearance=(float(q["max_clearance"]) if q.get("max_clearance") else None),
                        part=self.part(q["part"])[1], assembly=self.assembly(q["assembly"])[1])
        import re
        name = lambda p: re.sub(r"[^A-Za-z0-9_-]+", "_", os.path.relpath(p, self.root).split(".")[0])
        out = os.path.join(self.root, "out", "web", f"{name(part_full)}__{name(asm_full)}")
        try:
            os.makedirs(out, exist_ok=True)
            write_json(res, out)
            write_csv(res, out)
            lines = summary_lines(res)
            open(os.path.join(out, "summary.txt"), "w").write("\n".join(lines) + "\n")
            res["saved_to"] = os.path.relpath(out, self.root)
        except OSError as e:
            log.append(f"could not save results: {e}")
        res["log"] = log
        res["summary"] = summary_lines(res)
        return res


def make_handler(state: State, log: bool = True):
    """The HTTP request handler: GET serves the page files, POST /api/<name> calls State.api_<name>.
    So a new endpoint is just a new api_<name> method on State - no routing table to edit."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            if log and "/api/" in (args[0] if args else ""):
                print(time.strftime("%H:%M:%S"), fmt % args)

        def _send(self, code, body: bytes, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj, default=_jsonable).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                path = "/index.html"
            name = path.lstrip("/")
            if name in STATIC_FILES:          # only the page's own files are served, nothing else
                self._send(200, open(os.path.join(STATIC, name), "rb").read(), STATIC_FILES[name])
            elif path == "/favicon.ico":
                self._send(204, b"", "image/x-icon")
            elif path == "/api/files":
                self._json(200, state.api_files(None))
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            name = self.path.split("?")[0].rsplit("/", 1)[-1]
            fn = getattr(state, f"api_{name}", None) if name != "upload" else state.save_upload
            if fn is None:
                return self._json(404, {"error": f"unknown endpoint {name}"})
            try:
                n = int(self.headers.get("Content-Length", 0) or 0)
                if name == "upload":                      # raw file bytes, name in the query string
                    from urllib.parse import parse_qs, urlparse
                    if n > 1_000_000_000:
                        raise ValueError("file larger than 1 GB")
                    fname = parse_qs(urlparse(self.path).query).get("name", [""])[0]
                    data = self.rfile.read(n)
                    with state.lock:
                        out = state.save_upload(fname, data)
                    return self._json(200, out)
                q = json.loads(self.rfile.read(n) or b"{}")
                with state.lock:
                    out = fn(q)
                self._json(200, out)
            except (ValueError, FileNotFoundError, KeyError, IndexError, StopIteration) as e:
                self._json(400, {"error": f"{type(e).__name__}: {e}"})
            except Exception as e:
                traceback.print_exc()
                self._json(500, {"error": f"{type(e).__name__}: {e}"})
    return Handler


def serve(root=".", port=8765, part=None, assembly=None, open_browser=True):
    state = State(root, part, assembly)
    httpd = ThreadingHTTPServer(("127.0.0.1", port), make_handler(state))
    url = f"http://127.0.0.1:{port}/"
    print(f"fitfinder viewer: {url}   (files from {state.root}; Ctrl+C to stop)")
    if open_browser:
        try:
            import webbrowser
            webbrowser.open(url)
        except Exception:
            pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("stopped")
