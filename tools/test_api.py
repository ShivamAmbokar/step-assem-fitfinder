"""Tests of the viewer's server (fitfinder/web/server.py) - no browser, nothing extra to install.

  python tools/make_tests.py      # once: test geometry
  python tools/test_api.py

Starts the server inside this process on a free port, with a TEMPORARY copy of one test case as
its root folder, calls every endpoint the page uses, and checks the answers. To add a check:
copy one of the `check(...)` lines below and change the request and the condition.
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from fitfinder.web.server import State, make_handler  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(bool(ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   ({detail})" if detail and not ok else ""))


def call(base, path, body=None, raw=None):
    """GET (no body) / POST json (body) / POST bytes (raw). Returns (http status, parsed answer)."""
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
    req = urllib.request.Request(base + path, data=data, method="POST" if data is not None else "GET")
    try:
        with urllib.request.urlopen(req, timeout=300) as r:
            txt = r.read()
            return r.status, (json.loads(txt) if r.headers.get_content_type() == "application/json" else txt)
    except urllib.error.HTTPError as e:
        txt = e.read()
        try:
            return e.code, json.loads(txt)
        except ValueError:
            return e.code, txt


def main():
    case = os.path.join(ROOT, "tests_out", "I_cover_bolts")
    if not os.path.exists(os.path.join(case, "assembly.step")):
        print("run  python tools/make_tests.py  first")
        return 1
    root = tempfile.mkdtemp(prefix="fitfinder_api_")
    shutil.copytree(case, os.path.join(root, "job"))
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(State(root), log=False))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"
    try:
        print("page files")
        st, body = call(base, "/")
        check("GET / serves the page", st == 200 and b"fitfinder" in body)
        for f in ("app.js", "viewer3d.js", "style.css"):
            check(f"GET /{f}", call(base, "/" + f)[0] == 200)
        check("other files are not served", call(base, "/../README.md")[0] == 404)

        print("files and loading")
        st, f = call(base, "/api/files")
        check("files lists the assembly", st == 200 and "job/assembly.step" in f["files"], f)
        st, a = call(base, "/api/assembly", {"assembly": "job/assembly.step"})
        check("assembly: 4 placed parts", st == 200 and len(a["occurrences"]) == 4, a.get("error"))
        check("assembly: meshes have outline edges", all(d["edges"] for d in a["definitions"].values()))
        st, p = call(base, "/api/part", {"part": "job/part.step"})
        check("part: 5 faces, green = [3, 4]", st == 200 and len(p["faces"]) == 5 and p["green"] == [3, 4], p.get("error"))
        st, e = call(base, "/api/part", {"part": "../../etc/passwd"})
        check("paths outside the root are refused", st == 400 and "outside" in e["error"], e)
        bolt_def = next(o["def"] for o in a["occurrences"] if o["path"] == "bolt")
        st, fd = call(base, "/api/face", {"assembly": "job/assembly.step", "def": bolt_def, "fid": 4})
        check("face: bolt face 4 is the r=5 shank", st == 200 and "cylinder r=5" in fd["desc"], fd)

        print("assembly-only workflow")
        st, u = call(base, "/api/use_occurrence", {"assembly": "job/assembly.step", "path": "bolt"})
        check("use_occurrence: bolt becomes the part", st == 200 and u["occurrence"] == "bolt", u.get("error"))
        check("use_occurrence: saved under uploads/", u.get("part_path", "").startswith("uploads/"))
        check("use_occurrence: all 5 faces mapped", len(u.get("def_to_part", {})) == 5)
        st, m = call(base, "/api/match", {"part": u["part_path"], "assembly": "job/assembly.step", "faces": [3, 4]})
        check("match: 4 positions on 1 part (cover)", st == 200 and len(m["matches"]) == 4
              and [g["parts"] for g in m["groups"]] == [["cover"]], m.get("error"))
        check("match: the hole the bolt sits in is found in place",
              any("in-place" in x["found_by"] for x in m["matches"]))
        check("match: results saved", os.path.exists(os.path.join(root, m.get("saved_to", "?"), "matches.json")))
        st, e = call(base, "/api/match", {"part": u["part_path"], "assembly": "job/assembly.step", "faces": []})
        check("match without faces is refused", st == 400)

        print("upload")
        data = open(os.path.join(case, "part.step"), "rb").read()
        st, up = call(base, "/api/upload?name=my%20bolt.step", raw=data)
        check("upload a STEP file", st == 200 and up["path"] == "uploads/my_bolt.step", up)
        check("uploaded file is listed", "uploads/my_bolt.step" in call(base, "/api/files")[1]["files"])
        st, e = call(base, "/api/upload?name=evil.sh", raw=b"echo hi")
        check("non-STEP upload is refused", st == 400)
    finally:
        httpd.shutdown()
        shutil.rmtree(root, ignore_errors=True)
    print(f"\n{sum(RESULTS)}/{len(RESULTS)} server checks passed")
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
