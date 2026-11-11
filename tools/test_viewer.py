"""Browser test of the viewer: clicks through both workflows like a user and checks the results.

  pip install playwright && python -m playwright install chromium     # once (optional extra)
  python tools/make_tests.py                                          # once: test geometry
  python tools/test_viewer.py              # starts its own server on tests_out/, runs, stops it
  python tools/test_viewer.py --show       # same, with a visible browser window
  python tools/test_viewer.py --url http://127.0.0.1:8765   # use a server you started yourself

Screenshots go to out/ui/. Without Playwright installed the test is skipped (exit 0) - the
server API is still covered by tools/test_api.py, which needs nothing extra.
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHOTS = os.path.join(ROOT, "out", "ui")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def start_server(python, port):
    env = dict(os.environ, PYTHONPATH=ROOT)
    p = subprocess.Popen([python, "-m", "fitfinder", "serve", "--root", os.path.join(ROOT, "tests_out"),
                          "--port", str(port), "--no-browser"], cwd=ROOT, env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    for _ in range(100):
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/api/files", timeout=1)
            return p
        except OSError:
            time.sleep(0.2)
    p.kill()
    raise RuntimeError("server did not start")


CHECKS = []


def check(name, ok, detail=""):
    CHECKS.append((name, bool(ok)))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   ({detail})" if detail and not ok else ""))


def run(url, show):
    from playwright.sync_api import sync_playwright
    os.makedirs(SHOTS, exist_ok=True)
    with sync_playwright() as pw:
        args = ["--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"]
        exe = os.environ.get("CHROME")          # optional: a specific Chrome / Chromium binary
        b = pw.chromium.launch(headless=not show, args=args, **({"executable_path": exe} if exe else {}))
        page = b.new_page(viewport={"width": 1500, "height": 900})
        errors = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        S = lambda expr: page.evaluate(f"() => {expr}")  # noqa: E731  read app state
        wait_status = lambda text, t=120000: page.wait_for_function(  # noqa: E731
            f"document.getElementById('status').textContent.includes({text!r})", timeout=t)

        # ---------------------------------------------------------------- workflow 1: assembly only
        print("workflow 1: assembly only (I_cover_bolts), part picked in the model tree")
        page.goto(url)
        wait_status("STEP file")
        check("dark theme by default", page.get_attribute("html", "data-theme") == "dark")
        page.fill("#asmPath", "I_cover_bolts/assembly.step")
        page.click("#btnLoad")
        wait_status("Loaded")
        check("assembly loaded: 4 parts", S("fitfinder.S.occItems.length") == 4)
        check("model tree shows the parts", page.locator("#tree .item").count() == 4)
        page.click("#tree .item[data-path='bolt']")
        page.click("#btnMate")
        wait_status("Part to mate")
        check("bolt is the part to mate", S("fitfinder.S.mate && fitfinder.S.mate.path") == "bolt")
        check("MATE mark in the tree", page.locator("#tree .mate-mark").count() == 1)
        # click a face of the bolt directly in the assembly view
        xy = S("""(() => { const v = fitfinder.viewer, it = fitfinder.S.byPath['bolt'];
                   const m = it.model; return v.project([m[12], m[13], m[14] - 12]); })()""")
        box = page.locator("#canvas").bounding_box()
        page.mouse.click(box["x"] + xy[0], box["y"] + xy[1])
        page.wait_for_timeout(300)
        n_clicked = S("fitfinder.S.sel.size")
        check("clicking the bolt in the assembly selects a face", n_clicked == 1, f"selected {n_clicked}")
        page.screenshot(path=os.path.join(SHOTS, "1_mate_selected.png"))
        page.fill("#selIds", "3,4")
        page.dispatch_event("#selIds", "change")
        check("face IDs typed: 3,4", S("[...fitfinder.S.sel].sort().join(',')") == "3,4")
        page.click("#toolbar [data-pane=run]")
        page.click("#btnRun")
        wait_status("position")
        check("search: 1 part, 4 positions", S("fitfinder.S.result.matches.length") == 4
              and S("fitfinder.S.result.groups.length") == 1)
        check("bolt's own hole found in place", S("fitfinder.S.result.matches.some(m => m.found_by.includes('in-place'))"))
        page.click("#resList .pos[data-id='1']")
        page.wait_for_timeout(300)
        check("clicking a position shows the part placed (ghost)", S("fitfinder.viewer.scene.includes(fitfinder.S.ghostItem)"))
        check("properties show the match", "clearance" in page.text_content("#props"))
        page.screenshot(path=os.path.join(SHOTS, "2_match_dark.png"))
        page.click("#btnTheme")
        page.wait_for_timeout(200)
        check("light theme switch", page.get_attribute("html", "data-theme") == "light")
        page.screenshot(path=os.path.join(SHOTS, "3_match_light.png"))
        page.click("#btnTheme")

        # ---------------------------------------------------------------- workflow 2: separate part file
        print("workflow 2: separate part file (A_chuck_key), green faces")
        page.click("#toolbar [data-pane=setup]")
        page.check("input[name=partMode][value=file]")
        page.fill("#asmPath", "A_chuck_key/assembly.step")
        page.fill("#partPath", "A_chuck_key/part.step")
        page.click("#btnLoad")
        wait_status("Loaded")
        check("green faces pre-selected", S("[...fitfinder.S.sel].sort().join(',')") == "10,7,8,9")
        page.click("#toolbar [data-pane=run]")
        page.click("#btnRun")
        wait_status("position")
        check("search: 4 pinions", S("fitfinder.S.result.matches.length") == 4)
        check("near-miss pinion_wide listed", "pinion_wide" in page.text_content("#nearList"))
        page.click("[data-ctx=solid]")
        check("solid mode: every part opaque", S("fitfinder.S.occItems.every(i => i.alpha === 1)"))
        page.click("[data-ctx=xray]")
        page.screenshot(path=os.path.join(SHOTS, "4_chuck_results.png"))
        check("no JavaScript errors", not errors, "; ".join(errors))
        b.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", help="use an already running server")
    ap.add_argument("--python", default=sys.executable, help="python that has cadquery (for the server)")
    ap.add_argument("--show", action="store_true", help="show the browser window")
    a = ap.parse_args()
    try:
        import playwright  # noqa: F401
    except ImportError:
        print("SKIPPED: playwright is not installed (pip install playwright && python -m playwright install chromium)")
        return 0
    server = None
    url = a.url
    if not url:
        port = free_port()
        server = start_server(a.python, port)
        url = f"http://127.0.0.1:{port}/"
    try:
        run(url, a.show)
    finally:
        if server:
            server.terminate()
    bad = [n for n, ok in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(bad)}/{len(CHECKS)} viewer checks passed" + (f"  - failed: {bad}" if bad else ""))
    print(f"screenshots: {SHOTS}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
