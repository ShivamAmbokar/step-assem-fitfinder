// app.js - viewer behaviour: UI events, calls to server.py, and what viewer3d.js draws.
// All state lives in S; change S, then call refresh().

import { Viewer, mat } from "./viewer3d.js";

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const fmt = (v) => "(" + v.map((x) => (Math.abs(x) < 5e-4 ? 0 : +x.toFixed(2))).join(", ") + ")";

/** Call the server. GET when body is undefined, else POST JSON. Throws with the server's message. */
async function api(path, body) {
  const opt = body === undefined ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const r = await fetch(path, opt);
  const j = await r.json().catch(() => ({ error: r.statusText }));
  if (!r.ok) throw new Error(j.error || r.statusText);
  return j;
}
/** Status bar message. kind: "" | "busy" | "err" */
function status(msg, kind = "") { const s = $("status"); s.textContent = msg; s.className = kind; }
function pickinfo(msg) { $("pickinfo").textContent = msg || ""; }

const COL = {
  sel: [0.29, 0.56, 1.0],      // selected faces (blue)
  match: [1.0, 0.48, 0.12],    // matched faces (orange)
  ghost: [0.24, 0.78, 0.35],   // the part placed in a fit (green)
  near: [0.96, 0.8, 0.1],      // near-miss faces (yellow)
};
const GREY = { dark: [0.6, 0.62, 0.66], light: [0.78, 0.79, 0.81] };       // ordinary parts
const MATE = { dark: [0.52, 0.6, 0.76], light: [0.66, 0.73, 0.88] };       // the part to mate (blue-grey)
const PICKED = { dark: [0.72, 0.74, 0.62], light: [0.86, 0.85, 0.7] };     // part clicked / chosen in tree

const S = {
  theme: "dark",
  asmPath: "", asm: null,            // assembly file and the server's answer (/api/assembly)
  occItems: [], byPath: {},          // one 3-D item per placed part; path -> item
  hidden: new Set(),                 // parts hidden with the eye in the model tree
  partMode: "pick",                  // "pick" = part taken from the assembly, "file" = separate file
  partPath: "", part: null,          // part file and the server's answer (/api/part or /api/use_occurrence)
  partItem: null, ghostItem: null,   // the part alone (Part tab) and its green copy placed in a fit
  mate: null,                        // when picked from the assembly: {path, defToPart, partToDef}
  sel: new Set(),                    // selected faces, as face IDs of the PART FILE
  picked: null,                      // path of the assembly part last clicked / chosen in the tree
  result: null, focus: null,         // last search result; the match or near-miss being looked at
  view: "asm", ctx: "xray",          // which tab; how uninvolved parts are drawn
};

const viewer = new Viewer($("canvas"), { onPick });

function setTheme(t) {
  S.theme = t;
  document.documentElement.dataset.theme = t;
  viewer.setTheme(t);
  try { localStorage.setItem("fitfinder-theme", t); } catch (e) { /* private mode: ignore */ }
  refresh();
}
$("btnTheme").onclick = () => setTheme(S.theme === "dark" ? "light" : "dark");
$("btnInspector").onclick = () => { document.body.classList.toggle("inspector-closed"); $("btnInspector").classList.toggle("on"); viewer.redraw(); };

function showPane(name) {
  document.body.classList.remove("panel-closed");
  document.querySelectorAll("#toolbar .icon[data-pane]").forEach((b) => b.classList.toggle("on", b.dataset.pane === name));
  document.querySelectorAll("#panel .pane").forEach((p) => p.classList.toggle("on", p.id === "pane-" + name));
  if (name === "run") updateRunSummary();
  viewer.redraw();
}
document.querySelectorAll("#toolbar .icon[data-pane]").forEach((b) => {
  b.onclick = () => {   // clicking the active tool again folds the panel away (more room for the view)
    if (b.classList.contains("on") && !document.body.classList.contains("panel-closed")) {
      document.body.classList.add("panel-closed"); b.classList.remove("on"); viewer.redraw();
    } else showPane(b.dataset.pane);
  };
});

async function loadFileList() {
  const f = await api("/api/files");
  $("rootDir").textContent = f.root;
  $("fileList").innerHTML = f.files.map((p) => `<option value="${esc(p)}">`).join("");
  return f;
}

document.querySelectorAll("input[name=partMode]").forEach((r) => r.onchange = () => {
  S.partMode = document.querySelector("input[name=partMode]:checked").value;
  $("partPick").style.display = S.partMode === "pick" ? "" : "none";
  $("partFile").style.display = S.partMode === "file" ? "flex" : "none";
});

// upload a STEP file from this computer; the server saves it in <root>/uploads/
let uploadTarget = null;
$("upAsm").onclick = () => { uploadTarget = "asmPath"; $("fileInput").value = ""; $("fileInput").click(); };
$("upPart").onclick = () => { uploadTarget = "partPath"; $("fileInput").value = ""; $("fileInput").click(); };
$("fileInput").onchange = async () => {
  const f = $("fileInput").files[0]; if (!f) return;
  status(`Uploading ${f.name} (${(f.size / 1e6).toFixed(1)} MB)…`, "busy");
  try {
    const r = await fetch("/api/upload?name=" + encodeURIComponent(f.name), { method: "POST", body: f });
    const j = await r.json(); if (!r.ok) throw new Error(j.error);
    $(uploadTarget).value = j.path; await loadFileList();
    status(`Uploaded to ${j.path}. Press Load.`);
  } catch (e) { status("Upload failed: " + e.message, "err"); }
};

/** Load the assembly (and, in "file" mode, the separate part file). */
async function load() {
  S.asmPath = $("asmPath").value.trim();
  if (!S.asmPath) { status("Choose an assembly file first.", "err"); return; }
  $("btnLoad").disabled = true;
  try {
    status("Loading assembly… (big files take a while the first time)", "busy");
    const a = await api("/api/assembly", { assembly: S.asmPath });
    const geoms = {};
    for (const [k, m] of Object.entries(a.definitions)) geoms[k] = viewer.makeGeom(m);   // one mesh per shape
    S.asm = a;
    S.occItems = a.occurrences.map((o) => viewer.makeItem(geoms[o.def], mat.fromRows(o.world), { kind: "occ", path: o.path, name: o.name, def: o.def, defName: a.definitions[o.def].name, nfaces: a.definitions[o.def].nfaces }));
    S.byPath = Object.fromEntries(S.occItems.map((i) => [i.path, i]));
    Object.assign(S, { hidden: new Set(), part: null, partItem: null, ghostItem: null, mate: null, sel: new Set(),
                       picked: null, result: null, focus: null, partPath: "" });
    buildTree(); renderResults(); showProps(null); pickinfo(""); syncSel();
    $("empty").style.display = "none";
    if (S.partMode === "file" && $("partPath").value.trim()) await loadPartFile($("partPath").value.trim());
    setView("asm"); viewer.setView("iso");
    status(`Loaded ${a.occurrences.length} parts (${Object.keys(a.definitions).length} different shapes) in ${a.seconds}s.` +
           (S.partMode === "pick" && !S.part ? "  Next: pick the part to mate in the Model tree." : ""));
    if (!S.part) showPane(S.partMode === "pick" ? "tree" : "setup"); else showPane("faces");
  } catch (e) { status(e.message, "err"); }
  $("btnLoad").disabled = false;
  updateSteps();
}
$("btnLoad").onclick = load;

/** Separate part file -> the part to mate. */
async function loadPartFile(path) {
  status("Loading part…", "busy");
  const p = await api("/api/part", { part: path });
  S.mate = null; S.partPath = path;
  setPart(p, path.split("/").pop());
}

/** A part of the loaded assembly -> the part to mate (the assembly-only workflow). */
async function useOccurrence(path) {
  status(`Using ${path} as the part to mate…`, "busy");
  try {
    const r = await api("/api/use_occurrence", { assembly: S.asmPath, path });
    const partToDef = {};
    for (const [d, p] of Object.entries(r.def_to_part)) partToDef[p] = +d;
    S.mate = { path, defToPart: Object.fromEntries(Object.entries(r.def_to_part).map(([d, p]) => [+d, p])), partToDef };
    S.partPath = r.part_path;
    setPart(r, path);
    buildTree();
    viewer.fit([S.byPath[path]], 3);
    status(`Part to mate: ${path}. Click its mating faces in the view (or use green / the face list).`);
    showPane("faces");
  } catch (e) { status(e.message, "err"); }
}

/** Common part set-up: Part tab item, ghost item, face list, initial selection (green faces). */
function setPart(p, label) {
  S.part = p; S.partLabel = label;
  const g = viewer.makeGeom(p.mesh);
  S.partItem = viewer.makeItem(g, mat.ident(), { kind: "part" });
  S.ghostItem = viewer.makeItem(g, mat.ident(), { kind: "ghost", pickable: false, alpha: 0.4 });
  S.sel = new Set(p.green); S.result = null; S.focus = null;
  $("facesPartName").textContent = label;
  $("faceList").innerHTML = p.faces.map((f) =>
    `<div class="item" data-fid="${f.id}"><span class="dot" style="background:${p.green.includes(f.id) ? "#3dc75a" : "transparent"}"></span>` +
    `<span class="grow mono" title="${esc(f.desc)}">${esc(f.desc)}</span></div>`).join("");
  document.querySelectorAll("#faceList .item").forEach((el) => el.onclick = () => toggleFace(+el.dataset.fid));
  $("facesNoPart").style.display = "none"; $("facesBody").style.display = "";
  renderResults(); syncSel(); updateSteps();
}

function updateSteps() {
  $("stepAsm").classList.toggle("done", !!S.asm);
  $("stepPart").classList.toggle("done", !!S.part);
  $("crumbAsm").textContent = S.asmPath ? S.asmPath.split("/").pop() : "—";
  $("crumbPart").textContent = S.part ? S.partLabel : "—";
}

function buildTree() {
  const groups = {};                    // "headstock/chuck_A" -> [items]
  for (const it of S.occItems) {
    const k = it.path.includes("/") ? it.path.slice(0, it.path.lastIndexOf("/")) : "";
    (groups[k] = groups[k] || []).push(it);
  }
  const row = (it) => `<div class="item${S.picked === it.path ? " on" : ""}" data-path="${esc(it.path)}">` +
    `<button class="eye${S.hidden.has(it.path) ? " off" : ""}" title="show / hide">👁</button>` +
    `<span class="grow">${esc(it.name)}</span>` +
    (S.mate && S.mate.path === it.path ? `<span class="mate-mark">MATE</span>` : "") +
    `<span class="meta">${it.defName !== it.name ? esc(it.defName) + " · " : ""}${it.nfaces} faces</span></div>`;
  $("tree").innerHTML = Object.keys(groups).sort().map((k) => k === ""
    ? groups[k].map(row).join("")
    : `<details class="group" open><summary>${esc(k)}</summary>${groups[k].map(row).join("")}</details>`).join("");
  document.querySelectorAll("#tree .item").forEach((el) => {
    el.onclick = () => pickPart(el.dataset.path, null);
    el.ondblclick = () => viewer.fit([S.byPath[el.dataset.path]], 2);
    el.querySelector(".eye").onclick = (ev) => {
      ev.stopPropagation();
      const p = el.dataset.path; S.hidden.has(p) ? S.hidden.delete(p) : S.hidden.add(p);
      ev.target.classList.toggle("off", S.hidden.has(p)); refresh();
    };
  });
  filterTree();
}
function filterTree() {
  const q = $("treeFilter").value.trim().toLowerCase();
  document.querySelectorAll("#tree .item").forEach((el) => { el.style.display = !q || el.dataset.path.toLowerCase().includes(q) ? "" : "none"; });
}
$("treeFilter").oninput = filterTree;

/** A part was clicked (in the tree or the view): remember it and show its properties. */
function pickPart(path, fid) {
  S.picked = path;
  document.querySelectorAll("#tree .item").forEach((el) => el.classList.toggle("on", el.dataset.path === path));
  showProps({ type: "part", path, fid });
  refresh();
}

function toggleFace(fid) {
  if (!S.part || !fid) return;
  S.sel.has(fid) ? S.sel.delete(fid) : S.sel.add(fid);
  const f = S.part.faces[fid - 1];
  pickinfo(`${S.sel.has(fid) ? "selected" : "deselected"}  ${f ? f.desc : "face " + fid}`);
  showProps({ type: "face", fid });
  syncSel();
}
function syncSel() {
  const ids = [...S.sel].sort((a, b) => a - b);
  $("selIds").value = ids.join(",");
  document.querySelectorAll("#faceList .item").forEach((el) => el.classList.toggle("on", S.sel.has(+el.dataset.fid)));
  $("badgeSel").textContent = ids.length || "";
  $("crumbSel").textContent = `${ids.length} face${ids.length === 1 ? "" : "s"}`;
  $("selinfo").textContent = S.part ? `${ids.length} face(s) selected` : "";
  updateRunSummary(); refresh();
}
$("selIds").onchange = () => { S.sel = new Set($("selIds").value.split(/[\s,;]+/).map(Number).filter((n) => n > 0)); syncSel(); };
$("btnGreen").onclick = () => { if (!S.part) return; S.sel = new Set(S.part.green); syncSel(); if (!S.part.green.length) status("This part has no green faces."); };
$("btnClear").onclick = () => { S.sel = new Set(); syncSel(); };

function refresh() {
  const grey = GREY[S.theme], R = S.result, F = S.focus;
  if (S.partItem) viewer.paint(S.partItem, (f) => (S.sel.has(f) ? COL.sel : MATE[S.theme]));
  if (S.view === "part") { viewer.scene = S.partItem ? [S.partItem] : []; viewer.redraw(); return; }

  // which faces are orange / yellow, and which parts are "involved" (drawn normally)
  const orange = {}, yellow = {}, involved = new Set();
  if (R) {
    const ms = F ? (F.type === "match" ? [F.m] : []) : R.matches;
    for (const m of ms) for (const [p, ids] of Object.entries(m.faces)) {
      (orange[p] = orange[p] || new Set()); ids.forEach((i) => orange[p].add(i)); involved.add(p);
    }
    if (F && F.type === "near") { yellow[F.n.occurrence] = new Set(F.n.faces); involved.add(F.n.occurrence); }
    if (!F) R.source_occurrences.forEach((p) => involved.add(p));
  }
  if (S.mate && !F) involved.add(S.mate.path);
  const contextMode = !!(R || S.mate);       // only then are the other parts x-rayed / hidden

  for (const it of S.occItems) {
    const o = orange[it.path], y = yellow[it.path], isMate = S.mate && S.mate.path === it.path;
    viewer.paint(it, (f) =>
      o && o.has(f) ? COL.match :
      y && y.has(f) ? COL.near :
      isMate && S.sel.has(S.mate.defToPart[f]) ? COL.sel :
      isMate ? MATE[S.theme] :
      S.picked === it.path ? PICKED[S.theme] : grey);
    const other = contextMode && !involved.has(it.path);
    it.alpha = S.hidden.has(it.path) ? 0 : !other || S.ctx === "solid" ? 1 : S.ctx === "hidden" ? 0 : 0.15;
    it.xray = other && S.ctx === "xray";
  }
  const scene = S.occItems.filter((i) => i.alpha > 0);
  if (F && F.type === "match" && S.ghostItem) {          // the part, placed where it fits (green)
    S.ghostItem.model = mat.fromRows(F.m.transform);
    viewer.paint(S.ghostItem, (f) => (S.sel.has(f) ? COL.sel : COL.ghost));
    scene.push(S.ghostItem);
  }
  viewer.scene = scene; viewer.redraw();
}

function onPick(hit) {
  if (!hit) { pickinfo(""); return; }
  const it = hit.item;
  if (it.kind === "part") { toggleFace(hit.fid); return; }                 // Part tab: select faces
  if (it.kind === "occ" && S.mate && it.path === S.mate.path) {            // the part to mate, in place
    toggleFace(S.mate.defToPart[hit.fid]); return;
  }
  pickPart(it.path, hit.fid);                                              // any other part: inspect it
  pickinfo(`${it.path} · face ${hit.fid}`);
}

async function showProps(what) {
  const el = $("props");
  if (!what) { el.innerHTML = "Click a part or a face in the view."; return; }
  if (what.type === "face" && S.part) {
    const f = S.part.faces[what.fid - 1];
    el.innerHTML = `<dl class="props"><dt>Part to mate</dt><dd>${esc(S.partLabel)}</dd><dt>Face</dt><dd>${what.fid}</dd>` +
      `<dt>Type</dt><dd class="mono">${esc(f ? f.desc : "?")}</dd><dt>Area</dt><dd>${f ? f.area : "?"} mm²</dd>` +
      `<dt>Selected</dt><dd>${S.sel.has(what.fid) ? "yes" : "no"}</dd></dl>`;
    return;
  }
  if (what.type === "part") {
    const it = S.byPath[what.path]; if (!it) return;
    const t = [it.model[12], it.model[13], it.model[14]];
    el.innerHTML = `<dl class="props"><dt>Part</dt><dd><b>${esc(it.name)}</b></dd><dt>Path</dt><dd class="mono">${esc(it.path)}</dd>` +
      `<dt>Shape</dt><dd>${esc(it.defName)} (${it.nfaces} faces)</dd><dt>Position</dt><dd class="mono">${fmt(t)}</dd>` +
      (what.fid ? `<dt>Face</dt><dd class="mono" id="propFace">${what.fid} …</dd>` : "") + `</dl>` +
      `<div class="row" style="margin-top:12px"><button class="btn primary" id="btnMate">Set as part to mate</button></div>` +
      `<div class="row"><button class="btn" id="btnZoom">Zoom to</button><button class="btn" id="btnHide">Hide</button></div>`;
    $("btnMate").onclick = () => useOccurrence(what.path);
    $("btnZoom").onclick = () => viewer.fit([it], 2);
    $("btnHide").onclick = () => { S.hidden.add(what.path); buildTree(); refresh(); };
    if (what.fid) {
      try { const d = await api("/api/face", { assembly: S.asmPath, def: it.def, fid: what.fid });
            if ($("propFace")) $("propFace").textContent = d.desc; } catch (e) { /* not important */ }
    }
    return;
  }
  if (what.type === "match") {
    const m = what.m, T = m.transform;
    el.innerHTML = `<dl class="props"><dt>Match</dt><dd>#${m.id} on <b>${esc(m.parts.join(", "))}</b></dd>` +
      `<dt>Fit</dt><dd><span class="tag ${m.fit}">${m.fit}</span> ${m.clearance} mm</dd><dt>Joint</dt><dd>${m.joint}</dd>` +
      `<dt>Behaviour</dt><dd>${esc(m.behaviour)}</dd><dt>Found by</dt><dd>${esc(m.found_by)}</dd>` +
      `<dt>Faces</dt><dd class="mono">${Object.entries(m.faces).map(([p, ids]) => esc(p) + ": " + ids.join(",")).join("<br>")}</dd>` +
      `<dt>Part goes to</dt><dd class="mono">${fmt([T[0][3], T[1][3], T[2][3]])}</dd>` +
      (m.free_directions && m.free_directions.length ? `<dt>Slides along</dt><dd class="mono">${m.free_directions.map(fmt).join(" ")}</dd>` : "") +
      `<dt>Overlap</dt><dd>${m.mean_overlap ?? "-"}</dd></dl>`;
    return;
  }
  if (what.type === "near") {
    const n = what.n;
    el.innerHTML = `<dl class="props"><dt>Near-miss</dt><dd><b>${esc(n.part)}</b></dd><dt>Why</dt><dd>${esc(n.reason)}` +
      `${n.value == null ? "" : " (" + n.value + " mm)"}</dd><dt>Faces</dt><dd class="mono">${n.faces.join(",")}</dd></dl>` +
      `<div class="hint" style="margin-top:10px">Right shape, but not a fit. The reason says which check failed.</div>`;
  }
}

function updateRunSummary() {
  $("runSummary").innerHTML = `<dt>Assembly</dt><dd>${esc(S.asmPath || "—")}</dd><dt>Part to mate</dt><dd>${esc(S.part ? S.partLabel : "—")}</dd>` +
    `<dt>Faces</dt><dd>${S.sel.size ? [...S.sel].sort((a, b) => a - b).join(", ") : "none selected"}</dd>`;
}

async function run() {
  if (!S.asm) { status("Load an assembly first.", "err"); showPane("setup"); return; }
  if (!S.part) { status("Choose the part to mate first.", "err"); showPane(S.partMode === "pick" ? "tree" : "setup"); return; }
  if (!S.sel.size) { status("Select at least one face.", "err"); showPane("faces"); return; }
  $("btnRun").disabled = true; status("Searching for fits…", "busy");
  try {
    const r = await api("/api/match", { part: S.partPath, assembly: S.asmPath, faces: [...S.sel],
                                        near: $("near").value.trim(), margin: +$("margin").value || 150,
                                        max_clearance: +$("maxClear").value || 1 });
    S.result = r; S.focus = null;
    renderResults(); showPane("results"); refresh(); viewer.fit(viewer.scene);
    const g = r.groups || [];
    status(`${r.status}: ${g.length} part(s), ${r.matches.length} position(s), ${r.near_misses.length} near-miss(es) in ${r.seconds}s` +
           (r.saved_to ? `  ·  saved to ${r.saved_to}/` : ""));
  } catch (e) { status(e.message, "err"); }
  $("btnRun").disabled = false;
}
$("btnRun").onclick = run;

function renderResults() {
  const r = S.result;
  $("badgeRes").textContent = r ? (r.groups || []).length || "0" : "";
  if (!r) { $("resHead").textContent = "No search yet."; $("resList").innerHTML = $("nearList").innerHTML = ""; $("nearHead").style.display = "none"; $("runWarn").innerHTML = ""; return; }
  const byId = Object.fromEntries(r.matches.map((m) => [m.id, m]));
  const groups = r.groups || r.matches.map((m) => ({ group: m.id, parts: m.parts, match_ids: [m.id] }));
  $("resHead").innerHTML = `<div style="display:flex;gap:8px;align-items:center"><span style="flex:1"><b>${groups.length}</b> part(s), ` +
    `<b>${r.matches.length}</b> position(s) fit faces ${r.selected_faces.join(", ")}</span>` +
    `<button class="btn" id="btnAll" style="padding:2px 8px">Show all</button></div>`;
  $("btnAll").onclick = () => focusOn(null);
  $("resList").innerHTML = groups.map((g) => {
    const ms = g.match_ids.map((i) => byId[i]);
    return `<div class="result"><div class="title">${esc(g.parts.join(", "))}` +
      (ms.length > 1 ? `<span class="tag">${ms.length} positions</span>` : "") + `</div>` +
      ms.map((m) => `<div class="pos" data-id="${m.id}">#${m.id} <span class="tag ${m.fit}">${m.fit} ${m.clearance} mm</span>` +
        `<span class="tag">${m.joint}</span><div class="mono" style="color:var(--muted)">faces ${Object.values(m.faces).map((v) => v.join(",")).join(" | ")} · ${esc(m.found_by)}</div></div>`).join("") +
      `</div>`;
  }).join("") || `<div class="hint">No fit found${r.near_misses.length ? " - see the near-misses below." : "."}</div>`;
  $("nearHead").style.display = r.near_misses.length ? "" : "none";
  $("nearList").innerHTML = r.near_misses.map((n, i) => `<div class="item" data-i="${i}"><span class="grow">${esc(n.part)}</span>` +
    `<span class="tag near">${esc(n.reason)}${n.value == null ? "" : " " + (n.value > 0 ? "+" : "") + n.value}</span></div>`).join("");
  document.querySelectorAll("#resList .pos").forEach((el) => el.onclick = () => focusOn({ type: "match", m: byId[+el.dataset.id] }, el));
  document.querySelectorAll("#nearList .item").forEach((el) => el.onclick = () => focusOn({ type: "near", n: r.near_misses[+el.dataset.i] }, el));
  $("log").textContent = [...(r.warnings || []), "", ...(r.log || []), "", ...(r.summary || [])].join("\n");
  $("runWarn").innerHTML = (r.warnings || []).filter((w) => /SIZE_NOT|UNDER_CONSTRAINED|UNSUPPORTED|MERGED/.test(w))
    .map((w) => `<div class="warn">⚠ ${esc(w.replace(/^[A-Z_]+:\s*/, ""))}</div>`).join("");
  $("resHead").insertAdjacentHTML("beforeend", $("runWarn").innerHTML);
}

/** Look at one match / near-miss (or all again with f = null). */
function focusOn(f, el) {
  document.querySelectorAll("#resList .pos, #nearList .item").forEach((x) => x.classList.remove("on"));
  if (el) el.classList.add("on");
  S.focus = f; setView("asm"); refresh();
  if (!f) { viewer.fit(viewer.scene); showProps(null); return; }
  const paths = new Set(f.type === "match" ? f.m.occurrences : [f.n.occurrence]);
  const its = S.occItems.filter((i) => paths.has(i.path));
  if (f.type === "match") its.push(S.ghostItem);
  viewer.fit(its, 2.2);
  showProps(f);
  pickinfo(f.type === "match" ? `#${f.m.id}: ${f.m.parts.join(", ")} · ${f.m.fit} (${f.m.clearance} mm) · ${f.m.joint}` : `near-miss ${f.n.part}: ${f.n.reason}`);
}

function setView(v) {
  S.view = v;
  $("tabAsm").classList.toggle("on", v === "asm"); $("tabPart").classList.toggle("on", v === "part");
  refresh();
}
$("tabAsm").onclick = () => { setView("asm"); viewer.fit(viewer.scene); };
$("tabPart").onclick = () => { setView("part"); viewer.fit(viewer.scene); };
document.querySelectorAll("[data-view]").forEach((b) => b.onclick = () => viewer.setView(b.dataset.view));
function setCtx(mode) {
  S.ctx = mode;
  document.querySelectorAll("[data-ctx]").forEach((b) => b.classList.toggle("on", b.dataset.ctx === mode));
  refresh();
}
document.querySelectorAll("[data-ctx]").forEach((b) => b.onclick = () => setCtx(b.dataset.ctx));
function toggleEdges() { viewer.edges = !viewer.edges; $("btnEdges").classList.toggle("on", viewer.edges); viewer.redraw(); }
$("btnEdges").onclick = toggleEdges;

window.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT" || e.ctrlKey || e.metaKey || e.altKey) return;    // typing in a box
  const k = e.key.toLowerCase();
  const views = { f: "fit", 1: "iso", 2: "top", 3: "front", 4: "right" };
  if (views[k]) viewer.setView(views[k]);
  else if (k === "a") { setView("asm"); viewer.fit(viewer.scene); }
  else if (k === "p") { setView("part"); viewer.fit(viewer.scene); }
  else if (k === "x") setCtx("xray"); else if (k === "s") setCtx("solid"); else if (k === "h") setCtx("hidden");
  else if (k === "e") toggleEdges();
  else if (k === "t") setTheme(S.theme === "dark" ? "light" : "dark");
  else if (k === "escape") focusOn(null);
  else if (e.key === "?") showPane("help");
});

(async function start() {
  let theme = "dark";
  try { theme = localStorage.getItem("fitfinder-theme") || "dark"; } catch (e) { /* ignore */ }
  setTheme(theme);
  try {
    const f = await loadFileList();
    status(`${f.files.length} STEP file(s) found. Choose an assembly and press Load.`);
    if (f.assembly) $("asmPath").value = f.assembly;          // pre-filled by: serve --assembly ... --part ...
    if (f.part) { $("partPath").value = f.part; document.querySelector("input[value=file]").click(); }
    if (f.assembly) load();
  } catch (e) { status("Server not reachable: " + e.message, "err"); }
})();

// exposed for the browser console (F12)
window.fitfinder = { S, viewer, load, useOccurrence, run, refresh, focusOn };
