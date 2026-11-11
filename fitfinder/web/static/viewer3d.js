// viewer3d.js - WebGL2 drawing, camera and face picking. Usage:
//   const viewer = new Viewer(canvas, { onPick });      // onPick({item, fid}, event) on a click
//   const geom = viewer.makeGeom(meshFromServer);       // triangles of one part shape (once per shape)
//   const item = viewer.makeItem(geom, matrix, {...});  // one placed copy of it (any extra fields kept)
//   viewer.paint(item, fid => [r, g, b]);               // colour of every face of that copy
//   item.alpha = 0.15; item.xray = true; item.pickable = false;   // how it is drawn
//   viewer.scene = [item, ...];  viewer.redraw();       // what is drawn
//   viewer.fit(items); viewer.setView("iso" | "top" | "front" | "right"); viewer.setTheme("dark")
// Matrices are column-major; mat.fromRows() converts the row-major matrices the server sends.

export const vec = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: (a) => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
};
export const mat = {
  ident() { const o = new Float32Array(16); o[0] = o[5] = o[10] = o[15] = 1; return o; },
  mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s;
    }
    return o;
  },
  persp(fov, asp, n, f) {
    const t = 1 / Math.tan(fov / 2), o = new Float32Array(16);
    o[0] = t / asp; o[5] = t; o[10] = (f + n) / (n - f); o[11] = -1; o[14] = 2 * f * n / (n - f); return o;
  },
  look(e, c, u) {
    const z = vec.norm(vec.sub(e, c)), x = vec.norm(vec.cross(u, z)), y = vec.cross(z, x), o = new Float32Array(16);
    o[0] = x[0]; o[4] = x[1]; o[8] = x[2]; o[12] = -vec.dot(x, e);
    o[1] = y[0]; o[5] = y[1]; o[9] = y[2]; o[13] = -vec.dot(y, e);
    o[2] = z[0]; o[6] = z[1]; o[10] = z[2]; o[14] = -vec.dot(z, e); o[15] = 1;
    return { m: o, x, y, z };
  },
  fromRows(r) { const o = new Float32Array(16); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) o[j * 4 + i] = r[i][j]; return o; },
  apply(m, p) {
    return [m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12], m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
            m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]];
  },
};

function b64(s, T) {             // server sends arrays as base64 text -> typed array
  if (!s) return new T(0);
  const bin = atob(s), u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return new T(u8.buffer);
}

const VS = `#version 300 es
layout(location=0) in vec3 aPos; layout(location=1) in vec3 aNor; layout(location=2) in float aFid;
uniform mat4 uModel, uVP; flat out int vFid; out vec3 vN;
void main(){ vFid = int(aFid + 0.5); vN = mat3(uModel) * aNor; gl_Position = uVP * uModel * vec4(aPos, 1.0); }`;
// faces: colour from a per-part table (one entry per face id), simple head-light shading;
// x-ray: surfaces seen edge-on darker and more opaque so the outline stays readable
const FS = `#version 300 es
precision highp float; precision highp int;
flat in int vFid; in vec3 vN; uniform sampler2D uCol; uniform float uAlpha; uniform float uXray; uniform vec3 uEye; out vec4 o;
void main(){ vec4 c = texelFetch(uCol, ivec2(vFid & 1023, vFid >> 10), 0);
  float facing = abs(dot(normalize(vN), uEye));
  if (uXray > 0.5) { float rim = pow(1.0 - facing, 2.0); o = vec4(c.rgb * mix(0.8, 0.4, rim), mix(uAlpha, 0.75, rim)); }
  else { o = vec4(c.rgb * (0.38 + 0.62 * facing), uAlpha); } }`;
// picking: draw (object number, face id) as a colour into a hidden image, read the pixel under the mouse
const FS_PICK = `#version 300 es
precision highp float; precision highp int;
flat in int vFid; in vec3 vN; uniform int uId; out vec4 o;
void main(){ o = vec4(float(uId & 255), float((uId >> 8) & 255), float(vFid & 255), float((vFid >> 8) & 255)) / 255.0; }`;
const VS_LINE = `#version 300 es
layout(location=0) in vec3 aPos; uniform mat4 uModel, uVP;
void main(){ gl_Position = uVP * uModel * vec4(aPos, 1.0); }`;
const FS_LINE = `#version 300 es
precision highp float; uniform vec4 uLine; out vec4 o; void main(){ o = uLine; }`;

const VIEWS = {   // camera direction presets: yaw (around Z), pitch (up from the XY plane)
  iso: [-Math.PI / 4, 0.6155], top: [-Math.PI / 2, 1.5607], front: [-Math.PI / 2, 0], right: [0, 0],
};

export class Viewer {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    const gl = canvas.getContext("webgl2", { antialias: true, alpha: true, premultipliedAlpha: true });
    if (!gl) throw new Error("This browser has no WebGL2. Use a current Chrome, Edge or Firefox.");
    this.gl = gl;
    this.prog = this._program(VS, FS, ["uModel", "uVP", "uCol", "uAlpha", "uXray", "uEye"]);
    this.pickProg = this._program(VS, FS_PICK, ["uModel", "uVP", "uId"]);
    this.lineProg = this._program(VS_LINE, FS_LINE, ["uModel", "uVP", "uLine"]);
    this.scene = [];                  // items to draw - set by app.js
    this.edges = true;                // draw outline edges
    this.lineColor = [0.85, 0.88, 0.93];
    this.cam = { target: [0, 0, 0], dist: 200, yaw: VIEWS.iso[0], pitch: VIEWS.iso[1], fov: 40 * Math.PI / 180 };
    this.onPick = opts.onPick || (() => {});
    this._dirty = true;
    this._pickFB = { fb: null, rb: [], w: 0, h: 0 };
    this._mouse();
    new ResizeObserver(() => this.redraw()).observe(canvas);
    const loop = () => { if (this._dirty) { this._dirty = false; this._draw(); } requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }

  redraw() { this._dirty = true; }

  setTheme(name) {
    this.lineColor = name === "dark" ? [0.85, 0.88, 0.93] : [0.16, 0.18, 0.22];
    this.redraw();
  }

  /** Triangles + edges of one part shape, from the server's mesh payload. */
  makeGeom(m) {
    const gl = this.gl;
    const P = b64(m.pos, Float32Array), I = b64(m.idx, Uint32Array), F = b64(m.fid, Uint32Array);
    const nt = F.length, pos = new Float32Array(nt * 9), nor = new Float32Array(nt * 9), fid = new Float32Array(nt * 3);
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let t = 0; t < nt; t++) {            // flat shading: each triangle gets its own 3 vertices
      const v = [I[t * 3], I[t * 3 + 1], I[t * 3 + 2]].map((k) => [P[k * 3], P[k * 3 + 1], P[k * 3 + 2]]);
      const n = vec.norm(vec.cross(vec.sub(v[1], v[0]), vec.sub(v[2], v[0])));
      for (let k = 0; k < 3; k++) for (let j = 0; j < 3; j++) {
        pos[t * 9 + k * 3 + j] = v[k][j]; nor[t * 9 + k * 3 + j] = n[j];
        lo[j] = Math.min(lo[j], v[k][j]); hi[j] = Math.max(hi[j], v[k][j]);
      }
      fid[t * 3] = fid[t * 3 + 1] = fid[t * 3 + 2] = F[t];
    }
    const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
    [[pos, 3], [nor, 3], [fid, 1]].forEach(([arr, size], loc) => {
      const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.bufferData(gl.ARRAY_BUFFER, arr, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(loc); gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0);
    });
    const E = b64(m.edges || "", Float32Array);
    let evao = null;
    if (E.length) {
      evao = gl.createVertexArray(); gl.bindVertexArray(evao);
      const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); gl.bufferData(gl.ARRAY_BUFFER, E, gl.STATIC_DRAW);
      gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    }
    gl.bindVertexArray(null);
    return { vao, count: nt * 3, nfaces: m.nfaces, lo, hi, evao, ecount: E.length / 3 };
  }

  /** One placed copy of a geometry. `props` fields are kept on the item (e.g. path, kind). */
  makeItem(geom, model, props = {}) {
    return Object.assign({ geom, model: model || mat.ident(), alpha: 1, xray: false, pickable: true,
                           tex: this._colorTable(geom.nfaces) }, props);
  }

  /** Colour every face of an item: colourOf(faceId) -> [r, g, b] in 0..1 */
  paint(item, colourOf) {
    const t = item.tex, gl = this.gl;
    for (let f = 0; f <= t.n; f++) {
      const c = colourOf(f), i = f * 4;
      t.data[i] = c[0] * 255; t.data[i + 1] = c[1] * 255; t.data[i + 2] = c[2] * 255; t.data[i + 3] = 255;
    }
    gl.bindTexture(gl.TEXTURE_2D, t.t);
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, t.w, t.h, gl.RGBA, gl.UNSIGNED_BYTE, t.data);
    this.redraw();
  }

  /** Bounding box of items in world coordinates, or null. */
  box(items) {
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (const it of items) {
      const g = it.geom; if (!isFinite(g.lo[0])) continue;
      for (let k = 0; k < 8; k++) {
        const p = mat.apply(it.model, [k & 1 ? g.hi[0] : g.lo[0], k & 2 ? g.hi[1] : g.lo[1], k & 4 ? g.hi[2] : g.lo[2]]);
        for (let j = 0; j < 3; j++) { lo[j] = Math.min(lo[j], p[j]); hi[j] = Math.max(hi[j], p[j]); }
      }
    }
    return isFinite(lo[0]) ? { lo, hi } : null;
  }

  /** Move the camera so these items fill the view (zoomOut > 1 shows more around them). */
  fit(items, zoomOut = 1) {
    const b = this.box(items || this.scene); if (!b) return;
    this.cam.target = vec.mul(vec.add(b.lo, b.hi), 0.5);
    const r = Math.max(1, 0.5 * Math.hypot(b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]));
    this.cam.dist = r / Math.sin(this.cam.fov / 2) * 1.05 * zoomOut;
    this.redraw();
  }

  setView(name) {
    if (name === "fit") return this.fit();
    const v = VIEWS[name]; if (!v) return;
    this.cam.yaw = v[0]; this.cam.pitch = v[1]; this.fit();
  }

  /** Which (item, face id) is under canvas pixel (x, y)? Solid items first; if the click hits
   *  none, see-through ones. Items with pickable = false (e.g. the green ghost) are never picked. */
  pick(x, y) {
    const solid = (it) => it.pickable && it.alpha >= 1, seeThrough = (it) => it.pickable && it.alpha > 0;
    return this._pickPass(x, y, solid) || this._pickPass(x, y, seeThrough);
  }

  _pickPass(x, y, use) {
    const gl = this.gl; this._resize();
    const w = this.canvas.width, h = this.canvas.height, F = this._pickFB;
    if (F.w !== w || F.h !== h) {
      if (F.fb) { gl.deleteFramebuffer(F.fb); F.rb.forEach((r) => gl.deleteRenderbuffer(r)); }
      const fb = gl.createFramebuffer(), c = gl.createRenderbuffer(), d = gl.createRenderbuffer();
      gl.bindRenderbuffer(gl.RENDERBUFFER, c); gl.renderbufferStorage(gl.RENDERBUFFER, gl.RGBA8, w, h);
      gl.bindRenderbuffer(gl.RENDERBUFFER, d); gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT24, w, h);
      gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
      gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.RENDERBUFFER, c);
      gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, d);
      Object.assign(F, { fb, rb: [c, d], w, h });
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, F.fb); gl.viewport(0, 0, w, h);
    gl.clearColor(1, 1, 1, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.BLEND); gl.depthMask(true);
    gl.useProgram(this.pickProg.p); gl.uniformMatrix4fv(this.pickProg.u.uVP, false, this._camera().vp);
    this.scene.forEach((it, k) => {
      if (!use(it)) return;
      gl.uniform1i(this.pickProg.u.uId, k); gl.uniformMatrix4fv(this.pickProg.u.uModel, false, it.model);
      gl.bindVertexArray(it.geom.vao); gl.drawArrays(gl.TRIANGLES, 0, it.geom.count);
    });
    const d = window.devicePixelRatio || 1, px = new Uint8Array(4);
    gl.readPixels(Math.round(x * d), h - 1 - Math.round(y * d), 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null); gl.bindVertexArray(null); this.redraw();
    const id = px[0] + px[1] * 256, fid = px[2] + px[3] * 256;
    return id === 65535 ? null : { item: this.scene[id], fid };
  }

  /** Canvas pixel position of a world point. */
  project(p) {
    const v = this._camera().vp, x = v[0] * p[0] + v[4] * p[1] + v[8] * p[2] + v[12],
      y = v[1] * p[0] + v[5] * p[1] + v[9] * p[2] + v[13], w = v[3] * p[0] + v[7] * p[1] + v[11] * p[2] + v[15];
    return [(x / w + 1) / 2 * this.canvas.clientWidth, (1 - y / w) / 2 * this.canvas.clientHeight];
  }

  _program(vs, fs, names) {
    const gl = this.gl, p = gl.createProgram();
    for (const [type, src] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
      const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      gl.attachShader(p, s);
    }
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    const u = {}; for (const n of names) u[n] = gl.getUniformLocation(p, n);
    return { p, u };
  }

  _colorTable(nfaces) {           // a small texture: 1024 face colours per row
    const gl = this.gl, w = 1024, h = Math.floor((nfaces + 1) / 1024) + 1, t = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, t);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    return { t, w, h, n: nfaces, data: new Uint8Array(w * h * 4) };
  }

  _camera() {
    const c = this.cam, w = this.canvas.clientWidth, h = this.canvas.clientHeight, cp = Math.cos(c.pitch);
    const eye = vec.add(c.target, vec.mul([cp * Math.cos(c.yaw), cp * Math.sin(c.yaw), Math.sin(c.pitch)], c.dist));
    const L = mat.look(eye, c.target, [0, 0, 1]);
    const P = mat.persp(c.fov, w / Math.max(1, h), c.dist * 0.01, c.dist * 20);
    return { vp: mat.mul(P, L.m), eye: vec.norm(vec.sub(eye, c.target)), x: L.x, y: L.y };
  }

  _resize() {
    const d = window.devicePixelRatio || 1, cv = this.canvas;
    const w = Math.round(cv.clientWidth * d), h = Math.round(cv.clientHeight * d);
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  }

  _draw() {
    const gl = this.gl; this._resize();
    const C = this._camera(), items = this.scene;
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clearColor(0, 0, 0, 0);       // transparent: the CSS gradient behind the canvas is the background
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST); gl.disable(gl.CULL_FACE);
    gl.useProgram(this.prog.p);
    gl.uniformMatrix4fv(this.prog.u.uVP, false, C.vp); gl.uniform3fv(this.prog.u.uEye, C.eye);
    gl.uniform1i(this.prog.u.uCol, 0); gl.activeTexture(gl.TEXTURE0);
    const one = (it) => {
      gl.bindTexture(gl.TEXTURE_2D, it.tex.t); gl.uniformMatrix4fv(this.prog.u.uModel, false, it.model);
      gl.uniform1f(this.prog.u.uAlpha, it.alpha); gl.uniform1f(this.prog.u.uXray, it.xray ? 1 : 0);
      gl.bindVertexArray(it.geom.vao); gl.drawArrays(gl.TRIANGLES, 0, it.geom.count);
    };
    gl.enable(gl.POLYGON_OFFSET_FILL); gl.polygonOffset(1, 1);     // faces a hair back: edges draw on top
    gl.disable(gl.BLEND); gl.depthMask(true);
    items.filter((i) => i.alpha >= 1).forEach(one);                // 1. solid parts
    gl.enable(gl.BLEND); gl.blendFuncSeparate(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA, gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    gl.depthMask(false);
    items.filter((i) => i.alpha < 1).forEach(one);                 // 2. see-through parts
    gl.disable(gl.POLYGON_OFFSET_FILL);
    if (this.edges) {                                              // 3. outline edges
      gl.useProgram(this.lineProg.p); gl.uniformMatrix4fv(this.lineProg.u.uVP, false, C.vp);
      const [r, g, b] = this.lineColor;
      for (const it of items) {
        if (!it.geom.evao) continue;
        gl.uniform4f(this.lineProg.u.uLine, r, g, b, it.alpha >= 1 ? 0.5 : it.kind === "ghost" ? 0.3 : 0.4);
        gl.uniformMatrix4fv(this.lineProg.u.uModel, false, it.model);
        gl.bindVertexArray(it.geom.evao); gl.drawArrays(gl.LINES, 0, it.geom.ecount);
      }
    }
    gl.depthMask(true); gl.bindVertexArray(null);
  }

  _mouse() {
    const cv = this.canvas; let drag = null;
    cv.addEventListener("contextmenu", (e) => e.preventDefault());
    cv.addEventListener("mousedown", (e) => {
      drag = { x: e.clientX, y: e.clientY, x0: e.clientX, y0: e.clientY, pan: e.button === 2 || e.shiftKey };
      cv.style.cursor = "grabbing";
    });
    window.addEventListener("mouseup", (e) => {
      if (!drag) return; cv.style.cursor = "grab";
      const moved = Math.hypot(e.clientX - drag.x0, e.clientY - drag.y0) > 4, wasPan = drag.pan; drag = null;
      if (!moved && !wasPan) {
        const r = cv.getBoundingClientRect();
        this.onPick(this.pick(e.clientX - r.left, e.clientY - r.top), e);
      }
    });
    window.addEventListener("mousemove", (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y, c = this.cam; drag.x = e.clientX; drag.y = e.clientY;
      if (drag.pan) {
        const C = this._camera(), s = 2 * c.dist * Math.tan(c.fov / 2) / cv.clientHeight;
        c.target = vec.add(c.target, vec.add(vec.mul(C.x, -dx * s), vec.mul(C.y, dy * s)));
      } else { c.yaw -= dx * 0.008; c.pitch = Math.max(-1.56, Math.min(1.56, c.pitch + dy * 0.008)); }
      this.redraw();
    });
    cv.addEventListener("wheel", (e) => { e.preventDefault(); this.cam.dist *= Math.exp(e.deltaY * 0.0012); this.redraw(); },
                        { passive: false });
  }
}
