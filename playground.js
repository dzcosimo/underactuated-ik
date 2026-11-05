/* Toy-arm playground: tendon-driven double pendulum with parallel springs (the paper's running example).
   Both closed-loop IK strategies run live in the browser. */
(function () {
  "use strict";
  const root = document.getElementById("playground");
  if (!root) return;

  // ---------- model ----------
  const P = { l1: 1, l2: 1, m1: 1, m2: 1, g: 9.81, k1: 0, k2: 10, th: Math.PI / 8 };
  const S = { q: [0, 0], xbar: 1.2, algo: 0, running: false, trail: [], gain: 1.5, showSettle: true, msg: "" };
  var H = { ex: [], gu: [] };

  const G = (q) => {
    const s1 = Math.sin(q[0]), s12 = Math.sin(q[0] + q[1]);
    return [P.k1 * q[0] + P.g * (P.m1 + P.m2) * P.l1 * s1 + P.g * P.m2 * P.l2 * s12,
            P.k2 * q[1] + P.g * P.m2 * P.l2 * s12];
  };
  const Gu = (q) => { const g = G(q); return g[1] * Math.cos(P.th) - g[0] * Math.sin(P.th); };
  const X = (q) => P.l1 * Math.sin(q[0]) + P.l2 * Math.sin(q[0] + q[1]);
  const A = () => [Math.cos(P.th), Math.sin(P.th)];      // actuated direction
  const D = () => [Math.sin(P.th), -Math.cos(P.th)];     // passive direction (tendon locked)
  const qaOf = (q) => { const a = A(); return a[0] * q[0] + a[1] * q[1]; };
  const grad = (f, q, e = 1e-6) => [(f([q[0] + e, q[1]]) - f([q[0] - e, q[1]])) / (2 * e),
                                    (f([q[0], q[1] + e]) - f([q[0], q[1] - e])) / (2 * e)];

  // stiffness along the passive direction with the tendon locked (> 0: stable equilibrium)
  const kD = (q) => { const d = D(), g = grad(Gu, q); return -(g[0] * d[0] + g[1] * d[1]); };

  // held configuration with the same tendon length as q: solve Gu = 0 along the passive direction
  function settle(q, sGuess) {
    const a = A(), d = D(), qa = qaOf(q);
    const base = [qa * a[0], qa * a[1]];
    const at = (s) => [base[0] + s * d[0], base[1] + s * d[1]];
    const f = (s) => Gu(at(s));
    let s0 = sGuess !== undefined ? sGuess : d[0] * q[0] + d[1] * q[1];
    // Newton from the guess
    let s = s0;
    for (let i = 0; i < 30; i++) {
      const v = f(s), dv = (f(s + 1e-6) - f(s - 1e-6)) / 2e-6;
      if (Math.abs(dv) < 1e-9) break;
      const step = Math.max(-0.5, Math.min(0.5, v / dv));
      s -= step;
      if (Math.abs(step) < 1e-10) break;
    }
    if (Math.abs(f(s)) < 1e-6 && Math.abs(s - s0) < 2.5) return { q: at(s), s };
    // fallback: nearest sign change around s0
    let best = null;
    for (let r = 0.02; r < 4 && !best; r += 0.02) {
      for (const sg of [1, -1]) {
        const a1 = s0 + sg * (r - 0.02), b1 = s0 + sg * r;
        if (f(a1) * f(b1) <= 0) {
          let lo = a1, hi = b1;
          for (let k = 0; k < 50; k++) { const m = (lo + hi) / 2; if (f(lo) * f(m) <= 0) hi = m; else lo = m; }
          best = (lo + hi) / 2; break;
        }
      }
    }
    if (best === null) return null;
    return { q: at(best), s: best };
  }

  // ---------- algorithms ----------
  let sMem;
  function stepAlg1(dt) {
    // q stays on E; integrate the actuator coordinate
    const a = A(), d = D();
    const f = (qa) => {
      const base = [qa * a[0], qa * a[1]];
      const r = settle(base, sMem); return r ? { x: X(r.q), q: r.q, s: r.s } : null;
    };
    const qa = qaOf(S.q);
    const c = f(qa); if (!c) { S.msg = "No held configuration nearby"; return false; }
    const e = 1e-5, cp = f(qa + e), cm = f(qa - e);
    if (!cp || !cm) { S.msg = "No held configuration nearby"; return false; }
    const J = (cp.x - cm.x) / (2 * e);
    if (Math.abs(J) < 1e-4) { S.msg = "Singular: the tendon cannot move the tip here"; return false; }
    let v = S.gain * (S.xbar - c.x) / J;
    v = Math.max(-2.5, Math.min(2.5, v));
    const n = f(qa + dt * v); if (!n) return false;
    sMem = n.s; S.q = n.q; S.msg = "";
    return true;
  }
  function stepPlain(dt) {
    // classic inverse kinematics: moves q as if every joint were actuated, ignoring the statics
    // weighted pseudoinverse (W = diag(1, 0.1)): the task is redundant; this choice mostly moves the
    // shoulder and keeps the elbow bend, as a rigid-arm IK would
    const W = [1, 0.1], gx = grad(X, S.q), n2 = W[0] * gx[0] ** 2 + W[1] * gx[1] ** 2;
    if (n2 < 1e-8) { S.msg = "Singular: the tip cannot move along x here"; return false; }
    const r = S.gain * (S.xbar - X(S.q));
    let v = [W[0] * gx[0] * r / n2, W[1] * gx[1] * r / n2];
    const n = Math.hypot(v[0], v[1]); if (n > 2.5) v = [v[0] * 2.5 / n, v[1] * 2.5 / n];
    S.q = [S.q[0] + dt * v[0], S.q[1] + dt * v[1]]; S.msg = "";
    return true;
  }
  function stepAlg2(dt) {
    const gx = grad(X, S.q), gu = grad(Gu, S.q);
    const r1 = S.gain * (S.xbar - X(S.q)), r2 = -S.gain * Gu(S.q);
    // damped least squares on the 2x2 system [gx; gu] qdot = [r1; r2]
    const M = [[gx[0], gx[1]], [gu[0], gu[1]]];
    const MtM = [[M[0][0] ** 2 + M[1][0] ** 2 + 1e-4, M[0][0] * M[0][1] + M[1][0] * M[1][1]],
                 [M[0][0] * M[0][1] + M[1][0] * M[1][1], M[0][1] ** 2 + M[1][1] ** 2 + 1e-4]];
    const Mtr = [M[0][0] * r1 + M[1][0] * r2, M[0][1] * r1 + M[1][1] * r2];
    const det = MtM[0][0] * MtM[1][1] - MtM[0][1] * MtM[1][0];
    let v = [(MtM[1][1] * Mtr[0] - MtM[0][1] * Mtr[1]) / det, (-MtM[1][0] * Mtr[0] + MtM[0][0] * Mtr[1]) / det];
    const n = Math.hypot(v[0], v[1]); if (n > 2.5) v = [v[0] * 2.5 / n, v[1] * 2.5 / n];
    S.q = [S.q[0] + dt * v[0], S.q[1] + dt * v[1]];
    S.msg = "";
    return true;
  }

  // ---------- drawing helpers ----------
  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const armCv = root.querySelector("#pg-arm"), csCv = root.querySelector("#pg-cs");
  const QR = { x: [-Math.PI, Math.PI], y: [-Math.PI, Math.PI] };

  function fit(cv) {
    const r = cv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const w = Math.max(10, Math.round(r.width * dpr)), h = Math.max(10, Math.round(r.height * dpr));
    if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
    return { w, h, dpr };
  }

  // marching squares on a grid -> list of segments
  function contour(f, nx, ny, level = 0) {
    const xs = [], ys = [], V = [];
    for (let i = 0; i <= nx; i++) xs.push(QR.x[0] + (QR.x[1] - QR.x[0]) * i / nx);
    for (let j = 0; j <= ny; j++) ys.push(QR.y[0] + (QR.y[1] - QR.y[0]) * j / ny);
    for (let i = 0; i <= nx; i++) { V.push([]); for (let j = 0; j <= ny; j++) V[i].push(f([xs[i], ys[j]]) - level); }
    const segs = [];
    const lerp = (a, b, va, vb) => a + (b - a) * (va / (va - vb));
    for (let i = 0; i < nx; i++) for (let j = 0; j < ny; j++) {
      const v = [V[i][j], V[i + 1][j], V[i + 1][j + 1], V[i][j + 1]];
      const p = [[xs[i], ys[j]], [xs[i + 1], ys[j]], [xs[i + 1], ys[j + 1]], [xs[i], ys[j + 1]]];
      const pts = [];
      for (let k = 0; k < 4; k++) {
        const a = v[k], b = v[(k + 1) % 4];
        if ((a < 0) !== (b < 0)) {
          const pa = p[k], pb = p[(k + 1) % 4];
          pts.push([lerp(pa[0], pb[0], a, b), lerp(pa[1], pb[1], a, b)]);
        }
      }
      if (pts.length === 2) segs.push(pts);
      else if (pts.length === 4) { segs.push([pts[0], pts[1]]); segs.push([pts[2], pts[3]]); }
    }
    return segs;
  }
  let Esegs = [], Zsegs = [];
  const recomputeCurves = () => { Esegs = contour(Gu, 150, 150); recomputeZ(); };
  const recomputeZ = () => { Zsegs = contour(X, 150, 150, S.xbar); };

  function drawArm() {
    const { w, h, dpr } = fit(armCv), c = armCv.getContext("2d");
    c.clearRect(0, 0, w, h);
    const sc = Math.min(w, h) / 5.0, bx = w / 2, by = h * 0.16;
    const P2 = (x, y) => [bx + x * sc, by - y * sc];
    const fk = (q) => [[0, 0], [P.l1 * Math.sin(q[0]), -P.l1 * Math.cos(q[0])],
      [P.l1 * Math.sin(q[0]) + P.l2 * Math.sin(q[0] + q[1]), -P.l1 * Math.cos(q[0]) - P.l2 * Math.cos(q[0] + q[1])]];
    // target line
    const tx = P2(S.xbar, 0)[0];
    c.setLineDash([8 * dpr, 7 * dpr]); c.strokeStyle = css("--target"); c.lineWidth = 2.5 * dpr;
    c.beginPath(); c.moveTo(tx, 8 * dpr); c.lineTo(tx, h - 8 * dpr); c.stroke(); c.setLineDash([]);
    c.fillStyle = css("--target"); c.font = `${15 * dpr}px ${css("--font-ui")}`;
    c.fillText("x̄  (drag)", tx + 8 * dpr, h - 14 * dpr);
    // gravity arrow
    c.strokeStyle = css("--muted"); c.fillStyle = css("--muted"); c.lineWidth = 2 * dpr;
    c.beginPath(); c.moveTo(22 * dpr, h * 0.42); c.lineTo(22 * dpr, h * 0.58); c.stroke();
    c.beginPath(); c.moveTo(16 * dpr, h * 0.56); c.lineTo(22 * dpr, h * 0.6); c.lineTo(28 * dpr, h * 0.56); c.fill();
    c.fillText("g", 32 * dpr, h * 0.52);
    // base bar
    c.strokeStyle = css("--muted"); c.lineWidth = 3 * dpr;
    c.beginPath(); c.moveTo(bx - 26 * dpr, by - 10 * dpr); c.lineTo(bx + 26 * dpr, by - 10 * dpr); c.stroke();
    const drawChain = (q, col, alpha, rings) => {
      const p = fk(q).map(([x, y]) => P2(x, y));
      c.globalAlpha = alpha; c.strokeStyle = col; c.lineWidth = 8 * dpr; c.lineCap = "round";
      c.beginPath(); c.moveTo(...p[0]); c.lineTo(...p[1]); c.lineTo(...p[2]); c.stroke();
      if (rings) for (const pt of [p[0], p[1]]) {
        c.strokeStyle = css("--actuator"); c.lineWidth = 3 * dpr;
        c.beginPath(); c.arc(pt[0], pt[1], 11 * dpr, 0, 2 * Math.PI); c.stroke();
      }
      c.fillStyle = col; c.beginPath(); c.arc(p[2][0], p[2][1], 6 * dpr, 0, 2 * Math.PI); c.fill();
      c.globalAlpha = 1; return p;
    };
    // where the arm would settle with this tendon length
    if (S.showSettle) {
      const st = settle(S.q);
      if (st && Math.hypot(st.q[0] - S.q[0], st.q[1] - S.q[1]) > 0.02) drawChain(st.q, css("--muted"), 0.45, false);
    }
    const p = drawChain(S.q, css("--fg"), 1, true);
    // tip trail
    c.strokeStyle = css("--accent2"); c.lineWidth = 2 * dpr; c.beginPath();
    S.trail.forEach((q, i) => { const t = P2(X(q), -P.l1 * Math.cos(q[0]) - P.l2 * Math.cos(q[0] + q[1])); i ? c.lineTo(...t) : c.moveTo(...t); });
    c.stroke();
    armGeom = { P2, sc, bx, by };
  }
  let armGeom = null;

  function drawCS() {
    const { w, h, dpr } = fit(csCv), c = csCv.getContext("2d");
    c.clearRect(0, 0, w, h);
    const pad = 34 * dpr;
    const toPx = (q) => [pad + (q[0] - QR.x[0]) / (QR.x[1] - QR.x[0]) * (w - 2 * pad),
                         h - pad - (q[1] - QR.y[0]) / (QR.y[1] - QR.y[0]) * (h - 2 * pad)];
    // frame + axes
    c.strokeStyle = css("--rule"); c.lineWidth = 1 * dpr; c.strokeRect(pad, pad, w - 2 * pad, h - 2 * pad);
    const o = toPx([0, 0]);
    c.beginPath(); c.moveTo(pad, o[1]); c.lineTo(w - pad, o[1]); c.moveTo(o[0], pad); c.lineTo(o[0], h - pad); c.stroke();
    c.fillStyle = css("--muted"); c.font = `${14 * dpr}px ${css("--font-ui")}`;
    c.fillText("q₁", w - pad + 6 * dpr, o[1] + 5 * dpr); c.fillText("q₂", o[0] - 6 * dpr, pad - 10 * dpr);
    c.fillText("−π", pad - 6 * dpr, h - pad + 18 * dpr); c.fillText("π", w - pad - 6 * dpr, h - pad + 18 * dpr);
    const segs = (S, col, wid, dash) => {
      c.strokeStyle = col; c.lineWidth = wid * dpr; c.setLineDash(dash ? [6 * dpr, 6 * dpr] : []);
      c.beginPath(); for (const [a, b] of S) { const pa = toPx(a), pb = toPx(b); c.moveTo(...pa); c.lineTo(...pb); } c.stroke();
      c.setLineDash([]);
    };
    segs(Zsegs, css("--target"), 2.5, true);
    segs(Esegs, css("--held"), 3.5, false);
    // trail and current point
    c.strokeStyle = css("--accent2"); c.lineWidth = 2 * dpr; c.beginPath();
    S.trail.forEach((q, i) => { const t = toPx(q); i ? c.lineTo(...t) : c.moveTo(...t); }); c.stroke();
    const p = toPx(S.q);
    c.fillStyle = css("--fg"); c.beginPath(); c.arc(p[0], p[1], 6 * dpr, 0, 2 * Math.PI); c.fill();
    // legend
    c.font = `${13 * dpr}px ${css("--font-ui")}`;
    c.fillStyle = css("--held"); c.fillText("E: held configurations", pad + 8 * dpr, pad + 18 * dpr);
    c.fillStyle = css("--target"); c.fillText("h(q) = x̄", pad + 8 * dpr, pad + 36 * dpr);
    csGeom = { toPx, pad, w, h };
  }
  let csGeom = null;

  // ---------- UI ----------
  const $ = (id) => root.querySelector("#" + id);
  const readout = () => {
    const ex = Math.abs(S.xbar - X(S.q)), gu = Math.abs(Gu(S.q));
    $("pg-ex").textContent = ex.toFixed(3);
    $("pg-gu").textContent = gu.toFixed(3);
    const eq = gu < 0.05, held = eq && kD(S.q) > 0, there = ex < 0.005;
    const pill = $("pg-state");
    pill.textContent = S.msg ? S.msg : held && there ? "At the target and held" : eq && there ? "At the target, on an unstable equilibrium" : held ? "Held, not at the target" : eq ? "On an unstable equilibrium" : there ? "At the target, but the arm cannot hold this shape" : "Neither held nor at the target";
    pill.dataset.state = S.msg ? "warn" : held && there ? "ok" : held ? "mid" : "bad";
  };
  const draw = () => { drawArm(); drawCS(); readout(); drawPlot(); };

  function bindSlider(id, key, fmt, toModel = (v) => v) {
    const el = $(id), out = $(id + "-v");
    const upd = () => { P[key] = toModel(parseFloat(el.value)); out.textContent = fmt(parseFloat(el.value)); recomputeCurves(); sMem = undefined; draw(); };
    el.addEventListener("input", upd); upd();
  }
  bindSlider("pg-g", "g", (v) => v.toFixed(1));
  bindSlider("pg-k1", "k1", (v) => v.toFixed(0));
  bindSlider("pg-k2", "k2", (v) => v.toFixed(0));
  bindSlider("pg-th", "th", (v) => v.toFixed(0) + "°", (v) => v * Math.PI / 180);
  const xs = $("pg-x");
  xs.addEventListener("input", () => { S.xbar = parseFloat(xs.value); $("pg-x-v").textContent = S.xbar.toFixed(2); recomputeZ(); draw(); });

  root.querySelectorAll("[data-algo]").forEach((b) => b.addEventListener("click", () => {
    S.algo = +b.dataset.algo;
    root.querySelectorAll("[data-algo]").forEach((x) => x.setAttribute("aria-pressed", x === b ? "true" : "false"));
    if (S.algo === 0) { setRun(false); S.q = [0, -1]; S.msg = ""; }
    if (S.algo === 1) { const st = settle(S.q); if (st) { S.q = st.q; sMem = st.s; } }
    S.trail = [S.q.slice()]; clearH(); draw();
  }));
  const runBtn = $("pg-run");
  const setRun = (r) => { S.running = r; runBtn.textContent = r ? "Pause" : "Run"; runBtn.setAttribute("aria-pressed", r ? "true" : "false"); };
  runBtn.addEventListener("click", () => { S.msg = ""; setRun(!S.running); if (S.running) { S.trail = [S.q.slice()]; clearH(); loop(); } });
  $("pg-reset").addEventListener("click", () => { setRun(false); S.q = [1.25, -1.0]; if (S.algo === 1) { const st = settle(S.q); if (st) S.q = st.q; } S.trail = [S.q.slice()]; clearH(); sMem = undefined; S.msg = ""; draw(); });
  $("pg-settle").addEventListener("change", (e) => { S.showSettle = e.target.checked; draw(); });

  // click / drag in configuration space sets the start
  let dragCS = false;
  const csPick = (ev) => {
    const r = csCv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const px = (ev.clientX - r.left) * dpr, py = (ev.clientY - r.top) * dpr, { pad, w, h } = csGeom;
    const q = [QR.x[0] + (px - pad) / (w - 2 * pad) * (QR.x[1] - QR.x[0]), QR.y[0] + (h - pad - py) / (h - 2 * pad) * (QR.y[1] - QR.y[0])];
    if (Math.abs(q[0]) > Math.PI || Math.abs(q[1]) > Math.PI) return;
    S.q = q; sMem = undefined;
    if (S.algo === 1) { const st = settle(S.q); if (st) { S.q = st.q; sMem = st.s; } }
    S.trail = [S.q.slice()]; clearH(); S.msg = ""; draw();
  };
  csCv.addEventListener("pointerdown", (e) => { dragCS = true; csCv.setPointerCapture(e.pointerId); csPick(e); });
  csCv.addEventListener("pointermove", (e) => { if (dragCS) csPick(e); });
  csCv.addEventListener("pointerup", () => { dragCS = false; });
  // drag the target on the arm canvas
  let dragT = false;
  const tPick = (ev) => {
    const r = armCv.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const x = ((ev.clientX - r.left) * dpr - armGeom.bx) / armGeom.sc;
    S.xbar = Math.max(-1.95, Math.min(1.95, x)); xs.value = S.xbar; $("pg-x-v").textContent = S.xbar.toFixed(2);
    recomputeZ(); draw();
  };
  armCv.addEventListener("pointerdown", (e) => { dragT = true; armCv.setPointerCapture(e.pointerId); tPick(e); });
  armCv.addEventListener("pointermove", (e) => { if (dragT) tPick(e); });
  armCv.addEventListener("pointerup", () => { dragT = false; });


  // ---------- convergence plot (log scale: exponential decay shows as a straight line) ----------
  function recordH() { H.ex.push(Math.abs(S.xbar - X(S.q))); H.gu.push(Math.abs(Gu(S.q))); if (H.ex.length > 900) { H.ex.shift(); H.gu.shift(); } }
  function clearH() { H.ex = []; H.gu = []; recordH(); }
  function drawPlot() {
    const cv = root.querySelector("#pg-plot"); if (!cv) return;
    const { w, h, dpr } = fit(cv), c = cv.getContext("2d");
    c.clearRect(0, 0, w, h);
    const L = 46 * dpr, R = 12 * dpr, T = 10 * dpr, B = 22 * dpr, lo = -4, hi = 1.5;
    const n = Math.max(200, H.ex.length);
    const px = (i) => L + (w - L - R) * i / (n - 1);
    const py = (v) => { const l = Math.log10(Math.max(v, 1e-4)); return T + (h - T - B) * (hi - Math.min(hi, l)) / (hi - lo); };
    c.strokeStyle = css("--rule"); c.lineWidth = 1 * dpr; c.font = `${12 * dpr}px ${css("--font-ui")}`; c.fillStyle = css("--muted");
    for (const e of [1, 0, -1, -2, -3, -4]) { const y = py(10 ** e); c.beginPath(); c.moveTo(L, y); c.lineTo(w - R, y); c.stroke(); c.fillText(e === 0 ? "1" : e === 1 ? "10" : "1e" + e, 6 * dpr, y + 4 * dpr); }
    if (w / dpr > 520) c.fillText("time →", w - R - 46 * dpr, h - 6 * dpr);
    const line = (arr, col) => { c.strokeStyle = col; c.lineWidth = 2.2 * dpr; c.beginPath(); arr.forEach((v, i) => i ? c.lineTo(px(i), py(v)) : c.moveTo(px(i), py(v))); c.stroke(); };
    line(H.ex, css("--target")); line(H.gu, css("--held"));
    c.fillStyle = css("--target"); c.fillText("task error |x̄ − x|", L + 8 * dpr, h - 6 * dpr);
    c.fillStyle = css("--held"); c.fillText("statics residual |Gᵤ|", L + 124 * dpr, h - 6 * dpr);
  }

  // ---------- guided experiments ----------
  const DEF = { "pg-g": 9.81, "pg-k1": 0, "pg-k2": 10, "pg-th": 22.5, "pg-x": 1.2 };
  function setDefaults() { for (const id in DEF) { const el = $(id); el.value = DEF[id]; el.dispatchEvent(new Event("input")); } }
  const setAlgo = (n) => root.querySelector(`[data-algo="${n}"]`).click();
  function sagAnim() {
    const st = settle(S.q); if (!st) return;
    const q0 = S.q.slice(), q1 = st.q; let t0 = null;
    const step = (ts) => { if (t0 === null) t0 = ts; const a = Math.min(1, (ts - t0) / 1400), e = a * a;
      S.q = [q0[0] + (q1[0] - q0[0]) * e, q0[1] + (q1[1] - q0[1]) * e]; S.trail.push(S.q.slice()); recordH(); draw();
      if (a < 1) requestAnimationFrame(step); };
    requestAnimationFrame(step);
  }
  new ResizeObserver(drawPlot).observe(root.querySelector("#pg-plot"));

  let raf = null;
  function loop() {
    if (!S.running) return;
    for (let k = 0; k < 2; k++) {
      const ok = S.algo === 0 ? stepPlain(0.012) : S.algo === 1 ? stepAlg1(0.012) : stepAlg2(0.012);
      if (!ok) { setRun(false); break; }
    }
    S.trail.push(S.q.slice()); if (S.trail.length > 1500) S.trail.shift(); recordH();
    const ex = Math.abs(S.xbar - X(S.q));
    const done = S.algo === 0 ? ex < 1e-4 : ex < 1e-4 && Math.abs(Gu(S.q)) < 1e-3;
    draw();
    if (done) { setRun(false); if (S.algo === 0) setTimeout(sagAnim, 700); return; }
    raf = requestAnimationFrame(loop);
  }

  // initial state: plain IK selected, elbow bent, nothing run yet
  S.msg = "Pick a method and press Run";
  S.q = [0, -1.0]; S.trail = [S.q.slice()]; clearH();
  recomputeCurves(); draw();
  new ResizeObserver(draw).observe(armCv); new ResizeObserver(draw).observe(csCv);
  const mq = window.matchMedia("(prefers-color-scheme: dark)"); mq.addEventListener && mq.addEventListener("change", draw);
  new MutationObserver(draw).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });
})();
