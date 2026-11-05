/* 3D sheet viewer: a 3-joint arm with 2 tendons. Its held configurations form a 2D sheet E inside the
   3D configuration space, drawn in actuation coordinates (q_a1, q_a2, q_u). Both strategies run live. */
(function () {
  "use strict";
  const root = document.getElementById("sheet3d");
  if (!root) return;

  // ---------- model ----------
  const L = [0.5, 0.5, 0.5], Ms = [0.4, 0.4, 0.4], K = [3, 3, 3];
  let g = 9.81;
  const C = [[1, 1, 0], [0, 1, 1], [1 / 3, -1 / 3, 1 / 3]];
  const inv3 = (m) => {
    const [a, b, c] = m[0], [d, e, f] = m[1], [h, i, j] = m[2];
    const A = e * j - f * i, B = -(d * j - f * h), Cc = d * i - e * h, det = a * A + b * B + c * Cc;
    return [[A / det, -(b * j - c * i) / det, (b * f - c * e) / det],
            [B / det, (a * j - c * h) / det, -(a * f - c * d) / det],
            [Cc / det, -(a * i - b * h) / det, (a * e - b * d) / det]];
  };
  const Ci = inv3(C);
  const mv = (m, v) => m.map((r) => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
  const fromAQ = (a1, a2, u) => mv(Ci, [a1, a2, u]);
  const toAQ = (q) => mv(C, q);
  const cum = (q) => [q[0], q[0] + q[1], q[0] + q[1] + q[2]];
  function Gq(q) {
    const c = cum(q), s = c.map(Math.sin), out = [];
    for (let k = 0; k < 3; k++) {
      let t = 0;
      for (let i = k; i < 3; i++) { let sum = 0; for (let j = k; j <= i; j++) sum += L[j] * s[j]; t += Ms[i] * sum; }
      out.push(K[k] * q[k] + g * t);
    }
    return out;
  }
  const Gu = (q) => { const G = Gq(q); return Ci[0][2] * G[0] + Ci[1][2] * G[1] + Ci[2][2] * G[2]; };
  const tip = (q) => { const c = cum(q); let x = 0, y = 0; for (let i = 0; i < 3; i++) { x += L[i] * Math.sin(c[i]); y -= L[i] * Math.cos(c[i]); } return [x, y]; };
  function quBar(a1, a2, guess = 0) {
    const f = (u) => Gu(fromAQ(a1, a2, u));
    let u = guess;
    for (let i = 0; i < 40; i++) { const v = f(u), d = (f(u + 1e-6) - f(u - 1e-6)) / 2e-6; const st = v / d; u -= Math.max(-0.4, Math.min(0.4, st)); if (Math.abs(st) < 1e-11) break; }
    if (Math.abs(f(u)) < 1e-7) return u;
    let lo = -3, hi = 3; for (let k = 0; k < 60; k++) { const m = (lo + hi) / 2; if (f(lo) * f(m) <= 0) hi = m; else lo = m; }
    return (lo + hi) / 2;
  }
  const ha = (a) => fromAQ(a[0], a[1], quBar(a[0], a[1]));
  let XBAR;
  { const g0 = g; g = 9.81; XBAR = tip(ha([1.0, 0.8])); g = g0; }

  const solve3 = (M, b) => { const I = inv3(M); return mv(I, b); };
  const jac = (f, x, e = 1e-6) => {
    const n = x.length, f0 = f(x), cols = [];
    for (let i = 0; i < n; i++) { const xp = x.slice(), xm = x.slice(); xp[i] += e; xm[i] -= e; const a = f(xp), b = f(xm); cols.push(a.map((v, k) => (v - b[k]) / (2 * e))); }
    return f0.map((_, r) => cols.map((c) => c[r]));
  };
  function runAlg1(a0, steps = 350, dt = 0.02) {
    let a = a0.slice(); const out = [];
    for (let k = 0; k < steps; k++) {
      const f = (x) => tip(ha(x));
      const J = jac(f, a), e = f(a), r = [XBAR[0] - e[0], XBAR[1] - e[1]];
      // damped least squares 2x2
      const JtJ = [[J[0][0] ** 2 + J[1][0] ** 2 + 1e-3, J[0][0] * J[0][1] + J[1][0] * J[1][1]], [0, J[0][1] ** 2 + J[1][1] ** 2 + 1e-3]];
      JtJ[1][0] = JtJ[0][1];
      const Jr = [J[0][0] * r[0] + J[1][0] * r[1], J[0][1] * r[0] + J[1][1] * r[1]];
      const det = JtJ[0][0] * JtJ[1][1] - JtJ[0][1] * JtJ[1][0];
      let v = [1.2 * (JtJ[1][1] * Jr[0] - JtJ[0][1] * Jr[1]) / det, 1.2 * (-JtJ[1][0] * Jr[0] + JtJ[0][0] * Jr[1]) / det];
      const n = Math.hypot(...v); if (n > 2.5) v = v.map((x) => x * 2.5 / n);
      a = [a[0] + dt * v[0], a[1] + dt * v[1]];
      out.push(ha(a));
    }
    return out;
  }
  function runAlg2(q0, steps = 350, dt = 0.02) {
    let q = q0.slice(); const out = [q.slice()];
    for (let k = 0; k < steps; k++) {
      const Jh = jac(tip, q), gu = jac((x) => [Gu(x)], q)[0];
      const M = [Jh[0], Jh[1], gu], e = tip(q);
      const b = [1.2 * (XBAR[0] - e[0]), 1.2 * (XBAR[1] - e[1]), -1.2 * Gu(q)];
      const MtM = [0, 1, 2].map((i) => [0, 1, 2].map((j) => M[0][i] * M[0][j] + M[1][i] * M[1][j] + M[2][i] * M[2][j] + (i === j ? 1e-6 : 0)));
      const Mtb = [0, 1, 2].map((i) => M[0][i] * b[0] + M[1][i] * b[1] + M[2][i] * b[2]);
      let v = solve3(MtM, Mtb); const n = Math.hypot(...v); if (n > 2.5) v = v.map((x) => x * 2.5 / n);
      q = q.map((x, i) => x + dt * v[i]); out.push(q.slice());
    }
    return out;
  }
  function zCurve() {
    const q0 = ha([1.0, 0.8]); const pts = [];
    for (const d of [1, -1]) {
      let q = q0.slice(); const seg = [];
      for (let t = 0; t < 90; t++) {
        const q3 = q0[2] + d * t * 0.03; let x = [q[0], q[1]];
        let ok = false;
        for (let it = 0; it < 30; it++) {
          const f = (y) => { const p = tip([y[0], y[1], q3]); return [p[0] - XBAR[0], p[1] - XBAR[1]]; };
          const F = f(x), J = jac(f, x), det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
          if (Math.abs(det) < 1e-9) break;
          const dx = [(J[1][1] * F[0] - J[0][1] * F[1]) / det, (-J[1][0] * F[0] + J[0][0] * F[1]) / det];
          x = [x[0] - dx[0], x[1] - dx[1]];
          if (Math.hypot(...F) < 1e-10) { ok = true; break; }
        }
        if (!ok) break;
        q = [x[0], x[1], q3]; const a = toAQ(q);
        if (Math.abs(a[0]) > 1.6 || Math.abs(a[1]) > 1.6 || Math.abs(a[2]) > 0.6) break;
        seg.push(a);
      }
      if (d === 1) pts.push(...seg); else pts.unshift(...seg.reverse());
    }
    return pts;
  }

  // ---------- scene ----------
  const R = 1.4, N = 22, ZS = 2.4;
  let surf = [], star = null, Z = [];
  function build() {
    surf = [];
    for (let i = 0; i <= N; i++) { surf.push([]); for (let j = 0; j <= N; j++) { const a1 = -R + 2 * R * i / N, a2 = -R + 2 * R * j / N; let u = quBar(a1, a2, j ? surf[i][j - 1][2] : 0); surf[i].push([a1, a2, u]); } }
    const tr = runAlg1([1.0, 0.8], 250); star = toAQ(tr[tr.length - 1]);
    Z = zCurve();
  }
  const runs = []; const COLS = ["#5fb3e6", "#53c2a8", "#b393e6", "#e68fb0", "#e6c35f", "#8fd06b"];
  let yaw = -0.85, pitch = 0.95, auto = true, last = 0;

  const cv = root.querySelector("#s3-canvas"), armCv = root.querySelector("#s3-arm");
  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const fit = (c) => { const r = c.getBoundingClientRect(), d = window.devicePixelRatio || 1; const w = Math.max(10, Math.round(r.width * d)), h = Math.max(10, Math.round(r.height * d)); if (c.width !== w || c.height !== h) { c.width = w; c.height = h; } return { w, h, d }; };
  const hex2 = (h) => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
  const ramp = (u) => {   // blue -> teal -> green, by height q_u
    const stops = [[-0.45, "#2a5d8f"], [0, "#2f9c97"], [0.45, "#8bc96a"]];
    const t = Math.max(-0.45, Math.min(0.45, u));
    const k = t < 0 ? 0 : 1, a = stops[k], b = stops[k + 1], s = (t - a[0]) / (b[0] - a[0]);
    const A = hex2(a[1]), B = hex2(b[1]); return A.map((x, i) => x + (B[i] - x) * s);
  };

  function proj(p, w, h) {
    // world: x = q_a1, y = q_a2, z = q_u * ZS
    const x = p[0], y = p[1], z = p[2] * ZS;
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    const x1 = cy * x - sy * y, y1 = sy * x + cy * y;
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const yv = cp * y1 - sp * z, zv = sp * y1 + cp * z;   // zv: screen up, yv: depth
    const sc = Math.min(w, h) / 4.6, f = 6 / (6 + yv);
    return [w / 2 + x1 * sc * f, h * 0.53 - zv * sc * f, yv];
  }

  function draw(ts) {
    const { w, h, d } = fit(cv), c = cv.getContext("2d");
    c.clearRect(0, 0, w, h);
    // flat sheet (fully actuated picture), optional
    if (root.querySelector("#s3-flat").checked) {
      const corners = [[-R, -R, 0], [R, -R, 0], [R, R, 0], [-R, R, 0]].map((p) => proj(p, w, h));
      c.fillStyle = css("--muted"); c.globalAlpha = 0.16; c.beginPath(); corners.forEach((p, i) => i ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1])); c.closePath(); c.fill(); c.globalAlpha = 1;
    }
    // axes
    const ax = [[[-1.6, 0, 0], [1.7, 0, 0], "q_a1"], [[0, -1.6, 0], [0, 1.7, 0], "q_a2"], [[0, 0, -0.55], [0, 0, 0.62], "q_u"]];
    c.lineWidth = 1.2 * d; c.strokeStyle = css("--muted"); c.fillStyle = css("--muted"); c.font = `${13 * d}px ${css("--font-ui")}`;
    for (const [a, b, lab] of ax) { const pa = proj(a, w, h), pb = proj(b, w, h); c.beginPath(); c.moveTo(pa[0], pa[1]); c.lineTo(pb[0], pb[1]); c.stroke(); c.fillText(lab.replace("_", " "), pb[0] + 5 * d, pb[1]); }
    // surface quads, painter's order
    const quads = [];
    const light = [-0.4, -0.5, 0.77];
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
      const P4 = [surf[i][j], surf[i + 1][j], surf[i + 1][j + 1], surf[i][j + 1]];
      const S = P4.map((p) => proj(p, w, h));
      const depth = (S[0][2] + S[1][2] + S[2][2] + S[3][2]) / 4;
      const ux = [P4[1][0] - P4[0][0], P4[1][1] - P4[0][1], (P4[1][2] - P4[0][2]) * ZS], vx = [P4[3][0] - P4[0][0], P4[3][1] - P4[0][1], (P4[3][2] - P4[0][2]) * ZS];
      const nrm = [ux[1] * vx[2] - ux[2] * vx[1], ux[2] * vx[0] - ux[0] * vx[2], ux[0] * vx[1] - ux[1] * vx[0]];
      const nl = Math.hypot(...nrm); const lam = Math.abs((nrm[0] * light[0] + nrm[1] * light[1] + nrm[2] * light[2]) / nl);
      const col = ramp((P4[0][2] + P4[2][2]) / 2).map((x) => Math.round(x * (0.55 + 0.45 * lam)));
      quads.push({ S, depth, col });
    }
    quads.sort((a, b) => b.depth - a.depth);
    for (const q of quads) {
      c.fillStyle = `rgba(${q.col[0]},${q.col[1]},${q.col[2]},0.88)`; c.strokeStyle = c.fillStyle; c.lineWidth = 0.6 * d;
      c.beginPath(); q.S.forEach((p, k) => k ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1])); c.closePath(); c.fill(); c.stroke();
    }
    // grid lines of constant actuation on the sheet
    c.strokeStyle = css("--actuator"); c.lineWidth = 0.9 * d; c.globalAlpha = 0.35;
    for (let i = 0; i <= N; i += 2) {
      c.beginPath(); for (let j = 0; j <= N; j++) { const p = proj(surf[i][j], w, h); j ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); } c.stroke();
      c.beginPath(); for (let j = 0; j <= N; j++) { const p = proj(surf[j][i], w, h); j ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); } c.stroke();
    }
    c.globalAlpha = 1;
    // target set h(q) = x̄
    c.strokeStyle = css("--target"); c.lineWidth = 3 * d; c.beginPath();
    Z.forEach((a, k) => { const p = proj(a, w, h); k ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); }); c.stroke();
    // trajectories
    const now = performance.now();
    for (const r of runs) {
      const n = Math.min(r.pts.length, Math.floor((now - r.t0) / 12) + 1); r.n = n;
      c.strokeStyle = r.col; c.lineWidth = 3.6 * d; c.beginPath();
      for (let k = 0; k < n; k++) { const p = proj(r.pts[k], w, h); k ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); } c.stroke();
      const p = proj(r.pts[n - 1], w, h); c.fillStyle = r.col; c.beginPath(); c.arc(p[0], p[1], 5 * d, 0, 2 * Math.PI); c.fill();
      const p0 = proj(r.pts[0], w, h); c.strokeStyle = r.col; c.lineWidth = 2 * d; c.beginPath(); c.arc(p0[0], p0[1], 5 * d, 0, 2 * Math.PI); c.stroke();
    }
    if (star) { const p = proj(star, w, h); c.strokeStyle = css("--fg"); c.lineWidth = 2.5 * d; c.beginPath(); c.arc(p[0], p[1], 9 * d, 0, 2 * Math.PI); c.stroke(); }
    drawArm();
    if (auto) yaw += (ts - last || 16) * 0.00012;
    last = ts;
    requestAnimationFrame(draw);
  }

  function drawArm() {
    const { w, h, d } = fit(armCv), c = armCv.getContext("2d");
    c.clearRect(0, 0, w, h);
    const sc = Math.min(w, h) / 3.6, bx = w * 0.42, by = h * 0.12;
    const P2 = (x, y) => [bx + x * sc, by - y * sc];
    c.strokeStyle = css("--muted"); c.lineWidth = 3 * d; c.beginPath(); c.moveTo(bx - 20 * d, by - 8 * d); c.lineTo(bx + 20 * d, by - 8 * d); c.stroke();
    const t = P2(XBAR[0], XBAR[1]); c.strokeStyle = css("--target"); c.lineWidth = 3 * d;
    c.beginPath(); c.moveTo(t[0] - 8 * d, t[1] - 8 * d); c.lineTo(t[0] + 8 * d, t[1] + 8 * d); c.moveTo(t[0] + 8 * d, t[1] - 8 * d); c.lineTo(t[0] - 8 * d, t[1] + 8 * d); c.stroke();
    const chains = runs.length ? runs.map((r) => [r.qs[Math.max(0, (r.n || 1) - 1)], r.col]) : [[ha([1.0, 0.8]), css("--fg")]];
    for (const [q, col] of chains) {
      const cc = cum(q); let x = 0, y = 0; const pts = [P2(0, 0)];
      for (let i = 0; i < 3; i++) { x += L[i] * Math.sin(cc[i]); y -= L[i] * Math.cos(cc[i]); pts.push(P2(x, y)); }
      c.strokeStyle = col; c.lineWidth = 6 * d; c.lineCap = "round"; c.beginPath(); pts.forEach((p, k) => k ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1])); c.stroke();
      c.strokeStyle = css("--actuator"); c.lineWidth = 2 * d;
      for (let k = 0; k < 3; k++) { c.beginPath(); c.arc(pts[k][0], pts[k][1], 8 * d, 0, 2 * Math.PI); c.stroke(); }
    }
  }

  // ---------- UI ----------
  let colI = 0;
  const addRun = (kind) => {
    let qs;
    if (kind === 1) { const a0 = [-1.3 + 2.6 * Math.random(), -1.3 + 2.6 * Math.random()]; qs = [ha(a0)].concat(runAlg1(a0)); }
    else { const a = [-1.3 + 2.6 * Math.random(), -1.3 + 2.6 * Math.random(), -0.45 + 0.9 * Math.random()]; qs = runAlg2(fromAQ(...a)); }
    runs.push({ qs, pts: qs.map(toAQ), col: COLS[colI++ % COLS.length], t0: performance.now() });
    if (runs.length > 6) runs.shift();
  };
  root.querySelector("#s3-on").addEventListener("click", () => addRun(1));
  root.querySelector("#s3-any").addEventListener("click", () => addRun(2));
  root.querySelector("#s3-clear").addEventListener("click", () => { runs.length = 0; });
  const gs = root.querySelector("#s3-g"), gv = root.querySelector("#s3-g-v");
  gs.addEventListener("input", () => { g = parseFloat(gs.value); gv.textContent = g.toFixed(1); runs.length = 0; build(); });
  let drag = null;
  cv.addEventListener("pointerdown", (e) => { drag = [e.clientX, e.clientY, yaw, pitch]; auto = false; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener("pointermove", (e) => { if (!drag) return; yaw = drag[2] + (e.clientX - drag[0]) * 0.008; pitch = Math.max(0.15, Math.min(1.45, drag[3] + (e.clientY - drag[1]) * 0.006)); });
  cv.addEventListener("pointerup", () => { drag = null; });
  if (window.matchMedia("(prefers-reduced-motion: reduce)").matches) auto = false;

  build();
  addRun(2); addRun(1);
  requestAnimationFrame(draw);
})();
