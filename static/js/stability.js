/* =====================================================================
   POIL — interactive Point-set BCSDM contraction demo.

   Terminal-phase dynamics, exactly as analysed in the paper:

       W      = sum_i  x_i^b x_i^b^T          (point-set inertia, x_i^b in body frame)
       A      = tr(W) I - W                   (unit-mass inertia of the point set)
       w(R)   = A^{-1} ( R^T W - W R )^v
       Rdot   = R [w(R)]
       D(R)   = tr(W) - tr(W R)               Ddot = -||w(R)||_A^2 <= 0

   D is the classical navigation function on SO(3). With distinct eigenvalues
   of W it has exactly four critical points: the goal R = I and the three
   pi-rotations about the principal axes of W, all of which are non-minima.
   Their stable manifolds have dimension at most two, so every trajectory
   outside a measure-zero set converges to I.

   The box has three different edge lengths, which is what makes the
   eigenvalues of W distinct. Rendering is plain canvas 2D, no dependencies.
   ===================================================================== */
(function () {
  'use strict';

  var root = document.getElementById('stability-demo');
  if (!root) return;

  /* ---------------- tiny 3x3 / vector helpers ---------------- */

  function matMul(a, b) {
    var o = new Float64Array(9);
    for (var i = 0; i < 3; i++)
      for (var j = 0; j < 3; j++) {
        var s = 0;
        for (var k = 0; k < 3; k++) s += a[i * 3 + k] * b[k * 3 + j];
        o[i * 3 + j] = s;
      }
    return o;
  }
  function matVec(m, v) {
    return [
      m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
      m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
      m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
    ];
  }
  function identity() { return new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]); }

  // Rodrigues: rotation of angle about a unit axis.
  function axisAngle(axis, ang) {
    var n = Math.hypot(axis[0], axis[1], axis[2]);
    if (n < 1e-12) return identity();
    var x = axis[0] / n, y = axis[1] / n, z = axis[2] / n;
    var c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
    return new Float64Array([
      t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
      t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
      t * x * z - s * y, t * y * z + s * x, t * z * z + c
    ]);
  }

  // Axis-angle of a rotation, returned as the ball-model point  u = axis * (angle / pi).
  function ballPoint(R) {
    var tr = R[0] + R[4] + R[8];
    var c = Math.max(-1, Math.min(1, (tr - 1) / 2));
    var ang = Math.acos(c);
    if (ang < 1e-9) return [0, 0, 0];
    var ax;
    if (Math.PI - ang > 1e-4) {
      var s = 2 * Math.sin(ang);
      ax = [(R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s];
    } else {
      // near pi: read the axis off the symmetric part
      var d = [(R[0] + 1) / 2, (R[4] + 1) / 2, (R[8] + 1) / 2];
      var k = d[0] > d[1] ? (d[0] > d[2] ? 0 : 2) : (d[1] > d[2] ? 1 : 2);
      ax = [0, 0, 0];
      ax[k] = Math.sqrt(Math.max(0, d[k]));
      var o1 = (k + 1) % 3, o2 = (k + 2) % 3;
      ax[o1] = (R[k * 3 + o1] + R[o1 * 3 + k]) / (4 * ax[k] || 1e-9);
      ax[o2] = (R[k * 3 + o2] + R[o2 * 3 + k]) / (4 * ax[k] || 1e-9);
    }
    var n = Math.hypot(ax[0], ax[1], ax[2]) || 1;
    var r = ang / Math.PI;
    return [ax[0] / n * r, ax[1] / n * r, ax[2] / n * r];
  }

  // Re-orthonormalise (Gram-Schmidt) so numerical drift never leaves SO(3).
  function reortho(R) {
    var c0 = [R[0], R[3], R[6]], c1 = [R[1], R[4], R[7]];
    var n0 = Math.hypot(c0[0], c0[1], c0[2]);
    c0 = [c0[0] / n0, c0[1] / n0, c0[2] / n0];
    var d = c0[0] * c1[0] + c0[1] * c1[1] + c0[2] * c1[2];
    c1 = [c1[0] - d * c0[0], c1[1] - d * c0[1], c1[2] - d * c0[2]];
    var n1 = Math.hypot(c1[0], c1[1], c1[2]);
    c1 = [c1[0] / n1, c1[1] / n1, c1[2] / n1];
    var c2 = [
      c0[1] * c1[2] - c0[2] * c1[1],
      c0[2] * c1[0] - c0[0] * c1[2],
      c0[0] * c1[1] - c0[1] * c1[0]
    ];
    return new Float64Array([
      c0[0], c1[0], c2[0],
      c0[1], c1[1], c2[1],
      c0[2], c1[2], c2[2]
    ]);
  }

  /* ---------------- the object and its dynamics ---------------- */

  // Half-extents of the box. Three different values -> distinct eigenvalues of W,
  // which is the condition the analysis needs. Fixed.
  var half = [0.85, 0.65, 0.48];

  function corners() {
    var p = [];
    for (var sx = -1; sx <= 1; sx += 2)
      for (var sy = -1; sy <= 1; sy += 2)
        for (var sz = -1; sz <= 1; sz += 2)
          p.push([sx * half[0], sy * half[1], sz * half[2]]);
    return p;
  }

  // W = sum x x^T over the corners; diagonal because the box is axis-aligned in body frame.
  function inertia() {
    var w = [0, 0, 0];
    corners().forEach(function (p) {
      w[0] += p[0] * p[0]; w[1] += p[1] * p[1]; w[2] += p[2] * p[2];
    });
    return w;                       // diag(W)
  }

  // --- the controller, run literally ---------------------------------------
  //
  //   1. every tracked point gets a contraction velocity toward its target
  //         v_i = xdot_i*(tau*) + eta (x_i*(tau*) - x_i)      here xdot* = 0, eta = 1
  //   2. the object is held in a fixed grasp, so only ONE rigid motion is available:
  //         (v, w) = argmin sum_i || v + w x (x_i - xbar) - v_i ||^2
  //
  // Solving 2 in closed form gives the paper's A^{-1}(R^T W - W R)^v; we keep the two
  // steps apart so the per-point field is available to draw. Both agree to ~1e-15.

  // Solve a symmetric positive-definite 3x3 system by Cramer's rule.
  function det3(m) {
    return m[0] * (m[4] * m[8] - m[5] * m[7])
         - m[1] * (m[3] * m[8] - m[5] * m[6])
         + m[2] * (m[3] * m[7] - m[4] * m[6]);
  }
  function solve3(M, b) {
    var d = det3(M);
    if (Math.abs(d) < 1e-14) return [0, 0, 0];
    var out = [];
    for (var c = 0; c < 3; c++) {
      var Mc = M.slice();
      Mc[c] = b[0]; Mc[3 + c] = b[1]; Mc[6 + c] = b[2];
      out.push(det3(Mc) / d);
    }
    return out;
  }
  function cross(a, b) {
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  }

  // Step 1: where every tracked point is, and how the contraction field wants it to move.
  function pointField(R, eta) {
    if (eta === undefined) eta = 1;
    var targets = corners();            // x_i*(tau*) — the goal configuration, R = I
    var x = [], v = [];
    for (var i = 0; i < targets.length; i++) {
      var xi = matVec(R, targets[i]);
      x.push(xi);
      v.push([
        eta * (targets[i][0] - xi[0]),
        eta * (targets[i][1] - xi[1]),
        eta * (targets[i][2] - xi[2])
      ]);
    }
    return { x: x, v: v };
  }

  // Step 2: least-squares rigid-body twist through that field. Returns the BODY
  // angular velocity, so Rdot = R [w]. The linear part is zero here (centroid fixed).
  function omega(R) {
    var f = pointField(R);
    var n = f.x.length;
    var xbar = [0, 0, 0], vbar = [0, 0, 0], i;
    for (i = 0; i < n; i++) {
      xbar[0] += f.x[i][0]; xbar[1] += f.x[i][1]; xbar[2] += f.x[i][2];
      vbar[0] += f.v[i][0]; vbar[1] += f.v[i][1]; vbar[2] += f.v[i][2];
    }
    for (i = 0; i < 3; i++) { xbar[i] /= n; vbar[i] /= n; }

    // normal equations:  [ sum_i (r.r) I - r r^T ] w = sum_i r x (v_i - vbar)
    var M = new Float64Array(9), rhs = [0, 0, 0];
    for (i = 0; i < n; i++) {
      var r = [f.x[i][0] - xbar[0], f.x[i][1] - xbar[1], f.x[i][2] - xbar[2]];
      var dv = [f.v[i][0] - vbar[0], f.v[i][1] - vbar[1], f.v[i][2] - vbar[2]];
      var rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
      for (var a = 0; a < 3; a++)
        for (var b = 0; b < 3; b++)
          M[a * 3 + b] += (a === b ? rr : 0) - r[a] * r[b];
      var c = cross(r, dv);
      rhs[0] += c[0]; rhs[1] += c[1]; rhs[2] += c[2];
    }
    var ws = solve3(M, rhs);            // spatial angular velocity
    return [                             // -> body frame:  w_body = R^T w_spatial
      R[0] * ws[0] + R[3] * ws[1] + R[6] * ws[2],
      R[1] * ws[0] + R[4] * ws[1] + R[7] * ws[2],
      R[2] * ws[0] + R[5] * ws[1] + R[8] * ws[2]
    ];
  }

  function potential(R) {
    var w = inertia();
    var trW = w[0] + w[1] + w[2];
    return trW - (w[0] * R[0] + w[1] * R[4] + w[2] * R[8]);
  }

  // One integration step of Rdot = R [w(R)], on the manifold via the exponential map.
  function step(R, dt) {
    var om = omega(R);
    var n = Math.hypot(om[0], om[1], om[2]);
    if (n < 1e-14) return R;
    return reortho(matMul(R, axisAngle(om, n * dt)));
  }

  function angleToGoal(R) {
    var c = Math.max(-1, Math.min(1, (R[0] + R[4] + R[8] - 1) / 2));
    return Math.acos(c) * 180 / Math.PI;
  }

  /* ---------------- tracers ---------------- */

  var COLORS = {
    generic: '#143163',   // converges
    saddle:  '#b3261e',   // sits on an unstable critical point forever
    nudged:  '#d9822b'    // starts a hair off it, lingers, then escapes
  };

  var tracers = [];

  function makeTracer(key, label, R0) {
    return { key: key, label: label, R0: R0, R: R0, trail: [ballPoint(R0)], hist: [potential(R0)] };
  }

  function randomRotation() {
    // uniform-ish: random axis, random angle away from the goal so the run is visible
    var v = [Math.random() * 2 - 1, Math.random() * 2 - 1, Math.random() * 2 - 1];
    var n = Math.hypot(v[0], v[1], v[2]) || 1;
    var ang = 1.6 + Math.random() * 1.4;      // ~92deg .. ~172deg
    return axisAngle([v[0] / n, v[1] / n, v[2] / n], ang);
  }

  var saddleAxis = 1;   // which principal axis the stuck tracer sits on

  function resetTracers(newRandom) {
    var e = [[1, 0, 0], [0, 1, 0], [0, 0, 1]][saddleAxis];
    var R0generic = newRandom || !tracers.length ? randomRotation() : tracers[0].R0;
    var Rsaddle = axisAngle(e, Math.PI);
    // Half a degree short of the pi-rotation. That offset lies along the ONE direction
    // that is always unstable at R*_k -- rotation about e_k itself, whose Hessian entry is
    // -(lambda_i + lambda_j) < 0 -- so this start provably leaves the saddle.
    var Rnudged = axisAngle(e, Math.PI - 0.5 * Math.PI / 180);

    tracers = [
      makeTracer('generic', 'Random start', R0generic),
      makeTracer('nudged', '0.5° off a saddle', Rnudged),
      makeTracer('saddle', 'Exactly on it', Rsaddle)
    ];
    time = 0;
  }

  /* ---------------- camera + projection ---------------- */

  var cam = { yaw: -0.62, pitch: 0.42 };

  function camMatrix() {
    var cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw);
    var cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
    // yaw about z (up in object terms), then pitch about the camera x
    var Ry = new Float64Array([cy, -sy, 0, sy, cy, 0, 0, 0, 1]);
    var Rp = new Float64Array([1, 0, 0, 0, cp, -sp, 0, sp, cp]);
    return matMul(Rp, Ry);
  }

  // project a world point to canvas pixels (orthographic; y up, z toward viewer)
  function project(p, C, cx, cy, scale) {
    var q = matVec(C, p);
    return { x: cx + q[0] * scale, y: cy - q[2] * scale, depth: q[1] };
  }

  function dpr() { return Math.min(window.devicePixelRatio || 1, 2); }

  function fit(canvas, cssHeight) {
    var r = dpr();
    var w = canvas.clientWidth || 380;
    canvas.width = Math.round(w * r);
    canvas.height = Math.round(cssHeight * r);
    canvas.style.height = cssHeight + 'px';
    var ctx = canvas.getContext('2d');
    ctx.setTransform(r, 0, 0, r, 0, 0);
    return { ctx: ctx, w: w, h: cssHeight };
  }

  /* ---------------- panel 1: the box ---------------- */

  var boxCanvas = document.getElementById('demo-box');

  var EDGES = [
    [0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3],
    [2, 6], [3, 7], [4, 5], [4, 6], [5, 7], [6, 7]
  ];

  var SHORT = { generic: 'random', nudged: '0.5\u00b0 off', saddle: 'on the saddle' };
  var MARK = 0;    // the tracked point we draw large, to expose the box's own symmetry
  var ARROW = 0.30;  // how long a unit of per-point velocity is drawn, in body units

  function drawBox() {
    var f = fit(boxCanvas, 300), ctx = f.ctx;
    ctx.clearRect(0, 0, f.w, f.h);
    var C = camMatrix();
    var pts = corners();

    var n = tracers.length || 1;
    var slotW = f.w / n;
    var cy = f.h * 0.48;
    var scale = Math.min(slotW * 0.36, f.h * 0.29);

    tracers.forEach(function (t, i) {
      var cx = slotW * (i + 0.5);
      var col = COLORS[t.key];

      // goal pose (R = I), a faint dashed ghost behind each run
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = 'rgba(93,100,116,.4)';
      ctx.lineWidth = 1;
      drawWire(ctx, pts, identity(), C, cx, cy, scale);
      ctx.setLineDash([]);

      // current pose
      ctx.strokeStyle = col;
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      drawWire(ctx, pts, t.R, C, cx, cy, scale);

      // step 1 of the controller: the per-point contraction velocities.
      // At the saddle these are plainly non-zero, yet no rigid motion fits them --
      // which is exactly why that run does not move.
      var fld = pointField(t.R);
      ctx.strokeStyle = col; ctx.fillStyle = col;
      ctx.lineWidth = 1.2; ctx.globalAlpha = 0.55;
      fld.x.forEach(function (xi, k) {
        var vk = fld.v[k];
        var L = Math.hypot(vk[0], vk[1], vk[2]);
        if (L < 1e-3) return;
        var tip = [xi[0] + vk[0] * ARROW, xi[1] + vk[1] * ARROW, xi[2] + vk[2] * ARROW];
        var a = project(xi, C, cx, cy, scale);
        var b = project(tip, C, cx, cy, scale);
        var dx = b.x - a.x, dy = b.y - a.y, d = Math.hypot(dx, dy);
        if (d < 2) return;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        var ux = dx / d, uy = dy / d, hw = 2.2, hl = 5;
        ctx.beginPath();
        ctx.moveTo(b.x, b.y);
        ctx.lineTo(b.x - ux * hl - uy * hw, b.y - uy * hl + ux * hw);
        ctx.lineTo(b.x - ux * hl + uy * hw, b.y - uy * hl - ux * hw);
        ctx.closePath(); ctx.fill();
      });
      ctx.globalAlpha = 1;

      // the tracked points themselves
      pts.forEach(function (p) {
        var q = project(matVec(t.R, p), C, cx, cy, scale);
        ctx.fillStyle = col;
        ctx.globalAlpha = q.depth > 0 ? 0.4 : 1;
        ctx.beginPath(); ctx.arc(q.x, q.y, 2.8, 0, 6.2832); ctx.fill();
        ctx.globalAlpha = 1;
      });

      // A pi-rotation about a principal axis maps the box onto itself, so the stuck run
      // is shape-identical to the goal. Mark one point to show it is a *different* point
      // sitting there -- which is exactly what the point-set potential sees.
      var mGoal = project(pts[MARK], C, cx, cy, scale);
      ctx.strokeStyle = 'rgba(93,100,116,.75)'; ctx.lineWidth = 1.3;
      ctx.beginPath(); ctx.arc(mGoal.x, mGoal.y, 5.5, 0, 6.2832); ctx.stroke();

      var mNow = project(matVec(t.R, pts[MARK]), C, cx, cy, scale);
      ctx.fillStyle = col;
      ctx.beginPath(); ctx.arc(mNow.x, mNow.y, 5.5, 0, 6.2832); ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();

      // label + how far this run still is from the goal
      ctx.textAlign = 'center';
      ctx.fillStyle = col;
      ctx.font = '600 11px Poppins, system-ui, sans-serif';
      ctx.fillText(SHORT[t.key] || t.key, cx, f.h - 26);
      ctx.fillStyle = '#5d6474';
      ctx.font = '11px ui-monospace, monospace';
      ctx.fillText('\u2220 ' + angleToGoal(t.R).toFixed(1) + '\u00b0', cx, f.h - 10);
      ctx.textAlign = 'left';
    });

    ctx.fillStyle = '#5d6474';
    ctx.font = '11px ui-monospace, monospace';
    ctx.fillText('dashed = goal R = I', 12, 18);
    ctx.fillText('\u25cb one point at its goal   \u25cf where it is now', 12, 34);
    ctx.fillText('arrows = per-point contraction velocity', 12, 50);
  }

  function drawWire(ctx, pts, R, C, cx, cy, scale) {
    ctx.beginPath();
    EDGES.forEach(function (e) {
      var a = project(matVec(R, pts[e[0]]), C, cx, cy, scale);
      var b = project(matVec(R, pts[e[1]]), C, cx, cy, scale);
      ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
    });
    ctx.stroke();
  }

  /* ---------------- panel 2: the rotation ball ---------------- */

  var ballCanvas = document.getElementById('demo-ball');

  function drawBall() {
    var f = fit(ballCanvas, 300), ctx = f.ctx;
    ctx.clearRect(0, 0, f.w, f.h);
    var C = camMatrix(), cx = f.w / 2, cy = f.h / 2;
    var scale = Math.min(f.w, f.h) * 0.34;

    // wireframe sphere: the boundary is the set of pi-rotations
    ctx.strokeStyle = '#d7dbe5'; ctx.lineWidth = 1;
    for (var k = -2; k <= 2; k++) {
      var lat = k * Math.PI / 6;
      ctx.beginPath();
      for (var i = 0; i <= 64; i++) {
        var t = i / 64 * Math.PI * 2;
        var p = [Math.cos(lat) * Math.cos(t), Math.cos(lat) * Math.sin(t), Math.sin(lat)];
        var q = project(p, C, cx, cy, scale);
        i ? ctx.lineTo(q.x, q.y) : ctx.moveTo(q.x, q.y);
      }
      ctx.stroke();
    }
    for (var m = 0; m < 6; m++) {
      var lon = m * Math.PI / 6;
      ctx.beginPath();
      for (var j = 0; j <= 64; j++) {
        var s = -Math.PI / 2 + j / 64 * Math.PI;
        var p2 = [Math.cos(s) * Math.cos(lon), Math.cos(s) * Math.sin(lon), Math.sin(s)];
        var q2 = project(p2, C, cx, cy, scale);
        j ? ctx.lineTo(q2.x, q2.y) : ctx.moveTo(q2.x, q2.y);
      }
      ctx.stroke();
    }
    // silhouette
    ctx.strokeStyle = '#b9bfcd'; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.arc(cx, cy, scale, 0, 6.2832); ctx.stroke();

    // the three unstable critical points: pi-rotations about the principal axes.
    // +e_k and -e_k are the same rotation, so the six marks are three points.
    var axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    axes.forEach(function (a, i) {
      [1, -1].forEach(function (s) {
        var q = project([a[0] * s, a[1] * s, a[2] * s], C, cx, cy, scale);
        ctx.globalAlpha = q.depth > 0 ? .35 : 1;
        ctx.fillStyle = '#b3261e';
        ctx.beginPath(); ctx.arc(q.x, q.y, 5, 0, 6.2832); ctx.fill();
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.4; ctx.stroke();
        if (s > 0) {
          ctx.fillStyle = '#b3261e';
          ctx.font = '10px ui-monospace, monospace';
          var lx = Math.min(q.x + 16, f.w - 26);
          ctx.fillText('R*' + (i + 1), lx, Math.max(12, q.y - 6));
        }
        ctx.globalAlpha = 1;
      });
    });

    // goal at the centre
    var g = project([0, 0, 0], C, cx, cy, scale);
    ctx.fillStyle = '#1f8a4c';
    ctx.beginPath(); ctx.arc(g.x, g.y, 5.5, 0, 6.2832); ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.6; ctx.stroke();
    ctx.fillStyle = '#1f8a4c'; ctx.font = '10px ui-monospace, monospace';
    ctx.fillText('R = I', g.x + 9, g.y + 4);

    // trails, stuck run last so it is never painted over
    var order = tracers.slice().sort(function (a, b) {
      return (a.key === 'saddle' ? 1 : 0) - (b.key === 'saddle' ? 1 : 0);
    });
    order.forEach(function (t) {
      ctx.strokeStyle = COLORS[t.key];
      ctx.lineWidth = 3;
      ctx.lineJoin = ctx.lineCap = 'round';
      ctx.beginPath();
      t.trail.forEach(function (p, i) {
        var q = project(p, C, cx, cy, scale);
        i ? ctx.lineTo(q.x, q.y) : ctx.moveTo(q.x, q.y);
      });
      ctx.stroke();

      // where the run started
      var tail = project(t.trail[0], C, cx, cy, scale);
      ctx.fillStyle = '#fff';
      ctx.beginPath(); ctx.arc(tail.x, tail.y, 3.6, 0, 6.2832); ctx.fill();
      ctx.strokeStyle = COLORS[t.key]; ctx.lineWidth = 2; ctx.stroke();

      var head = project(t.trail[t.trail.length - 1], C, cx, cy, scale);
      if (t.key === 'saddle') {
        ctx.strokeStyle = COLORS[t.key]; ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.arc(head.x, head.y, 11, 0, 6.2832); ctx.stroke();
        ctx.beginPath(); ctx.arc(head.x, head.y, 15, 0, 6.2832);
        ctx.globalAlpha = .45; ctx.stroke(); ctx.globalAlpha = 1;
      }
      ctx.fillStyle = COLORS[t.key];
      ctx.beginPath(); ctx.arc(head.x, head.y, 5, 0, 6.2832); ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.6; ctx.stroke();
    });

    ctx.fillStyle = '#5d6474'; ctx.font = '11px ui-monospace, monospace';
    ctx.fillText('radius = angle / π', 12, 18);
    ctx.fillText('drag to orbit', 12, f.h - 12);
  }

  /* ---------------- panel 3: D over time ---------------- */

  var plotCanvas = document.getElementById('demo-plot');
  var HIST = 900;

  function drawPlot() {
    var f = fit(plotCanvas, 130), ctx = f.ctx;
    ctx.clearRect(0, 0, f.w, f.h);
    var padL = 44, padR = 10, padT = 14, padB = 22;
    var w = f.w - padL - padR, h = f.h - padT - padB;

    var w0 = inertia();
    var Dmax = 2 * (w0[0] + w0[1] + w0[2]);   // upper bound on tr(W) - tr(WR)

    ctx.strokeStyle = '#e6e8ee'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + h); ctx.lineTo(padL + w, padT + h);
    ctx.stroke();

    ctx.fillStyle = '#5d6474'; ctx.font = '10px ui-monospace, monospace';
    ctx.fillText('D(R)', 6, padT + 8);
    ctx.fillText('0', 30, padT + h + 3);
    ctx.fillText('time', padL + w - 24, padT + h + 15);

    var span = Math.max(120, tracers.reduce(function (m, t) {
      return Math.max(m, t.hist.length);
    }, 0));
    tracers.forEach(function (t) {
      ctx.strokeStyle = COLORS[t.key]; ctx.lineWidth = 2;
      // the two saddle runs sit on top of each other until the nudged one escapes,
      // so dash it and the stuck run stays visible underneath
      ctx.setLineDash(t.key === 'nudged' ? [6, 4] : []);
      ctx.beginPath();
      var n = t.hist.length;
      for (var i = 0; i < n; i++) {
        var x = padL + (i / (span - 1)) * w;
        var y = padT + h - (t.hist[i] / Dmax) * h;
        i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    });
  }

  /* ---------------- loop ---------------- */

  var time = 0, running = true, settleFrames = 0;

  var DT = 0.012;   // integration step; the flow is dimensionless here

  function tick() {
    if (running) {
      for (var s = 0; s < 3; s++) {
        tracers.forEach(function (t) {
          t.R = step(t.R, DT);
        });
        time += DT;
      }
      tracers.forEach(function (t) {
        var p = ballPoint(t.R);
        var last = t.trail[t.trail.length - 1];
        if (Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) > 0.004) t.trail.push(p);
        if (t.trail.length > 1200) t.trail.shift();
        t.hist.push(potential(t.R));
        if (t.hist.length > HIST) t.hist.shift();
      });
      updateReadout();

      // once the runs that can converge have, hold a beat and start over
      var settled = tracers.every(function (t) {
        return t.key === 'saddle' || angleToGoal(t.R) < 0.4;
      });
      settleFrames = settled ? settleFrames + 1 : 0;
      if (settleFrames > 100) { resetTracers(true); settleFrames = 0; }
    }
    drawBox(); drawBall(); drawPlot();
    if (!STILL && running) requestAnimationFrame(tick);
  }

  // ?still=N renders a single frame N integration steps in and stops (N defaults to 700),
  // for screenshots and for reduced-motion users.
  var stillArg = /[?&]still=(\d+)/.exec(location.search);
  var STILL = !!stillArg ||
    (window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches);
  var STILL_STEPS = stillArg && +stillArg[1] > 1 ? +stillArg[1] : 700;

  /* ---------------- readout + controls ---------------- */

  var readout = document.getElementById('demo-readout');

  function updateReadout() {
    if (!readout) return;
    readout.innerHTML = tracers.map(function (t) {
      return '<span class="ro-item"><i style="background:' + COLORS[t.key] + '"></i>' +
        t.label + '<b>∠ ' + angleToGoal(t.R).toFixed(1) + '°</b>' +
        '<b>D = ' + potential(t.R).toFixed(2) + '</b></span>';
    }).join('');
  }

  function bind(id, fn) {
    var el = document.getElementById(id);
    if (el) el.addEventListener('click', fn);
  }

  bind('demo-new', function () { resetTracers(true); });
  bind('demo-replay', function () { resetTracers(false); });
  bind('demo-nudge', function () {
    // Kick along the unstable direction (about the saddle's own axis), so it really leaves.
    var t = tracers.filter(function (x) { return x.key === 'saddle'; })[0];
    if (!t) return;
    var e = [[1, 0, 0], [0, 1, 0], [0, 0, 1]][saddleAxis];
    t.R = reortho(matMul(t.R, axisAngle(e, -2 * Math.PI / 180)));
    t.trail.push(ballPoint(t.R));
  });
  var manualPause = false;
  bind('demo-pause', function () {
    manualPause = !manualPause;
    running = !manualPause;
    this.textContent = manualPause ? 'Play' : 'Pause';
    if (running) requestAnimationFrame(tick);
  });

  var axSel = document.getElementById('demo-axis');
  if (axSel) {
    axSel.value = String(saddleAxis);
    axSel.addEventListener('change', function () {
      saddleAxis = parseInt(axSel.value, 10);
      resetTracers(false);
    });
  }

  // orbit: drag either 3D panel
  [boxCanvas, ballCanvas].forEach(function (cv) {
    if (!cv) return;
    var dragging = false, lx = 0, ly = 0;
    cv.addEventListener('pointerdown', function (e) {
      dragging = true; lx = e.clientX; ly = e.clientY; cv.setPointerCapture(e.pointerId);
    });
    cv.addEventListener('pointermove', function (e) {
      if (!dragging) return;
      cam.yaw -= (e.clientX - lx) * 0.008;
      cam.pitch = Math.max(-1.4, Math.min(1.4, cam.pitch + (e.clientY - ly) * 0.008));
      lx = e.clientX; ly = e.clientY;
    });
    var end = function () { dragging = false; };
    cv.addEventListener('pointerup', end);
    cv.addEventListener('pointercancel', end);
  });

  window.addEventListener('resize', function () { drawBox(); drawBall(); drawPlot(); });

  // don't burn CPU while the demo is off screen
  if ('IntersectionObserver' in window && !STILL) {
    var onScreen = true;
    new IntersectionObserver(function (entries) {
      var wasOff = !onScreen;
      onScreen = entries[0].isIntersecting;
      running = onScreen && !manualPause;
      if (onScreen && wasOff) requestAnimationFrame(tick);
    }, { threshold: 0 }).observe(root);
  }

  // exposed so the two-step controller can be checked against the paper's closed form
  window.POILDemo = {
    omega: omega, pointField: pointField, potential: potential, inertia: inertia,
    halfExtents: function () { return half.slice(); }
  };

  resetTracers(true);
  if (STILL) {
    for (var w = 0; w < STILL_STEPS; w++) {
      tracers.forEach(function (t) { t.R = step(t.R, DT); });
      tracers.forEach(function (t) {
        var p = ballPoint(t.R);
        var last = t.trail[t.trail.length - 1];
        if (Math.hypot(p[0] - last[0], p[1] - last[1], p[2] - last[2]) > 0.004) t.trail.push(p);
        t.hist.push(potential(t.R));
      });
    }
    running = false;
  }
  updateReadout();
  requestAnimationFrame(tick);
})();
