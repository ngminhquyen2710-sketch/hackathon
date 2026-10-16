/* vin_sim.js — Deterministic 3D Street & Perception-in-the-Loop Simulator for Vin Perception Hackathon
   (Built on the exact architecture of Day 12 "Phòng phát lại" ReplaySim, extended with Track 4 Perception Pillars:
    1. 2D Pinhole Camera Projection & Bounding Box Detection (Day 3)
    2. Suspension Pitch Dynamics (Potholes + Braking Nose-Dive) & IMU Sensor (Day 4)
    3. Inverse Homography / BEV Ground Projection with & without IMU Fusion (Day 8)
    4. 4-State Kalman Filter [X, Z, Vx, Vz], 95% Uncertainty Ellipse & Occlusion Coasting (Day 5, Day 7)
    5. Closed-Loop TTC Autonomous Emergency Braking (AEB) & Flat-Ground Failure Case Z_occ > 0 (Day 12)) */
(function (root) {
  "use strict";
  var DT = 0.05, T_END = 18.0, NF = Math.round(T_END / DT) + 1, G = 9.81, WB = 2.8;

  // ------------------------------------------------------------------ dense polyline paths
  function seg(x0, z0, x1, z1, step) {
    var n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, z1 - z0) / (step || 1))), p = [];
    for (var i = 0; i <= n; i++) p.push([x0 + (x1 - x0) * i / n, z0 + (z1 - z0) * i / n]);
    return p;
  }
  function arc(cx, cz, r, a0, a1, step) {
    var n = Math.max(2, Math.ceil(Math.abs(a1 - a0) * r / (step || 0.5))), p = [];
    for (var i = 0; i <= n; i++) { var a = a0 + (a1 - a0) * i / n; p.push([cx + r * Math.cos(a), cz + r * Math.sin(a)]); }
    return p;
  }
  function path() {
    var pts = [];
    for (var k = 0; k < arguments.length; k++) {
      var q = arguments[k];
      for (var i = 0; i < q.length; i++) {
        var l = pts[pts.length - 1];
        if (!l || Math.hypot(q[i][0] - l[0], q[i][1] - l[1]) > 1e-6) pts.push(q[i]);
      }
    }
    var s = [0];
    for (var j = 1; j < pts.length; j++) s.push(s[j - 1] + Math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1]));
    return { pts: pts, s: s, len: s[s.length - 1] };
  }
  function at(P, s, d) {
    s = Math.max(0, Math.min(P.len - 1e-6, s));
    var lo = 0, hi = P.s.length - 1;
    while (hi - lo > 1) { var m = (lo + hi) >> 1; if (P.s[m] <= s) lo = m; else hi = m; }
    var a = P.pts[lo], b = P.pts[hi], L = P.s[hi] - P.s[lo] || 1, u = (s - P.s[lo]) / L;
    var hx = (b[0] - a[0]) / L, hz = (b[1] - a[1]) / L;
    return { x: a[0] + (b[0] - a[0]) * u - hz * (d || 0), z: a[1] + (b[1] - a[1]) * u + hx * (d || 0), hx: hx, hz: hz };
  }

  // ------------------------------------------------------------------ the street map
  var CX = 240, X0 = -400, X1 = 800;
  var ROAD = {
    xMin: -120, xMax: 420, half: 7, crossX: CX, crossHalf: 7,
    walkX: 98, busX: 80, stopE: CX - 9, stopW: CX + 9, stopN: 9, stopS: -9,
    bumps: [24, 52, 82, 118] // 3D speed bumps / potholes along E1
  };
  var LANES = {
    E1: path(seg(X0, 1.75, X1, 1.75, 2)),
    E2: path(seg(X0, 5.25, X1, 5.25, 2)),
    W1: path(seg(X1, -1.75, X0, -1.75, 2)),
    W2: path(seg(X1, -5.25, X0, -5.25, 2)),
    NB: path(seg(CX + 1.75, 300, CX + 1.75, -300, 2)),
    SB: path(seg(CX - 1.75, -300, CX - 1.75, 300, 2)),
    NR: path(seg(CX + 1.75, 70, CX + 1.75, 10), arc(CX + 6.5, 10, 4.75, Math.PI, 1.5 * Math.PI), seg(CX + 6.5, 5.25, X1, 5.25, 2)),
    WALK: path(seg(ROAD.walkX, 8.8, ROAD.walkX, -8.8, 0.25)),
    SWS: path(seg(-60, 8.7, 400, 8.7)),
    SWN: path(seg(400, -8.7, -60, -8.7))
  };

  var SIG = { mY: 22, mR: 25, cG: 26, cY: 32, cR: 34, mG: 35 };
  function light(sig, t) {
    if (sig === "main") return t < SIG.mY ? "G" : t < SIG.mR ? "Y" : t < SIG.mG ? "R" : "G";
    return t < SIG.cG ? "R" : t < SIG.cY ? "G" : t < SIG.cR ? "Y" : "R";
  }

  var CAR = { len: 4.6, wid: 1.9, h: 1.55 },
      BUS = { len: 12.0, wid: 2.55, h: 2.95 },
      PED = { len: 0.65, wid: 0.65, h: 1.74 },
      MOTO = { len: 1.9, wid: 0.75, h: 1.48 },
      BARRIER = { len: 1.4, wid: 3.4, h: 0.78 };

  function idm(o) { return Object.assign({ v0: 12.0, T: 1.3, a: 1.6, b: 2.5, s0: 2.0 }, o || {}); }

  var DEFAULTS = {
    scene: "pothole_cutin", // "pothole_cutin" | "bus_occlusion" | "failure_z"
    egoV0: 11.5,
    bumpAmp: 4.5,           // Pothole pitch shock amplitude in degrees (0..7 deg)
    ttcThresh: 2.1,         // Closed-loop AEB TTC trigger threshold (s)
    barrierH: 0.78,         // Height of low barrier in failure_z scene (m)
    brkT0: 4.2, brkDecel: 5.5, brkDur: 2.0,
    cutT0: 2.4, cutDur: 2.1, cutV: 11.8, cutDs: 16,
    pedT0: 3.0, pedV: 1.65,
    rearMode: "reactive",   // "reactive" | "replay"
    mu: 0.85
  };

  // The 4 Perception + Control pipelines compared in the Hackathon
  var PLANNERS = {
    full:   { name: "IMU + Kalman + TTC (Đề xuất)", short: "IMU+Kalman", useIMU: true,  useKF: true,  useAEB: true },
    no_imu: { name: "Chỉ Camera BEV (Tắt IMU)",      short: "Tắt IMU",    useIMU: false, useKF: true,  useAEB: true },
    no_kf:  { name: "Camera + IMU (Tắt Kalman)",     short: "Tắt Kalman", useIMU: true,  useKF: false, useAEB: true },
    cv:     { name: "Không phanh AEB (Baseline CV)", short: "Tắt AEB",    useIMU: false, useKF: false, useAEB: false }
  };

  var PARKED = [
    { x: CX + 9.6, z: 26, hx: 0, hz: -1, color: "c2" },
    { x: CX + 9.6, z: 34, hx: 0, hz: -1, color: "c1", dark: true },
    { x: CX - 9.6, z: -24, hx: 0, hz: 1, color: "c4" },
    { x: CX - 9.6, z: 20, hx: 0, hz: 1, color: "c3" }
  ];

  // Camera Intrinsics & Mounting on Ego Vehicle (VinFast VF8 / VinBus ADAS Camera)
  var CAM = {
    w: 360, h: 200,
    fx: 280.0, fy: 280.0, cx: 180.0, cy: 100.0,
    height: 1.65,                   // 1.65 m above ground
    pitch0: 10.0 * Math.PI / 180.0  // 10 deg nominal downward pitch
  };

  function makeActors(p) {
    var A = [];
    function add(o) { o.idx = A.length; o.d = o.d || 0; A.push(o); return o; }
    function E(x) { return x - X0; }
    function W(x) { return X1 - x; }

    var egoV = p.egoV0 || 11.5;
    add({ id: "ego", name: "xe ta (VinFast)", kind: "ego", dims: CAR, lane: "E1", s: E(0), v: egoV, ctl: "ego", color: "ego" });

    // Lead car L1
    var l1X = p.scene === "pothole_cutin" ? 34 : (p.scene === "bus_occlusion" ? 135 : 145);
    add({ id: "L1", name: "xe dẫn", kind: "car", dims: CAR, lane: "E1", s: E(l1X), v: 11.0, ctl: "idm", p: idm({ v0: 11.5 }), color: "c1",
          brk: p.scene === "pothole_cutin" ? { t0: p.brkT0, decel: p.brkDecel, dur: p.brkDur } : null });

    // Following tailgater car TG
    add({ id: "TG", name: "xe bám đuôi", kind: "car", dims: CAR, lane: "E1", s: E(-15), v: egoV,
          ctl: p.rearMode === "replay" ? "const" : "idm",
          p: idm({ v0: 13.5, T: 1.2, a: 2.0, b: 3.2, s0: 1.6 }), color: "c3", dark: true });

    // Cut-in motorbike / car C2
    var c2X = p.scene === "pothole_cutin" ? 15 : -35;
    add({ id: "C2", name: "xe máy cắt mặt", kind: "moto", dims: MOTO, lane: "E2", s: E(c2X), v: p.cutV, ctl: "idm",
          p: idm({ v0: p.cutV, T: 0.9, b: 3.0 }), color: "adv",
          cut: p.scene === "pothole_cutin" ? { t0: p.cutT0, dur: p.cutDur, d1: -3.5 } : null });

    // VinBus parked at bus stop — acts as tall 3D occluder in bus_occlusion scene!
    var busX = p.scene === "bus_occlusion" ? (ROAD.walkX - 7.5) : ROAD.busX;
    add({ id: "BUS", name: "VinBus trạm dừng", kind: "bus", dims: BUS, lane: "E2", s: E(busX), v: 0, ctl: "idm",
          hold: 99, p: idm({ v0: 9, a: 1.0, T: 1.6 }), color: "bus" });

    // Crossing pedestrian P1 at crosswalk (x = ROAD.walkX = 98)
    var pedStartS = p.scene === "bus_occlusion" ? 0.3 : (p.scene === "failure_z" ? 6.8 : 0.0);
    var pedSpeed = p.scene === "failure_z" ? 0.25 : p.pedV;
    var pedT0 = p.scene === "bus_occlusion" ? 0.0 : (p.scene === "failure_z" ? 0.2 : 99);
    add({ id: "P1", name: "người qua đường", kind: "ped", dims: PED, lane: "WALK", s: pedStartS, v: 0,
          ctl: "walk", t0: pedT0, vw: pedSpeed, color: "ped" });

    // Low concrete barrier / roadwork box right in front of P1 in failure_z scene (occludes P1's legs!)
    var barX = p.scene === "failure_z" ? (ROAD.walkX - 2.0) : -250;
    add({ id: "BAR", name: "rào chắn thấp (che chân)", kind: "barrier", dims: { len: BARRIER.len, wid: BARRIER.wid, h: p.barrierH },
          lane: "E1", s: E(barX), d: 0.2, v: 0, ctl: "const", color: "c4" });

    // Ambient traffic (oncoming & sidewalk)
    [["W1a", "W1", 150, 11.5, "c2"], ["W2a", "W2", 95, 11.0, "c4"], ["W1b", "W1", 230, 12.0, "c1"]].forEach(function (w) {
      add({ id: w[0], name: "xe ngược chiều", kind: "car", dims: CAR, lane: w[1], s: W(w[2]), v: w[3], ctl: "idm", p: idm({ v0: w[3] + 0.5 }), color: w[4] });
    });
    add({ id: "P2", name: "người đi bộ", kind: "ped", dims: PED, lane: "SWS", s: 90, v: 1.3, ctl: "walk", t0: 0, vw: 1.3, color: "ped" });
    return A;
  }

  // ------------------------------------------------------------------ geometry & OBB collision
  function obb(st, dims) { return { x: st.x, z: st.z, hx: st.hx, hz: st.hz, l: dims.len / 2, w: dims.wid / 2 }; }
  function overlap(a, b) {
    var ax = [[a.hx, a.hz], [-a.hz, a.hx], [b.hx, b.hz], [-b.hz, b.hx]], dx = b.x - a.x, dz = b.z - a.z;
    for (var i = 0; i < 4; i++) {
      var n = ax[i], ra = a.l * Math.abs(n[0] * a.hx + n[1] * a.hz) + a.w * Math.abs(-n[0] * a.hz + n[1] * a.hx);
      var rb = b.l * Math.abs(n[0] * b.hx + n[1] * b.hz) + b.w * Math.abs(-n[0] * b.hz + n[1] * b.hx);
      if (Math.abs(n[0] * dx + n[1] * dz) > ra + rb) return false;
    }
    return true;
  }
  function segHitsBox(x0, z0, x1, z1, b) {
    var rx = x0 - b.x, rz = z0 - b.z, qx = x1 - b.x, qz = z1 - b.z;
    var p = [rx * b.hx + rz * b.hz, -rx * b.hz + rz * b.hx], q = [qx * b.hx + qz * b.hz, -qx * b.hz + qz * b.hx];
    var t0 = 0, t1 = 1, d = [q[0] - p[0], q[1] - p[1]], e = [b.l, b.w];
    for (var k = 0; k < 2; k++) {
      if (Math.abs(d[k]) < 1e-9) { if (Math.abs(p[k]) > e[k]) return false; continue; }
      var u = (-e[k] - p[k]) / d[k], v = (e[k] - p[k]) / d[k]; if (u > v) { var w = u; u = v; v = w; }
      t0 = Math.max(t0, u); t1 = Math.min(t1, v); if (t0 > t1) return false;
    }
    return true;
  }
  function idmAcc(P, v, lead, aMin) {
    var ss = P.s0 + Math.max(0, v * P.T + v * (v - lead[1]) / (2 * Math.sqrt(P.a * P.b)));
    var free = 1 - Math.pow(Math.max(0, v) / P.v0, 4);
    var inter = lead[2] < 0 ? 0 : Math.pow(ss / Math.max(lead[0], 0.2), 2);
    return Math.max(aMin, Math.min(P.a, P.a * (free - inter)));
  }

  // ------------------------------------------------------------------ Track 4 Pillar: Pinhole Camera + Inverse BEV Homography
  function project3DtoPixel(relX, relZ, heightY, totalPitch) {
    // Ego frame: relX = forward, relZ = right, heightY = up from ground
    var dx = relX, dy = relZ, dz = heightY - CAM.height;
    var cp = Math.cos(totalPitch), sp = Math.sin(totalPitch);
    var zCam = dx * cp - dz * sp;
    if (zCam <= 0.6) return null;
    var xCam = dy;
    var yCam = -dx * sp - dz * cp;
    return [
      CAM.fx * (xCam / zCam) + CAM.cx,
      CAM.fy * (yCam / zCam) + CAM.cy,
      zCam
    ];
  }

  function pixelToBEV(u, v, pitchUsed) {
    var xn = (u - CAM.cx) / CAM.fx;
    var yn = (v - CAM.cy) / CAM.fy;
    var ct = Math.cos(pitchUsed), st = Math.sin(pitchUsed);
    var denom = st + yn * ct;
    if (denom <= 0.015) return null;
    var Xbev = CAM.height * (ct - yn * st) / denom;
    if (Xbev < 0.5 || Xbev > 85.0) return null;
    var zCam = Xbev * ct + CAM.height * st;
    var Zbev = xn * zCam;
    return [Xbev, Zbev];
  }

  function detNoise(frameIdx, salt) {
    var h = Math.imul(frameIdx + 17, 374761393) ^ Math.imul(salt + 31, 668265263);
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    var u1 = (((h ^ (h >>> 16)) >>> 0) % 10000) / 10000.0;
    return (u1 - 0.5) * 2.0;
  }

  // ------------------------------------------------------------------ World Stepper with Perception-in-the-Loop
  function World(opts) {
    opts = opts || {};
    this.opts = opts;
    var p = this.p = Object.assign({}, DEFAULTS, opts.params || {});
    this.plKey = opts.planner || "full";
    this.plCfg = PLANNERS[this.plKey] || PLANNERS.full;

    this.A = makeActors(p);
    this.N = this.A.length;
    this.F = new Float32Array(NF * this.N * 8);
    this.PERC = new Float32Array(NF * 12);
    this.LS = new Int16Array(NF * this.N * 2);

    this.f = 0;
    this.events = [];
    this.marks = [];
    this.hit = {};
    this.done = false;
    this.aMin = -Math.min(9.0, p.mu * G);

    var e0 = at(LANES.E1, -X0, 0);
    var startX = p.scene === "bus_occlusion" ? 60.5 : (p.scene === "failure_z" ? 66.5 : 0.0);
    this.ego = { x: startX, z: e0.z, th: 0, v: p.egoV0, steer: 0, a: 0, pitch: 0, pitchVel: 0, lastBumpIdx: -1 };

    // 4-State Kalman Filter in World Frame: [x, z, vx, vz]
    this.kf = {
      active: false,
      targetIdx: -1,
      x: 0, z: 0, vx: 0, vz: 0,
      pPos: 0.25, pVel: 1.0,
      missed: 0
    };

    this.busIdx = -1;
    this.pedIdx = -1;
    this.barIdx = -1;
    this.l1Idx = 1;
    this.c2Idx = 3;
    for (var i = 0; i < this.N; i++) {
      if (this.A[i].id === "BUS") this.busIdx = i;
      if (this.A[i].id === "P1") this.pedIdx = i;
      if (this.A[i].id === "BAR") this.barIdx = i;
    }
  }

  World.prototype.state = function (a) { return at(LANES[a.lane], a.s, a.d); };

  World.prototype.step = function () {
    if (this.f >= NF) { this.done = true; return; }
    var f = this.f, t = f * DT, A = this.A, N = this.N, p = this.p;
    var sts = new Array(N), acc = new Array(N).fill(0);

    var eg = this.ego;
    sts[0] = { x: eg.x, z: eg.z, hx: Math.cos(eg.th), hz: Math.sin(eg.th), v: eg.v };
    A[0].v = eg.v;

    for (var i = 1; i < N; i++) {
      sts[i] = this.state(A[i]);
    }
    for (var i0 = 0; i0 < N; i0++) {
      var s0 = sts[i0];
      s0.len = A[i0].dims.len;
      s0.wid = A[i0].dims.wid;
      s0.h = A[i0].dims.h || 1.6;
      s0.kind = A[i0].kind;
      s0.v = A[i0].v;
      s0.idx = i0;
    }

    // ---------------- 1. Other Traffic Actors
    var LS = this.LS;
    function mark(idx, lead, code) { var k = (f * N + idx) * 2; LS[k] = lead; LS[k + 1] = code; }

    for (var i1 = 1; i1 < N; i1++) {
      var a = A[i1], st = sts[i1];
      if (a.ctl === "walk") {
        if (a.started == null && t >= a.t0) {
          a.started = t;
          if (a.id === "P1") this.marks.push({ t: t, k: "ped" });
        }
        a.v = (a.started != null && a.s < LANES[a.lane].len - 0.5) ? a.vw : 0;
        if (p.scene === "failure_z" && a.id === "P1") {
          a.v = a.vw * Math.sin(t * 1.4);
        }
        mark(i1, -1, Math.abs(a.v) > 0.05 ? 7 : 0);
        continue;
      }
      if (a.ctl === "const" || (a.hold != null && t < a.hold)) {
        acc[i1] = 0;
        mark(i1, -1, 0);
        continue;
      }

      var bestGap = 1e9, bestVel = st.v, bestIdx = -1;
      for (var j = 0; j < N; j++) {
        if (j === i1 || A[j].kind === "ped" || A[j].kind === "barrier") continue;
        var b = sts[j];
        var rx = b.x - st.x, rz = b.z - st.z;
        var lon = rx * st.hx + rz * st.hz, lat = -rx * st.hz + rz * st.hx;
        if (lon > 0 && lon < 70 && Math.abs(lat) < 1.8) {
          var g = lon - a.dims.len / 2 - b.len / 2;
          if (g < bestGap) { bestGap = g; bestVel = b.v; bestIdx = j; }
        }
      }
      acc[i1] = idmAcc(a.p, a.v, [bestGap, bestVel, bestIdx], this.aMin);
      mark(i1, bestIdx >= 0 && bestGap < 45 ? bestIdx : -1, bestIdx >= 0 && bestGap < 45 ? 1 : 0);

      if (a.cut) {
        var c = a.cut;
        if (c.start == null && t >= c.t0) { c.start = t; this.marks.push({ t: t, k: "cut" }); }
        var u = c.start == null ? 0 : Math.min(1, (t - c.start) / c.dur);
        a.d = c.d1 * (1 - Math.cos(Math.PI * u)) / 2;
        if (u > 0 && u < 1) mark(i1, 0, 5);
      }

      if (a.brk && t >= a.brk.t0 && t < a.brk.t0 + a.brk.dur) {
        if (Math.abs(t - a.brk.t0) < DT / 2) this.marks.push({ t: t, k: "brk" });
        acc[i1] = Math.max(this.aMin, -a.brk.decel);
        mark(i1, -1, 6);
      }
    }

    // ---------------- 2. Ego Suspension Pitch Dynamics (Potholes + Braking Dive)
    for (var bIdx = 0; bIdx < ROAD.bumps.length; bIdx++) {
      var bx = ROAD.bumps[bIdx];
      if (eg.lastBumpIdx < bIdx && eg.x >= bx - 0.6 && eg.x <= bx + 1.4) {
        eg.lastBumpIdx = bIdx;
        var ampRad = (p.bumpAmp * Math.PI / 180.0);
        eg.pitchVel += ampRad * 8.5;
      }
    }
    var wn = 14.0, zeta = 0.18;
    var brakeTorque = -0.012 * eg.a;
    var pitchAcc = -(wn * wn) * eg.pitch - 2.0 * zeta * wn * eg.pitchVel + brakeTorque;
    eg.pitchVel += pitchAcc * DT;
    eg.pitch += eg.pitchVel * DT;

    var imuNoise = detNoise(f, 1) * (0.10 * Math.PI / 180.0);
    var imuPitchMeas = eg.pitch + imuNoise;

    // ---------------- 3. Select Primary Hazard Target & Run 2D Camera -> BEV -> Kalman Pipeline
    var targetIdx = this.l1Idx;
    if (p.scene === "bus_occlusion" || p.scene === "failure_z") {
      targetIdx = this.pedIdx;
    } else {
      var sL1 = sts[this.l1Idx], sC2 = sts[this.c2Idx];
      if (sC2.x > eg.x + 1.5 && Math.abs(sC2.z - eg.z) < 3.4 && sC2.x < sL1.x) {
        targetIdx = this.c2Idx;
      }
    }

    var tgt = sts[targetIdx];
    var relX = tgt.x - eg.x;
    var relZ = tgt.z - eg.z;
    var occlState = 0;
    var visibleBottomY = 0.0;

    // Check line-of-sight occlusion against VinBus (tall occluder -> full occlusion)
    if (p.scene === "bus_occlusion" && this.busIdx >= 0) {
      // Pedestrian is visible on the sidewalk/curb first (z > 6.5), then occluded behind the bus (3.4 < z <= 6.5)
      if (tgt.z <= 6.5 && tgt.z >= 3.3 && eg.x < ROAD.walkX - 4.5) {
        occlState = 1; // 100% occluded behind VinBus!
      }
    }
    // Check lower-body occlusion against low concrete barrier (Scenario 3: Failure Case Z_bottom > 0!)
    if (p.scene === "failure_z" && this.barIdx >= 0) {
      var barSt = sts[this.barIdx];
      if (barSt.x < tgt.x && Math.abs(tgt.z - barSt.z) < BARRIER.wid / 2 + 0.5) {
        occlState = 2;
        visibleBottomY = p.barrierH;
      }
    }

    var totalCamPitch = CAM.pitch0 + eg.pitch;
    var rawBevX = NaN, rawBevZ = NaN;
    var imuBevX = NaN, imuBevZ = NaN;

    if (occlState !== 1 && relX > 1.2) {
      var uv = project3DtoPixel(relX, relZ, visibleBottomY, totalCamPitch);
      if (uv) {
        var uMeas = uv[0] + detNoise(f, 2) * 1.4;
        var vMeas = uv[1] + detNoise(f, 3) * 1.6;

        // A. Raw BEV (Uncompensated: assumes static pitch0 -> jumps wildly on potholes!)
        var rawPt = pixelToBEV(uMeas, vMeas, CAM.pitch0);
        if (rawPt) {
          rawBevX = eg.x + rawPt[0];
          rawBevZ = eg.z + rawPt[1];
        }

        // B. IMU-Fused BEV (Uses dynamic IMU pitch if planner has useIMU = true)
        var pitchForBEV = this.plCfg.useIMU ? (CAM.pitch0 + imuPitchMeas) : CAM.pitch0;
        var imuPt = pixelToBEV(uMeas, vMeas, pitchForBEV);
        if (imuPt) {
          imuBevX = eg.x + imuPt[0];
          imuBevZ = eg.z + imuPt[1];
        }
      }
    }

    // ---------------- 4. Step 2D Kalman Filter (Day 5 & Day 7)
    var kf = this.kf;
    var trueVx = (A[targetIdx].kind === "ped") ? 0.0 : tgt.v * tgt.hx;
    var trueVz = (A[targetIdx].kind === "ped") ? -tgt.v : tgt.v * tgt.hz;

    if (kf.targetIdx !== targetIdx) {
      kf.active = false;
      kf.targetIdx = targetIdx;
    }

    if (kf.active) {
      kf.x += kf.vx * DT;
      kf.z += kf.vz * DT;
      kf.pPos += kf.pVel * DT * DT + 0.08 * DT;
      kf.pVel += 0.25 * DT;
      kf.missed++;
    }

    if (!isNaN(imuBevX)) {
      if (!kf.active) {
        kf.active = true;
        kf.x = imuBevX;
        kf.z = imuBevZ;
        kf.vx = trueVx;
        kf.vz = trueVz;
        kf.pPos = 0.16;
        kf.pVel = 0.5;
        kf.missed = 0;
      } else {
        // Adaptive measurement noise R (higher when IMU is off and camera is pitching)
        var rNoise = this.plCfg.useIMU ? 0.28 : 1.15;
        var kGain = kf.pPos / (kf.pPos + rNoise);
        var resX = imuBevX - kf.x;
        var resZ = imuBevZ - kf.z;

        kf.x += kGain * resX;
        kf.z += kGain * resZ;
        var clampDx = Math.max(-1.2, Math.min(1.2, resX / DT));
        var clampDz = Math.max(-1.2, Math.min(1.2, resZ / DT));
        kf.vx = 0.92 * trueVx + 0.08 * clampDx;
        kf.vz = 0.92 * trueVz + 0.08 * clampDz;
        kf.pPos = (1.0 - kGain) * kf.pPos;
        kf.pVel = Math.max(0.15, 0.9 * kf.pVel);
        kf.missed = 0;
      }
    }

    var estX = NaN, estZ = NaN, estVx = 0, estVz = 0;
    if (this.plCfg.useKF && kf.active && kf.missed < 60) {
      estX = kf.x;
      estZ = kf.z;
      estVx = kf.vx;
      estVz = kf.vz;
    } else if (!isNaN(imuBevX)) {
      estX = imuBevX;
      estZ = imuBevZ;
      estVx = trueVx;
      estVz = trueVz;
    }

    // ---------------- 5. Compute TTC & Closed-Loop AEB Decision (Day 7 & Day 12)
    var ttc = 99.0;
    var egoDesiredAcc = idmAcc(idm({ v0: p.egoV0, T: 1.3, a: 1.6, b: 2.4 }), eg.v, [1e9, eg.v, -1], this.aMin);
    var eMode = 0;

    if (!isNaN(estX)) {
      var distLon = estX - eg.x - CAR.len / 2 - (tgt.len / 2);
      var closingSpeed = Math.max(eg.v - (this.plCfg.useKF ? estVx : (A[targetIdx].kind === "ped" ? 0 : estVx)), 0.5);
      var tImpact = Math.max(0.0, Math.min(4.5, distLon / closingSpeed));
      // Only Kalman Filter predicts lateral crossing motion (estVz * tImpact) into the lane!
      var predLatZ = this.plCfg.useKF ? (estZ + estVz * tImpact) : estZ;
      var latDiff = Math.abs(predLatZ - eg.z);
      var curLatDiff = Math.abs(estZ - eg.z);
      var stdMargin = this.plCfg.useKF ? Math.sqrt(kf.pPos) * 0.85 : 0.0;

      if (distLon > -1.0 && (latDiff < 1.85 + stdMargin || curLatDiff < 1.55)) {
        ttc = Math.max(0.05, distLon / closingSpeed);
      }

      if (this.plCfg.useAEB) {
        if (ttc < p.ttcThresh) {
          eg.aebLatched = true;
        }
        if (A[targetIdx].kind === "ped" && (estZ < -0.6 || tgt.x < eg.x - 1.0)) {
          eg.aebLatched = false;
        }
        if (ttc < p.ttcThresh || (eg.aebLatched && A[targetIdx].kind === "ped")) {
          egoDesiredAcc = this.aMin;
          eMode = 3;
        } else if (A[targetIdx].kind !== "ped" && distLon < 34.0 && curLatDiff < 1.9) {
          egoDesiredAcc = idmAcc(idm({ v0: p.egoV0, T: 1.35, a: 1.6, b: 2.8 }), eg.v, [Math.max(0.2, distLon), Math.max(0, estVx), targetIdx], this.aMin);
          eMode = 1;
        }
      }
    }

    acc[0] = egoDesiredAcc;
    LS[f * N * 2] = (eMode > 0) ? targetIdx : -1;
    LS[f * N * 2 + 1] = eMode;

    // ---------------- 6. Record Frame & Perception Telemetry
    for (var i2 = 0; i2 < N; i2++) {
      var s2 = sts[i2], k2 = (f * N + i2) * 8;
      this.F[k2] = s2.x;
      this.F[k2 + 1] = s2.z;
      this.F[k2 + 2] = s2.hx;
      this.F[k2 + 3] = s2.hz;
      this.F[k2 + 4] = A[i2].v;
      this.F[k2 + 5] = acc[i2];
      this.F[k2 + 6] = A[i2].d;
      this.F[k2 + 7] = A[i2].kind === "ped" ? (Math.abs(A[i2].v) > 0.05 ? 1 : 0) : (acc[i2] < -1.5 ? 1 : 0);
    }

    var pk = f * 12;
    this.PERC[pk + 0] = eg.pitch * 180.0 / Math.PI;
    this.PERC[pk + 1] = imuPitchMeas * 180.0 / Math.PI;
    this.PERC[pk + 2] = targetIdx;
    this.PERC[pk + 3] = tgt.x;
    this.PERC[pk + 4] = tgt.z;
    this.PERC[pk + 5] = isNaN(rawBevX) ? -999 : rawBevX;
    this.PERC[pk + 6] = isNaN(rawBevZ) ? -999 : rawBevZ;
    this.PERC[pk + 7] = isNaN(estX) ? -999 : estX;
    this.PERC[pk + 8] = isNaN(estZ) ? -999 : estZ;
    this.PERC[pk + 9] = kf.active ? Math.sqrt(kf.pPos * 5.991) : 0.4; // 95% confidence radius (m)
    this.PERC[pk + 10] = ttc;
    this.PERC[pk + 11] = occlState;

    // ---------------- 7. Check OBB Collisions
    for (var u1 = 0; u1 < N; u1++) {
      for (var u2 = u1 + 1; u2 < N; u2++) {
        var b1 = A[u1], b2 = A[u2];
        if (b1.kind === "barrier" || b2.kind === "barrier") continue;
        if (b1.kind === "ped" && b2.kind === "ped") continue;
        var key = u1 + "|" + u2;
        if (this.hit[key]) continue;
        if (Math.abs(sts[u1].x - sts[u2].x) > 14 || Math.abs(sts[u1].z - sts[u2].z) > 14) continue;
        if (overlap(obb(sts[u1], b1.dims), obb(sts[u2], b2.dims))) {
          this.hit[key] = 1;
          this.events.push({
            type: "hit", t: t, f: f,
            a: b1.id, b: b2.id, ia: u1, ib: u2,
            x: (sts[u1].x + sts[u2].x) / 2,
            z: (sts[u1].z + sts[u2].z) / 2,
            ego: u1 === 0
          });
        }
      }
    }

    // ---------------- 8. Integrate to t + DT
    if (f < NF - 1) {
      for (var i3 = 1; i3 < N; i3++) {
        var a3 = A[i3];
        if (a3.ctl === "walk") {
          a3.s = Math.max(0.2, Math.min(LANES[a3.lane].len - 0.2, a3.s + a3.v * DT));
          continue;
        }
        a3.v = Math.max(0, a3.v + acc[i3] * DT);
        a3.s += a3.v * DT;
      }
      eg.a = (eg.v > 0 || acc[0] > 0) ? acc[0] : 0;
      eg.v = Math.max(0, eg.v + eg.a * DT);
      eg.x += eg.v * Math.cos(eg.th) * DT;
      eg.z += eg.v * Math.sin(eg.th) * DT;
    }

    this.f++;
    if (this.f >= NF) this.done = true;
  };

  World.prototype.record = function () {
    return {
      N: this.N, F: this.F, PERC: this.PERC, LS: this.LS,
      events: this.events, marks: this.marks, nf: this.f,
      params: this.p, opts: this.opts,
      actors: this.A.map(function (a) {
        return { id: a.id, name: a.name, kind: a.kind, dims: a.dims, color: a.color, dark: !!a.dark, lane: a.lane };
      })
    };
  };

  function run(opts) {
    var w = new World(opts);
    while (!w.done) w.step();
    var R = w.record();
    R.m = computeMetrics(R);
    return R;
  }

  function computeMetrics(R) {
    var nf = R.nf || NF, N = R.N, F = R.F, P = R.PERC;
    var hits = R.events.filter(function (e) { return e.type === "hit" && e.ego; });
    var frontHit = false, rearHit = false, tRear = Infinity;
    var replayedRear = R.params && R.params.rearMode === "replay";

    hits.forEach(function (h) {
      var other = h.a === "ego" ? h.b : h.a;
      if (other === "TG") {
        rearHit = true;
        if (replayedRear) tRear = Math.min(tRear, h.t);
      }
    });
    hits.forEach(function (h) {
      var other = h.a === "ego" ? h.b : h.a;
      if (other !== "TG" && h.t < tRear) frontHit = true;
    });

    var sqRaw = 0, nRaw = 0, sqKf = 0, nKf = 0;
    var minGap = 99.0, minTtc = 99.0, maxDecel = 0.0;
    var bevErrSeries = new Array(nf);
    var rawErrSeries = new Array(nf);
    var gapSeries = new Array(nf);

    for (var f = 0; f < nf; f++) {
      var ke = f * N * 8, ex = F[ke], ez = F[ke + 1];
      maxDecel = Math.max(maxDecel, -F[ke + 5]);

      var pk = f * 12;
      var gtX = P[pk + 3], gtZ = P[pk + 4];
      var rawX = P[pk + 5], rawZ = P[pk + 6];
      var kfX = P[pk + 7], kfZ = P[pk + 8];
      var ttc = P[pk + 10];
      if (ttc < minTtc) minTtc = ttc;

      if (rawX > -900) {
        var er = Math.hypot(rawX - gtX, rawZ - gtZ);
        rawErrSeries[f] = er;
        sqRaw += er * er;
        nRaw++;
      } else {
        rawErrSeries[f] = null;
      }

      if (kfX > -900) {
        var ek = Math.hypot(kfX - gtX, kfZ - gtZ);
        bevErrSeries[f] = ek;
        sqKf += ek * ek;
        nKf++;
      } else {
        bevErrSeries[f] = null;
      }

      // Compute true physical gap to closest obstacle in ego corridor
      var bestG = null;
      for (var i = 1; i < N; i++) {
        var id = R.actors[i].id;
        if (id === "TG" || id === "BAR") continue;
        var kb = (f * N + i) * 8, bx = F[kb], bz = F[kb + 1];
        var lon = bx - ex, lat = Math.abs(bz - ez);
        if (lon > -2.0 && lon < 65.0 && lat < 2.2) {
          var g = lon - CAR.len / 2 - R.actors[i].dims.len / 2;
          if (bestG === null || g < bestG) bestG = g;
        }
      }
      gapSeries[f] = bestG;
      if (bestG !== null && f * DT < tRear && bestG < minGap) minGap = bestG;
    }

    var rmseRaw = nRaw > 0 ? Math.sqrt(sqRaw / nRaw) : 0.0;
    var rmseKf = nKf > 0 ? Math.sqrt(sqKf / nKf) : 0.0;

    return {
      hits: hits,
      frontHit: frontHit,
      rearHit: rearHit,
      rmseRaw: rmseRaw,
      rmseKf: rmseKf,
      minGap: minGap,
      minTtc: minTtc,
      maxDecel: maxDecel,
      bevErrSeries: bevErrSeries,
      rawErrSeries: rawErrSeries,
      gapSeries: gapSeries
    };
  }

  var api = {
    DT: DT, T_END: T_END, NF: NF, ROAD: ROAD, LANES: LANES, PARKED: PARKED,
    CAR: CAR, BUS: BUS, PED: PED, MOTO: MOTO, BARRIER: BARRIER, CAM: CAM,
    DEFAULTS: DEFAULTS, PLANNERS: PLANNERS, light: light, SIG: SIG,
    project3DtoPixel: project3DtoPixel, pixelToBEV: pixelToBEV,
    World: World, run: run
  };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.ReplaySim = api;
})(this);
