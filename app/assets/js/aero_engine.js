/**
 * BSERC DEF-SPACE AUTUMN INTERNSHIP 2026 — AIRCRAFT DESIGN PROJECT 4
 * CORE AERODYNAMIC, FLIGHT MECHANICS & 3D PHONG/CFD STEALTH RENDERING ENGINE
 * Configuration: Tailless Cropped Delta Reflex Wing UAV (CD-UAV 3.5)
 * Mentor: Dr. J. Sarathkumar Sebastin (VIT Bhopal University)
 *
 * Certified Reference Standards:
 * - NASA TM-85767 / TP-1803: Low-speed cropped delta stability & reflex trim
 * - ICAO Doc 7488 / ISO 2533: Standard Atmosphere ISA barometric density model
 * - AIAA Education Series (Raymer, 6th Ed.): Trimmed drag polar & static stability
 * - BSERC Course Material: Chapters 8, 10, 12, 14, 15, 16
 * - UIUC Propeller Database: APC 10x6 Thin Electric efficiency model (eta_prop = 0.72)
 */

const AERO_CORE = (function () {
  'use strict';

  // --- CERTIFIED CONSTANTS (Project 4 DOCX & ISA Standards) ---
  const GEOMETRY = Object.freeze({
    Cr: 0.90,                                      // Root chord [m]
    Ct: 0.15,                                      // Tip chord [m]
    b: 1.50,                                       // Wingspan [m]
    e: 0.89,                                       // Oswald efficiency factor [-]
    lambda: 0.15 / 0.90,                           // Taper ratio = 0.16667 (1/6)
    S: 0.5 * 1.50 * 0.90 * (1 + 0.15 / 0.90),      // Wing area = 0.7875 m^2
    AR: (1.50 * 1.50) / 0.7875,                    // Aspect ratio = 2.8571 (20/7)
    k: 1 / (Math.PI * 0.89 * ((1.50 * 1.50) / 0.7875)), // Induced drag factor = 0.1252
    MAC: (2 / 3) * 0.90 * ((1 + (1 / 6) + (1 / 36)) / (1 + (1 / 6))), // 0.6143 m
    y_MAC: (1.50 / 6) * ((1 + 2 * (1 / 6)) / (1 + (1 / 6)))            // 0.2857 m
  });

  const DERIVATIVES = Object.freeze({
    CD0: 0.030,                // Zero-lift parasite drag coefficient [-]
    CL0: 0.000,                // Zero-AoA lift coefficient [-]
    Cm0: 0.010,                // Zero-lift pitching moment (Reflex camber) [-]
    CLa: 2.920,                // Wing lift curve slope [rad^-1]
    CLde: 0.265,               // Elevon lift effectiveness [rad^-1]
    Cma_baseline: -0.292,      // Pitch static stability derivative [rad^-1]
    Cmde: -0.400               // Elevon pitch control power [rad^-1]
  });

  const INERTIAL = Object.freeze({
    mass: 3.50,                // UAV takeoff mass [kg]
    g: 10.0,                   // Gravitational acceleration [m/s^2]
    W: 35.00,                  // Weight = m * g = 35.00 N
    wingLoading: 35.00 / 0.7875 // 44.44 N/m^2
  });

  const PROPULSION = Object.freeze({
    eta_prop: 0.72,            // Propeller propulsive efficiency (UIUC APC 10x6)
    eta_elec: 0.88,            // Motor + ESC electrical conversion efficiency
    P_avionics: 8.0,           // Constant autopilot + servo draw [W]
    V_batt_nominal: 14.8,      // 4S Li-Po nominal voltage [V]
    Q_Ah: 5.0,                 // Battery capacity [Ah] (5000 mAh)
    f_usable: 0.80,            // Depth of Discharge (80% safe reserve)
    E_usable_Wh: 0.80 * 14.8 * 5.0 // 59.20 Wh usable energy
  });

  const MASS_BREAKDOWN = Object.freeze([
    { id: 'airframe', name: 'Carbon-Epoxy Blended Delta Airframe & Spars', mass: 1.85, pct: 52.9, color: '#d4af37' },
    { id: 'battery',  name: '4S 5000mAh 35C Li-Po Battery Pack',          mass: 0.54, pct: 15.4, color: '#e63946' },
    { id: 'payload',  name: 'EO/IR Gimbal & Sensor Payload Suite',        mass: 0.50, pct: 14.3, color: '#38bdf8' },
    { id: 'motor',    name: 'Brushless Outrunner Motor, ESC & APC 10x6',  mass: 0.35, pct: 10.0, color: '#10b981' },
    { id: 'avionics', name: 'Autopilot, GPS, Pitot & Digital Metal Servos', mass: 0.26, pct: 7.4,  color: '#a855f7' }
  ]);

  function getAirDensity(altM) {
    if (altM <= 0) return 1.225;
    return 1.225 * Math.pow(1 - 2.2558e-5 * altM, 4.2559);
  }

  function solveFlightState(alphaDeg, altM = 0, smPct = 10.0, bankDeg = 0) {
    const rho = getAirDensity(altM);
    const alphaRad = alphaDeg * (Math.PI / 180.0);
    const bankRad = bankDeg * (Math.PI / 180.0);

    const Cma = -(smPct / 100.0) * DERIVATIVES.CLa;
    const deRad = -(DERIVATIVES.Cm0 + Cma * alphaRad) / DERIVATIVES.Cmde;
    const deDeg = deRad * (180.0 / Math.PI);

    const CL_untrimmed = DERIVATIVES.CL0 + DERIVATIVES.CLa * alphaRad;
    const CL_elevon = DERIVATIVES.CLde * deRad;
    const CL = Math.max(0.005, CL_untrimmed + CL_elevon);

    const nLoad = 1.0 / Math.cos(bankRad);
    const effectiveWeight = INERTIAL.W * nLoad;

    const V = Math.sqrt((2.0 * effectiveWeight) / (rho * GEOMETRY.S * CL));
    const Mach = V / 340.29;
    const qDyn = 0.5 * rho * V * V;

    const CDi = GEOMETRY.k * CL * CL;
    const CD = DERIVATIVES.CD0 + CDi;

    const Tr = qDyn * GEOMETRY.S * CD;
    const Pr = Tr * V;
    const LD = CL / CD;
    const endurParam = Math.pow(CL, 1.5) / CD;

    const Pshaft = Pr / PROPULSION.eta_prop;
    const Pbatt = (Pshaft / PROPULSION.eta_elec) + PROPULSION.P_avionics;
    const Ibatt = Pbatt / PROPULSION.V_batt_nominal;
    const endurHr = PROPULSION.E_usable_Wh / Pbatt;
    const endurMin = endurHr * 60.0;
    const rangeKm = (V * endurHr * 3600.0) / 1000.0;

    const servoMarginDeg = 20.0 - Math.abs(deDeg);
    const turnRadiusM = bankDeg > 0.5 ? (V * V) / (INERTIAL.g * Math.tan(bankRad)) : Infinity;
    const turnRateDegS = bankDeg > 0.5 ? ((INERTIAL.g * Math.tan(bankRad)) / V) * (180.0 / Math.PI) : 0.0;

    return {
      alphaDeg, alphaRad, altM, rho, smPct, bankDeg, nLoad,
      deRad, deDeg, CL_untrimmed, CL_elevon, CL, CDi, CD, V, Mach, qDyn,
      Tr, Pr, Pshaft, Pbatt, Ibatt, LD, endurParam,
      endurMin, rangeKm, servoMarginDeg, turnRadiusM, turnRateDegS
    };
  }

  function generateFullSweep(altM = 0, smPct = 10.0) {
    const states = [];
    for (let i = 0; i <= 24; i++) {
      states.push(solveFlightState(i * 0.5, altM, smPct, 0));
    }
    return states;
  }

  function generateVnEnvelope(altM = 0) {
    const rho = getAirDensity(altM);
    const CLmax = 1.2;
    const CLminNeg = 0.8;
    const nMax = 3.8;
    const nMin = -1.52;
    const Vs = Math.sqrt((2 * INERTIAL.W) / (rho * GEOMETRY.S * CLmax));
    const Vcorner = Vs * Math.sqrt(nMax);
    const VcornerNeg = Math.sqrt((2 * INERTIAL.W * Math.abs(nMin)) / (rho * GEOMETRY.S * CLminNeg));
    const Vne = 45.0;

    const stallCurvePos = [];
    for (let v = 0; v <= Vcorner; v += 0.5) {
      const n = (0.5 * rho * v * v * GEOMETRY.S * CLmax) / INERTIAL.W;
      stallCurvePos.push({ v, n: Math.min(n, nMax) });
    }
    return { Vs, Vcorner, VcornerNeg, Vne, nMax, nMin, stallCurvePos };
  }

  // ===========================================================================
  // HIGH-TESSELLATION 3D PHONG / CFD / X-RAY STEALTH UCAV ENGINE
  // ===========================================================================
  let _propAngle = 0;
  let _headingDeg = 0;
  const _windTunnelParticles = [];
  for (let i = 0; i < 110; i++) {
    _windTunnelParticles.push({
      x: (Math.random() - 0.5) * 1.65,
      y: (Math.random() - 0.5) * 0.45,
      z: -1.35 + Math.random() * 2.7,
      spd: 0.028 + Math.random() * 0.018
    });
  }

  /**
   * Computes exact 3D upper and lower surface height (y in aircraft coords, where -y is UP)
   * for our Blended-Wing-Body Cropped Delta Reflex Airframe at spanwise station x and chordwise fraction xi in [0,1].
   */
  function sampleBWBProfile(absX, xi) {
    const halfB = 0.75; // 1.50 m span / 2
    const eta = Math.min(1.0, absX / halfB); // normalized span [0..1]

    // Local chord and leading edge Z
    // Root (eta=0): zLE = -0.48 (extended nosecone to -0.54), zTE = +0.45
    // Tip  (eta=1): zLE = +0.30, zTE = +0.45 (Ct = 0.15m)
    let zLE, zTE;
    if (eta < 0.18) {
      // Smooth blended chine nose extension
      const blend = eta / 0.18;
      zLE = -0.54 + blend * (-0.45 + 0.54) + eta * 0.75;
    } else {
      zLE = -0.45 + eta * 0.75;
    }
    zTE = 0.45;
    const chord = zTE - zLE;
    const z = zLE + xi * chord;

    // NACA-style thickness distribution + BWB central fuselage humped spine
    const tRatio = (1.0 - 0.55 * eta) * 0.11 + (eta < 0.28 ? Math.pow(1.0 - eta / 0.28, 2) * 0.095 : 0);
    const yt = 5.0 * tRatio * chord * (
      0.2969 * Math.sqrt(Math.max(0, xi)) -
      0.1260 * xi -
      0.3516 * xi * xi +
      0.2843 * xi * xi * xi -
      0.1015 * xi * xi * xi * xi
    );

    // Reflexed S-camber line (positive nose-up Cm0 = +0.010)
    const reflexCamber = 0.022 * chord * Math.sin(Math.PI * xi * 1.35) * (1.0 - 1.32 * xi) * (1.0 - 0.4 * eta);

    // Canopy bulge near centerline (eta < 0.19, xi in [0.14, 0.58])
    let canopyBulge = 0;
    let isCanopy = false;
    if (eta < 0.175 && xi > 0.13 && xi < 0.56) {
      const cxNorm = (xi - 0.345) / 0.215;
      const cyNorm = eta / 0.175;
      const r2 = cxNorm * cxNorm + cyNorm * cyNorm;
      if (r2 < 1.0) {
        canopyBulge = 0.052 * Math.pow(1.0 - r2, 1.35);
        isCanopy = true;
      }
    }

    // Note: in screen/camera space, negative Y is UP
    const yUpper = -(yt + reflexCamber + canopyBulge);
    const yLower = +(yt * 0.55 - reflexCamber * 0.5);

    return { z, yUpper, yLower, eta, xi, isCanopy, chord, zLE, zTE };
  }

  /**
   * Pre-builds the base spanwise/chordwise grid stations for high-speed 60FPS dynamic mesh shading
   */
  const SPAN_STATIONS = [-0.75, -0.66, -0.55, -0.44, -0.33, -0.23, -0.15, -0.09, -0.04, 0.0, 0.04, 0.09, 0.15, 0.23, 0.33, 0.44, 0.55, 0.66, 0.75];
  const CHORD_STATIONS = [0.0, 0.06, 0.14, 0.24, 0.35, 0.46, 0.57, 0.68, 0.78, 0.86, 0.93, 1.0];

  function colormapCp(cpNorm) {
    // Maps normalized suction/pressure [-1.2 .. +0.6] to vibrant CFD palette (Blue -> Cyan -> Emerald -> Gold -> Crimson)
    const t = Math.max(0, Math.min(1, (cpNorm + 1.1) / 1.6));
    let r, g, b;
    if (t < 0.25) {
      const u = t / 0.25;
      r = Math.round(230 - u * 30); g = Math.round(45 + u * 135); b = Math.round(65 - u * 45);
    } else if (t < 0.55) {
      const u = (t - 0.25) / 0.30;
      r = Math.round(200 - u * 170); g = Math.round(180 + u * 35); b = Math.round(20 + u * 100);
    } else if (t < 0.80) {
      const u = (t - 0.55) / 0.25;
      r = Math.round(30 - u * 15); g = Math.round(215 - u * 65); b = Math.round(120 + u * 125);
    } else {
      const u = (t - 0.80) / 0.20;
      r = Math.round(15 + u * 25); g = Math.round(150 - u * 90); b = Math.round(245 - u * 45);
    }
    return `rgb(${r},${g},${b})`;
  }

  function renderStealthDelta3D(canvas, st, rotX, rotY, opts = {}) {
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const cw = canvas.clientWidth || 800;
    const ch = canvas.clientHeight || 500;
    if (canvas.width !== Math.floor(cw * dpr) || canvas.height !== Math.floor(ch * dpr)) {
      canvas.width = Math.floor(cw * dpr);
      canvas.height = Math.floor(ch * dpr);
    }

    ctx.save();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, cw, ch);

    const visualMode = opts.visualMode || 'stealth'; // 'stealth' | 'cfd' | 'xray'
    const showFlow = opts.showFlow !== false;
    const showVectors = opts.showVectors !== false;
    const showElevons = opts.showElevons !== false;
    const showGrid = opts.showGrid !== false;
    const showHUD = opts.showHUD !== false;

    // 1. Deep Aerospace Hangar / Wind-Tunnel Atmosphere Background
    const bgGrad = ctx.createRadialGradient(cw * 0.5, ch * 0.48, 10, cw * 0.5, ch * 0.5, cw * 0.68);
    if (visualMode === 'cfd') {
      bgGrad.addColorStop(0, '#101829');
      bgGrad.addColorStop(0.55, '#090d16');
      bgGrad.addColorStop(1, '#05070b');
    } else if (visualMode === 'xray') {
      bgGrad.addColorStop(0, '#0c1622');
      bgGrad.addColorStop(0.6, '#070b12');
      bgGrad.addColorStop(1, '#04060a');
    } else {
      bgGrad.addColorStop(0, '#181e2c');
      bgGrad.addColorStop(0.55, '#0b0e16');
      bgGrad.addColorStop(1, '#05070b');
    }
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, cw, ch);

    const cx = cw * 0.5;
    const cy = ch * 0.50;
    const zoom = opts.zoom || 1.0;
    const fov = Math.min(cw, ch) * 0.92 * zoom;

    const cosX = Math.cos(rotX), sinX = Math.sin(rotX);
    const cosY = Math.cos(rotY), sinY = Math.sin(rotY);

    function worldToCam(wx, wy, wz) {
      const x1 = wx * cosY + wz * sinY;
      const z1 = -wx * sinY + wz * cosY;
      const y2 = wy * cosX - z1 * sinX;
      const z2 = wy * sinX + z1 * cosX;
      const dist = 2.55 + z2;
      const s = fov / Math.max(0.25, dist);
      return { px: cx + x1 * s, py: cy + y2 * s, cx: x1, cy: y2, cz: z2, s };
    }

    // Pitch (alpha) and Roll (bank) transformation
    const pitchRad = (st.alphaDeg * Math.PI) / 180.0;
    const rollRad = (st.bankDeg * Math.PI) / 180.0;
    const cP = Math.cos(pitchRad), sP = Math.sin(pitchRad);
    const cR = Math.cos(rollRad), sR = Math.sin(rollRad);

    function bodyToWorld(bx, by, bz) {
      // Pitch around X first (nose up = -by for negative z), then Roll around Z
      const py = by * cP + bz * sP;
      const pz = -by * sP + bz * cP;
      const rx = bx * cR - py * sR;
      const ry = bx * sR + py * cR;
      return { wx: rx, wy: ry, wz: pz };
    }

    function transBody(bx, by, bz) {
      const w = bodyToWorld(bx, by, bz);
      return worldToCam(w.wx, w.wy, w.wz);
    }

    // 2. Perspective Wind-Tunnel Floor Grid & Shadow Footprint
    if (showGrid) {
      const floorY = 0.36;
      ctx.strokeStyle = visualMode === 'xray' ? 'rgba(56, 189, 248, 0.11)' : 'rgba(212, 175, 55, 0.09)';
      ctx.lineWidth = 1;
      for (let gz = -1.2; gz <= 1.21; gz += 0.24) {
        const p1 = worldToCam(-1.2, floorY, gz);
        const p2 = worldToCam(1.2, floorY, gz);
        ctx.beginPath(); ctx.moveTo(p1.px, p1.py); ctx.lineTo(p2.px, p2.py); ctx.stroke();
      }
      for (let gx = -1.2; gx <= 1.21; gx += 0.24) {
        const p1 = worldToCam(gx, floorY, -1.2);
        const p2 = worldToCam(gx, floorY, 1.2);
        ctx.beginPath(); ctx.moveTo(p1.px, p1.py); ctx.lineTo(p2.px, p2.py); ctx.stroke();
      }

      // Ground Shadow Silhouette under aircraft
      const shNose = worldToCam(0, floorY, -0.52);
      const shTipR = worldToCam(0.74 * cR, floorY, 0.30);
      const shTER  = worldToCam(0.74 * cR, floorY, 0.45);
      const shTEL  = worldToCam(-0.74 * cR, floorY, 0.45);
      const shTipL = worldToCam(-0.74 * cR, floorY, 0.30);
      ctx.fillStyle = 'rgba(0, 0, 0, 0.48)';
      ctx.beginPath();
      ctx.moveTo(shNose.px, shNose.py);
      ctx.lineTo(shTipR.px, shTipR.py);
      ctx.lineTo(shTER.px, shTER.py);
      ctx.lineTo(shTEL.px, shTEL.py);
      ctx.lineTo(shTipL.px, shTipL.py);
      ctx.closePath();
      ctx.fill();
    }

    // 3. Generate Depth-Sorted 3D Quad/Triangle Faces of the Blended Cropped Delta
    const deflRad = showElevons ? (st.deDeg * 2.2 * Math.PI) / 180.0 : 0.0;
    const faces = [];

    // Light sources in camera/world space (Key light top-left-front, Gold rim light right)
    const Lx = -0.38, Ly = -0.82, Lz = -0.42;

    function getVertex3D(x, xi, isUpper) {
      const prof = sampleBWBProfile(Math.abs(x), xi);
      let y = isUpper ? prof.yUpper : prof.yLower;
      let z = prof.z;
      let isElevon = false;

      // Trailing-edge Elevon articulation for |x| in [0.15, 0.70] and xi >= 0.86
      if (Math.abs(x) >= 0.145 && Math.abs(x) <= 0.71 && xi >= 0.855) {
        isElevon = true;
        const zHinge = prof.zLE + 0.86 * prof.chord;
        const dz = z - zHinge;
        if (dz > 0) {
          // Deflect around zHinge
          y += dz * Math.sin(deflRad);
          z = zHinge + dz * Math.cos(deflRad);
        }
      }
      const w = bodyToWorld(x, y, z);
      const cam = worldToCam(w.wx, w.wy, w.wz);
      return {
        bx: x, by: y, bz: z,
        wx: w.wx, wy: w.wy, wz: w.wz,
        px: cam.px, py: cam.py, cz: cam.cz, s: cam.s,
        eta: prof.eta, xi: xi, isCanopy: prof.isCanopy && isUpper, isElevon, isUpper
      };
    }

    // Build upper & lower surface mesh quads
    for (let i = 0; i < SPAN_STATIONS.length - 1; i++) {
      const x0 = SPAN_STATIONS[i];
      const x1 = SPAN_STATIONS[i + 1];
      for (let j = 0; j < CHORD_STATIONS.length - 1; j++) {
        const xi0 = CHORD_STATIONS[j];
        const xi1 = CHORD_STATIONS[j + 1];

        for (let surf = 0; surf < 2; surf++) {
          const isUpper = surf === 0;
          const v00 = getVertex3D(x0, xi0, isUpper);
          const v10 = getVertex3D(x1, xi0, isUpper);
          const v11 = getVertex3D(x1, xi1, isUpper);
          const v01 = getVertex3D(x0, xi1, isUpper);

          // Compute 3D surface normal in world space
          const ux = v10.wx - v00.wx, uy = v10.wy - v00.wy, uz = v10.wz - v00.wz;
          const vx = v01.wx - v00.wx, vy = v01.wy - v00.wy, vz = v01.wz - v00.wz;
          let nx = uy * vz - uz * vy;
          let ny = uz * vx - ux * vz;
          let nz = ux * vy - uy * vx;
          if (!isUpper) { nx = -nx; ny = -ny; nz = -nz; }
          const nLen = Math.hypot(nx, ny, nz) || 1.0;
          nx /= nLen; ny /= nLen; nz /= nLen;

          const avgCz = 0.25 * (v00.cz + v10.cz + v11.cz + v01.cz);
          const avgEta = 0.5 * (v00.eta + v10.eta);
          const avgXi = 0.5 * (xi0 + xi1);
          const isCanopy = v00.isCanopy || v11.isCanopy;
          const isElevon = v00.isElevon && v11.isElevon;

          faces.push({
            pts: [v00, v10, v11, v01],
            cz: avgCz,
            nx, ny, nz,
            eta: avgEta,
            xi: avgXi,
            isUpper,
            isCanopy,
            isElevon
          });
        }
      }
    }

    // Add Canted Stealth Wingtip Winglets / Fin Plates at x = ±0.75m
    [-0.75, 0.75].forEach(sideX => {
      const sign = sideX > 0 ? 1 : -1;
      const wBaseLE = transBody(sideX, 0.005, 0.30);
      const wBaseTE = transBody(sideX, 0.005, 0.45);
      const wTipLE  = transBody(sideX + sign * 0.025, -0.068, 0.35);
      const wTipTE  = transBody(sideX + sign * 0.025, -0.062, 0.455);
      faces.push({
        pts: [wBaseLE, wBaseTE, wTipTE, wTipLE],
        cz: 0.25 * (wBaseLE.cz + wBaseTE.cz + wTipTE.cz + wTipLE.cz),
        nx: sign * 0.85, ny: -0.45, nz: 0.1,
        eta: 1.0, xi: 0.5, isUpper: true, isWinglet: true
      });
    });

    // Sort faces back-to-front (Painter's Z-buffer algorithm)
    faces.sort((a, b) => b.cz - a.cz);

    // Render all 3D mesh faces with Phong Specular / CFD Pressure / X-Ray shading
    faces.forEach(f => {
      const p = f.pts;
      ctx.beginPath();
      ctx.moveTo(p[0].px, p[0].py);
      ctx.lineTo(p[1].px, p[1].py);
      ctx.lineTo(p[2].px, p[2].py);
      ctx.lineTo(p[3].px, p[3].py);
      ctx.closePath();

      // Diffuse + Specular lighting
      const diff = Math.max(0.08, -(f.nx * Lx + f.ny * Ly + f.nz * Lz));
      const spec = Math.pow(Math.max(0, diff), 14);

      if (visualMode === 'xray') {
        ctx.fillStyle = f.isElevon
          ? 'rgba(230, 57, 70, 0.28)'
          : f.isCanopy
          ? 'rgba(212, 175, 55, 0.22)'
          : 'rgba(15, 25, 42, 0.38)';
        ctx.fill();
        ctx.strokeStyle = f.isElevon
          ? 'rgba(230, 57, 70, 0.75)'
          : 'rgba(56, 189, 248, 0.28)';
        ctx.lineWidth = 0.65;
        ctx.stroke();
      } else if (visualMode === 'cfd') {
        // Real aerodynamic surface Cp pressure distribution
        const alphaEff = st.alphaDeg / 12.0;
        let cp;
        if (f.isUpper) {
          // Suction peak near leading edge & vortex core around eta ~ 0.65
          const leSuction = -1.15 * alphaEff * Math.exp(-f.xi * 3.2) * (1.0 + 0.45 * Math.sin(f.eta * Math.PI));
          const reflexRear = f.isElevon ? (st.deDeg * 0.06) : (0.18 * Math.pow(f.xi, 2));
          cp = leSuction + reflexRear - 0.15;
        } else {
          cp = 0.45 * alphaEff * (1.0 - 0.5 * f.xi) + 0.1;
        }
        ctx.fillStyle = colormapCp(cp);
        ctx.fill();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
        ctx.lineWidth = 0.45;
        ctx.stroke();
      } else {
        // STEALTH CARBON-COMPOSITE PBR MODE
        if (f.isCanopy) {
          // Gold-tinted radar-absorbent fighter canopy glass
          const r = Math.min(255, Math.round(35 + diff * 95 + spec * 185));
          const g = Math.min(255, Math.round(28 + diff * 72 + spec * 145));
          const b = Math.min(255, Math.round(22 + diff * 45 + spec * 65));
          ctx.fillStyle = `rgb(${r},${g},${b})`;
          ctx.fill();
          ctx.strokeStyle = 'rgba(212, 175, 55, 0.45)';
          ctx.lineWidth = 0.6;
          ctx.stroke();
        } else if (f.isElevon) {
          const isUp = st.deDeg < -0.05;
          const baseR = isUp ? 165 : 155;
          const baseG = isUp ? 32  : 125;
          const baseB = isUp ? 45  : 35;
          const r = Math.min(255, Math.round(baseR * (0.45 + 0.7 * diff) + spec * 90));
          const g = Math.min(255, Math.round(baseG * (0.45 + 0.7 * diff) + spec * 70));
          const b = Math.min(255, Math.round(baseB * (0.45 + 0.7 * diff) + spec * 70));
          ctx.fillStyle = `rgb(${r},${g},${b})`;
          ctx.fill();
          ctx.strokeStyle = isUp ? 'rgba(255, 110, 120, 0.65)' : 'rgba(243, 229, 171, 0.65)';
          ctx.lineWidth = 0.7;
          ctx.stroke();
        } else if (f.isWinglet) {
          ctx.fillStyle = '#1b202e';
          ctx.fill();
          ctx.strokeStyle = '#e63946';
          ctx.lineWidth = 1.1;
          ctx.stroke();
        } else {
          // Obsidian/Anthracite Stealth Carbon Skin with anisotropic metallic specular sheen
          const lum = f.isUpper ? (16 + diff * 34 + spec * 68) : (10 + diff * 18);
          const r = Math.min(255, Math.round(lum * 0.92 + spec * 24));
          const g = Math.min(255, Math.round(lum * 0.98 + spec * 18));
          const b = Math.min(255, Math.round(lum * 1.18));
          ctx.fillStyle = `rgb(${r},${g},${b})`;
          ctx.fill();
          ctx.strokeStyle = f.isUpper ? 'rgba(212, 175, 55, 0.08)' : 'rgba(255, 255, 255, 0.03)';
          ctx.lineWidth = 0.5;
          ctx.stroke();
        }
      }
    });

    // 4. In X-Ray Mode: Render Internal Carbon Spars, CG, NP, and MAC!
    if (visualMode === 'xray') {
      // Main Front Spar & Rear Elevon Hinge Spar
      ctx.strokeStyle = '#d4af37';
      ctx.lineWidth = 2.4;
      const spF_L = transBody(-0.72, -0.01, 0.31);
      const spF_C = transBody(0.0, -0.02, -0.10);
      const spF_R = transBody(0.72, -0.01, 0.31);
      ctx.beginPath(); ctx.moveTo(spF_L.px, spF_L.py); ctx.lineTo(spF_C.px, spF_C.py); ctx.lineTo(spF_R.px, spF_R.py); ctx.stroke();

      const spR_L = transBody(-0.72, -0.005, 0.38);
      const spR_R = transBody(0.72, -0.005, 0.38);
      ctx.strokeStyle = '#e63946';
      ctx.lineWidth = 2.0;
      ctx.beginPath(); ctx.moveTo(spR_L.px, spR_L.py); ctx.lineTo(spR_R.px, spR_R.py); ctx.stroke();

      // Mean Aerodynamic Chord (MAC = 0.6143 m at y_MAC = 0.2857 m)
      const macLE = transBody(0.2857, -0.015, -0.164);
      const macTE = transBody(0.2857, -0.015, 0.45);
      ctx.strokeStyle = '#f3e5ab';
      ctx.setLineDash([4, 3]);
      ctx.lineWidth = 2.0;
      ctx.beginPath(); ctx.moveTo(macLE.px, macLE.py); ctx.lineTo(macTE.px, macTE.py); ctx.stroke();
      ctx.setLineDash([]);

      // CG & NP Spheres
      const ptCG = transBody(0.0, -0.02, 0.02);
      const ptNP = transBody(0.0, -0.02, 0.02 + 0.6143 * (st.smPct / 100.0));
      ctx.fillStyle = '#10b981';
      ctx.beginPath(); ctx.arc(ptCG.px, ptCG.py, 5.5, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = '700 10px "JetBrains Mono"';
      ctx.fillText('CG', ptCG.px + 8, ptCG.py - 2);

      ctx.fillStyle = '#d4af37';
      ctx.beginPath(); ctx.arc(ptNP.px, ptNP.py, 5.5, 0, Math.PI * 2); ctx.fill();
      ctx.fillText('NP', ptNP.px + 8, ptNP.py + 10);
    }

    // 5. Stealth Panel Trim Accents, IAF Roundel, and Stencils (in Stealth Mode)
    if (visualMode === 'stealth') {
      // Glowing Crimson Stealth Seam Lines
      ctx.strokeStyle = 'rgba(230, 57, 70, 0.85)';
      ctx.lineWidth = 1.6;
      [-1, 1].forEach(side => {
        const p1 = transBody(side * 0.16, -0.022, -0.22);
        const p2 = transBody(side * 0.69, -0.006, 0.31);
        const p3 = transBody(side * 0.69, -0.005, 0.39);
        const p4 = transBody(side * 0.16, -0.018, 0.39);
        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py); ctx.lineTo(p2.px, p2.py);
        ctx.lineTo(p3.px, p3.py); ctx.lineTo(p4.px, p4.py);
        ctx.stroke();
      });

      // GoldLeading-Edge Highlight
      ctx.strokeStyle = 'rgba(212, 175, 55, 0.72)';
      ctx.lineWidth = 1.5;
      const leNose = transBody(0, -0.002, -0.54);
      const leTipR = transBody(0.75, 0.005, 0.30);
      const leTipL = transBody(-0.75, 0.005, 0.30);
      ctx.beginPath();
      ctx.moveTo(leTipL.px, leTipL.py);
      ctx.lineTo(leNose.px, leNose.py);
      ctx.lineTo(leTipR.px, leTipR.py);
      ctx.stroke();

      // IAF Roundel on Starboard Wing
      const vRnd = transBody(0.47, -0.018, 0.25);
      const rR = Math.max(4.5, vRnd.s * 0.036);
      ctx.fillStyle = '#ff9933';
      ctx.beginPath(); ctx.arc(vRnd.px, vRnd.py, rR, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#ffffff';
      ctx.beginPath(); ctx.arc(vRnd.px, vRnd.py, rR * 0.64, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = '#138808';
      ctx.beginPath(); ctx.arc(vRnd.px, vRnd.py, rR * 0.32, 0, Math.PI * 2); ctx.fill();

      // Stencil "CD-UAV 3.5"
      const vSt = transBody(0.30, -0.025, 0.14);
      ctx.fillStyle = 'rgba(243, 229, 171, 0.88)';
      ctx.font = '700 10px "JetBrains Mono", monospace';
      ctx.fillText('CD-UAV 3.5', vSt.px - 28, vSt.py);
    }

    // 6. Rear Brushless Pusher Motor Cone & 3-Blade APC 10x6 Propeller
    const speedRatio = Math.max(0.5, Math.min(3.0, st.V / 12.0));
    _propAngle += 0.42 * speedRatio;
    _headingDeg = (_headingDeg + st.turnRateDegS * 0.016) % 360;

    const vSpinnerBase = transBody(0, -0.016, 0.45);
    const vSpinnerTip  = transBody(0, -0.016, 0.51);
    const propR = 0.145;

    // Spinning Propeller Disc Motion Blur Ring
    ctx.strokeStyle = 'rgba(212, 175, 55, 0.32)';
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    for (let a = 0; a <= Math.PI * 2.05; a += 0.2) {
      const pt = transBody(Math.cos(a) * propR, -0.016 + Math.sin(a) * propR, 0.48);
      if (a === 0) ctx.moveTo(pt.px, pt.py); else ctx.lineTo(pt.px, pt.py);
    }
    ctx.stroke();

    // 3 Twisted Airfoil Blades
    for (let bIdx = 0; bIdx < 3; bIdx++) {
      const ang = _propAngle + (bIdx * Math.PI * 2) / 3;
      const bTip = transBody(Math.cos(ang) * propR, -0.016 + Math.sin(ang) * propR, 0.48);
      const bMid1 = transBody(Math.cos(ang - 0.16) * (propR * 0.55), -0.016 + Math.sin(ang - 0.16) * (propR * 0.55), 0.475);
      const bMid2 = transBody(Math.cos(ang + 0.16) * (propR * 0.55), -0.016 + Math.sin(ang + 0.16) * (propR * 0.55), 0.485);
      ctx.fillStyle = '#d4af37';
      ctx.beginPath();
      ctx.moveTo(vSpinnerBase.px, vSpinnerBase.py);
      ctx.lineTo(bMid1.px, bMid1.py);
      ctx.lineTo(bTip.px, bTip.py);
      ctx.lineTo(bMid2.px, bMid2.py);
      ctx.closePath();
      ctx.fill();
    }
    ctx.fillStyle = '#e63946';
    ctx.beginPath(); ctx.arc(vSpinnerTip.px, vSpinnerTip.py, 4.5, 0, Math.PI * 2); ctx.fill();

    // 7. Volumetric 3D Polhamus Leading-Edge Vortex (LEV) Ribbons & Smoke Streamlines
    if (showFlow) {
      const vortIntensity = Math.max(0.15, st.alphaDeg / 10.0);
      [-1, 1].forEach(side => {
        ctx.beginPath();
        for (let s = 0; s <= 42; s++) {
          const prog = s / 42.0; // 0 at apex (-0.48), 1 at wake (+0.85)
          const bz = -0.46 + prog * 1.28;
          const spanTrack = side * (0.04 + prog * 0.62);
          const helixA = prog * 16.0 - _propAngle * 0.85 * side;
          const radius = (0.012 + prog * 0.065) * vortIntensity;
          const bx = spanTrack + Math.cos(helixA) * radius;
          const by = -0.035 - prog * 0.08 * vortIntensity + Math.sin(helixA) * radius;
          const pt = transBody(bx, by, bz);
          if (s === 0) ctx.moveTo(pt.px, pt.py);
          else ctx.lineTo(pt.px, pt.py);
        }
        ctx.strokeStyle = side > 0 ? 'rgba(230, 57, 70, 0.72)' : 'rgba(212, 175, 55, 0.72)';
        ctx.lineWidth = 2.0;
        ctx.stroke();
      });

      // Wind-tunnel freestream smoke particles
      _windTunnelParticles.forEach(p => {
        p.z += p.spd * speedRatio;
        if (p.z > 1.35) {
          p.z = -1.35;
          p.x = (Math.random() - 0.5) * 1.65;
          p.y = (Math.random() - 0.5) * 0.45;
        }
        const pt1 = worldToCam(p.x, p.y, p.z);
        const pt2 = worldToCam(p.x, p.y, p.z - 0.095 * speedRatio);
        ctx.strokeStyle = 'rgba(243, 229, 171, 0.32)';
        ctx.lineWidth = 1.1;
        ctx.beginPath();
        ctx.moveTo(pt1.px, pt1.py);
        ctx.lineTo(pt2.px, pt2.py);
        ctx.stroke();
      });
    }

    // 8. 3D Aerodynamic Force Vectors (Lift, Weight, Thrust, Drag)
    if (showVectors) {
      const vCG = transBody(0, -0.02, 0.02);
      const liftLen = 0.34 * Math.min(1.6, st.nLoad);
      const vLift   = transBody(0, -0.02 - liftLen, 0.02);
      const vWeight = worldToCam(vCG.cx, vCG.cy + 0.32, vCG.cz);
      const vThrust = transBody(0, -0.02, -0.39);
      const vDrag   = transBody(0, -0.02, 0.42);

      function drawArrow3D(from, to, col, text) {
        ctx.strokeStyle = col;
        ctx.fillStyle = col;
        ctx.lineWidth = 2.3;
        ctx.beginPath();
        ctx.moveTo(from.px, from.py);
        ctx.lineTo(to.px, to.py);
        ctx.stroke();

        const ang = Math.atan2(to.py - from.py, to.px - from.px);
        ctx.beginPath();
        ctx.moveTo(to.px, to.py);
        ctx.lineTo(to.px - 9 * Math.cos(ang - 0.4), to.py - 9 * Math.sin(ang - 0.4));
        ctx.lineTo(to.px - 9 * Math.cos(ang + 0.4), to.py - 9 * Math.sin(ang + 0.4));
        ctx.closePath();
        ctx.fill();

        ctx.font = '700 10.5px "JetBrains Mono", monospace';
        ctx.fillText(text, to.px + 7, to.py + 3);
      }

      drawArrow3D(vCG, vLift,   '#10b981', `L = ${(INERTIAL.W * st.nLoad).toFixed(1)} N`);
      drawArrow3D(vCG, vWeight, '#38bdf8', `W = 35.0 N`);
      drawArrow3D(vCG, vThrust, '#d4af37', `Tr = ${st.Tr.toFixed(2)} N`);
      drawArrow3D(vCG, vDrag,   '#e63946', `D = ${st.Tr.toFixed(2)} N`);
    }

    // 9. FIGHTER JET PRIMARY FLIGHT DISPLAY (PFD) & HUD OVERLAY (when enabled on full simulator)
    if (showHUD && cw >= 520) {
      ctx.save();
      // Subtle green/gold tactical HUD reticle & pitch ladder in center
      const hudColor = 'rgba(212, 175, 55, 0.48)';
      ctx.strokeStyle = hudColor;
      ctx.fillStyle = hudColor;
      ctx.lineWidth = 1.1;

      // Center Bore-sight Crosshair
      ctx.beginPath();
      ctx.moveTo(cx - 18, cy); ctx.lineTo(cx - 6, cy);
      ctx.moveTo(cx + 6, cy);  ctx.lineTo(cx + 18, cy);
      ctx.moveTo(cx, cy - 12); ctx.lineTo(cx, cy - 4);
      ctx.stroke();

      // Bank Angle Arc at top center
      ctx.beginPath();
      ctx.arc(cx, cy, 145, -Math.PI * 0.72, -Math.PI * 0.28);
      ctx.stroke();

      // Artificial Horizon Bar rotated by Bank Angle and shifted by Pitch (AoA)
      ctx.save();
      ctx.translate(cx, cy + st.alphaDeg * 5.5);
      ctx.rotate(-rollRad);
      ctx.strokeStyle = 'rgba(16, 185, 129, 0.38)';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(-110, 0); ctx.lineTo(-38, 0);
      ctx.moveTo(38, 0);   ctx.lineTo(110, 0);
      ctx.stroke();
      ctx.restore();

      // CFD Colorbar Legend when in CFD mode
      if (visualMode === 'cfd') {
        const cbX = cw - 155, cbY = 66, cbW = 135, cbH = 12;
        const grad = ctx.createLinearGradient(cbX, 0, cbX + cbW, 0);
        grad.addColorStop(0, colormapCp(-1.1));
        grad.addColorStop(0.5, colormapCp(-0.3));
        grad.addColorStop(1, colormapCp(0.5));
        ctx.fillStyle = 'rgba(8, 10, 15, 0.85)';
        ctx.fillRect(cbX - 8, cbY - 18, cbW + 16, 44);
        ctx.strokeStyle = 'rgba(212, 175, 55, 0.35)';
        ctx.strokeRect(cbX - 8, cbY - 18, cbW + 16, 44);
        ctx.fillStyle = grad;
        ctx.fillRect(cbX, cbY, cbW, cbH);
        ctx.fillStyle = '#f3e5ab';
        ctx.font = '700 9.5px "JetBrains Mono"';
        ctx.fillText('SURFACE Cp HEATMAP', cbX, cbY - 5);
        ctx.fillStyle = '#94a3b8';
        ctx.fillText('-1.2 (Suction)', cbX, cbY + 22);
        ctx.fillText('+0.5', cbX + cbW - 26, cbY + 22);
      }

      ctx.restore();
    }

    ctx.restore();
  }

  // ===========================================================================
  // INSTITUTIONAL HEADER & BACKEND API SYNC INJECTOR
  // ===========================================================================
  function injectInstitutionalHeader() {
    const mainEl = document.querySelector('main.app-main');
    if (!mainEl || mainEl.querySelector('.inst-prestige-bar')) return;

    const banner = document.createElement('div');
    banner.className = 'inst-prestige-bar';
    banner.innerHTML = `
      <div class="inst-left">
        <div class="inst-emblem">✦</div>
        <div class="inst-left-text">
          <strong>BSERC Def-Space</strong>
          <span>Autumn Internship Programme 2026</span>
        </div>
      </div>
      <div class="inst-center">
        <h2>Aircraft Design Technology</h2>
        <p>Computational Aerodynamics &nbsp;|&nbsp; Flight Mechanics &nbsp;|&nbsp; UAV Propulsion Integration</p>
      </div>
      <div class="inst-right">
        <strong>VIT Bhopal University</strong>
        <span>Under the Mentorship of Dr. J. Sarathkumar Sebastin</span>
      </div>
    `;
    mainEl.insertBefore(banner, mainEl.firstChild);

    const navEl = document.querySelector('nav.sidebar-nav');
    if (navEl && !navEl.querySelector('a[href*="index.html"]')) {
      const isSubdir = window.location.pathname.includes('/app/');
      const homeHref = isSubdir ? '../index.html' : 'index.html';
      const homeLink = document.createElement('a');
      homeLink.href = homeHref;
      homeLink.className = 'nav-link';
      homeLink.innerHTML = `<span class="nav-icon">🏛️</span> Command Hub (Overview)`;
      navEl.insertBefore(homeLink, navEl.firstChild);
    }

    if (window.location.protocol.startsWith('http')) {
      fetch('/api/health')
        .then(r => r.ok ? r.json() : null)
        .then(data => {
          if (data && data.status === 'OPERATIONAL') {
            const badgeSpan = document.querySelector('.sys-badge span');
            if (badgeSpan) badgeSpan.textContent = 'FASTAPI ENGINE: ONLINE';
          }
        })
        .catch(() => {});
    }
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', injectInstitutionalHeader);
    } else {
      injectInstitutionalHeader();
    }
  }

  return {
    GEOMETRY,
    DERIVATIVES,
    INERTIAL,
    PROPULSION,
    MASS_BREAKDOWN,
    getAirDensity,
    solveFlightState,
    generateFullSweep,
    generateVnEnvelope,
    renderStealthDelta3D
  };
})();

if (typeof module !== 'undefined' && module.exports) {
  module.exports = AERO_CORE;
} else if (typeof window !== 'undefined') {
  window.AERO_CORE = AERO_CORE;
}
