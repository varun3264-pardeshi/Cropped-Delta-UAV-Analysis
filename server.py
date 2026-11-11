# -*- coding: utf-8 -*-
"""
BSERC DEF-SPACE AUTUMN INTERNSHIP PROGRAMME 2026
AIRCRAFT DESIGN PROJECT 4 — BACKEND COMPUTATIONAL SERVER (FastAPI)
"Trim, Thrust Required, and Power Required Analysis of a Tailless Cropped Delta Wing UAV"
Mentor: Dr. J. Sarathkumar Sebastin (VIT Bhopal University)

Provides:
1. RESTful Aerodynamic & Flight Mechanics API (/api/*)
2. Multi-Page Web Application Routing (/, /simulator, /performance, /propulsion, /envelope, /docs)
3. Static Asset & Deliverable Serving (/app/*, /Plots/*, downloadable .m, .xls, .csv, .docx, .pptx)
"""

import math
import os
from pathlib import Path
from typing import Dict, List, Any

from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

BASE_DIR = Path(__file__).resolve().parent
APP_DIR = BASE_DIR / "app"
PLOTS_DIR = BASE_DIR / "Plots"

app = FastAPI(
    title="CD-UAV 3.5 | Aerospace Command-Centre Backend API",
    description="Certified Computational Flight Mechanics & Trim Equilibrium Engine (BSERC Def-Space 2026)",
    version="4.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# =============================================================================
# CERTIFIED AEROSPACE CONSTANTS (Project 4 DOCX, BSERC Material, NASA TM-85767)
# =============================================================================
Cr = 0.90                       # Root chord [m]
Ct = 0.15                       # Tip chord [m]
b = 1.50                        # Wingspan [m]
e_oswald = 0.89                 # Oswald span efficiency [-]
taper_lambda = Ct / Cr          # 0.166667 (1/6)
S_wing = 0.5 * b * Cr * (1.0 + taper_lambda)  # 0.7875 m^2
AR_wing = (b ** 2) / S_wing     # 2.857143 (20/7)
k_induced = 1.0 / (math.pi * e_oswald * AR_wing)  # 0.125185
MAC = (2.0 / 3.0) * Cr * ((1.0 + taper_lambda + taper_lambda ** 2) / (1.0 + taper_lambda))  # 0.614286 m
y_MAC = (b / 6.0) * ((1.0 + 2.0 * taper_lambda) / (1.0 + taper_lambda))  # 0.285714 m

GEOMETRY = {
    "Cr": Cr,
    "Ct": Ct,
    "b": b,
    "e": e_oswald,
    "lambda": round(taper_lambda, 6),
    "S": round(S_wing, 6),
    "AR": round(AR_wing, 6),
    "k": round(k_induced, 6),
    "MAC": round(MAC, 6),
    "y_MAC": round(y_MAC, 6),
}

DERIVATIVES = {
    "CD0": 0.030,               # Zero-lift parasite drag coefficient [-]
    "CL0": 0.000,               # Zero-AoA lift coefficient [-]
    "Cm0": 0.010,               # Reflex camber zero-lift pitching moment [-]
    "CLa": 2.920,               # Lift curve slope [rad^-1]
    "CLde": 0.265,              # Elevon lift effectiveness [rad^-1]
    "Cma_baseline": -0.292,     # Longitudinal static stability derivative [rad^-1]
    "Cmde": -0.400,             # Elevon pitch control derivative [rad^-1]
}

INERTIAL = {
    "mass": 3.50,               # All-up mass [kg]
    "g": 10.0,                  # Gravitational acceleration [m/s^2] (per Project 4 spec)
    "W": 35.00,                 # Total aircraft weight [N]
    "wingLoading": round(35.00 / S_wing, 4),  # 44.4444 N/m^2
}

PROPULSION = {
    "eta_prop": 0.72,           # APC 10x6 Thin Electric propeller efficiency (UIUC database)
    "eta_elec": 0.88,           # Brushless motor + ESC efficiency
    "P_avionics": 8.0,          # Flight controller + digital servos continuous draw [W]
    "V_batt_nominal": 14.8,     # 4S Li-Po nominal voltage [V]
    "Q_Ah": 5.0,                # Battery capacity [Ah] (5000 mAh)
    "f_usable": 0.80,           # Safe usable depth of discharge (80%)
    "E_usable_Wh": 59.20,       # 14.8 * 5.0 * 0.80 = 59.20 Wh
}

MASS_BREAKDOWN = [
    {"id": "airframe", "component": "Carbon-Epoxy Blended Delta Airframe & Spars", "mass_kg": 1.85, "pct": 52.86, "x_cg_m": 0.410, "color": "#d4af37"},
    {"id": "battery",  "component": "4S 5000mAh 35C Li-Po Battery Pack",          "mass_kg": 0.54, "pct": 15.43, "x_cg_m": 0.340, "color": "#e63946"},
    {"id": "payload",  "component": "EO/IR Gimbal & Multispectral Sensor Suite",   "mass_kg": 0.50, "pct": 14.29, "x_cg_m": 0.360, "color": "#38bdf8"},
    {"id": "motor",    "component": "Brushless Outrunner Motor, 40A ESC & APC 10x6", "mass_kg": 0.35, "pct": 10.00, "x_cg_m": 0.780, "color": "#10b981"},
    {"id": "avionics", "component": "Pixhawk Cube Autopilot, GPS, Pitot & Digital Servos", "mass_kg": 0.26, "pct": 7.43, "x_cg_m": 0.440, "color": "#a855f7"},
]


def get_air_density(alt_m: float) -> float:
    """ICAO Doc 7488 Troposphere Standard Atmosphere density model."""
    if alt_m <= 0:
        return 1.225
    return 1.225 * ((1.0 - 2.2558e-5 * alt_m) ** 4.2559)


def solve_flight_state(alpha_deg: float, alt_m: float = 0.0, sm_pct: float = 10.0, bank_deg: float = 0.0) -> Dict[str, Any]:
    """Solves exact longitudinal pitch equilibrium (Cm = 0) and steady flight performance."""
    rho = get_air_density(alt_m)
    alpha_rad = math.radians(alpha_deg)
    bank_rad = math.radians(bank_deg)

    # Pitch static stability derivative from Static Margin: SM = -Cma / CLa
    Cma = -(sm_pct / 100.0) * DERIVATIVES["CLa"]

    # 1. Pitching moment equilibrium: Cm0 + Cma*alpha + Cmde*de = 0
    de_rad = -(DERIVATIVES["Cm0"] + Cma * alpha_rad) / DERIVATIVES["Cmde"]
    de_deg = math.degrees(de_rad)

    # 2. Trimmed Lift Coefficient
    CL_untrimmed = DERIVATIVES["CL0"] + DERIVATIVES["CLa"] * alpha_rad
    CL_elevon = DERIVATIVES["CLde"] * de_rad
    CL = max(0.005, CL_untrimmed + CL_elevon)

    # 3. Banked turn load factor
    n_load = 1.0 / math.cos(bank_rad)
    W_eff = INERTIAL["W"] * n_load

    # 4. Equilibrium Velocity
    V = math.sqrt((2.0 * W_eff) / (rho * S_wing * CL))
    Mach = V / 340.29
    q_dyn = 0.5 * rho * (V ** 2)

    # 5. Parabolic Drag Polar
    CDi = k_induced * (CL ** 2)
    CD = DERIVATIVES["CD0"] + CDi

    # 6. Thrust & Aerodynamic Power Required
    Tr = q_dyn * S_wing * CD
    Pr = Tr * V
    LD = CL / CD
    endur_param = (CL ** 1.5) / CD

    # 7. Electric Propulsion & Battery Sizing
    P_shaft = Pr / PROPULSION["eta_prop"]
    P_batt = (P_shaft / PROPULSION["eta_elec"]) + PROPULSION["P_avionics"]
    I_batt = P_batt / PROPULSION["V_batt_nominal"]
    endur_hr = PROPULSION["E_usable_Wh"] / P_batt
    endur_min = endur_hr * 60.0
    range_km = (V * endur_hr * 3600.0) / 1000.0

    # 8. Turn Kinematics & Control Margins
    servo_margin_deg = 20.0 - abs(de_deg)
    turn_radius_m = (V ** 2) / (INERTIAL["g"] * math.tan(bank_rad)) if bank_deg > 0.5 else None
    turn_rate_deg_s = math.degrees((INERTIAL["g"] * math.tan(bank_rad)) / V) if bank_deg > 0.5 else 0.0

    return {
        "alphaDeg": round(alpha_deg, 3),
        "alphaRad": round(alpha_rad, 5),
        "altM": round(alt_m, 1),
        "rho": round(rho, 4),
        "smPct": round(sm_pct, 2),
        "bankDeg": round(bank_deg, 1),
        "nLoad": round(n_load, 3),
        "deRad": round(de_rad, 5),
        "deDeg": round(de_deg, 3),
        "CL_untrimmed": round(CL_untrimmed, 4),
        "CL_elevon": round(CL_elevon, 4),
        "CL": round(CL, 4),
        "CD0": DERIVATIVES["CD0"],
        "CDi": round(CDi, 5),
        "CD": round(CD, 5),
        "V": round(V, 3),
        "V_kmh": round(V * 3.6, 2),
        "Mach": round(Mach, 4),
        "qDyn": round(q_dyn, 2),
        "Tr": round(Tr, 3),
        "Pr": round(Pr, 3),
        "Pshaft": round(P_shaft, 2),
        "Pbatt": round(P_batt, 2),
        "Ibatt": round(I_batt, 2),
        "LD": round(LD, 3),
        "endurParam": round(endur_param, 3),
        "endurMin": round(endur_min, 2),
        "rangeKm": round(range_km, 2),
        "servoMarginDeg": round(servo_margin_deg, 2),
        "turnRadiusM": round(turn_radius_m, 2) if turn_radius_m is not None else None,
        "turnRateDegS": round(turn_rate_deg_s, 2),
    }


# =============================================================================
# REST API ENDPOINTS
# =============================================================================

@app.get("/api/health")
def api_health():
    return {
        "status": "OPERATIONAL",
        "engine": "CD-UAV 3.5 Computational Aerodynamics Backend",
        "project": "BSERC Def-Space Autumn Internship 2026 — Project 4",
        "mentor": "Dr. J. Sarathkumar Sebastin (VIT Bhopal University)",
    }


@app.get("/api/constants")
def api_constants():
    CL_opt_range = math.sqrt(DERIVATIVES["CD0"] / k_induced)
    LD_max_analytical = 1.0 / (2.0 * math.sqrt(k_induced * DERIVATIVES["CD0"]))
    CL_opt_endur = math.sqrt(3.0 * DERIVATIVES["CD0"] / k_induced)
    alpha_self_trim_deg = math.degrees(-DERIVATIVES["Cm0"] / DERIVATIVES["Cma_baseline"])

    return {
        "geometry": GEOMETRY,
        "derivatives": DERIVATIVES,
        "inertial": INERTIAL,
        "propulsion": PROPULSION,
        "massBreakdown": MASS_BREAKDOWN,
        "analyticalMilestones": {
            "alphaSelfTrimDeg": round(alpha_self_trim_deg, 3),
            "CL_maxRange": round(CL_opt_range, 4),
            "LD_maxAnalytical": round(LD_max_analytical, 3),
            "Tr_minAnalytical": round(INERTIAL["W"] / LD_max_analytical, 3),
            "CL_minPower": round(CL_opt_endur, 4),
        },
    }


@app.get("/api/solve")
def api_solve(
    alpha: float = Query(10.0, ge=0.0, le=15.0),
    alt: float = Query(0.0, ge=0.0, le=5000.0),
    sm: float = Query(10.0, ge=2.0, le=25.0),
    bank: float = Query(0.0, ge=0.0, le=75.0),
):
    return solve_flight_state(alpha, alt, sm, bank)


@app.get("/api/sweep")
def api_sweep(
    alt: float = Query(0.0, ge=0.0, le=5000.0),
    sm: float = Query(10.0, ge=2.0, le=25.0),
):
    states = []
    for i in range(25):
        alpha = i * 0.5
        states.append(solve_flight_state(alpha, alt, sm, 0.0))

    alpha_self = math.degrees(-DERIVATIVES["Cm0"] / (-(sm / 100.0) * DERIVATIVES["CLa"]))
    self_trim_state = solve_flight_state(alpha_self, alt, sm, 0.0)
    max_range_state = max(states, key=lambda s: s["LD"])
    max_endur_state = max(states, key=lambda s: s["endurParam"])

    return {
        "altM": alt,
        "smPct": sm,
        "rho": round(get_air_density(alt), 4),
        "count": len(states),
        "states": states,
        "milestones": {
            "selfTrim": self_trim_state,
            "maxRange": max_range_state,
            "maxEndurance": max_endur_state,
        },
    }


@app.get("/api/multi-altitude")
def api_multi_altitude():
    altitudes = [0, 1000, 2000]
    series = {}
    for h in altitudes:
        states = [solve_flight_state(i * 0.5, h, 10.0, 0.0) for i in range(25)]
        series[str(h)] = {
            "altM": h,
            "rho": round(get_air_density(h), 4),
            "states": states,
            "selfTrim": solve_flight_state(1.9616, h, 10.0, 0.0),
            "maxRange": solve_flight_state(10.0, h, 10.0, 0.0),
            "maxEndurance": solve_flight_state(12.0, h, 10.0, 0.0),
        }
    return series


@app.get("/api/vn-envelope")
def api_vn_envelope(alt: float = Query(0.0, ge=0.0, le=5000.0)):
    rho = get_air_density(alt)
    CL_max_pos = 1.20
    CL_max_neg = 0.80
    n_max = 3.80
    n_min = -1.52
    n_ult = 5.70
    W = INERTIAL["W"]

    Vs_1g = math.sqrt((2.0 * W) / (rho * S_wing * CL_max_pos))
    V_corner = Vs_1g * math.sqrt(n_max)
    V_corner_neg = math.sqrt((2.0 * W * abs(n_min)) / (rho * S_wing * CL_max_neg))
    V_max_level = 28.0
    V_ne = 45.0

    pos_curve = []
    v = 0.0
    while v <= V_ne + 0.01:
        n_stall = (0.5 * rho * (v ** 2) * S_wing * CL_max_pos) / W
        pos_curve.append({"v": round(v, 2), "n": round(min(n_stall, n_max), 3)})
        v += 0.5

    neg_curve = []
    v = 0.0
    while v <= V_ne + 0.01:
        n_stall_neg = -(0.5 * rho * (v ** 2) * S_wing * CL_max_neg) / W
        neg_curve.append({"v": round(v, 2), "n": round(max(n_stall_neg, n_min), 3)})
        v += 0.5

    return {
        "altM": alt,
        "rho": round(rho, 4),
        "Vs1g": round(Vs_1g, 2),
        "Vcorner": round(V_corner, 2),
        "VcornerNeg": round(V_corner_neg, 2),
        "VmaxLevel": V_max_level,
        "Vne": V_ne,
        "nMax": n_max,
        "nMin": n_min,
        "nUlt": n_ult,
        "posCurve": pos_curve,
        "negCurve": neg_curve,
    }


@app.get("/api/matlab-code")
def api_matlab_code():
    m_path = BASE_DIR / "Cropped_Delta_UAV_Analysis.m"
    if m_path.exists():
        return {"filename": "Cropped_Delta_UAV_Analysis.m", "code": m_path.read_text(encoding="utf-8", errors="replace")}
    return {"filename": "Cropped_Delta_UAV_Analysis.m", "code": "% MATLAB file not found"}


# =============================================================================
# PAGE ROUTES & STATIC MOUNTING
# =============================================================================

@app.get("/")
def serve_root():
    return FileResponse(BASE_DIR / "index.html")


@app.get("/simulator")
def serve_simulator():
    return RedirectResponse(url="/app/simulator.html")


@app.get("/performance")
def serve_performance():
    return RedirectResponse(url="/app/performance.html")


@app.get("/propulsion")
def serve_propulsion():
    return RedirectResponse(url="/app/propulsion.html")


@app.get("/envelope")
def serve_envelope():
    return RedirectResponse(url="/app/envelope.html")


@app.get("/docs-hub")
def serve_docs_hub():
    return RedirectResponse(url="/app/docs.html")


# Mount Plots, app, and root deliverables
if PLOTS_DIR.exists():
    app.mount("/Plots", StaticFiles(directory=str(PLOTS_DIR)), name="Plots")
if APP_DIR.exists():
    app.mount("/app", StaticFiles(directory=str(APP_DIR), html=True), name="app")
app.mount("/files", StaticFiles(directory=str(BASE_DIR)), name="files")


if __name__ == "__main__":
    import uvicorn
    print("\n" + "=" * 72)
    print("  CD-UAV 3.5 | BSERC DEF-SPACE AEROSPACE COMMAND-CENTRE SERVER")
    print("  URL: http://localhost:8000")
    print("=" * 72 + "\n")
    uvicorn.run(app, host="127.0.0.1", port=8000)
