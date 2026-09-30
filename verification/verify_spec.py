"""Independent verification of the Leviathan-class Build Specification, Rev H.

Recomputes every figure the document derives from others, runs a volumetric
clash check of the Annex B/C allocation against a correctly offset plating
envelope, and tests the surface layout for fittings that claim the same
ground. Writes verification/results.json and verification/results.md.

    python3 verification/verify_spec.py
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import shapely
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "model"))
import lev_geom as G  # noqa: E402

PI = math.pi
D2R = G.D2R
RESULTS: list[dict] = []
FINDINGS: list[dict] = []


def check(ref, claim, stated, computed, tol, note=""):
    ok = abs(computed - stated) <= tol
    RESULTS.append(dict(ref=ref, claim=claim, stated=stated, computed=round(computed, 4),
                        tol=tol, status="PASS" if ok else "FLAG", note=note))
    return ok


def check_true(ref, claim, ok, note=""):
    RESULTS.append(dict(ref=ref, claim=claim, stated="yes", computed="yes" if ok else "no", tol="",
                        status="PASS" if ok else "FLAG", note=note))
    return ok


def finding(ref, severity, title, detail, fix=""):
    FINDINGS.append(dict(ref=ref, severity=severity, title=title, detail=detail, fix=fix))


# =============================================================== geometry
def section_polygon(t, **kw):
    return Polygon(G.hull_profile(t, **kw)).buffer(0)


def revolve_moment(poly):
    """Integral of r dA over a section polygon (for Pappus)."""
    if poly.is_empty:
        return 0.0
    return poly.area * poly.centroid.x


def hull_volume(collar_well=True, z_max=None, with_fairing=True, knuckle="ellipse", n_aft=900):
    """Hull solid volume (Praetorian cavity and collar well included in the
    section; Lance cavities subtracted separately), optionally clipped to
    z < z_max."""
    def poly_at(t):
        p = section_polygon(t if with_fairing else 0.0, collar_well=collar_well)
        if knuckle == "cone":
            # straight chord from (1050, -470) to (1130, -630) in place of the ellipse:
            # the lens between chord and arc is cavity for the ellipse, hull for the cone
            lens = Polygon([G.knuckle_rz(PI / 2 * i / 200) for i in range(201)])
            p = p.union(lens).buffer(0)
        if z_max is not None:
            p = p.intersection(Polygon([(-1, -2000), (3000, -2000), (3000, z_max), (-1, z_max)]))
        return p
    fwd = revolve_moment(poly_at(0.0)) * (270.0 * D2R)
    aft = 0.0
    dt = 90.0 / n_aft
    for i in range(n_aft):
        t = 135.0 + (i + 0.5) * dt
        aft += revolve_moment(poly_at(t)) * dt * D2R
    return fwd + aft


def lance_cavity_volume(radius_bowl, radius_well, z_max=None, n=400):
    """Void a Lance leaves in the hull: upper hemisphere + well cylinder down to the crown."""
    c = G.LANCES[0]
    zc = c.zc
    # hemisphere above the equator (entirely below the dorsal surface)
    if z_max is None or z_max >= zc + radius_bowl:
        hemi = 2.0 / 3.0 * PI * radius_bowl ** 3
    elif z_max <= zc:
        hemi = 0.0
    else:
        h = z_max - zc
        hemi = PI * (radius_bowl ** 2 * h - h ** 3 / 3.0)
    # cylinder from the crown up to the equator (clipped)
    xs = np.linspace(-radius_well, radius_well, n)
    X, Y = np.meshgrid(xs, xs)
    inside = X ** 2 + Y ** 2 <= radius_well ** 2
    r = np.hypot(c.r + X, Y)
    zb = G.CROWN_TOP - 50.0 * (r / G.R_CROWN) ** 2
    top = zc if z_max is None else min(zc, z_max)
    hgt = np.clip(top - zb, 0, None)
    cyl = float(np.sum(hgt[inside]) * (xs[1] - xs[0]) ** 2)
    return hemi + cyl


def sphere_segment_below(R, zc, z):
    """Volume of a sphere (centre zc) below the plane z."""
    h = min(max(z - (zc - R), 0.0), 2 * R)
    return PI * h * h * (3 * R - h) / 3.0


def cavity_volume(z_top=None):
    """Open dish between the rim plane and the ventral moulded surface, r < 1,130."""
    rs = np.linspace(0, G.R_DISH, 20001)
    zb = np.array([G.bottom_z(r) for r in rs])
    return float(np.trapz(2 * PI * rs * (zb - G.Z_VENTRAL), rs))


def well_annuli_volume():
    """Free volume inside the nine wells with cores seated (well minus core, crown to
    equator), plus the 2 m bowl clearance."""
    tot = 0.0
    # Lances
    c = G.LANCES[0]
    n = 500
    xs = np.linspace(-c.well_r, c.well_r, n)
    X, Y = np.meshgrid(xs, xs)
    rho = np.hypot(X, Y)
    inside = rho <= c.well_r
    r = np.hypot(c.r + X, Y)
    zb = G.CROWN_TOP - 50.0 * (r / G.R_CROWN) ** 2
    # core lower surface under each column
    core_bot = np.where(rho < c.rc, c.zc - np.sqrt(np.clip(c.rc ** 2 - rho ** 2, 0, None)), c.zc)
    free = np.clip(c.zc - zb, 0, None) - np.clip(np.minimum(c.zc, np.maximum(zb, core_bot)) * 0 + (c.zc - np.maximum(core_bot, zb)), 0, None) * (rho < c.rc)
    lance = float(np.sum(free[inside]) * (xs[1] - xs[0]) ** 2)
    lance += 2.0 / 3.0 * PI * (c.bowl_inner ** 3 - c.rc ** 3)
    tot += 8 * lance
    # Praetorian (bottom flush with the rim plane, below the crown)
    p = G.PRAETORIAN
    xs = np.linspace(-p.well_r, p.well_r, n)
    X, Y = np.meshgrid(xs, xs)
    rho = np.hypot(X, Y)
    inside = rho <= p.well_r
    zb = G.CROWN_TOP - 50.0 * (rho / G.R_CROWN) ** 2
    core_bot = np.where(rho < p.rc, p.zc - np.sqrt(np.clip(p.rc ** 2 - rho ** 2, 0, None)), p.zc)
    free = np.clip(p.zc - zb, 0, None) - np.clip(p.zc - np.maximum(core_bot, zb), 0, None) * (rho < p.rc)
    tot += float(np.sum(free[inside]) * (xs[1] - xs[0]) ** 2) + 2.0 / 3.0 * PI * (p.bowl_inner ** 3 - p.rc ** 3)
    return tot


def core_volume_below(z):
    return sum(sphere_segment_below(c.rc, c.zc, z) for c in G.CORES)


def main():
    # ------------------------------------------------------ Section 1 / B.2
    check("§1", "Overall diameter, m", 3356.9, 2 * G.R_RIM, 0.05)
    check("§1", "Overall length over drive fairing, m", 3536.9, G.R_RIM + G.r_out(180), 0.05)
    check("§1", "Overall height, collar extended, m", 730, 100 - G.Z_VENTRAL, 0.1)
    check("§1", "Overall height, collar retracted, m", 710, G.DOME_TOP - G.Z_VENTRAL, 0.1)
    check("§1", "Hull thickness at the hub, m", 475, G.cap_z(0) - G.CROWN_TOP, 0.5)
    check("§1/§2.3", "Outer band thickness at the dish rim, m", 660, G.cap_z(1130) - G.Z_VENTRAL, 1.0)
    check("§2.1/B.2", "Dorsal rise, rim to axis, m", 55, G.cap_z(0), 0.05)
    check("B.2", "Dorsal surface slope at the rim, deg", 3.75, math.degrees(math.asin(G.R_RIM / G.R_CAP)), 0.01)
    cap_vol = PI * 55 * (3 * G.R_RIM ** 2 + 55 ** 2) / 6
    check("§2.1", "Volume added by the dorsal curve, M m³", 244, cap_vol / 1e6, 1.0)
    check("§2.1", "Rise of a 20 m skin panel, mm", 2, 20 ** 2 / (8 * G.R_CAP) * 1000, 0.1)
    check("A.8", "Dorsal rise-to-span ratio (1:x)", 61, 2 * G.R_RIM / 55, 0.2)
    check("B.2", "Fixed cap meets dorsal curve at r 250 (dome z − cap z), m", 0, G.dome_z(250) - G.cap_z(250), 0.1)
    dome_ang = math.degrees(math.atan(-G.dome_slope(250)) - math.atan(-G.cap_slope(250)))
    check("B.2", "Fixed cap / dorsal curve knuckle angle at r 250 (not tangent), deg", 11.4, dome_ang, 0.2,
          "B.2 says only 'not tangent'; stated value here is the computed one, recorded for the model")
    check("B.2", "Collar well depth below local surface at the inner wall, m", 64, G.cap_z(250) - G.Z_WELL_FLOOR, 0.5)
    check("§2.2", "Collar well clear depth at r 250 ('65 m'), m", 65, G.cap_z(250) - G.Z_WELL_FLOOR, 0.5,
          "§2.2 says 65, B.2 says 64; the surface at r 250 is +53.8 so the depth to z −10 is 63.8")
    p = G.PRAETORIAN
    check("§2.2", "Central core radius at the datum z = 0, m", 204, math.sqrt(p.rc ** 2 - (0 - p.zc) ** 2), 0.5)
    check("§2.2", "Core surface at r 150 (well floor limit), z", 32, p.zc + math.sqrt(p.rc ** 2 - 150 ** 2), 0.5)
    check("§2.2", "Core reaches r 250 at z", -40, p.zc + math.sqrt(p.rc ** 2 - 250 ** 2), 0.5)
    bowl_at_250 = p.zc + math.sqrt(p.bowl_moulded ** 2 - 250 ** 2)
    finding("§2.2, B.2", "minor", "The §2.2 well-depth argument uses the bare core radius, not the bowl",
            f"Praetorian's bowl moulded surface (r_c + 14 = 362 m) reaches r = 250 at z = {bowl_at_250:.1f}, "
            f"not −40. That leaves {G.Z_WELL_FLOOR - bowl_at_250:.1f} m of structure under the collar well floor at z −10. "
            "The floor still clears the bowl, so the design holds. The argument just overstates the margin by about 20 m.",
            "State the margin against the bowl shell (≈10 m under the floor), or drop the −40 figure.")

    # ventral / knuckle
    check("B.2", "Ventral crown at r 1,050, z", -470, G.crown_z(1050), 0.01)
    check("§3.2/B.2", "Lance south pole stands proud of the crown at its station, m", 50,
          G.crown_z(800) - (G.LANCES[0].zc - 210), 1.5)
    check("§3.2", "Praetorian stands proud of the crown, m", 210, G.crown_z(0) - (p.zc - p.rc), 0.5)
    kink = math.degrees(math.atan(-G.crown_slope(1050)))
    finding("B.2", "minor", "Crown and knuckle are not tangent at r 1,050",
            f"B.2 gives the knuckle a horizontal tangent where it meets the crown. The crown paraboloid slopes "
            f"{kink:.2f}° there (dz/dr = {G.crown_slope(1050):.4f}). So the ventral face has a {kink:.1f}° crease at r 1,050, "
            "which shows in reflections and in the cavity floodlighting.",
            "Either accept the crease (the model keeps it, as specified) or tilt the ellipse to match the crown slope.")

    # §2.3 masking angles
    check("§2.3/A.8", "Hub crown masked within, deg", 10.5, math.degrees(math.atan(210 / 1130)), 0.1)
    h = 130
    check("A.8", "Lance south pole masked across the dish, deg", 3.9, math.degrees(math.atan(h / (800 + 1130))), 0.1)
    check("A.8", "Lance south pole masked sideways, deg", 9.3, math.degrees(math.atan(h / math.sqrt(1130 ** 2 - 800 ** 2))), 0.1)
    check("A.8", "Lance south pole masked outward, deg", 21.5, math.degrees(math.atan(h / (1130 - 800))), 0.1)
    check("§17.2/A.8", "Limb depression at 300 km over an Earth-sized world, deg", 17,
          math.degrees(math.acos(6371 / (6371 + 300))), 0.3)
    check("§2.3", "Cavity depth, m", 210, G.CROWN_TOP - G.Z_VENTRAL, 0)
    check("§2.3", "Cavity opening, m", 2260, 2 * G.R_DISH, 0)

    # §2.4 fairing
    lune = 0.0
    n = 20000
    for i in range(n):
        t = 135 + 90 * (i + 0.5) / n
        lune += 0.5 * (G.r_out(t) ** 2 - G.R_RIM ** 2) * (90 / n) * D2R
    check("§2.4", "Fairing lune plan area, M m²", 0.315, lune / 1e6, 0.001)
    check("§2.4", "Fairing added volume, M m³", 198, lune * 630 / 1e6, 1)
    chord = 2 * G.R_RIM * math.sin(45 * D2R)
    check("§2.4/§17.4", "Emitter chord across the aft 90°, m", 2374, chord, 1)
    arc = G.fairing_arc_length()
    check("B.8", "Drive emitter segment face width (arc/16), m", 169, arc / 16, 3,
          f"The fairing's aft boundary is {arc:.0f} m long, so each of 16 segments is {arc / 16:.0f} m. "
          f"The rim-circle arc would give {G.R_RIM * PI / 2 / 16:.0f} m and the chord {chord / 16:.0f} m.")

    # §1 / §20 volume and density
    lance_void = lance_cavity_volume(210 + 2, 210 + 6)
    lance_void_moulded = lance_cavity_volume(210 + 14, 210 + 14)
    v_full = hull_volume() - 8 * lance_void
    v_nofair = hull_volume(with_fairing=False) - 8 * lance_void
    v_cone = hull_volume(with_fairing=False, knuckle="cone") - 8 * lance_void
    v_cone_full = v_cone + (v_full - v_nofair)
    check("§20", "Enclosed hull volume incl. fairing (A.8 method, straight-cone knuckle), B m³", 4.83, v_cone_full / 1e9, 0.02,
          f"Voids at clearance radius (bowl r_c+2, well r_c+6), collar well open. With the Annex B ellipse knuckle the volume "
          f"is {v_full / 1e9:.3f} B m³. With moulded-radius voids (r_c+14) it is {(hull_volume() - 8 * lance_void_moulded) / 1e9:.3f}.")
    check("A.8", "Enclosed hull volume excl. fairing (straight cone), B m³", 4.63, v_cone / 1e9, 0.02,
          f"Ellipse knuckle: {v_nofair / 1e9:.3f}")
    check("A.8", "Elliptical knuckle vs straight cone, M m³", 25, (v_cone - v_nofair) / 1e6, 2)
    check("§20", "Hull density 4.47 Bt / enclosed volume, t/m³", 0.93, 4.47e9 / v_cone_full, 0.01,
          f"{4.47e9 / v_full:.3f} on the true (ellipse) volume")
    core_vol = sum(4 / 3 * PI * c.rc ** 3 for c in G.CORES)
    check("§20", "Nine cores, volume, M m³", 487, core_vol / 1e6, 1)
    check("§20", "Core density, t/m³", 0.62, 300e6 / core_vol, 0.005)
    check("§20", "Plating grade raise adds (8 m × 8.86 M m² × 8 t/m³), Bt", 0.57, 8 * 8.86e6 * 8 / 1e9, 0.01)
    dorsal_area = 2 * PI * G.R_CAP * 55
    check("§5.5/§16", "Dorsal face area, M m²", 8.86, dorsal_area / 1e6, 0.01)
    check("§16", "Dorsal plating mass, Bt", 0.71, dorsal_area * 10 * 8 / 1e9, 0.01)
    check("A.7", "Fairing crown plating mass, Bt", 0.03, lune * 10 * 8 / 1e9, 0.01)
    check("§17.1", "Sublight acceleration, cores deployed, G", 2130, 2000 * 4.8 / 4.5, 5)
    check("§1", "Mated dry mass, Bt", 4.8, 4.5 + 0.3, 0.001)

    # §2.7 emplacement and flotation
    check("§2.7", "Hull density quoted in §2.7, t/m³", 0.96, 4.47e9 / v_full, 0.01,
          "§2.7 uses 0.96, which is the pre-fairing figure. §1, §20 and A.8 give 0.93.")
    footprint = PI * G.R_RIM ** 2
    check("§2.7", "Footprint, M m²", 8.85, footprint / 1e6, 0.01, f"The fairing adds {lune / 1e6:.3f} M m², giving {(footprint + lune) / 1e6:.2f}.")
    check("§2.7", "Mean contact pressure at 1 g, MPa", 5.3, 4.8e12 * 9.80665 / footprint / 1e6, 0.05)
    check("§2.7", "…at Martian gravity, MPa", 2.0, 4.8e12 * 3.721 / footprint / 1e6, 0.05)
    check("§2.7", "…at lunar gravity, MPa", 0.9, 4.8e12 * 1.62 / footprint / 1e6, 0.05)
    check("§2.7", "Rock overburden at 630 m (2.7 t/m³), MPa", 16.7, 2700 * 9.80665 * 630 / 1e6, 0.1)
    check("§2.7", "'Carries less afterwards … about 7.4 MPa'", 7.4, 4.8e12 * 9.80665 / footprint / 1e6, 0.3,
          "Cannot be reproduced. Mated weight over the footprint is 5.3 MPa (the §2.7 figure one line later); "
          "a 630 m column at 0.96 t/m³ is 5.9 MPa. 7.4 MPa implies a 1.2 t/m³ column.")
    # displacement
    below0_hull = hull_volume(z_max=0.0) - 8 * lance_void
    below0_hull_nofair = hull_volume(z_max=0.0, with_fairing=False) - 8 * lance_void
    disp_flood = below0_hull + core_volume_below(0.0)
    disp_flood_nofair = below0_hull_nofair + core_volume_below(0.0)
    cav = cavity_volume()
    annuli = well_annuli_volume()
    check("A.8", "Displacement, cavity flooded, as the hull now stands (with fairing), B m³", 4.87, disp_flood / 1e9, 0.03,
          f"Reproduces only without the fairing ({disp_flood_nofair / 1e9:.3f})")
    check("A.8", "Displacement, cavity flooded, pre-fairing hull, B m³", 4.87, disp_flood_nofair / 1e9, 0.03)
    check("§2.7", "Volume closed by the hardened screen (cavity + well annuli), M m³", 740, (cav + annuli) / 1e6, 25,
          f"Cavity {cav / 1e6:.0f} M m³ + well annuli and bowl clearance {annuli / 1e6:.0f} M m³. The spec's own 5.62 − 4.87 is 750.")
    disp_closed_nofair = disp_flood_nofair + cav + annuli
    disp_closed = disp_flood + cav + annuli
    check("A.8", "Displacement, cavity closed, B m³", 5.62, disp_closed_nofair / 1e9, 0.03,
          f"Without the fairing. With the fairing: {disp_closed / 1e9:.3f}.")
    check("§2.7", "Mean immersed density, flooded, t/m³", 0.99, 4.8e9 / disp_flood_nofair, 0.01,
          f"With the fairing: {4.8e9 / disp_flood:.3f}")
    check("§2.7", "Mean immersed density, screen hardened, t/m³", 0.85, 4.8e9 / disp_closed_nofair, 0.01,
          f"With the fairing: {4.8e9 / disp_closed:.3f}")

    def draft(rho, closed, fairing):
        wp = footprint + (lune if fairing else 0.0)
        base = (disp_closed if closed else disp_flood) if fairing else (disp_closed_nofair if closed else disp_flood_nofair)
        need = 4.8e9 / rho
        # the section above the waterline is a prism of the waterplane down to the cavity crown
        return 630 - (base - need) / wp

    drafts = {}
    for name, rho, dfl, dsc in [("Fresh water", 1.000, 622, 538), ("Seawater", 1.025, 609, 524), ("Dense brine", 1.240, 517, 433)]:
        a, b = draft(rho, False, False), draft(rho, True, False)
        af, bf = draft(rho, False, True), draft(rho, True, True)
        drafts[name] = dict(flooded=a, closed=b, flooded_fairing=af, closed_fairing=bf)
        check("§2.7", f"Draft, {name}, cavity flooded (pre-fairing method), m", dfl, a, 2.5, f"Including the fairing: {af:.0f} m")
        check("§2.7", f"Draft, {name}, screen hardened (pre-fairing method), m", dsc, b, 2.5, f"Including the fairing: {bf:.0f} m")
    check("§2.7", "Draft recovered by hardening the screen, m", 84, (cav + annuli) / footprint, 3)
    check("§2.7", "Underside pressure on the screen (fresh water, 538 m), 10¹³ N", 2.1,
          1000 * 9.80665 * 538 * PI * G.R_DISH ** 2 / 1e13, 0.05)
    check("§2.7", "…as a share of weight, %", 45, 1000 * 9.80665 * 538 * PI * G.R_DISH ** 2 / (4.8e12 * 9.80665) * 100, 1)
    check("§6.1", "Cavity air at 1 atm, kt", 900, 740e6 * 1.225 / 1e6 * 1000 / 1000, 15)
    check("§2.7", "Pendulum arm for a 60 s period, m", 894, 9.80665 * (60 / (2 * PI)) ** 2, 1)
    bm = PI * G.R_RIM ** 4 / 4 / 4.8e9
    check("§2.7", "Metacentric radius BM (order of a km), m", 1000, bm, 400)
    check("§2.7", "Water/dorsal areal density ratio at 1 km", 12, 1000 / (10 * 8), 0.6)
    # core flotation
    def float_depth(R, frac):
        lo, hi = 0.0, 2 * R
        for _ in range(80):
            mid = (lo + hi) / 2
            if PI * mid * mid * (3 * R - mid) / 3 < frac * 4 / 3 * PI * R ** 3:
                lo = mid
            else:
                hi = mid
        return lo
    dl = float_depth(210, 0.62)
    check("§2.7", "Lance draft in fresh water, m", 244, dl, 1.5)
    check("§2.7", "Lance waterline latitude, deg N", 9, math.degrees(math.asin((dl - 210) / 210)), 0.5)
    check("§2.7", "Praetorian draft, m", 404, float_depth(348, 0.62), 1.5)
    finding("§2.7", "significant", "The Lance flotation sentence contradicts itself",
            "With the waterline at 9° N (pole down, 244 m draft), the whole southern band (30°–50° S) is 110–160 m under water: "
            "gear bays, ramps and southern batteries. The sentence says the band stays dry, then puts 'the gear' under.",
            "Read: '…which leaves every hangar mouth dry and puts the southern band, the gear, the entry face and the shield face under.'")
    finding("§2.7, A.8", "significant", "Flotation figures leave out the drive fairing",
            f"4.87 and 5.62 B m³ are reproduced only by excluding the fairing's 198 M m³ (and its 0.315 M m² of waterplane). "
            f"With the fairing the displacements are {disp_flood / 1e9:.2f} and {disp_closed / 1e9:.2f} B m³. Immersed densities become "
            f"{4.8e9 / disp_flood:.3f} and {4.8e9 / disp_closed:.3f} t/m³. Drafts (fresh / sea / brine): flooded "
            + " / ".join(f"{drafts[k]['flooded_fairing']:.0f}" for k in drafts) + " m, hardened "
            + " / ".join(f"{drafts[k]['closed_fairing']:.0f}" for k in drafts) + " m. "
            "§2.7 also uses 0.96 t/m³ for the hull; since the fairing was added the figure is 0.93.",
            "Rebase §2.7 and A.8 on the 4.83 B m³ hull.")

    # §3 cores
    lc = G.LANCES[0]
    check("§9.1", "Lance lower hangar deck z at 15° N", -236, lc.zc + lc.rc * math.sin(15 * D2R), 0.5)
    check("§9.1", "Lance upper hangar deck z at 35° N", -170, lc.zc + lc.rc * math.sin(35 * D2R), 0.5)
    check("§9.1", "Praetorian lower hangar deck z at 7.6° N", -236, p.zc + p.rc * math.sin(7.6 * D2R), 0.5)
    check("§9.1", "Praetorian upper hangar deck z at 18.8° N", -170, p.zc + p.rc * math.sin(18.8 * D2R), 0.5)
    check("§9.1", "Gallery chord through a Lance at 15° N, m", 406, 2 * lc.rc * math.cos(15 * D2R), 1)
    check("§9.1", "Gallery chord through a Lance at 35° N, m", 344, 2 * lc.rc * math.cos(35 * D2R), 1)
    check("§3.3", "Lance gear leg at 30° S (pole clearance), m", 105, lc.rc * (1 - math.sin(30 * D2R)), 1)
    check("§3.3", "Lance gear leg at 50° S, m", 50, lc.rc * (1 - math.sin(50 * D2R)), 1.5)
    finding("§3.3, D.8", "minor", "Gear-leg lengths hold for a Lance only",
            f"105 m at 30° S and 50 m at 50° S stand a 210 m sphere on its pole. Praetorian (348 m) needs "
            f"{p.rc * (1 - math.sin(30 * D2R)):.0f} m and {p.rc * (1 - math.sin(50 * D2R)):.0f} m. D.8 applies '50 to 105 m' to every core.",
            "Give Praetorian's legs separately (≈174 m and ≈81 m).")
    wm = math.degrees(math.asin((G.crown_z(800) - lc.zc) / lc.rc))
    check("§3.3", "Lance well mouth latitude, deg S", 49.6, -wm, 0.5,
          "Crown at the station centre is z −449.0, giving 49.2° S. 49.6° corresponds to z −449.8.")
    # Praetorian grid anchoring
    off_lg = (G.DECK0 - p.zc) % 6
    finding("B.6, B.3", "significant", "Praetorian's own deck grid cannot carry the galleries",
            f"B.6 anchors each core's internal grid on its own equator. That works for a Lance: the galleries sit 54 m and "
            f"120 m above its equator (z −290), 9 and 20 levels. Praetorian's equator is at z −282, so the galleries sit 46 m and "
            f"112 m above it. Neither is a whole number of 6 m levels (remainder {off_lg:.0f} m), so a gallery can't run into "
            "Praetorian without a step. The z +28 command deck is on the hull grid (n = 44), not on an equator-anchored one.",
            "Anchor Praetorian's grid on the hull grid (a level at z −284, 2 m under its equator). The centre can't move, "
            "because the south cap has to stay flush at z −630.")
    finding("B.9, §15", "significant", "The flag command deck is not below the flag plot",
            "B.9 puts Praetorian's command deck at z +28, 'one level below the plot'. But the plot spans n = 42–46 "
            "(z +16 to +40), and z +28 is n = 44, in the middle of that span. One level below the plot would be z +10 (n = 41). "
            f"z +28 is also {math.degrees(math.asin((28 - p.zc) / p.rc)):.1f}° N on the core, inside the northern cap "
            "that B.6 gives to the drive cluster and its plant (above 60° N).",
            "Move the command deck to z +10 (n = 41, 57.5° N), just under the drive cap, or move the plot up.")
    # keys vs hangar mouths on Praetorian
    finding("§3.1, B.6", "significant", "Praetorian's keys and hangar mouths share the same eight longitudes",
            "B.6 clocks all eight keys (50 m wide, 20 m proud) to the web longitudes. It also puts Praetorian's sixteen hangar "
            "mouths (60 × 20 m, at 7.6° N and 18.8° N) 'on the spur longitudes', which are the same longitudes. Each key channel "
            "would run through two mouths, and each spur would enter the bowl through the key. On a Lance the two are 22.5° apart, "
            "so the conflict is Praetorian's alone.",
            "Clock Praetorian's keys to 22.5° + 45n, or split each key above 22° N. The model uses the split key.")
    # thruster wells vs mouths
    half_mouth = math.degrees(30 / (p.rc * math.cos(7.6 * D2R)))
    finding("§3.3, B.6", "significant", "Praetorian's thruster wells can't be evenly spaced",
            f"There are 48 wells a belt at ±8°. Evenly spaced on 7.5°, the wells nearest each spur longitude sit 3.75° off it, "
            f"inside the 60 m mouth at 7.6° N, which spans ±{half_mouth:.2f}° of longitude. So even spacing would place wells "
            "inside mouths, contrary to 'never share a longitude with one'.",
            "Group the wells six to each 35° gap between mouths (≈5.8° spacing). The model does this.")

    # §9 / B.7 openings
    check("§9.2/B.7", "Knuckle z at the mouth centre r 1,118", -545, G.knuckle_z(1118), 1)
    check("B.7", "Knuckle z at the mouth sill r 1,125", -575, G.knuckle_z(1125), 1)
    check("B.7", "Knuckle z at the mouth head r 1,106", -515, G.knuckle_z(1106), 1)
    check("§9.2", "Two-thirds radius, m", 1118, G.R_RIM * 2 / 3, 1.5)
    check("B.7", "Mouth pair ± angle for 240 m centres at r 1,118, deg", 6.15, math.degrees(120 / 1118), 0.01)
    check("B.3/B.7", "Mouth sill on the deck grid (n = −57), z", -578, -575, 0.5,
          "B.3 puts the mouth sill at n = −57 (z −578). B.7 puts it at z −575, 3 m (half a level) off the grid.")
    check("B.7", "Upper boom root on the knuckle, z at r 1,108", -520, G.knuckle_z(1108), 1)
    # lower root: horizontal distance to curve
    zr = -596
    s = math.sqrt(1 - ((zr - G.Z_VENTRAL) / G.KN_B) ** 2)
    check("B.7", "Lower boom root on the knuckle, r at z −596", 1128, G.R_CROWN + G.KN_A * s, 1)
    check("B.8", "Screen emitter on the knuckle, z at r 1,097", -500, G.knuckle_z(1097), 1)
    # segment budget along the knuckle
    circ = 2 * PI * G.MOUTH_R
    used = 16 * G.MOUTH_W + 16 * G.BOOM_RECESS[0] + 8 * G.SCREEN_EMITTER[2]
    check_true("B.13", "16 mouths, 16 boom recesses and 8 screen emitters fit the knuckle circumference", circ - used > 0,
               f"They use {used:.0f} m of {circ:.0f} m at r 1,118, leaving {circ - used:.0f} m.")

    # §8 / B.8 batteries
    circ_rim = 2 * PI * G.R_RIM
    check("§8", "Rim circumference, m", 10546, circ_rim, 1)
    check("§8", "Forward 270° of rim, m", 7910, circ_rim * 0.75, 1)
    check("B.8", "Heavy mounts per row (whole 40 m pitches in 270°)", 197, math.floor(circ_rim * 0.75 / 40), 0)
    check("B.8", "Heavy mounts, three rows", 591, 3 * 197, 0)
    check("B.8", "Medium mounts, one-third band r 559.5 (30 m pitch)", 117, G.ring_count(559.5, 30), 0)
    check("B.8", "Medium mounts, dorsal two-thirds r 1,119", 234, G.ring_count(1119, 30), 0)
    check("B.8", "Medium mounts, ventral outer band r 1,331", 278, G.ring_count(1331, 30), 0)
    check("C.12", "Medium ventral outer count in C.12", 261, G.ring_count(1331, 30), 0,
          "C.12 gives 261. B.8 gives 278, which is what 30 m pitch at r 1,331 yields.")
    check("§2.1", "Collar horizon: dorsal slope at rim, deg ('about four')", 4, math.degrees(math.asin(G.R_RIM / G.R_CAP)), 0.5)

    # r = 1,119 claims
    c2 = 2 * PI * 1119
    c1 = 2 * PI * 559.5
    finding("§6, §7, B.8", "significant", "Three fittings claim the dorsal r 1,119 circle",
            f"The dorsal two-thirds medium band puts 234 mounts on a 30 m pitch at r 1,119, which uses {234 * 30:.0f} m of the "
            f"{c2:.0f} m circumference. Section 6 puts the 8 dorsal shield rings (60 m apertures) at two-thirds radius, and §7 puts "
            f"16 dorsal sensors (20 × 8 m) there too. That circle has {c2 - 234 * 30:.0f} m to spare. The one-third circle has the "
            f"same problem: 117 mounts use {117 * 30:.0f} of {c1:.0f} m, and 16 more sensors are specified on it. "
            "The HTML datum model draws all of these on the same circles.",
            "Offset the emitters and sensors radially clear of the mount swept envelopes. The model puts the emitters at "
            "r 1,175 (inside their generator hall at r 1,019–1,219), the two-thirds sensors at r 1,060 and the one-third "
            "sensors at r 505.")
    finding("B.8, D.2", "minor", "Rim shield emitter height is not given, and three rim rings have no rim",
            "B.8 puts the rim generator hall between battery rows at z −288 to −148 but gives no emitter height. The HTML model puts "
            "the emitter at z −315, on the middle battery row. Of the eight rim rings, those at 135° and 225° fall on the "
            "boundary with the drive emitter, and the one at 180° falls on the emitter face (D.2 counts only five on the "
            "battery rim).",
            "The model puts rim emitters at z −218 (centre of the generator hall, between rows). The 180° ring goes on the "
            "emitter face at a segment boundary. Please confirm.")

    # B.4/B.5 structure
    check("B.4", "Ring frames at 20 m to the rim", 84, math.ceil(G.R_RIM / 20), 0)
    check("B.4", "Radial frames at the rim", 1024, 64 * 2 ** 4, 0)
    check("B.4", "Panel width just inside r 200 (64 frames), m", 19.6, 2 * PI * 200 / 64, 0.1)
    check("B.4", "Panel width at the rim (1,024 frames), m", 10.3, 2 * PI * G.R_RIM / 1024, 0.1)
    check("B.3", "Upper gallery is exactly eleven levels above the lower", 11, (-170 - G.DECK0) / 6, 0)
    check("B.3", "Levels at the rim, n −65 to 37", 103, 37 - (-65) + 1, 0)
    check("B.3", "Levels at the hub, n −30 to 46", 77, 46 - (-30) + 1, 0)
    check("B.3", "Gross deck area, hull (volume / 6 m), M m²", 805, v_cone_full / 6 / 1e6, 5)
    check("B.3", "Gross deck area, cores, M m²", 81, core_vol / 6 / 1e6, 1)
    check("§14", "Local mid-plane at r 800, z", -203, (G.cap_z(800) + G.crown_z(800)) / 2, 0.5)
    mid_hub = (G.cap_z(470) + G.crown_z(470)) / 2
    finding("§14, B.3, B.9", "minor", "The 'citadel mid-plane' is 100 m below the hub's mid-plane",
            f"§14 puts the citadel at the hub mid-plane, 'deepest material in every direction'. B.3 calls n = −9 (z −290) the "
            f"citadel mid-plane, and the combat bridge sits there at r 380–560. The geometric mid-plane at r 470 is z {mid_hub:.0f}. "
            f"At z −290 the bridge has {-290 - G.crown_z(560):.0f} m of hull under it, toward the cavity (the ship's weak bearing), "
            f"and {G.cap_z(560) + 290:.0f} m over it.",
            "If the intent is to align with Praetorian's equator (z −282), say so. Otherwise raise the citadel to about n = 8 (z −188).")
    # flag plot vs fixed cap
    r_lim = math.sqrt(G.DOME_R ** 2 - (46 + 14 - G.DOME_C) ** 2)
    finding("B.9, B.3", "minor", "The flag plot is squeezed at its outer edge",
            f"The fixed cap clears the core crown by 14 m on the axis (10 m grade + 4 m structure). But the dome falls to +53.8 at "
            f"r 250, so the underside of its structure there is at +39.8. B.3 calls n = 46 (deck at +40) the highest hub level. "
            f"If that level is usable (4.5 m clear), it fits under the cap only inboard of r ≈ {r_lim:.0f}. The plot is "
            "specified out to r 250. Separately, r 250 is the collar well's inner moulded wall, so a plot running to r 250 "
            "leaves no room for that wall's 4 m plating. The envelope check flags it.",
            "Run the plot from r 210 to 246, with its top level stopping at r ≈ 215.")
    finding("B.8, B.10", "significant", "The ordnance cells break through the dish at their inboard foot",
            "The ordnance blocks run from r 1,050 at levels n = −40 to −10, so their floor is at z −476. At r 1,050 the ventral "
            "moulded surface is at z −470 (the crown edge), and it only drops below −476 at r ≈ 1,072 (below −480, which clears "
            "the 4 m grade, at r ≈ 1,078). The inboard 22 m of the blocks' bottom level sits outside the hull, opening the "
            "magazines onto the cavity. That is the most heavily protected volume on the ship and the bearing it is most "
            "exposed on. Neither the Annex B.13 check nor the HTML model finds it, because both sample at up to 40 m.",
            "Start the ordnance blocks at r 1,080, or raise their floor to n = −38 (z −464).")
    # growing decks tiers
    finding("B.3, B.11", "significant", "The growing decks' z range holds 14 tiers, not 16",
            "n = −25 to −18 and z −386 to −344 span 42 m (seven pitches). Sixteen 3 m tiers, or 'eight primary bays', need 48 m. "
            "C.12's ordnance-block volume (71 M m³) reproduces only if a range 'n = a to b' means deck planes a to b (−476 to −296), "
            "which confirms the 42 m reading. At 14 tiers the growing area is 515,200 × 14 = 7.21 M m², "
            "short of the 8.25 M m² §12 requires.",
            "Extend the blocks to z −338 (n = −25 to −17), or make them 7.5 % larger in plan.")
    finding("§14, B.11", "significant", "The r = 1,500 rule is broken by three volumes the spec itself places",
            "§14 and B.11 say the endurance plant stops at r 1,500 and nothing but weapons stands outboard of it between z −142 and "
            "−532. But B.11 places the growing decks at r 1,350–1,550, and B.8 places the rim (r 1,500–1,674, z −288 to −148) and "
            "ventral (r 1,500–1,630, z −483 to −343) shield generator halls outboard of r 1,500. The generator halls are meant to be "
            "there ('set in the gaps between battery rows'), so the rule's wording is at fault. The growing decks are not.",
            "Reword the rule to exclude the generator halls, and pull the growing decks in to r 1,300–1,500.")
    finding("B.2, B.13, B.11", "minor", "The count of identical segments is inconsistent, and too high in both places",
            "B.2 says 'seven of the eight … identical' plus two unique aft segments (9 ≠ 8). B.13 says six identical and two aft. "
            "But the aft block (135°–225°) spans half of the 135° and 225° segments and all of the 180° segment. B.11 gives the "
            "135° and 225° sectors their own growing-deck layout. That leaves five identical segments (0°, 45°, 90°, 270°, "
            "315°), a mirrored 135°/225° pair, and a unique 180° segment.",
            "State five identical segments, one mirrored pair and one stern segment.")
    finding("B.8", "minor", "'Segment' means two different things in B.8",
            "Cavity defence is '640 mounts, forty a segment' and cavity arrays are 'three a segment, 48 in all'. Both imply 16 "
            "segments, but everywhere else a segment is one of eight 45° inter-aperture segments.",
            "Say 'eighty a segment' and 'six a segment', or define a 22.5° half-segment.")
    # booms stowage vs knuckle grade
    worst = 0.0
    worst_z = None
    for zz in np.linspace(-618, -578, 81):
        c = (zz - G.Z_VENTRAL) / G.KN_B
        r_surf = G.R_CROWN + G.KN_A * math.sqrt(max(0, 1 - c * c))
        nr, nz = G.knuckle_normal(r_surf)
        # plating inner face: 4 m along the inward normal (-n)
        r_in = r_surf - 4 * nr
        intr = r_in - 1130.0
        if intr > worst:
            worst, worst_z = intr, zz
    finding("B.7, Annex B.13 check", "minor", "The lower stowed boom cuts into the knuckle plating",
            f"The lower boom stows at r 1,130–1,170, z −618 to −578. There the knuckle is almost vertical and its moulded surface "
            f"is at r 1,125.6–1,129.8, so the 4 m grade's inner face lies at up to r {1130 + worst:.1f} (worst at z {worst_z:.0f}). "
            f"That puts the boom's inboard face up to {worst:.1f} m inside the structural ring plating. The Annex B.13 check "
            "(like the HTML model's) measures the grade vertically, which hides this on a near-vertical wall.",
            "Start the lower boom stowage at r 1,135, or measure the grade along the surface normal in the check.")
    # collar travel
    finding("§2.2, B.9, D.8", "significant", "A 45 m collar travel leaves the stowed crown inside the shutters",
            f"Extended crown +100; travel 45 m puts the stowed crown at +55. The dorsal surface across the well mouth is "
            f"+{G.cap_z(250):.1f} (inner) to +{G.cap_z(450):.1f} (outer), and the iris shutters carry the full 10 m grade under it, "
            f"so their underside is at +{G.cap_z(250) - 10:.1f} to +{G.cap_z(450) - 10:.1f}. A +55 crown stands 1–4 m proud of "
            "the surface, through the shutters. They can't close over a collar stowed that way. The well can't be deepened to fix it: "
            f"Praetorian's bowl shell reaches r 250 at z {bowl_at_250:.0f}.",
            "The collar body can be at most 50 m tall (well floor −10 to stowed crown +40), with 60 m of travel. Extended, it stands "
            "from +50 to +100 on its lift. The model uses these figures.")
    # hub cap highest point
    finding("§7", "minor", "The hub cap is the highest point only when the collar is stowed",
            "§7 calls the hub-cap sensors the 'highest point in either collar state'. With the collar extended, its crown (+100) "
            "stands 20 m over the fixed cap (+80) and screens the cap's apertures below about "
            f"{math.degrees(math.atan((100 - G.dome_z(150)) / (250 - 150))):.0f}° elevation.",
            "Say 'the highest point with the collar stowed', or accept that the collar screens them when extended.")
    # heavy magazines
    pitch_mag = 40 * 1534 / G.R_RIM
    finding("B.8, C.12", "minor", "Heavy ready magazines touch their neighbours if 40 m is the tangential width",
            f"Magazines of 40 × 40 × 30 m sit about 90–130 m inboard of the rim (r ≈ 1,514–1,554), where the 40 m mount pitch has "
            f"shrunk to about {pitch_mag:.1f} m. With 40 m tangential they would overlap by about {40 - pitch_mag:.1f} m.",
            "Orient the 30 m side tangentially.")
    # booms toward the bow
    finding("B.7", "minor", "'Toward the bow' is ambiguous for booms on the 0°, 180° and port spines",
            "Booms on the 0° and 180° spines have no bow side. On the port spines (225°–315°), 'toward the bow' is the opposite "
            "rotational sense to the starboard spines. The HTML model uses one rotational sense throughout (decreasing θ).",
            "State it as a rotational sense ('upper booms stow toward decreasing θ').")
    finding("B.3, B.12", "minor", "Two readings of the gallery z value, and B.12's layer ranges overlap",
            "B.3 numbers levels 'from the lower gallery deck', z −236 + 6n, which reads as a floor. B.12 gives the galleries as "
            "'n = 0, ±12 m' and 'n = 11, ±12 m', which is centred. B.12 also puts the citadel at n −45 to −1 and handling at "
            "n 1 to 10, both of which overlap the ±12 m gallery bands.",
            "Pick one. The model follows B.12 (centred) for the de-stack slabs, as the HTML model does.")
    # twelve powerplants
    finding("D.3, §3.3", "minor", "The core designations' face is misdescribed",
            "D.3 paints each core's designation on its southern cap, 'the face that is uppermost when a core is landed'. "
            "But §3.3 lands a core on that cap: the repulsorlift array there carries the landing load, the gear at 30°–50° S "
            "stands the core 'on its own pole clearance', and D.2 scorches the cap 'after any landing'. The southern cap is "
            "lowermost on the ground. It faces the ground only while the core descends, which is when a designation there "
            "could be read from the surface.",
            "Read: 'the face turned to the ground on descent'. The model paints the designations on the southern cap.")
    finding("B.8", "minor", "Drive emitter segment width",
            f"B.8 gives each of the sixteen segments 'about 169 m of face width'. The fairing's aft boundary (the cosine lune "
            f"from 135° to 225°) is {G.fairing_arc_length():.0f} m long, which is {G.fairing_arc_length() / 16:.0f} m a segment. "
            f"169 m does not match the rim arc ({G.R_RIM * PI / 2 / 16:.0f} m) or the chord ({2 * G.R_RIM * math.sin(PI / 4) / 16:.0f} m) either.",
            "Use 178 m.")
    finding("§13.3", "minor", "'Twelve powerplants' is the deployed count, but the mass quoted is mated",
            "4.8 Bt is the mated mass. Mated, the ship has 4 main + 8 auxiliary + 9 core plants = 21. Twelve is the count "
            "after the cores deploy.", "Say 'twenty-one powerplants', or quote 4.5 Bt.")

    # §9-§11 complement arithmetic
    check("§9", "Air group total", 1188, 576 + 288 + 144 + 48 + 48 + 48 + 18 + 18, 0)
    check("§9", "Craft per Lance", 36, 18 + 6 + 6 + 6, 0)
    check("§10", "Ship's company breakdown", 42500, 500 + 1500 + 8000 + 10000 + 4000 + 6000 + 2000 + 4000 + 1500 + 2000 + 3000, 0)
    check("§10", "Complement", 109592, 42500 + 8000 + 9280 + 49812, 0)
    check("§11", "Ground force total", 49812, 36864 + 6048 + 1700 + 1300 + 2000 + 400 + 1500, 0)
    check("§11", "Armour and artillery crews", 6048, 288 * 7 + 96 * 30 + 576 * 2, 0)
    check("§11", "Corps line strength (9 × 4⁶)", 36864, 9 * 4 ** 6, 0)
    check("§11.1", "Line per Lance", 4608, 36864 / 8, 0)
    check("B.6", "Lance berthing", 6880, 4608 + (6048 + 1700 + 2000 + 400) / 8 + 1000, 10)
    check("§14", "Hull berthing (ship's company + aircrew)", 51780, 42500 + 9280, 0)
    check("B.11", "Hull habitable claim at 110 m²/person, M m²", 5.7, 51780 * 110 / 1e6, 0.01)
    tf = 109592 + 8 * 700 + 4 * 7400 + 4 * 5200 + 18 * 750 + 4 * 900 + 8 * 73 + 8 * 8
    check("§18.3/C.1", "Task force personnel", 183340, tf, 0)
    check("A.1", "Task force with 100-crew Arquitens", 171600, tf - 18 * 650, 100)
    check("§18.3", "Task force hulls", 47, 1 + 8 + 4 + 4 + 18 + 4 + 8, 0)
    check("§18.3", "Starfighters", 2640, 864 + 4 * 420 + 4 * 24, 0)
    check("§18.3", "Core-based craft", 324, 1188 - 864, 0)
    check("§12", "Growing area at 45 m² for 183,300, M m²", 8.25, 45 * 183300 / 1e6, 0.01)
    check("§12", "Footprint at 16 tiers, m²", 516000, 45 * 183300 / 16, 1000)
    check("B.11", "Growing footprint, 7 sectors × 2 blocks × 184 × 200 m", 515200, 7 * 2 * 184 * 200, 0)
    check("C.1", "Design population (×1.1 × 1.1)", 130000, 109592 * 1.1 * 1.1, 3000)
    check("C.1", "O₂ per day at 0.84 kg/person, t", 92, 109592 * 0.84 / 1000, 1)
    check("C.1", "Metabolic heat at 137 W/person, MW", 15, 109592 * 137 / 1e6, 0.2)
    check("C.3", "Berthing compartments of 48", 2284, math.ceil(109592 / 48), 0)
    check("C.3", "Cubicles of six", 18266, math.ceil(109592 / 6), 0)
    check("C.3", "Personal locker volume at 0.37 m³, m³", 40300, 109592 * 0.37, 300)
    finding("C.3", "minor", "The locker volume is not 'larger than the citadel'",
            "40,300 m³ is less than one citadel hall: the combat bridge alone (180 × 120 m, 24 m clear) is 518,400 m³. The "
            "citadel's fabrication and magazine band, r < 600 m over n −20 to −1, is about 82 M m³ once Praetorian's bowl is excluded.",
            "Drop the comparison, or compare with something of that size (one heavy-battery ready magazine is 48,000 m³).")
    check("C.3", "Berthing at 2.4 m²/person, m²", 263000, 109592 * 2.4, 500)
    check("C.3", "Berthing share of total deck ('a third of one percent'), %", 0.333, 109592 * 2.4 / (805e6 + 81e6) * 100, 0.05,
          "263,000 m² of 886 M m² is 0.03 %, three hundredths of one percent.")
    check("C.4", "Washbasins at 1:12", 9133, math.ceil(109592 / 12), 0)
    check("C.4", "Water closets at 1:14", 7828, math.ceil(109592 / 14), 0)
    check("C.4", "Showers at 1:25", 4384, math.ceil(109592 / 25), 0)
    check("C.5", "Mess seats at 30 %", 32878, 109592 * 0.3, 1)
    check("C.6", "Fitness stations", 1468, 109592 * 1.5 / 112, 1)
    check("C.6", "Barber chairs at 1:300", 366, math.ceil(109592 / 300), 0)
    check("C.6", "Laundry per week from stated rates (≤ 5.4 kg/person), t", 1193, 109592 * 5.4 / 1000, 10,
          "Even at the officers' 5.4 kg for everyone, the rate gives 592 t/week. At the crew's 3.2 kg it is 351 t. "
          "1,193 t implies 10.9 kg a person a week.")
    check("C.6", "Dry cleaning at 1.8 kg/month, t", 198, 109592 * 1.8 / 1000, 1)
    check("C.7", "Beds at 5 per 1,000", 548, 109592 * 5 / 1000, 1)
    check("C.8", "Replenishment air at 2.4 L/s, m³/s", 260, 109592 * 2.4 / 1000, 5)
    check("C.13", "Escape pods at 12", 9133, math.ceil(109592 / 12), 0)
    shares = [25, 15, 10, 10, 8, 6, 5, 5, 4, 3, 3, 2.6, 2, 1]
    vols = [1210, 725, 480, 480, 385, 290, 240, 240, 195, 145, 145, 124, 95, 50]
    check("C.14", "Volume budget shares sum, %", 100, sum(shares), 0.05,
          f"The listed claims total {sum(vols)} M m³ of 4,830. {4830 - sum(vols)} M m³ is unallocated.")
    ord_vol = 2 * (26.5 / 360) * PI * (1400 ** 2 - 1050 ** 2) * 180
    check("C.12", "Ordnance block volume (26.5° × 2, r 1,050–1,400, 180 m), M m³", 71, ord_vol / 1e6, 1)
    heavy = 591 * (PI * 22 ** 2 * 30 + 15 * 15 * 90 + 40 * 40 * 30 + 30 * 30 * 20)
    med = 746 * (PI * 11 ** 2 * 45 + 8 * 8 * 40 + 20 * 20 * 15 + 12 * 12 * 10)
    pdv = 1920 * PI * 6 ** 2 * 20
    check("C.12", "Mount installation volume (barbettes, trunks, magazines, sinks), M m³", 53, (heavy + med + pdv) / 1e6, 5,
          f"Summing the C.12 stack as stated: heavy {heavy / 1e6:.0f}, medium {med / 1e6:.0f}, close-in {pdv / 1e6:.1f} M m³. "
          "The 53 M m³ figure is about what you get leaving out the barbettes.")
    halls = 4 * 300 * 300 * 498 + 8 * 200 * 200 * 300
    gens = 8 * (174 * 200 * 140 + 130 * 200 * 140 + 200 * 200 * 120 + 120 ** 3)
    check("C.14", "Machinery (reactor, auxiliary, shield and screen halls), M m³", 290, (halls + gens) / 1e6, 15,
          f"Reactor halls (≈300 × 300 × 498 m) and auxiliary blocks alone are {halls / 1e6:.0f} M m³. The 32 generator halls add "
          f"{gens / 1e6:.0f} M m³ at the HTML model's assumed tangential widths (200 m; 120 m for the screen halls), which "
          "B.8 does not give.")

    # ------------------------------------------------ volumetric clash check
    clash_check("spec")
    clash_check("model")

    # ------------------------------------------------ HTML datum model audit
    html_audit()

    out = dict(checks=RESULTS, findings=FINDINGS)
    with open(os.path.join(HERE, "results.json"), "w") as f:
        json.dump(out, f, indent=1)
    write_markdown()
    npass = sum(r["status"] == "PASS" for r in RESULTS)
    print(f"{npass}/{len(RESULTS)} derived figures reproduce; {len(RESULTS) - npass} flagged; {len(FINDINGS)} findings")


# ======================================================================
# Volumetric clash check (Annex B / C allocation as specified in Rev H)
# ======================================================================
VOID, SOFT, APERTURE, MATES = G.VOID, G.SOFT, G.APERTURE, G.MATES


def to_spine(th, x, y):
    c, s = math.cos(th * D2R), math.sin(th * D2R)
    return x * s + y * c, x * c - y * s   # u along the spine, v toward increasing theta


def from_spine(th, u, v):
    c, s = math.cos(th * D2R), math.sin(th * D2R)
    return u * s + v * c, u * c - v * s


def vol_samples(V, spacing):
    k, A = V["kind"], V["A"]
    z0, z1 = V["z0"], V["z1"]
    nz = max(2, int(round((z1 - z0) / spacing)))
    zs = z0 + (np.arange(nz) + 0.5) * (z1 - z0) / nz
    if k == "wedge":
        r0, r1, t0, t1 = A
        nr = max(2, int(round((r1 - r0) / spacing)))
        rm = (r0 + r1) / 2
        nt = max(2, int(round((t1 - t0) * D2R * rm / spacing)))
        rs = r0 + (np.arange(nr) + 0.5) * (r1 - r0) / nr
        ts = V["th"] + t0 + (np.arange(nt) + 0.5) * (t1 - t0) / nt
        RR, TT, ZZ = np.meshgrid(rs, ts, zs, indexing="ij")
        X, Y = RR * np.sin(TT * D2R), RR * np.cos(TT * D2R)
        return np.stack([X.ravel(), Y.ravel(), ZZ.ravel()], 1)
    if k == "sphere":
        rc, rho = A
        zc = (z0 + z1) / 2
        n = max(4, int(round(2 * rho / spacing)))
        g = -rho + (np.arange(n) + 0.5) * 2 * rho / n
        U, W, Z = np.meshgrid(g, g, g, indexing="ij")
        m = U ** 2 + W ** 2 + Z ** 2 <= rho ** 2
        X, Y = from_spine(V["th"], rc + U[m], W[m])
        return np.stack([X, Y, zc + Z[m]], 1)
    u0, u1, v0, v1 = A
    if k == "boxR":
        u1 = G.R_RIM + 200
    nu = max(2, int(round((u1 - u0) / spacing)))
    nv = max(2, int(round((v1 - v0) / spacing)))
    us = u0 + (np.arange(nu) + 0.5) * (u1 - u0) / nu
    vs = v0 + (np.arange(nv) + 0.5) * (v1 - v0) / nv
    U, W, Z = np.meshgrid(us, vs, zs, indexing="ij")
    X, Y = from_spine(V["th"], U.ravel(), W.ravel())
    P = np.stack([X, Y, Z.ravel()], 1)
    if k == "boxR":
        r = np.hypot(P[:, 0], P[:, 1])
        th = np.degrees(np.arctan2(P[:, 0], P[:, 1])) % 360
        ro = np.array([G.r_out(t) for t in th])
        P = P[r <= ro - V["param"]]
    return P


def vol_contains(V, P, eps=0.5):
    k, A = V["kind"], V["A"]
    z = P[:, 2]
    m = (z > V["z0"] + eps) & (z < V["z1"] - eps)
    if k == "wedge":
        r = np.hypot(P[:, 0], P[:, 1])
        th = np.degrees(np.arctan2(P[:, 0], P[:, 1])) % 360
        dt = (th - V["th"] + 180) % 360 - 180
        ea = eps / np.maximum(r, 1) / D2R
        return m & (r > A[0] + eps) & (r < A[1] - eps) & (dt > A[2] + ea) & (dt < A[3] - ea)
    u, v = to_spine(V["th"], P[:, 0], P[:, 1])
    if k == "sphere":
        zc = (V["z0"] + V["z1"]) / 2
        return (u - A[0]) ** 2 + v ** 2 + (z - zc) ** 2 < (A[1] - eps) ** 2
    if k == "boxR":
        r = np.hypot(P[:, 0], P[:, 1])
        th = np.degrees(np.arctan2(P[:, 0], P[:, 1])) % 360
        ro = np.array([G.r_out(t) for t in th])
        return m & (u > A[0] + eps) & (v > A[2] + eps) & (v < A[3] - eps) & (r < ro - V["param"] - eps)
    return m & (u > A[0] + eps) & (u < A[1] - eps) & (v > A[2] + eps) & (v < A[3] - eps)


_ALLOWED = {}


def allowed_region(th_bucket):
    """Hull section at longitude less its plating: 10 m under dorsal-facing
    surfaces, 12 m bowl shell on Praetorian's bowl, 4 m on every other face,
    all measured along the surface normal."""
    if th_bucket in _ALLOWED:
        return _ALLOWED[th_bucket]
    t = th_bucket
    prof = G.hull_profile(t, n_cap=96, n_knuckle=96, n_crown=96, n_bowl=128)
    poly = Polygon(prof).buffer(0)
    inner = poly.buffer(-G.GRADE_OTHER, join_style=2)
    # dorsal line: dome + cap (+ fairing crown), 10 m
    dors = [(r, G.dome_z(r)) for r in np.linspace(0, 250, 40)]
    dors_out = [(r, G.cap_z(r)) for r in np.linspace(450, G.R_RIM, 120)]
    ro = G.r_out(t)
    if ro > G.R_RIM:
        dors_out.append((ro, 0.0))
    # collar-well shutters carry the dorsal grade across the mouth; treat the well mouth as closed for grade purposes
    band = unary_union([LineString(dors).buffer(G.GRADE_DORSAL, cap_style=2),
                        LineString(dors_out).buffer(G.GRADE_DORSAL, cap_style=2)])
    p = G.PRAETORIAN
    bowl = LineString([(p.bowl_inner * math.cos(a), p.zc + p.bowl_inner * math.sin(a)) for a in np.linspace(0, PI / 2, 128)])
    band = band.union(bowl.buffer(12.0, cap_style=2))
    region = inner.difference(band)
    _ALLOWED[th_bucket] = region
    return region


def envelope_violation(P):
    """Boolean per point: outside the plated hull interior or inside a Lance keep-out."""
    r = np.hypot(P[:, 0], P[:, 1])
    th = np.degrees(np.arctan2(P[:, 0], P[:, 1])) % 360
    bad = np.zeros(len(P), bool)
    kind = np.array([""] * len(P), dtype=object)
    buckets = np.where((th >= 134) & (th <= 226), np.round(th * 2) / 2, 0.0)
    for b in np.unique(buckets):
        sel = buckets == b
        reg = allowed_region(float(b))
        inside = shapely.contains_xy(reg, r[sel], P[sel, 2])
        idx = np.where(sel)[0]
        bad[idx[~inside]] = True
        kind[idx[~inside]] = "hull"
    for c in G.LANCES:
        d3 = np.sqrt((P[:, 0] - c.x) ** 2 + (P[:, 1] - c.y) ** 2 + (P[:, 2] - c.zc) ** 2)
        dh = np.hypot(P[:, 0] - c.x, P[:, 1] - c.y)
        b1 = (P[:, 2] > c.zc) & (d3 < c.bowl_moulded)
        b2 = (P[:, 2] <= c.zc) & (dh < c.well_r + G.GRADE_OTHER)
        bad |= b1 | b2
        kind[b1 | b2] = "lance"
    return bad, kind


def clash_check(variant="spec"):
    V = G.allocation(variant)
    hard = [v for v in V if not (v["flags"] & (VOID | SOFT))]
    npts = 0
    viols = {}
    for v in V:
        if v["flags"] & (SOFT | APERTURE):
            continue
        P = vol_samples(v, 4.0 if (v["z1"] - v["z0"]) < 80 else 8.0)
        npts += len(P)
        bad, kind = envelope_violation(P)
        if v["flags"] & MATES:
            bad &= kind != "lance"
            # galleries and spurs may cross Praetorian's bowl too
            rr = np.hypot(P[:, 0], P[:, 1])
            bad &= ~(rr < G.PRAETORIAN.bowl_moulded + 1)
        if v["name"] == "Mouth lift trunk":
            bad &= P[:, 2] > -500   # the trunk opens onto the mouth head by design
        if bad.any():
            key = v["name"]
            e = viols.setdefault(key, dict(points=0, thetas=set(), zmin=1e9, zmax=-1e9, rmin=1e9, rmax=-1e9))
            e["points"] += int(bad.sum())
            e["thetas"].add(round(v["th"] % 360, 2))
            Q = P[bad]
            rq = np.hypot(Q[:, 0], Q[:, 1])
            e["zmin"] = min(e["zmin"], float(Q[:, 2].min()))
            e["zmax"] = max(e["zmax"], float(Q[:, 2].max()))
            e["rmin"] = min(e["rmin"], float(rq.min()))
            e["rmax"] = max(e["rmax"], float(rq.max()))
    clashes = {}
    for i, a in enumerate(hard):
        Pa = vol_samples(a, 6.0)
        for j, b in enumerate(hard):
            if j <= i or a.get("parent") == b["name"] or b.get("parent") == a["name"]:
                continue
            if a["z1"] <= b["z0"] or b["z1"] <= a["z0"]:
                continue
            inside = vol_contains(b, Pa)
            if inside.any():
                key = (a["name"], b["name"])
                clashes[key] = clashes.get(key, 0) + int(inside.sum())
    # voids (galleries, trunks, command spaces) must not run through hard volumes either
    voids = [v for v in V if (v["flags"] & VOID) and not (v["flags"] & APERTURE)]
    vclash = {}
    for a in voids:
        Pa = vol_samples(a, 6.0)
        for b in hard:
            if a["z1"] <= b["z0"] or b["z1"] <= a["z0"]:
                continue
            inside = vol_contains(b, Pa)
            if inside.any():
                key = (a["name"], b["name"])
                vclash[key] = vclash.get(key, 0) + int(inside.sum())
    tag = "Rev H as specified" if variant == "spec" else "as modelled (with fixes)"
    RESULTS.append(dict(ref="B.13", claim=f"Voids against hard volumes, {tag}", stated=0, computed=len(vclash), tol=0,
                        status="PASS" if not vclash else "FLAG",
                        note="; ".join(f"{a} × {b} ({n} pts)" for (a, b), n in vclash.items()) or "no void runs through a hard volume"))
    RESULTS.append(dict(ref="B.13", claim=f"Volumetric clash check, hard volumes pairwise, {tag}", stated=0, computed=len(clashes), tol=0,
                        status="PASS" if not clashes else "FLAG",
                        note="; ".join(f"{a} × {b} ({n} pts)" for (a, b), n in clashes.items()) or
                        f"{len(V)} volume instances, no hard-volume overlaps"))
    notes = []
    for k, e in viols.items():
        notes.append(f"{k}: {e['points']} pts at θ {sorted(e['thetas'])[:4]}…, r {e['rmin']:.0f}–{e['rmax']:.0f}, z {e['zmin']:.0f} to {e['zmax']:.0f}")
    RESULTS.append(dict(ref="B.13", claim=f"Envelope check with normal-offset grades, {tag}", stated=0, computed=len(viols), tol=0,
                        status="PASS" if not viols else "FLAG", note=" | ".join(notes) or f"{npts} sample points, no violations"))


# ======================================================================
def html_audit():
    finding("Datum model (HTML)", "minor", "File name and labels give three different revisions",
            "The file is named 'Revision_G_datum_model', its title says Revision H, and its interior toggle is labelled "
            "'Rev F as written / Rev H as specified' but keyed internally as 'G'.", "Rename to Rev H throughout.")
    finding("Datum model (HTML)", "significant", "The drawn fittings lag the Rev H allocation data in the same file",
            "Heavy rim battery: draws 198 a row (594) at 270/198° spacing. Rev H says 197 (591) on a 40 m pitch. "
            "Medium: the ventral outer band is drawn at r 1,119 on the knuckle (skipping mouths and screen emitters), not at "
            "r 1,331 on the flat annulus with 278 mounts. The legend says '702 medium' where B.8 gives 746 (C.12: 729). "
            "Boom recesses are drawn at r 1,078 and 1,130, not the Rev H roots at r 1,108 / z −520 and r 1,128 / z −596, "
            "which the file's own allocation data carries.", "Regenerate the fittings layer from the Rev H datums.")
    finding("Datum model (HTML)", "minor", "Fittings drawn on top of each other",
            "Rim shield emitters are drawn at z −315, on the middle heavy-battery row. Dorsal shield emitters and dorsal sensors "
            "are drawn on the battery circles (r 559.5 and 1,119). Praetorian's key channels are drawn across its hangar-mouth "
            "decals, and its 48 thruster wells on an even 7.5° pitch overlap the mouths.", "See the corresponding spec findings.")
    finding("Datum model (HTML)", "minor", "The clash check measures the grade vertically and samples too coarsely",
            "envelope() tests h < bottomZ(r) + 4, a vertical offset. On the near-vertical knuckle that is not a 4 m plate, so "
            "the lower stowed boom's intrusion is missed. The sampler's spacing is up to 40 m and it tests only cell centres. "
            "That can miss thin intrusions, such as the lift trunks' 0.7 m intrusion at their foot (which is intended, but "
            "is never exercised).", "Offset along the normal (as verification/verify_spec.py does) and sample at ≤ 4 m near surfaces.")


def write_markdown():
    lines = ["# Leviathan-class Rev H: verification results", "",
             "Generated by `verification/verify_spec.py`. Every figure the specification derives from other figures was "
             "recomputed independently from the Annex B datums. Volumes were found by revolving the actual section polygons, "
             "not by the straight-cone approximation.", ""]
    npass = sum(r["status"] == "PASS" for r in RESULTS)
    lines += [f"**{npass} of {len(RESULTS)} derived figures reproduce within tolerance. {len(RESULTS) - npass} are flagged.**", "",
              "| Ref | Claim | Stated | Computed | Tol | Status | Note |", "|---|---|---:|---:|---:|:---:|---|"]
    for r in RESULTS:
        st = "✅" if r["status"] == "PASS" else "⚠️"
        lines.append(f"| {r['ref']} | {r['claim']} | {r['stated']} | {r['computed']} | {r['tol']} | {st} | {r['note']} |")
    lines += ["", "## Findings", ""]
    for sev in ("significant", "minor"):
        for f in [f for f in FINDINGS if f["severity"] == sev]:
            lines += [f"### [{sev}] {f['title']} ({f['ref']})", "", f["detail"], ""]
            if f["fix"]:
                lines += [f"*Suggested resolution:* {f['fix']}", ""]
    with open(os.path.join(HERE, "results.md"), "w") as fh:
        fh.write("\n".join(lines))


if __name__ == "__main__":
    main()
