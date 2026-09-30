"""Render the image set from the built .blend.

    python3 model/render_views.py BLEND OUTDIR [--shots 01,02,...] [--scale 1.0] [--samples 96]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import bpy

D2R = math.pi / 180

CRUISING = dict(collar_extended=1.0, batteries_deployed=0.0, shields_energised=0.0, screen_state=1, mouths_open=1,
                booms_deployed=0, core_release=0.0, destack=0.0, section=0, section_longitude=0.0, lighting=2, drive_throttle=0.25)
ACTION = dict(CRUISING, collar_extended=0.0, batteries_deployed=1.0, shields_energised=1.0, screen_state=2, mouths_open=0, lighting=0,
              drive_throttle=0.6)

# name: (camera, controls, sun (rx, rz, energy), fill energy, exposure, extras)
SHOTS = {
    "01_dorsal_three_quarter": ("CAM 01 Dorsal three-quarter", CRUISING, (63, 150, 3.4), 0.25, 0.0, {}),
    "02_ventral_cavity_flight_ops": ("CAM 02 Ventral cavity", CRUISING, (118, 60, 2.0), 0.6, 0.3, {}),
    "03_stern_drive_emitter": ("CAM 03 Stern quarter", dict(CRUISING, drive_throttle=0.8), (70, 80, 3.0), 0.3, 0.0, {}),
    "04_beam_elevation": ("CAM 04 Beam profile (ortho)", CRUISING, (60, 110, 3.0), 0.5, 0.2, {"res": (2400, 800)}),
    "05_dorsal_plan": ("CAM 05 Dorsal plan (ortho)", CRUISING, (35, 160, 3.2), 0.2, 0.0, {"res": (2000, 2000)}),
    "06_section_cutaway_0deg_spine": ("CAM 06 Section cutaway", dict(CRUISING, section=1, section_longitude=0.0), (62, 100, 3.4), 0.6, 0.2, {}),
    "06b_section_through_lances": ("CAM 06b Section through Lances One and Five", dict(CRUISING, section=1, section_longitude=22.5), (62, 20, 3.4), 0.6, 0.2, {}),
    "07_destack": ("CAM 07 De-stack", dict(CRUISING, destack=1.0, lighting=1), (55, 140, 3.2), 0.6, 0.2, {}),
    "08_core_release": ("CAM 08 Core release", dict(CRUISING, core_release=0.55, screen_state=0, lighting=1), (120, 30, 2.6), 0.6, 0.1, {}),
    "09_bridge_collar": ("CAM 09 Bridge collar", CRUISING, (74, 205, 3.2), 0.3, 0.0, {}),
    "10_lance_released": ("CAM 10 Lance close-up", dict(CRUISING, core_release=0.2, screen_state=0, lighting=1), (100, 250, 3.0), 0.6, 0.3, {}),
    "11_action_state": ("CAM 11 Action state", ACTION, (76, 250, 3.4), 0.12, 0.0, {}),
    "12_campaign_posture": ("CAM 12 Campaign posture", dict(CRUISING, core_release=0.3, screen_state=3, booms_deployed=0), (74, 215, 3.0), 0.5, 0.0, {"planet": True}),
    "14_rim_batteries_deployed": ("CAM 14 Rim batteries deployed", ACTION, (70, 225, 3.4), 0.15, 0.2, {}),
    "13_cavity_interior": ("CAM 13 Cavity interior", dict(CRUISING, screen_state=0), (120, 40, 1.5), 0.4, 0.0, {}),
}


def apply(shot, scale, samples):
    cam, controls, sun, fill, exposure, extra = SHOTS[shot]
    sc = bpy.context.scene
    ctrl = bpy.data.objects["LEVIATHAN controls"]
    for k, v in controls.items():
        ctrl[k] = type(ctrl[k])(v)
    sc.camera = bpy.data.objects[cam]
    s = bpy.data.objects["Sun"]
    s.rotation_euler = (sun[0] * D2R, 0, sun[1] * D2R)
    s.data.energy = sun[2]
    bpy.data.objects["Planetshine (fill from below)"].data.energy = fill
    sc.view_settings.exposure = exposure
    res = extra.get("res", (1920, 1080))
    sc.render.resolution_x = int(res[0] * scale)
    sc.render.resolution_y = int(res[1] * scale)
    sc.cycles.samples = samples
    pl = bpy.data.objects["Planet (campaign posture, 300 km below)"]
    pl.hide_render = not extra.get("planet", False)
    ctrl.update_tag()
    bpy.context.view_layer.update()


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    ap = argparse.ArgumentParser()
    ap.add_argument("blend")
    ap.add_argument("outdir")
    ap.add_argument("--shots", default="")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--samples", type=int, default=96)
    a = ap.parse_args(argv)
    bpy.ops.wm.open_mainfile(filepath=a.blend)
    os.makedirs(a.outdir, exist_ok=True)
    shots = [s for s in SHOTS if not a.shots or any(s.startswith(p) for p in a.shots.split(","))]
    for shot in shots:
        t = time.time()
        apply(shot, a.scale, a.samples)
        bpy.context.scene.render.filepath = os.path.join(a.outdir, shot + ".png")
        bpy.ops.render.render(write_still=True)
        print(f"RENDERED {shot} in {time.time() - t:.0f}s", flush=True)


if __name__ == "__main__":
    main()
