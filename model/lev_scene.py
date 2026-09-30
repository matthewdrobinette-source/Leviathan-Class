"""Controls, drivers, section cutters, cameras, lights and render settings."""
from __future__ import annotations

import bpy
from mathutils import Vector

import lev_geom as G
from lev_materials import MATS, EMISSION_HOOKS

D2R = G.D2R
CTRL = None

# name: (default, min, max, description)
CONTROLS = {
    "collar_extended": (1.0, 0.0, 1.0, "Bridge collar: 0 stowed under the iris shutters, 1 extended to +100 (60 m travel)"),
    "batteries_deployed": (0.0, 0.0, 1.0, "Batteries: 0 stowed flush behind their shutters, 1 at full stroke (12 / 8 / 4 m)"),
    "shields_energised": (0.0, 0.0, 1.0, "Shield ring emitters: 1 stands them 8 m proud and lights them"),
    "screen_state": (1, 0, 3, "Cavity screen: 0 off, 1 attenuating at the rim plane, 2 hardened 10 m below, 3 drawn inboard against the dish"),
    "mouths_open": (1, 0, 1, "Launch mouth outer shutters: 1 open for flight operations"),
    "booms_deployed": (0, 0, 1, "Umbilical booms: 0 stowed in the outer band, 1 telescoped out to 900 m below the rim plane"),
    "core_release": (0.0, 0.0, 1.0, "Core release: opposed pairs One/Five, Two/Six, Three/Seven, Four/Eight, then Praetorian"),
    "destack": (0.0, 0.0, 1.0, "De-stack (B.12): slabs part 150 m, aft block draws 600 m aft, cores drop 900 m"),
    "section": (0, 0, 1, "Cross-section through the axis (removes the half facing the section normal)"),
    "section_longitude": (0.0, 0.0, 180.0, "Longitude of the section plane, degrees from the bow"),
    "lighting": (2, 0, 2, "Exterior lighting (D.4): 0 action (dark), 1 cruising, 2 flight operations"),
    "drive_throttle": (0.25, 0.0, 1.0, "Sublight emitter glow"),
}


def make_controls(coll):
    global CTRL
    ob = bpy.data.objects.new("LEVIATHAN controls", None)
    ob.empty_display_type = "PLAIN_AXES"
    ob.empty_display_size = 200.0
    ob.location = (0, 0, 180)
    coll.objects.link(ob)
    for k, (dv, lo, hi, desc) in CONTROLS.items():
        ob[k] = dv
        ui = ob.id_properties_ui(k)
        if isinstance(dv, int):
            ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, description=desc)
        else:
            ui.update(min=lo, max=hi, soft_min=lo, soft_max=hi, description=desc, step=1)
    CTRL = ob
    return ob


SIMPLE_FAIL = []


def drive(idb, path, expr, props, index=None):
    fc = idb.driver_add(path, index) if index is not None else idb.driver_add(path)
    if isinstance(fc, list):
        raise ValueError(f"array path needs an index: {path}")
    d = fc.driver
    d.type = "SCRIPTED"
    for name, prop in props.items():
        v = d.variables.new()
        v.name = name
        v.type = "SINGLE_PROP"
        v.targets[0].id_type = "OBJECT"
        v.targets[0].id = CTRL
        v.targets[0].data_path = f'["{prop}"]'
    d.expression = expr
    if not d.is_simple_expression:
        SIMPLE_FAIL.append((idb.name, path, expr))
    return fc


def phase(p0, p1, var="R"):
    """Clamp (var - p0) / (p1 - p0) to [0, 1] as a simple expression."""
    return f"min(1, max(0, ({var} - {p0}) / {p1 - p0}))"


ROLE_EXPR = {
    "nav": ("50 * (L >= 1)", {"L": "lighting"}),
    "beacon": ("90 * (L >= 1) * (C > 0.99)", {"L": "lighting", "C": "collar_extended"}),
    "flood": ("8 * (L >= 1) * M", {"L": "lighting", "M": "mouths_open"}),
    "flightops": ("25 * (L >= 2) * M", {"L": "lighting", "M": "mouths_open"}),
    "boom": ("25 * (L >= 1)", {"L": "lighting"}),
    "interior": ("0.12 + 0.22 * (L >= 1)", {"L": "lighting"}),
    "sensor": ("0.5", {}),
    "shield": ("0.15 + 1.1 * S", {"S": "shields_energised"}),
    "screen_emitter": ("0.3 + 3 * (Q >= 1)", {"Q": "screen_state"}),
    "screen_att": ("0.9", {}),
    "screen_hard": ("1.4", {}),
    "repulsor": ("0.15 + 2 * (R > 0.001)", {"R": "core_release"}),
    "drive": ("3 * T", {"T": "drive_throttle"}),
    "core_drive": ("25 * (R > 0.02)", {"R": "core_release"}),
}


def bind_material_drivers():
    for mname, node, inp, role in EMISSION_HOOKS:
        m = MATS[mname]
        expr, props = ROLE_EXPR[role]
        path = f'nodes["{node}"].outputs[0].default_value' if inp == "OUT" else f'nodes["{node}"].inputs["{inp}"].default_value'
        drive(m.node_tree, path, expr, props)


def drive_visibility(ob, expr, props):
    """expr evaluates to 1 when the object should be hidden."""
    drive(ob, "hide_viewport", expr, props)
    drive(ob, "hide_render", expr, props)


def section_cutters(coll):
    """Half-space boxes on the removed side of the section plane. One per
    section material, all children of one pivot the section_longitude turns."""
    piv = bpy.data.objects.new("Section plane", None)
    piv.empty_display_type = "SINGLE_ARROW"
    piv.empty_display_size = 400
    coll.objects.link(piv)
    drive(piv, "rotation_euler", f"-S * {D2R}", {"S": "section_longitude"}, index=2)
    cut = {}
    for key, mat in (("hull", "Section"), ("lance", "Section, Lance"), ("praetorian", "Section, Praetorian")):
        me = bpy.data.meshes.new(f"Section cutter, {key}")
        x0, x1, y, z = 0.0, 7000.0, 7000.0, 7000.0
        V = [(x, yy, zz) for x in (x0, x1) for yy in (-y, y) for zz in (-z, z)]
        F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        me.from_pydata(V, [], F)
        me.update()
        me.materials.append(MATS[mat])
        ob = bpy.data.objects.new(f"Section cutter, {key}", me)
        coll.objects.link(ob)
        ob.parent = piv
        ob.display_type = "WIRE"
        ob.hide_render = True
        ob.hide_viewport = True
        cut[key] = ob
    return piv, cut


def add_section_boolean(ob, cutter):
    m = ob.modifiers.new("Section", "BOOLEAN")
    m.operation, m.solver, m.object = "DIFFERENCE", "EXACT", cutter
    m.material_mode = "TRANSFER"
    m.use_hole_tolerant = True
    drive(ob, 'modifiers["Section"].show_viewport', "S >= 1", {"S": "section"})
    drive(ob, 'modifiers["Section"].show_render', "S >= 1", {"S": "section"})
    return m


def add_section_delete(ob, ng):
    m = ob.modifiers.new("Section delete", "NODES")
    m.node_group = ng
    ids = {it.name: it.identifier for it in ng.interface.items_tree if it.item_type == "SOCKET" and it.in_out == "INPUT"}
    drive(ob, f'modifiers["Section delete"]["{ids["Section"]}"]', "S >= 1", {"S": "section"})
    drive(ob, f'modifiers["Section delete"]["{ids["Section Normal"]}"]', f"cos(A * {D2R})", {"A": "section_longitude"}, index=0)
    drive(ob, f'modifiers["Section delete"]["{ids["Section Normal"]}"]', f"-sin(A * {D2R})", {"A": "section_longitude"}, index=1)
    return m


def look_at(ob, target):
    d = Vector(target) - ob.location
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


CAMERAS = {
    # name: (location, target, lens, ortho_scale or None)
    "CAM 01 Dorsal three-quarter": ((5000.0, 5500.0, 2750.0), (0.0, -150.0, -300.0), 50.0, None),
    "CAM 02 Ventral cavity": ((-3000.0, 2500.0, -2900.0), (0.0, 0.0, -420.0), 45.0, None),
    "CAM 03 Stern quarter": ((2300.0, -4700.0, 650.0), (0.0, -900.0, -300.0), 50.0, None),
    "CAM 04 Beam profile (ortho)": ((6000.0, 0.0, -315.0), (0.0, 0.0, -315.0), 50.0, 3900.0),
    "CAM 05 Dorsal plan (ortho)": ((0.0, -90.0, 6000.0), (0.0, -90.0, 0.0), 50.0, 3800.0),
    "CAM 06 Section cutaway": ((4200.0, 1500.0, 900.0), (0.0, 0.0, -300.0), 45.0, None),
    "CAM 06b Section through Lances One and Five": ((4700.0, -4300.0, 1250.0), (-60.0, 90.0, -330.0), 58.0, None),
    "CAM 07 De-stack": ((6900.0, 7300.0, 2600.0), (0.0, -200.0, -140.0), 55.0, None),
    "CAM 08 Core release": ((-2800.0, -3600.0, -2300.0), (0.0, 0.0, -900.0), 42.0, None),
    "CAM 09 Bridge collar": ((820.0, 960.0, 250.0), (0.0, 0.0, 60.0), 50.0, None),
    "CAM 10 Lance close-up": ((1450.0, 1700.0, -1450.0), (306.0, 739.0, -1690.0), 60.0, None),
    "CAM 11 Action state": ((-4400.0, 3700.0, 1500.0), (0.0, 0.0, -330.0), 50.0, None),
    "CAM 12 Campaign posture": ((3300.0, -3500.0, 120.0), (0.0, 0.0, -1000.0), 35.0, None),
    "CAM 14 Rim batteries deployed": ((-260.0, 2330.0, 60.0), (-660.0, 1545.0, -330.0), 40.0, None),
    "CAM 13 Cavity interior": ((0.0, -450.0, -600.0), (790.0, 790.0, -470.0), 22.0, None),
}


def make_cameras(coll):
    cams = {}
    for name, (loc, tgt, lens, ortho) in CAMERAS.items():
        cd = bpy.data.cameras.new(name)
        cd.lens = lens
        cd.clip_start = 1.0
        cd.clip_end = 2.0e7
        if ortho:
            cd.type = "ORTHO"
            cd.ortho_scale = ortho
        ob = bpy.data.objects.new(name, cd)
        coll.objects.link(ob)
        ob.location = loc
        look_at(ob, tgt)
        cams[name] = ob
    return cams


def lights(coll):
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN"))
    sun.data.energy = 3.2
    sun.data.angle = 0.53 * D2R
    sun.data.color = (1.0, 0.97, 0.93)
    coll.objects.link(sun)
    sun.rotation_euler = (52 * D2R, 0, 148 * D2R)
    fill = bpy.data.objects.new("Planetshine (fill from below)", bpy.data.lights.new("Planetshine", "SUN"))
    fill.data.energy = 0.25
    fill.data.color = (0.55, 0.68, 1.0)
    fill.data.angle = 25 * D2R
    coll.objects.link(fill)
    fill.rotation_euler = (160 * D2R, 0, -20 * D2R)
    floods = []
    for k in range(16):
        t = 22.5 * k + 11.25
        ld = bpy.data.lights.new(f"Cavity flood {k + 1:02d}", "AREA")
        ld.shape = "DISK"
        ld.size = 30.0
        ld.color = (1.0, 0.66, 0.30)
        ld.energy = 0.0
        ob = bpy.data.objects.new(f"Cavity flood {k + 1:02d}", ld)
        coll.objects.link(ob)
        x, y = G.xy(1050.0, t)
        ob.location = (x, y, -486.0)
        look_at(ob, (0.0, 0.0, -300.0))
        drive(ld, "energy", "3.5e5 * (L >= 1) * M", {"L": "lighting", "M": "mouths_open"})
        floods.append(ob)
    return sun, fill, floods


def render_settings(scene):
    scene.render.engine = "CYCLES"
    c = scene.cycles
    c.device = "CPU"
    c.samples = 96
    c.preview_samples = 16
    c.use_adaptive_sampling = True
    c.adaptive_threshold = 0.02
    c.use_denoising = True
    c.denoiser = "OPENIMAGEDENOISE"
    c.max_bounces = 6
    c.diffuse_bounces = 2
    c.glossy_bounces = 2
    c.transmission_bounces = 4
    c.transparent_max_bounces = 16
    c.sample_clamp_indirect = 6.0
    c.blur_glossy = 1.0
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX"
    try:
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.length_unit = "METERS"
    # compositor: a little bloom on lights and glazing
    scene.use_nodes = True
    nt = scene.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    rl = nt.nodes.new("CompositorNodeRLayers")
    gl = nt.nodes.new("CompositorNodeGlare")
    gl.glare_type = "FOG_GLOW"
    gl.quality = "HIGH"
    gl.inputs["Threshold"].default_value = 3.0
    gl.inputs["Strength"].default_value = 0.55
    gl.inputs["Size"].default_value = 0.55
    comp = nt.nodes.new("CompositorNodeComposite")
    nt.links.new(rl.outputs["Image"], gl.inputs["Image"])
    nt.links.new(gl.outputs["Image"], comp.inputs["Image"])
    rl.location, gl.location, comp.location = (0, 0), (300, 0), (600, 0)
