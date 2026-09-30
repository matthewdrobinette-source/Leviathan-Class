"""Build the Leviathan-class Rev H model as a .blend.

    python3 model/build_blend.py [output.blend]

Needs Blender's Python module (pip install bpy==4.5.4). Every surface comes
from the Annex B datum set in lev_geom.py; nothing is placed by hand.
"""
from __future__ import annotations

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bpy  # noqa: E402
import numpy as np  # noqa: E402

import lev_geom as G  # noqa: E402
import lev_mesh as LM  # noqa: E402
import lev_hull as H  # noqa: E402
import lev_interior as I  # noqa: E402
import lev_materials as MT  # noqa: E402
import lev_fittings as F  # noqa: E402
import lev_cores as LC  # noqa: E402
import lev_details as DT  # noqa: E402
import lev_scene as SC  # noqa: E402

T0 = time.time()
D2R = G.D2R


def log(*a):
    print(f"[{time.time() - T0:7.1f}s]", *a, flush=True)


def new_coll(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or bpy.context.scene.collection).children.link(c)
    return c


def apply_boolean(ob, cutter, op, use_self=False):
    m = ob.modifiers.new("b", "BOOLEAN")
    m.operation, m.solver, m.object = op, "EXACT", cutter
    m.use_self = use_self
    m.material_mode = "TRANSFER"
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.remove(m)
    ob.data = me
    me.name = ob.name
    H.mark_sharp(me)
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return ob


def copy_object(ob, name, coll):
    o = ob.copy()
    o.data = ob.data.copy()
    o.name = name
    coll.objects.link(o)
    return o


def mesh_obj(name, me, coll, mats=()):
    for m in mats:
        me.materials.append(MT.MATS[m])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def build(out_path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.name = "Leviathan-class, Rev H"
    MT.build_all()
    log("materials", len(MT.MATS))

    root = new_coll("LEVIATHAN")
    c_slabs = new_coll("Hull slabs (B.12 de-stack layers)", root)
    c_cores = new_coll("Cores", root)
    c_collar = new_coll("Bridge collar and iris shutters", root)
    c_fit = new_coll("Fittings (batteries, sensors, emitters)", root)
    c_open = new_coll("Openings, lighting and markings", root)
    c_drive = new_coll("Drive emitter", root)
    c_screen = new_coll("Cavity screen", root)
    c_booms = new_coll("Umbilical booms", root)
    c_bowl = new_coll("Bowl keys, dogs and repulsor arrays", root)
    c_cont = new_coll("Containment spheres", root)
    c_ghost = new_coll("Interior volumes (ghost view)", root)
    c_ctrl = new_coll("Controls, cameras and lights")
    c_env = new_coll("Environment")
    c_src = new_coll("Instance sources (hidden)")
    c_build = new_coll("Build cutters (hidden)")

    # ------------------------------------------------------------ hull
    me = H.build_hull_mesh("Hull")
    slot_names = ["Hull plate, dorsal", "Hull plate, rim", "Hull plate, ventral", "Hull plate, cavity",
                  "Bowl and well, alusteel", "Bowl and well, alusteel", "Collar well, alusteel", "Hull plate, dorsal"]
    # unique slots: the EXACT boolean's material transfer does not remap duplicate slots
    uniq = list(dict.fromkeys(slot_names))
    idx = np.zeros(len(me.polygons), np.int32)
    me.polygons.foreach_get("material_index", idx)
    remap = np.array([uniq.index(n) for n in slot_names], np.int32)
    me.polygons.foreach_set("material_index", remap[idx])
    hull = mesh_obj("Hull (assembled)", me, c_build, uniq)
    lc = mesh_obj("Cutter, Lance bowls and wells", H.lance_cutter_mesh(), c_build, ["Bowl and well, alusteel"])
    voids, hard, soft = I.cutter_sets("model")
    vm = I.volumes_mesh("Cutter, voids", voids)
    vcut = mesh_obj("Cutter, voids", vm, c_build, ["Gallery interior, alusteel joiner"])
    cats = list(G.CATEGORIES.keys())
    hard_cut = [v for v in hard if not v.get("parent")]
    hm = I.volumes_mesh("Cutter, hard volumes", hard_cut, mat_of=lambda v: cats.index(v["cat"]), shrink=0.05)
    hcut = mesh_obj("Cutter, hard volumes", hm, c_build, [f"Volume, {G.CATEGORIES[k][0]}" for k in cats])
    far = LM.mesh_from_arrays("far", LM.box_mesh_verts((0, 0, -9000), (1, 0, 0), (0, 1, 0), (0, 0, 1), 1, 1, 1), LM.BOX_FACES)
    farob = mesh_obj("far", far, c_build, ["Gallery interior, alusteel joiner"])
    apply_boolean(vcut, farob, "DIFFERENCE", use_self=True)
    log("cutters ready")
    apply_boolean(hull, lc, "DIFFERENCE")
    log("Lance bowls and wells cut")
    apply_boolean(hull, vcut, "DIFFERENCE")
    log("galleries, spurs, lifts, mouths and command voids cut")
    apply_boolean(hull, hcut, "DIFFERENCE")
    log("hard allocation cut", len(hull.data.polygons), "faces")

    # ------------------------------------------------------------ slabs
    pb = LM.PatchBuilder()
    plan = [G.xy(1000.0, t) for t in np.linspace(135, 225, 97)] + [G.xy(2600.0, t) for t in np.linspace(225, 135, 97)]
    I.prism(pb, plan, -900.0, 400.0)
    aftc = mesh_obj("Cutter, aft block", pb.build("aft prism", smooth=False), c_build, ["Section"])
    aftc.data.polygons.foreach_set("use_smooth", np.zeros(len(aftc.data.polygons), bool))
    slabs = {}
    aft = copy_object(hull, "Slab, aft block (draws aft)", c_slabs)
    apply_boolean(aft, aftc, "INTERSECT")
    slabs["aft"] = aft
    rest = copy_object(hull, "rest", c_build)
    apply_boolean(rest, aftc, "DIFFERENCE")
    log("aft block split")
    pb = LM.PatchBuilder()
    pb.add_grid(LM.lathe_local([(0.0, -10.0), (450.0, -10.0), (450.0, 400.0), (0.0, 400.0)], 256), wrap_u=True)
    capc = mesh_obj("Cutter, collar and cap", pb.build("cap cyl", smooth=False), c_build, ["Section"])
    capc.data.polygons.foreach_set("use_smooth", np.zeros(len(capc.data.polygons), bool))
    names = {"ventral": "Slab, ventral traffic and shell (draws ventral)", "citadel": "Slab, citadel and endurance plant",
             "lowergallery": "Slab, lower gallery", "handling": "Slab, handling and stores", "uppergallery": "Slab, upper gallery",
             "berthing": "Slab, berthing and air group", "dorsal": "Slab, dorsal shell and battery band"}
    for lid, z0, z1 in F.SLAB_Z:
        z0c, z1c = max(z0, -900.0), min(z1, 400.0)
        bm = LM.mesh_from_arrays("band", LM.box_mesh_verts((0, 0, (z0c + z1c) / 2), (1, 0, 0), (0, 1, 0), (0, 0, 1), 2700, 2700, (z1c - z0c) / 2), LM.BOX_FACES, smooth=False)
        band = mesh_obj(f"Cutter, band {lid}", bm, c_build, ["Section"])
        s = copy_object(rest, names[lid], c_slabs)
        apply_boolean(s, band, "INTERSECT")
        if lid == "dorsal":
            cap = copy_object(s, "Slab, collar and fixed cap", c_slabs)
            apply_boolean(cap, capc, "INTERSECT")
            slabs["cap"] = cap
            apply_boolean(s, capc, "DIFFERENCE")
        slabs[lid] = s
        log("slab", lid, len(s.data.polygons))
    bpy.data.objects.remove(rest)
    # the assembled hull is what renders while the stack is closed (no seams);
    # the slabs take over when the stack opens
    hull.name = "Hull (assembled)"
    c_build.objects.unlink(hull)
    c_slabs.objects.link(hull)
    DESTACK_K = {"ventral": -1, "citadel": 1, "lowergallery": 2, "handling": 3, "uppergallery": 4, "berthing": 5, "dorsal": 6, "cap": 7}

    # ------------------------------------------------------------ controls and section
    SC.make_controls(c_ctrl)
    piv, cutters = SC.section_cutters(c_ctrl)
    sdel = F.section_delete_group()
    SC.add_section_boolean(hull, cutters["hull"])
    SC.drive_visibility(hull, "D > 0.001", {"D": "destack"})
    for lid, ob in slabs.items():
        SC.drive_visibility(ob, "D <= 0.001", {"D": "destack"})
        if lid == "aft":
            SC.drive(ob, "location", "-600 * D", {"D": "destack"}, index=1)
        else:
            SC.drive(ob, "location", f"{150 * DESTACK_K[lid]} * D", {"D": "destack"}, index=2)
        SC.add_section_boolean(ob, cutters["hull"])
        ob["de_stack_layer"] = lid
    log("slabs driven")

    # ------------------------------------------------------------ interior display
    def cat_mesh(name, vols):
        return I.volumes_mesh(name, vols, mat_of=lambda v: cats.index(v["cat"]))
    ghost = mesh_obj("Interior volumes, hard allocation", cat_mesh("ghost", [v for v in hard if not v.get("parent")]),
                     c_ghost, [f"Volume, {G.CATEGORIES[k][0]}" for k in cats])
    SC.add_section_boolean(ghost, cutters["hull"])
    SC.drive_visibility(ghost, "D > 0.001", {"D": "destack"})
    gv = mesh_obj("Interior volumes, voids", I.volumes_mesh("ghost voids", voids), c_ghost, ["Volume, Galleries, spurs, lifts, mouths"])
    SC.add_section_boolean(gv, cutters["hull"])
    SC.drive_visibility(gv, "D > 0.001", {"D": "destack"})
    c_ghost.hide_render = True
    for v in [v for v in hard if v.get("parent")]:
        x, y = I.spine_xy(v["th"], v["A"][0], 0.0)
        r, t = G.polar(x, y)
        lid = "aft" if (G.in_aft_arc(t) and r >= 1000) else "citadel"
        sm = I.volumes_mesh("sphere", [v])
        ob = mesh_obj(f"{v['name']} {t:.0f}°", sm, c_cont, ["Volume, Hypermatter containment"])
        ob.parent = slabs[lid]
        SC.add_section_boolean(ob, cutters["hull"])
    log("interior volumes")

    # ------------------------------------------------------------ cores
    cts = {False: LC.CoreType(False), True: LC.CoreType(True)}
    core_mesh, core_fit = {}, {}
    for k, ct in cts.items():
        core_mesh[k], core_fit[k] = LC.build_core_mesh(ct, c_build)
    log("core meshes", {k: len(m.polygons) for k, m in core_mesh.items()})
    core_objs = []
    for c in G.CORES:
        ob = bpy.data.objects.new(c.name, core_mesh[c.central])
        c_cores.objects.link(ob)
        ob.location = (c.x, c.y, c.zc)
        ob.rotation_euler = (0, 0, (90.0 if c.central else 90.0 - c.th) * D2R)
        ob["station"] = f"r {c.r:.0f}, theta {c.th:.1f}, z {c.zc:.0f}; diameter {2 * c.rc:.0f} m"
        ob["release_pair"] = c.pair
        p0 = (c.pair - 1) * 0.2
        SC.drive(ob, "location", f"{c.zc} - 900 * D - 1500 * {SC.phase(p0, p0 + 0.2)}", {"D": "destack", "R": "core_release"}, index=2)
        SC.add_section_boolean(ob, cutters["praetorian" if c.central else "lance"])
        fit = bpy.data.objects.new(c.name + " fit-out", core_fit[c.central])
        c_cores.objects.link(fit)
        fit.parent = ob
        SC.add_section_delete(fit, sdel)
        des = DT.core_designation(c.name, c.rc, c_cores)
        des.parent = ob
        SC.add_section_delete(des, sdel)
        core_objs.append(ob)
    log("cores placed")

    # ------------------------------------------------------------ collar, shutters
    col, glz, beacon = DT.collar(c_collar)
    col.parent = slabs["cap"]
    SC.drive(col, "location", f"({SC.phase(0.3, 1.0, 'C')} - 1) * {DT.COLLAR_TRAVEL}", {"C": "collar_extended"}, index=2)
    for o in (col, glz):
        SC.add_section_boolean(o, cutters["hull"])
    SC.add_section_delete(beacon, sdel)
    leaves = DT.shutters(c_collar)
    for lf in leaves:
        lf.parent = slabs["cap"]
        b = lf["bearing"] * D2R
        d = f"212 * {SC.phase(0.1, 0.3, 'C')}"
        SC.drive(lf, "location", f"{math.sin(b):.6f} * {d}", {"C": "collar_extended"}, index=0)
        SC.drive(lf, "location", f"{math.cos(b):.6f} * {d}", {"C": "collar_extended"}, index=1)
        SC.drive(lf, "location", f"-16 * {SC.phase(0.0, 0.1, 'C')}", {"C": "collar_extended"}, index=2)
        SC.add_section_boolean(lf, cutters["hull"])
    log("collar and iris shutters")

    # ------------------------------------------------------------ drive emitter, screen, booms, bowls
    em = DT.drive_emitter(c_drive)
    em.parent = slabs["aft"]
    SC.add_section_boolean(em, cutters["hull"])
    scr = DT.cavity_screens(c_screen)
    for key, q in (("attenuating", 1), ("hardened", 2), ("inboard", 3)):
        SC.drive_visibility(scr[key], f"max(Q != {q}, D > 0.001)", {"Q": "screen_state", "D": "destack"})
        SC.add_section_delete(scr[key], sdel)
        scr[key].visible_shadow = False
    stowed, deployed = DT.booms(c_booms)
    for o in stowed:
        SC.drive_visibility(o, "max(B >= 1, D > 0.001)", {"B": "booms_deployed", "D": "destack"})
        SC.add_section_delete(o, sdel)
    for o in deployed:
        SC.drive_visibility(o, "max(B < 1, D > 0.001)", {"B": "booms_deployed", "D": "destack"})
        SC.add_section_delete(o, sdel)
    for o in DT.bowl_fittings(c_bowl, cts):
        SC.drive_visibility(o, "D > 0.001", {"D": "destack"})
        SC.add_section_delete(o, sdel)
    log("emitter, screen, booms, bowls")

    # ------------------------------------------------------------ openings, lights, markings
    op = DT.openings(c_open)
    SC.drive_visibility(op["mouth_shutters"], "max(M >= 1, D > 0.001)", {"M": "mouths_open", "D": "destack"})
    for k, o in op.items():
        if k != "mouth_shutters":
            SC.drive_visibility(o, "D > 0.001", {"D": "destack"})
        SC.add_section_delete(o, sdel)
    mk = DT.markings(c_open)
    mk["name"].parent = slabs["dorsal"]
    mk["division"].parent = slabs["aft"]
    mk["badge"].parent = slabs["cap"]
    for k, o in mk.items():
        if k not in ("name", "division", "badge"):
            SC.drive_visibility(o, "D > 0.001", {"D": "destack"})
        SC.add_section_delete(o, sdel)
    log("openings, lights, markings")

    # ------------------------------------------------------------ fittings
    ng = F.instancer_group()
    ids = {it.name: it.identifier for it in ng.interface.items_tree if it.item_type == "SOCKET" and it.in_out == "INPUT"}
    srcs = F.build_sources(c_src)
    clouds = F.build_clouds()
    counts = {}
    for key, cl in clouds.items():
        label, stroke, role = F.FITTING_SPECS[key]
        counts[key] = cl.count()
        for lid, items in cl.pts.items():
            if not items:
                continue
            ob = bpy.data.objects.new(f"Fittings, {label} [{lid}]", F.cloud_mesh(f"{key} {lid}", items))
            c_fit.objects.link(ob)
            ob.parent = slabs[lid]
            m = ob.modifiers.new("Fittings", "NODES")
            m.node_group = ng
            mount, stow, opn = srcs[key]
            m[ids["Mount"]] = mount
            if stow:
                m[ids["Stowed"]] = stow
            if opn:
                m[ids["Open"]] = opn
            m[ids["Stroke"]] = stroke
            if role == "batteries":
                SC.drive(ob, f'modifiers["Fittings"]["{ids["Deploy"]}"]', "B", {"B": "batteries_deployed"})
            elif role == "shields":
                SC.drive(ob, f'modifiers["Fittings"]["{ids["Deploy"]}"]', "S", {"S": "shields_energised"})
            else:
                m[ids["Deploy"]] = 1.0
                m[ids["Stroke"]] = 0.0
            SC.drive(ob, f'modifiers["Fittings"]["{ids["Section"]}"]', "S >= 1", {"S": "section"})
            SC.drive(ob, f'modifiers["Fittings"]["{ids["Section Normal"]}"]', f"cos(A * {D2R})", {"A": "section_longitude"}, index=0)
            SC.drive(ob, f'modifiers["Fittings"]["{ids["Section Normal"]}"]', f"-sin(A * {D2R})", {"A": "section_longitude"}, index=1)
            ob["count"] = len(items)
    for o in c_src.objects:
        o.hide_render = True
        o.hide_viewport = True
    log("fittings", counts)

    # ------------------------------------------------------------ scene
    SC.bind_material_drivers()
    SC.make_cameras(c_ctrl)
    sun, fill, floods = SC.lights(c_ctrl)
    for o in floods:
        o.parent = slabs["citadel"]
    scene.world = MT.world_space()
    R = 6.371e6
    pbp = LM.PatchBuilder()
    pbp.add_grid(LM.lathe_local([(R * math.sin(a), R * math.cos(a)) for a in np.linspace(0, math.pi, 97)], 192), wrap_u=True)
    planet = mesh_obj("Planet (campaign posture, 300 km below)", pbp.build("Planet"), c_env, ["Planet surface"])
    planet.location = (0, 0, -300e3 - R)
    planet.hide_render = True
    planet.hide_viewport = True
    SC.render_settings(scene)
    scene.camera = bpy.data.objects["CAM 01 Dorsal three-quarter"]
    for c in (c_build, c_src):
        c.hide_render = True
        c.hide_viewport = True
    for o in c_build.objects:
        o.hide_render = True
        o.hide_viewport = True
    # README and states script in the file
    txt = bpy.data.texts.new("README")
    txt.write(README)
    st = bpy.data.texts.new("configuration_states.py")
    st.write(STATES_SCRIPT)
    if SC.SIMPLE_FAIL:
        log("WARNING drivers needing Python:", SC.SIMPLE_FAIL[:5], len(SC.SIMPLE_FAIL))
    else:
        log("all drivers are simple expressions (no script auto-run needed)")
    # viewport clip for a 3.5 km ship
    scene["fitting_counts"] = str(counts)
    bpy.ops.wm.save_as_mainfile(filepath=out_path, compress=True)
    log("saved", out_path, f"{os.path.getsize(out_path) / 1e6:.1f} MB")
    return counts


README = """LEVIATHAN-CLASS CAPITAL ASSAULT CARRIER, BUILD SPECIFICATION REV H
Parametric model generated from the Annex B datum set (model/lev_geom.py).
Units: 1 Blender unit = 1 m. +Y is the bow, +X starboard, +Z dorsal.
The origin is on the axis at the dorsal rim plane (z = 0).

EXPLORING
- Select the 'LEVIATHAN controls' empty. Object Properties > Custom Properties
  has every configuration control as a slider:
    collar_extended     0 stowed under the iris shutters, 1 crown at +100
    batteries_deployed  591 heavy, 746 medium, ~1,240 PD and 640 cavity mounts rise
    shields_energised   the 24 hull ring emitters stand 8 m proud and light up
    screen_state        0 off, 1 attenuating, 2 hardened, 3 drawn inboard
    mouths_open         outer shutters of the sixteen launch mouths
    booms_deployed      sixteen umbilical booms telescoped to 900 m below the rim plane
    core_release        opposed pairs One/Five .. Four/Eight, then Praetorian
    destack             Annex B.12 exploded view
    section             cross-section through the axis; section_longitude turns it
    lighting            0 action (dark), 1 cruising, 2 flight operations
    drive_throttle      emitter glow
- Text Editor > configuration_states.py > Run Script sets the Section 17.1
  states (Cruising, Action, Deployed).
- Cameras 'CAM 01' to 'CAM 13' are the rendered views (Numpad 0 to look through).
- Collection 'Interior volumes (ghost view)': hide the hull slabs and show this
  to see the Annex B allocation as colour-coded volumes.
- The section runs exact booleans on the hull slabs; expect a few seconds per change.
- In the viewport, set View > Clip End to 50 km or more.

Every driver is a simple expression, so no script auto-run is needed.
"""

STATES_SCRIPT = '''import bpy
# Section 17.1 configuration states. Set STATE and press Run Script.
STATE = "Action"   # "Cruising", "Action" or "Deployed"
STATES = {
    "Cruising": dict(collar_extended=1.0, batteries_deployed=0.0, shields_energised=0.0, screen_state=1,
                     mouths_open=1, booms_deployed=0, core_release=0.0, lighting=2),
    "Action": dict(collar_extended=0.0, batteries_deployed=1.0, shields_energised=1.0, screen_state=2,
                   mouths_open=0, booms_deployed=0, core_release=0.0, lighting=0),
    "Deployed": dict(collar_extended=1.0, batteries_deployed=0.0, shields_energised=1.0, screen_state=3,
                     mouths_open=1, booms_deployed=0, core_release=1.0, lighting=1),
}
ctrl = bpy.data.objects["LEVIATHAN controls"]
for k, v in STATES[STATE].items():
    ctrl[k] = v
ctrl.update_tag()
bpy.context.view_layer.update()
'''

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "blender", "Leviathan_RevH.blend")
    build(os.path.abspath(out))
