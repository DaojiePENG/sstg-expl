#!/usr/bin/env python3
"""Generate a deterministic Gazebo world bundle from ``scene.yaml``.

The generator is intentionally limited to static boxes and registered task
panels.  That restriction keeps collision geometry, evaluator truth and the
visual scene derived from one auditable specification.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORLDS_ROOT = ROOT / "ros2_ws/src/sstg_gazebo/worlds/development"
ROBOT_CLEARANCE_M = 0.24


def _finite(value, label):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _text(parent, tag, value):
    child = ET.SubElement(parent, tag)
    child.text = str(value)
    return child


def _pose_text(x, y, z, yaw_rad=0.0):
    return f"{x:.6g} {y:.6g} {z:.6g} 0 0 {yaw_rad:.9g}"


def _size_text(values):
    return " ".join(f"{float(value):.6g}" for value in values)


def _material(visual, color):
    rgba = [float(value) for value in color] + [1.0]
    material = ET.SubElement(visual, "material")
    _text(material, "ambient", _size_text(rgba))
    diffuse = [min(1.0, value * 1.15) for value in rgba[:3]] + [1.0]
    _text(material, "diffuse", _size_text(diffuse))


def _box_model(world, spec, *, collision=True):
    model = ET.SubElement(world, "model", {"name": str(spec["id"])})
    _text(model, "static", "true")
    yaw = math.radians(_finite(spec.get("yaw_deg", 0.0), "box yaw"))
    _text(
        model,
        "pose",
        _pose_text(
            _finite(spec["x_m"], "box x"),
            _finite(spec["y_m"], "box y"),
            _finite(spec["z_m"], "box z"),
            yaw,
        ),
    )
    size = [_finite(value, "box size") for value in spec["size_m"]]
    if len(size) != 3 or min(size) <= 0.0:
        raise ValueError(f"box {spec['id']} requires three positive sizes")
    link = ET.SubElement(model, "link", {"name": "body"})
    if collision:
        collision_node = ET.SubElement(link, "collision", {"name": "collision"})
        geometry = ET.SubElement(collision_node, "geometry")
        _text(ET.SubElement(geometry, "box"), "size", _size_text(size))
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    _text(ET.SubElement(geometry, "box"), "size", _size_text(size))
    _material(visual, spec.get("color_rgb", [0.55, 0.55, 0.55]))


def _add_system_plugins(world):
    plugins = (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-contact-system", "gz::sim::systems::Contact"),
    )
    for filename, name in plugins:
        plugin = ET.SubElement(world, "plugin", {"filename": filename, "name": name})
        if name == "gz::sim::systems::Sensors":
            _text(plugin, "render_engine", "ogre2")


def _add_light(world):
    light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    _text(light, "cast_shadows", "true")
    _text(light, "pose", "0 0 10 0 0 0")
    _text(light, "diffuse", "0.85 0.85 0.85 1")
    _text(light, "specular", "0.2 0.2 0.2 1")
    attenuation = ET.SubElement(light, "attenuation")
    _text(attenuation, "range", "1000")
    _text(attenuation, "constant", "0.9")
    _text(attenuation, "linear", "0.01")
    _text(attenuation, "quadratic", "0.001")
    _text(light, "direction", "-0.4 0.2 -0.9")


def _boundary_boxes(width, height):
    return [
        {"id": "wall_north", "x_m": 0, "y_m": height / 2 - 0.1,
         "z_m": 1.25, "size_m": [width, 0.2, 2.5]},
        {"id": "wall_south", "x_m": 0, "y_m": -height / 2 + 0.1,
         "z_m": 1.25, "size_m": [width, 0.2, 2.5]},
        {"id": "wall_west", "x_m": -width / 2 + 0.1, "y_m": 0,
         "z_m": 1.25, "size_m": [0.2, height, 2.5]},
        {"id": "wall_east", "x_m": width / 2 - 0.1, "y_m": 0,
         "z_m": 1.25, "size_m": [0.2, height, 2.5]},
    ]


def _validate_scene(scene):
    if scene.get("schema") != "sstg_system_sim_scene_spec/v1":
        raise ValueError("scene.yaml has an unsupported schema")
    world_id = str(scene.get("world_id", "")).strip()
    if not world_id:
        raise ValueError("world_id is required")
    dimensions = scene.get("dimensions_m", {})
    width = _finite(dimensions.get("width"), "world width")
    height = _finite(dimensions.get("height"), "world height")
    if width <= 2.0 or height <= 2.0:
        raise ValueError("world dimensions are too small")
    identifiers = [str(box.get("id", "")) for box in scene.get("boxes", [])]
    if len(identifiers) != len(set(identifiers)) or not all(identifiers):
        raise ValueError("box IDs must be non-empty and unique")
    targets = scene.get("targets", [])
    target_ids = [str(target.get("target_id", "")) for target in targets]
    if not targets or len(target_ids) != len(set(target_ids)) or not all(target_ids):
        raise ValueError("target IDs must be non-empty and unique")
    starts = scene.get("starts", [])
    if not starts:
        raise ValueError("at least one start is required")
    for start in starts:
        x = _finite(start["x_m"], "start x")
        y = _finite(start["y_m"], "start y")
        _finite(start["yaw_deg"], "start yaw")
        if abs(x) >= width / 2 - ROBOT_CLEARANCE_M or abs(y) >= height / 2 - ROBOT_CLEARANCE_M:
            raise ValueError(f"start {start['start_id']} is too close to the boundary")
        for box in scene.get("boxes", []):
            yaw = math.radians(_finite(box.get("yaw_deg", 0), "box yaw"))
            dx, dy = x - float(box["x_m"]), y - float(box["y_m"])
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            sx, sy, _ = [float(value) for value in box["size_m"]]
            if (
                abs(local_x) <= sx / 2 + ROBOT_CLEARANCE_M
                and abs(local_y) <= sy / 2 + ROBOT_CLEARANCE_M
            ):
                raise ValueError(f"start {start['start_id']} intersects {box['id']}")
    return world_id, width, height


def render_bundle(scene):
    world_id, width, height = _validate_scene(scene)
    sdf = ET.Element("sdf", {"version": "1.10"})
    sdf.append(ET.Comment("Generated from scene.yaml; edit the specification, not this file."))
    world = ET.SubElement(sdf, "world", {"name": world_id})
    physics = ET.SubElement(world, "physics", {"name": "physics", "type": "ignored"})
    _text(physics, "max_step_size", "0.004")
    _text(physics, "real_time_factor", "1.0")
    _text(physics, "real_time_update_rate", "250")
    _text(world, "gravity", "0 0 -9.81")
    scene_node = ET.SubElement(world, "scene")
    _text(scene_node, "ambient", "0.35 0.35 0.38 1")
    _text(scene_node, "background", "0.72 0.78 0.86 1")
    _text(scene_node, "shadows", "true")
    _add_system_plugins(world)
    _add_light(world)
    _box_model(world, {
        "id": "floor", "x_m": 0, "y_m": 0, "z_m": -0.05,
        "size_m": [width, height, 0.1], "color_rgb": [0.58, 0.58, 0.58],
    })
    for box in _boundary_boxes(width, height):
        box["color_rgb"] = [0.76, 0.76, 0.72]
        _box_model(world, box)
    for box in scene.get("boxes", []):
        _box_model(world, box)
    target_colors = (
        [0.9, 0.1, 0.1], [0.1, 0.8, 0.2],
        [0.1, 0.25, 0.9], [0.9, 0.75, 0.1],
    )
    for index, target in enumerate(scene["targets"]):
        _box_model(world, {
            "id": target["target_id"],
            "x_m": target["x_m"],
            "y_m": target["y_m"],
            "z_m": target.get("z_m", 1.05),
            "yaw_deg": target["surface_normal_yaw_deg"],
            "size_m": target.get("size_m", [0.02, 0.35, 0.35]),
            "color_rgb": target.get(
                "color_rgb", target_colors[index % len(target_colors)]
            ),
        }, collision=False)
    ET.indent(sdf, space="  ")
    world_text = ET.tostring(sdf, encoding="unicode", xml_declaration=True) + "\n"

    metadata = {
        "schema": "sstg_system_sim_world/v1",
        "world_id": world_id,
        "split": "development",
        "backend": "gazebo_harmonic",
        "site_family": scene["site_family"],
        "layout_version": str(scene["layout_version"]),
        "dimensions_m": {"width": width, "height": height},
        "origin_m": {"x": -width / 2, "y": -height / 2},
        "truth_resolution_m": float(scene.get("truth_resolution_m", 0.05)),
        "room_count": int(scene["room_count"]),
        "door_opening_width_m": scene.get("door_opening_width_m", []),
        "furniture_profile": scene["furniture_profile"],
        "truth_access": "evaluator_only",
        "formal_result_eligible": False,
        "generated_from": "scene.yaml",
        "notes": scene["notes"],
    }
    starts = {
        "schema": "sstg_system_sim_starts/v1",
        "world_id": world_id,
        "starts": scene["starts"],
    }
    targets = {
        "schema": "sstg_system_sim_targets/v1",
        "world_id": world_id,
        "targets": scene["targets"],
    }
    return {
        "world.sdf": world_text,
        "metadata.yaml": yaml.safe_dump(metadata, sort_keys=False),
        "starts.yaml": yaml.safe_dump(starts, sort_keys=False),
        "targets.yaml": yaml.safe_dump(targets, sort_keys=False),
    }


def generate(bundle, check=False):
    bundle = bundle.resolve()
    scene_path = bundle / "scene.yaml"
    scene = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    outputs = render_bundle(scene)
    changed = []
    for name, content in outputs.items():
        path = bundle / name
        if not path.exists() or path.read_text(encoding="utf-8") != content:
            changed.append(name)
            if not check:
                path.write_text(content, encoding="utf-8")
    if check and changed:
        raise ValueError(
            f"generated files are stale in {bundle}: {', '.join(changed)}"
        )
    return changed


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", nargs="*", type=Path)
    parser.add_argument("--check", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    bundles = args.bundle or sorted(
        path.parent for path in WORLDS_ROOT.rglob("scene.yaml")
    )
    if not bundles:
        print("no scene.yaml bundles found", file=sys.stderr)
        return 2
    try:
        for bundle in bundles:
            changed = generate(bundle, check=args.check)
            print(f"{bundle}: {'updated ' + ', '.join(changed) if changed else 'up to date'}")
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
