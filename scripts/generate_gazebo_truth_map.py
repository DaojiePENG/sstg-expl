#!/usr/bin/env python3
"""Rasterize axis/planar-rotated SDF box collisions into evaluator truth."""
from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUNDLE = (
    ROOT / "ros2_ws/src/sstg_gazebo/worlds/development/"
    "multi_room_office/dev_office_01"
)


def pose(element):
    node = element.find("pose")
    if node is None or not (node.text or "").strip():
        return (0.0,) * 6
    values = [float(value) for value in node.text.split()]
    if len(values) != 6:
        raise ValueError(f"expected 6-value SDF pose, got {values}")
    return tuple(values)


def compose(parent, child):
    px, py, pz, proll, ppitch, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child
    cosine, sine = math.cos(pyaw), math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        pz + cz,
        proll + croll,
        ppitch + cpitch,
        pyaw + cyaw,
    )


def collision_boxes(world_path, slice_z):
    root = ElementTree.parse(world_path).getroot()
    boxes = []
    for model in root.findall(".//world/model"):
        if (model.findtext("static") or "false").strip().lower() != "true":
            continue
        model_pose = pose(model)
        for link in model.findall("link"):
            link_pose = compose(model_pose, pose(link))
            for collision in link.findall("collision"):
                size_node = collision.find("geometry/box/size")
                if size_node is None:
                    continue
                size = tuple(float(value) for value in size_node.text.split())
                if len(size) != 3:
                    raise ValueError(f"invalid box size in {model.get('name')}")
                box_pose = compose(link_pose, pose(collision))
                z_min = box_pose[2] - size[2] / 2.0
                z_max = box_pose[2] + size[2] / 2.0
                if z_min <= slice_z <= z_max:
                    boxes.append({
                        "model": model.get("name"),
                        "x": box_pose[0],
                        "y": box_pose[1],
                        "yaw": box_pose[5],
                        "width": size[0],
                        "height": size[1],
                    })
    return boxes


def validate_world_registry(world_path, metadata, targets, tolerance=1e-6):
    """Fail closed when evaluator registries drift from their SDF world."""
    root = ElementTree.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{world_path} does not contain a world")
    if world.get("name") != metadata["world_id"]:
        raise ValueError(
            f"world name {world.get('name')!r} does not match "
            f"world_id {metadata['world_id']!r}"
        )

    models = {model.get("name"): model for model in world.findall("model")}
    registered = {entry["target_id"] for entry in targets["targets"]}
    modeled = {name for name in models if name.startswith("target_panel_")}
    if registered != modeled:
        raise ValueError(
            "target registry/model mismatch: "
            f"registered={sorted(registered)}, modeled={sorted(modeled)}"
        )

    for entry in targets["targets"]:
        model = models[entry["target_id"]]
        x, y, z, _, _, yaw = pose(model)
        expected_xyz = (entry["x_m"], entry["y_m"], entry["z_m"])
        if any(
            not math.isclose(actual, float(expected), abs_tol=tolerance)
            for actual, expected in zip((x, y, z), expected_xyz)
        ):
            raise ValueError(
                f"target pose drift for {entry['target_id']}: "
                f"SDF={(x, y, z)}, registry={expected_xyz}"
            )
        expected_yaw = math.radians(float(entry["surface_normal_yaw_deg"]))
        yaw_error = math.atan2(
            math.sin(yaw - expected_yaw), math.cos(yaw - expected_yaw)
        )
        if not math.isclose(yaw_error, 0.0, abs_tol=tolerance):
            raise ValueError(
                f"target normal drift for {entry['target_id']}: "
                f"SDF yaw={yaw}, registry yaw={expected_yaw}"
            )
        size_node = model.find("link/visual/geometry/box/size")
        if size_node is None:
            raise ValueError(f"target {entry['target_id']} is not a box panel")
        size = tuple(float(value) for value in size_node.text.split())
        if len(size) != 3 or not size[0] < min(size[1:]):
            raise ValueError(
                f"target {entry['target_id']} must be thin along local +X"
            )


def rasterize(metadata, boxes):
    resolution = float(metadata["truth_resolution_m"])
    width_m = float(metadata["dimensions_m"]["width"])
    height_m = float(metadata["dimensions_m"]["height"])
    origin_x = float(metadata["origin_m"]["x"])
    origin_y = float(metadata["origin_m"]["y"])
    width = int(round(width_m / resolution))
    height = int(round(height_m / resolution))
    columns = origin_x + (np.arange(width) + 0.5) * resolution
    rows = origin_y + (np.arange(height) + 0.5) * resolution
    xx, yy = np.meshgrid(columns, rows)
    occupied = np.zeros((height, width), dtype=bool)
    for box in boxes:
        cosine, sine = math.cos(box["yaw"]), math.sin(box["yaw"])
        dx, dy = xx - box["x"], yy - box["y"]
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        occupied |= (
            (np.abs(local_x) <= box["width"] / 2.0 + 1e-12)
            & (np.abs(local_y) <= box["height"] / 2.0 + 1e-12)
        )
    return occupied, resolution, (origin_x, origin_y)


def write_pgm(path, occupied):
    # ROS map_server treats the first image row as the maximum world Y, while
    # the evaluator grid stores row zero at origin Y.
    image = np.where(np.flipud(occupied), 0, 254).astype(np.uint8)
    header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
    path.write_bytes(header + image.tobytes())


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", nargs="?", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--slice-z", type=float, default=0.15)
    return parser.parse_args()


def main():
    args = parse_args()
    bundle = args.bundle.resolve()
    metadata_path = bundle / "metadata.yaml"
    world_path = bundle / "world.sdf"
    starts_path = bundle / "starts.yaml"
    targets_path = bundle / "targets.yaml"
    scene_path = bundle / "scene.yaml"
    evaluation = bundle / "evaluation"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    targets = yaml.safe_load(targets_path.read_text(encoding="utf-8"))
    validate_world_registry(world_path, metadata, targets)
    boxes = collision_boxes(world_path, args.slice_z)
    if not boxes:
        raise RuntimeError("no collision boxes intersect the evaluator slice")
    occupied, resolution, origin = rasterize(metadata, boxes)
    evaluation.mkdir(parents=True, exist_ok=True)
    pgm_path = evaluation / "truth_map.pgm"
    yaml_path = evaluation / "truth_map.yaml"
    manifest_path = evaluation / "truth_map_manifest.yaml"
    write_pgm(pgm_path, occupied)
    yaml_path.write_text(yaml.safe_dump({
        "image": "truth_map.pgm",
        "resolution": resolution,
        "origin": [origin[0], origin[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }, sort_keys=False), encoding="utf-8")
    manifest = {
        "schema": "sstg_system_sim_truth_map/v1",
        "world_id": metadata["world_id"],
        "source_world": "../world.sdf",
        "slice_z_m": args.slice_z,
        "resolution_m": resolution,
        "origin_m": list(origin),
        "shape": [int(occupied.shape[0]), int(occupied.shape[1])],
        "occupied_cells": int(np.sum(occupied)),
        "free_cells": int(np.sum(~occupied)),
        "collision_models": [box["model"] for box in boxes],
        "target_count": len(targets["targets"]),
        "sha256": {
            "world.sdf": sha256(world_path),
            "metadata.yaml": sha256(metadata_path),
            "starts.yaml": sha256(starts_path),
            "targets.yaml": sha256(targets_path),
            "truth_map.pgm": sha256(pgm_path),
            "truth_map.yaml": sha256(yaml_path),
        },
    }
    if scene_path.is_file():
        manifest["generated_scene_spec"] = "../scene.yaml"
        manifest["sha256"]["scene.yaml"] = sha256(scene_path)
    manifest_path.write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    checksums = evaluation / "checksums.sha256"
    checked = [
        pgm_path,
        yaml_path,
        manifest_path,
        world_path,
        metadata_path,
        starts_path,
        targets_path,
    ]
    if scene_path.is_file():
        checked.append(scene_path)
    checksums.write_text("".join(
        f"{sha256(path)}  {path.relative_to(bundle)}\n" for path in checked
    ), encoding="utf-8")
    print(yaml.safe_dump(manifest, sort_keys=False))


if __name__ == "__main__":
    main()
