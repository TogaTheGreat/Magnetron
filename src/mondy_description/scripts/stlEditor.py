#!/usr/bin/env python3
"""
stl_tool.py - A practical Trimesh-based STL inspector/editor/viewer CLI.

Designed for ROS/URDF mesh cleanup:
- inspect bounds/extents/center
- guess likely units from extents
- scale meshes to meters
- translate meshes
- set bounds min to 0,0,0
- center origin at bounding-box center
- move origin to a chosen point
- rotate meshes
- export clean STL copies
- optionally view meshes using trimesh's viewer
- generate URDF <visual> snippets

Install:
    python3 -m pip install trimesh numpy

Examples:
    python3 stl_tool.py inspect meshes/mg995.STL
    python3 stl_tool.py inspect meshes/*.stl --guess-units

    # Convert mm STL to meter STL, then move min corner to 0,0,0
    python3 stl_tool.py edit meshes/mg995.STL --scale 0.001 --min0 -o meshes/mg995_min0_m.stl

    # Convert inch STL to meter STL
    python3 stl_tool.py edit meshes/servobracketu.STL --scale 0.0254 --min0 -o meshes/servobracketu_min0_m.stl

    # Center origin at bounding-box center after scaling
    python3 stl_tool.py edit part.stl --scale 0.001 --center-origin -o part_centered_m.stl

    # Translate by x y z, in current mesh units after scale/rotation
    python3 stl_tool.py edit part.stl --scale 0.001 --translate 0.01 0 0 -o part_shifted.stl

    # Rotate 90 degrees around Z
    python3 stl_tool.py edit part.stl --rotate-deg 0 0 90 -o part_rotated.stl

    # Measure a known point relative to current min corner / center
    python3 stl_tool.py inspect part.stl --known-point shaft 26.4 21.1 10

    # View one or more meshes
    python3 stl_tool.py view meshes/mg995_min0_m.stl meshes/servobracketu_min0_m.stl

    # Generate a visual snippet
    python3 stl_tool.py urdf-visual --name servo --filename package://mondy_description/meshes/mg995_min0_m.stl --xyz 0 0 0 --rpy 0 0 0
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh


def fmt_vec(v: Iterable[float]) -> str:
    a = np.asarray(v, dtype=float)
    return "[" + ", ".join(f"{x:.9g}" for x in a.tolist()) + "]"


def load_mesh(path: Path) -> trimesh.Trimesh:
    try:
        loaded = trimesh.load(path)
    except Exception as exc:
        raise SystemExit(f"ERROR: failed to load {path}: {exc}") from exc

    if isinstance(loaded, trimesh.Scene):
        if not loaded.geometry:
            raise SystemExit(f"ERROR: {path} loaded as an empty scene")
        mesh = trimesh.util.concatenate(tuple(loaded.geometry.values()))
    elif isinstance(loaded, trimesh.Trimesh):
        mesh = loaded
    else:
        try:
            mesh = trimesh.load_mesh(path, force="mesh")
        except Exception as exc:
            raise SystemExit(f"ERROR: {path} did not load as a mesh: {exc}") from exc

    if mesh is None or mesh.is_empty:
        raise SystemExit(f"ERROR: {path} loaded as empty mesh")

    return mesh


def print_mesh_info(path: Path, mesh: trimesh.Trimesh, guess_units: bool = False) -> None:
    bounds = mesh.bounds
    extents = mesh.extents
    center = bounds.mean(axis=0)

    print(f"\nFILE: {path}")
    print(f"vertices:         {len(mesh.vertices)}")
    print(f"faces:            {len(mesh.faces)}")
    print(f"watertight:       {mesh.is_watertight}")
    print(f"bounds min:       {fmt_vec(bounds[0])}")
    print(f"bounds max:       {fmt_vec(bounds[1])}")
    print(f"extents:          {fmt_vec(extents)}")
    print(f"bbox center:      {fmt_vec(center)}")
    print(f"centroid:         {fmt_vec(mesh.centroid)}")
    if mesh.is_watertight:
        print(f"volume:           {mesh.volume:.9g}")
    else:
        print("volume:           unavailable/non-watertight")
    print(f"surface area:     {mesh.area:.9g}")

    if guess_units:
        print_guess(extents)


def print_guess(extents: np.ndarray) -> None:
    largest = float(np.max(extents))
    print("unit guess:")
    if largest > 100:
        print("  likely raw units: millimeters or weird large CAD units")
        print("  try scale: 0.001 if expected dimensions are tens/hundreds of mm")
    elif 10 <= largest <= 100:
        print("  likely raw units: millimeters")
        print("  try scale: 0.001 for ROS/URDF meters")
    elif 1.0 <= largest < 10:
        print("  likely raw units: inches OR already huge meters")
        print("  try scale: 0.0254 if expected dimension is a few inches")
    elif 0.01 <= largest < 1.0:
        print("  likely raw units: meters")
        print("  try scale: 1.0")
    else:
        print("  very small model; check export units manually")


def rotation_matrix_xyz_deg(rx: float, ry: float, rz: float) -> np.ndarray:
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    mx = trimesh.transformations.rotation_matrix(rx, [1, 0, 0])
    my = trimesh.transformations.rotation_matrix(ry, [0, 1, 0])
    mz = trimesh.transformations.rotation_matrix(rz, [0, 0, 1])
    # Apply X, then Y, then Z.
    return trimesh.transformations.concatenate_matrices(mz, my, mx)


def auto_output_path(path: Path, suffix: str = "_edited") -> Path:
    return path.with_name(f"{path.stem}{suffix}.stl")


def edit_mesh(args: argparse.Namespace) -> None:
    path = Path(args.input).expanduser()
    mesh = load_mesh(path)

    print_mesh_info(path, mesh, guess_units=args.guess_units)

    # Order:
    # 1 scale
    # 2 rotate
    # 3 translate
    # 4 min/center/origin normalization
    if args.scale is not None:
        s = args.scale
        if len(s) == 1:
            scale = [s[0], s[0], s[0]]
        elif len(s) == 3:
            scale = s
        else:
            raise SystemExit("ERROR: --scale takes either one uniform value or three xyz values")
        mesh.apply_scale(scale)
        print(f"\nApplied scale: {fmt_vec(scale)}")

    if args.rotate_deg is not None:
        rx, ry, rz = args.rotate_deg
        mesh.apply_transform(rotation_matrix_xyz_deg(rx, ry, rz))
        print(f"Applied rotation deg XYZ: {fmt_vec([rx, ry, rz])}")

    if args.translate is not None:
        t = np.asarray(args.translate, dtype=float)
        mesh.apply_translation(t)
        print(f"Applied translation: {fmt_vec(t)}")

    normalization_count = sum(bool(x) for x in [
        args.min0,
        args.center_origin,
        args.origin_at is not None,
    ])
    if normalization_count > 1:
        raise SystemExit("ERROR: choose only one of --min0, --center-origin, or --origin-at")

    if args.min0:
        shift = -mesh.bounds[0]
        mesh.apply_translation(shift)
        print(f"Moved bounds min to 0,0,0 using translation: {fmt_vec(shift)}")

    if args.center_origin:
        shift = -mesh.bounds.mean(axis=0)
        mesh.apply_translation(shift)
        print(f"Moved bbox center to origin using translation: {fmt_vec(shift)}")

    if args.origin_at is not None:
        p = np.asarray(args.origin_at, dtype=float)
        mesh.apply_translation(-p)
        print(f"Moved chosen mesh point to origin using translation: {fmt_vec(-p)}")

    out = Path(args.output).expanduser() if args.output else auto_output_path(path)
    out.parent.mkdir(parents=True, exist_ok=True)

    try:
        mesh.export(out)
    except Exception as exc:
        raise SystemExit(f"ERROR: failed to export {out}: {exc}") from exc

    print_mesh_info(out, mesh, guess_units=False)
    print(f"\nWROTE: {out}")


def inspect_meshes(args: argparse.Namespace) -> None:
    for item in args.inputs:
        path = Path(item).expanduser()
        mesh = load_mesh(path)
        print_mesh_info(path, mesh, guess_units=args.guess_units)

        if args.known_point:
            name, x, y, z = args.known_point
            p = np.asarray([float(x), float(y), float(z)])
            print(f"known point '{name}': {fmt_vec(p)}")
            print(f"  vector min -> point:    {fmt_vec(p - mesh.bounds[0])}")
            print(f"  vector center -> point: {fmt_vec(p - mesh.bounds.mean(axis=0))}")


def view_meshes(args: argparse.Namespace) -> None:
    scene = trimesh.Scene()

    for i, item in enumerate(args.inputs):
        path = Path(item).expanduser()
        mesh = load_mesh(path)

        if args.spacing:
            mesh.apply_translation([i * args.spacing, 0, 0])

        scene.add_geometry(mesh, node_name=path.stem, geom_name=path.name)

    print("Opening viewer. If this fails over SSH/headless terminal, use inspect/edit only or open the output STL on a desktop.")
    scene.show()


def make_urdf_visual(args: argparse.Namespace) -> None:
    name = args.name
    filename = args.filename
    xyz = args.xyz or [0.0, 0.0, 0.0]
    rpy = args.rpy or [0.0, 0.0, 0.0]
    scale_attr = ""
    if args.scale is not None:
        s = args.scale
        if len(s) == 1:
            s = [s[0], s[0], s[0]]
        if len(s) != 3:
            raise SystemExit("ERROR: --scale takes either one uniform value or three xyz values")
        scale_attr = f' scale="{s[0]} {s[1]} {s[2]}"'

    snippet = (
        f'<visual name="{name}">\n'
        f'  <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>\n'
        f'  <geometry>\n'
        f'    <mesh filename="{filename}"{scale_attr}/>\n'
        f'  </geometry>\n'
        f'</visual>'
    )
    print(snippet)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Inspect, normalize, scale, rotate, translate, export, and view STL meshes using trimesh."
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    pi = sub.add_parser("inspect", help="Print mesh bounds/extents/center info.")
    pi.add_argument("inputs", nargs="+", help="STL files")
    pi.add_argument("--guess-units", action="store_true", help="Print rough unit guess from extents")
    pi.add_argument("--known-point", nargs=4, metavar=("NAME", "X", "Y", "Z"),
                    help="Print vectors from bounds min/center to a known point in raw mesh coordinates")
    pi.set_defaults(func=inspect_meshes)

    pe = sub.add_parser("edit", help="Edit one STL and export a new STL.")
    pe.add_argument("input", help="Input STL")
    pe.add_argument("-o", "--output", help="Output STL. Default: input_edited.stl")
    pe.add_argument("--guess-units", action="store_true", help="Print rough unit guess before editing")
    pe.add_argument("--scale", nargs="+", type=float,
                    help="Scale mesh. One value for uniform, or three for x y z.")
    pe.add_argument("--rotate-deg", nargs=3, type=float, metavar=("RX", "RY", "RZ"),
                    help="Rotate mesh by degrees around X Y Z, applied before translation/origin normalization.")
    pe.add_argument("--translate", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="Translate mesh after scale/rotation.")
    pe.add_argument("--min0", action="store_true", help="Move bounds min corner to 0,0,0.")
    pe.add_argument("--center-origin", action="store_true", help="Move bounding-box center to 0,0,0.")
    pe.add_argument("--origin-at", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="Move this mesh-coordinate point to 0,0,0.")
    pe.set_defaults(func=edit_mesh)

    pv = sub.add_parser("view", help="Open Trimesh viewer for one or more STL files.")
    pv.add_argument("inputs", nargs="+", help="STL files")
    pv.add_argument("--spacing", type=float, default=0.0,
                    help="Offset each mesh along X for side-by-side comparison.")
    pv.set_defaults(func=view_meshes)

    pu = sub.add_parser("urdf-visual", help="Generate a URDF <visual> snippet.")
    pu.add_argument("--name", default="mesh_visual")
    pu.add_argument("--filename", required=True, help="Example: package://mondy_description/meshes/part.stl")
    pu.add_argument("--xyz", nargs=3, type=float)
    pu.add_argument("--rpy", nargs=3, type=float)
    pu.add_argument("--scale", nargs="+", type=float)
    pu.set_defaults(func=make_urdf_visual)

    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
