"""Plot the reachable workspace of a single delta.

Default mode ("ik") sweeps a uniform (x, y, z) grid and keeps every point whose
inverse kinematics lands all three actuators inside their travel limits, which
gives uniform spatial density and shows the true workspace shape.

Mode "grid" instead walks the motor space -- a regular grid over the three
actuator travels, forward kinematics on each triple. That is the mechanism's
own parameterisation, but it aliases hard: the xy gain is about 6.8, so a single
motor grid step already displaces the tip ~7x as far radially, punching a
spurious hole through the centre of the cloud. Useful for seeing actuator
resolution, misleading as a workspace picture.

Pure kinematics either way -- no hardware or serial port.

Saves a top-down and a side-on render, then opens an interactive 3D window you
can drag to rotate.

    uv run python scripts/other/visualize_workspace.py
    uv run python scripts/other/visualize_workspace.py --samples 31 --no-show
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "python"))

from delta_control.constants import (  # noqa: E402
    BASE_TRIANGLE_SIDE_LEN,
    EE_Z_OFFSET,
    HOME_POSITION,
    LEG_LENGTH,
    MAX_JOINT_POS,
    MIN_JOINT_POS,
    PLATFORM_TRIANGLE_SIDE_LEN,
)
from delta_control.prismatic_delta import PrismaticDelta  # noqa: E402

# Task-space sampling is the default and is cheap (IK ~15 us vs FK ~190 us), so
# it can afford a much finer grid than the motor-space walk.
DEFAULT_SAMPLES = {"ik": 61, "grid": 25}
OUTPUT_DIR = REPO_ROOT / "generated_files" / "workspace"

# Actuator spread (max height difference across the three carriages) that
# scripts/basic/sweep_workspace.py keeps to on real hardware. Beyond this the
# platform tilts hard and the legs approach horizontal -- FK still solves, but
# those poses are not ones the array is driven through in practice.
PRACTICAL_SPREAD = 0.005

M_TO_MM = 1000.0


def task_space_bounds(delta: PrismaticDelta):
    """Bounding box that certainly contains the reachable set, from the geometry.

    A leg can at most lie flat (tip at carriage height) or stand vertical (tip a
    full leg length above it), which fixes z; the widest xy reach is a flat leg
    plus both triangle circumradii. IK then rejects everything outside the real
    workspace, so this only has to be generous, not tight.
    """
    max_radius = delta.lower_leg_length + delta.base_circumradius + delta.platform_circumradius
    z_min = MIN_JOINT_POS + delta.ee_z_offset
    z_max = MAX_JOINT_POS + delta.lower_leg_length + delta.ee_z_offset
    return max_radius, z_min, z_max


def sweep_task_space(delta: PrismaticDelta, samples: int):
    """IK over a uniform (x, y, z) grid; keep the poses the actuators can hold.

    Sampling task space rather than motor space gives uniform spatial density,
    so the picture shows workspace SHAPE. Walking the motor grid instead aliases
    badly: this delta has an xy gain around 6.8, so one motor grid step already
    throws the tip ~7x that far radially, leaving a spurious hole at the centre.

    Returns (heights, points): the (N, 3) actuator commands and the (N, 3) tip
    positions that produced them.
    """
    max_radius, z_min, z_max = task_space_bounds(delta)
    xy_axis = np.linspace(-max_radius, max_radius, samples)
    z_axis = np.linspace(z_min, z_max, samples)

    heights = []
    points = []
    for x in xy_axis:
        for y in xy_axis:
            # Skip the box corners outside the widest possible reach up front.
            if x * x + y * y > max_radius * max_radius:
                continue
            for z in z_axis:
                h = delta.ik([x, y, z])
                # NaN means the legs cannot close on that point at all; the
                # bounds check is the actuators' physical travel limit.
                if np.any(np.isnan(h)):
                    continue
                if np.any(h < MIN_JOINT_POS) or np.any(h > MAX_JOINT_POS):
                    continue
                heights.append(h)
                points.append((x, y, z))

    return np.array(heights), np.array(points)


def sweep_motor_space(delta: PrismaticDelta, samples: int):
    """FK over the full grid of actuator triples.

    Returns (heights, points): the (N, 3) actuator commands that produced a
    finite solution and the (N, 3) end-effector tip positions they map to.
    """
    axis = np.linspace(MIN_JOINT_POS, MAX_JOINT_POS, samples)

    heights = []
    points = []
    for h1 in axis:
        for h2 in axis:
            for h3 in axis:
                tip = np.asarray(delta.fk([h1, h2, h3]), dtype=float).reshape(3)
                # interx() returns NaN when the three leg spheres don't
                # intersect, i.e. the actuators are too far apart to close the
                # linkage. Those triples are simply not commandable.
                if np.any(np.isnan(tip)):
                    continue
                heights.append((h1, h2, h3))
                points.append(tip)

    return np.array(heights), np.array(points)


def actuator_spread(heights):
    """Max minus min carriage height for each actuator triple -- a tilt proxy."""
    return heights.max(axis=1) - heights.min(axis=1)


def _scatter_cloud(ax, heights, points):
    """Draw the tip cloud on a 3D axes, coloured by actuator spread."""
    pts = points * M_TO_MM
    spread = actuator_spread(heights) * M_TO_MM
    home = np.asarray(HOME_POSITION) * M_TO_MM

    # Draw the widely-splayed poses first so the tight, practical ones sit on top.
    order = np.argsort(-spread)
    sc = ax.scatter(pts[order, 0], pts[order, 1], pts[order, 2],
                    c=spread[order], cmap="viridis_r", s=3, alpha=0.4)
    ax.scatter(*home, color="red", s=45, marker="o", label="home", depthshade=False)

    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    # Equal data scaling, so the cloud isn't stretched by the axis ranges.
    spans = pts.max(axis=0) - pts.min(axis=0)
    ax.set_box_aspect(spans)
    ax.legend(loc="upper left", fontsize=8)
    return sc


def _hide_axis(ax, which: str) -> None:
    """Blank out an axis that is edge-on to the camera (its labels are noise)."""
    axis = getattr(ax, f"{which}axis")
    axis.set_ticks([])
    axis.line.set_linewidth(0)
    getattr(ax, f"set_{which}label")("")
    axis.pane.set_visible(False)


def _make_figure(heights, points, samples: int, title: str, mode: str):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    sc = _scatter_cloud(ax, heights, points)
    fig.colorbar(sc, ax=ax, label="actuator spread (mm)", shrink=0.6, pad=0.02)
    fig.suptitle(
        f"Single delta workspace -- {title}\n"
        f"{samples}^3 {'task-space' if mode == 'ik' else 'motor-space'} grid, "
        f"{len(points)} reachable poses",
        fontsize=12,
    )
    # 3D axes leave a lot of dead margin by default; claw most of it back.
    fig.subplots_adjust(left=0.0, right=0.92, top=0.94, bottom=0.0)
    return fig, ax


def save_views(heights, points, samples: int, out_dir: Path, mode: str) -> list[Path]:
    """Save a top-down and a side-on render of the tip cloud."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []

    # (name, elevation, azimuth, edge-on axis): down the -z axis, then level with x-z.
    for name, title, elev, azim, edge_on in (
        ("top", "top view (looking down -z)", 90, -90, "z"),
        ("side", "side view (looking along +y)", 0, -90, "y"),
    ):
        fig, ax = _make_figure(heights, points, samples, title, mode)
        ax.view_init(elev=elev, azim=azim)
        # Orthographic, so a saved view is a true projection with no perspective skew.
        ax.set_proj_type("ortho")
        _hide_axis(ax, edge_on)
        out_path = out_dir / f"workspace_{mode}_{name}_{samples}.png"
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"saved {out_path}")
        written.append(out_path)

    return written


def show_interactive(heights, points, samples: int, mode: str) -> None:
    """Open a rotatable 3D window (drag to orbit, scroll to zoom)."""
    fig, _ = _make_figure(heights, points, samples, "drag to rotate", mode)
    print("opening interactive window -- drag to rotate, close it to exit")
    plt.show()


def summarize(heights, points, samples: int, mode: str) -> None:
    total = samples**3
    label = "task-space grid" if mode == "ik" else "motor grid"
    print(f"{label}: {samples}^3 = {total} candidate points")
    print(f"joint range:     [{MIN_JOINT_POS * M_TO_MM:.2f}, {MAX_JOINT_POS * M_TO_MM:.2f}] mm")
    print(f"reachable:       {len(points)} ({100 * len(points) / total:.1f}%)")
    print("tip extents (mm):")
    for axis_name, col in zip("xyz", range(3)):
        lo, hi = points[:, col].min(), points[:, col].max()
        print(f"  {axis_name}: {lo * M_TO_MM:8.2f} .. {hi * M_TO_MM:8.2f}"
              f"   (span {(hi - lo) * M_TO_MM:.2f})")
    radius = np.linalg.norm(points[:, 0:2], axis=1)
    print(f"  max radius from z axis: {radius.max() * M_TO_MM:.2f} mm")
    print(f"home pose (mm):  {np.asarray(HOME_POSITION) * M_TO_MM}")

    # The full FK cloud includes heavily tilted poses; report the tighter subset
    # separately so the practical envelope isn't hidden by the extremes.
    practical = points[actuator_spread(heights) <= PRACTICAL_SPREAD]
    print(f"spread <= {PRACTICAL_SPREAD * M_TO_MM:.0f} mm: {len(practical)} poses")
    if len(practical):
        prac_radius = np.linalg.norm(practical[:, 0:2], axis=1)
        print(f"  z: {practical[:, 2].min() * M_TO_MM:8.2f} .. "
              f"{practical[:, 2].max() * M_TO_MM:8.2f}")
        print(f"  max radius from z axis: {prac_radius.max() * M_TO_MM:.2f} mm")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("ik", "grid"), default="ik",
                        help="ik: uniform task-space sweep (default). "
                             "grid: walk the motor space with FK")
    parser.add_argument("--samples", type=int, default=None,
                        help=f"grid points per axis (default {DEFAULT_SAMPLES['ik']} for ik, "
                             f"{DEFAULT_SAMPLES['grid']} for grid)")
    parser.add_argument("--no-show", action="store_true",
                        help="save the images without opening the interactive window")
    parser.add_argument("--csv", type=Path, default=None,
                        help="also write h1,h2,h3,x,y,z for every reachable pose here")
    args = parser.parse_args()

    samples = args.samples if args.samples is not None else DEFAULT_SAMPLES[args.mode]

    delta = PrismaticDelta(
        PLATFORM_TRIANGLE_SIDE_LEN, BASE_TRIANGLE_SIDE_LEN, LEG_LENGTH, EE_Z_OFFSET
    )

    if args.mode == "ik":
        heights, points = sweep_task_space(delta, samples)
    else:
        heights, points = sweep_motor_space(delta, samples)

    if not len(points):
        raise SystemExit("no reachable poses -- check the geometry constants")

    summarize(heights, points, samples, args.mode)

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(args.csv, np.hstack((heights, points)), delimiter=",",
                   header="h1,h2,h3,x,y,z", comments="")
        print(f"saved {args.csv}")

    save_views(heights, points, samples, OUTPUT_DIR, args.mode)

    if not args.no_show:
        show_interactive(heights, points, samples, args.mode)


if __name__ == "__main__":
    main()
