#!/usr/bin/env python3

from pathlib import Path
import math
import shutil


SOURCE_WORLD = (
    Path.home()
    / "prius_adas_ws/prius_adas_control/worlds/adas_figure8.world"
)

OUTPUT_WORLD = (
    Path.home()
    / "prius_adas_ws/prius_adas_control/worlds/adas_figure8_slam.world"
)

BACKUP_WORLD = (
    Path.home()
    / "prius_adas_ws/prius_adas_control/worlds/adas_figure8_original.backup"
)


# ============================================================
# FIGURE-8
# ============================================================

A = 22.0
B = 12.0


def centerline(t):

    x = A * math.sin(t)
    y = B * math.sin(2.0 * t)

    return x, y


def tangent(t):

    dx = A * math.cos(t)
    dy = 2.0 * B * math.cos(2.0 * t)

    mag = math.sqrt(dx * dx + dy * dy)

    return dx / mag, dy / mag


# ============================================================
# BARRIERS
# ============================================================

BARRIER_OFFSET = 3.5

BARRIER_HEIGHT = 1.50

BARRIER_THICKNESS = 0.20

NUM_SEGMENTS = 120


# ============================================================
# INTERSECTION OPENING
# ============================================================

# The figure-8 crosses at approximately:
#
#       x = 0
#       y = 0
#
# We remove barrier segments whose midpoint is close to
# the crossing.
#
# Larger value = wider opening.
#

INTERSECTION_OPENING_RADIUS = 4.5


def near_intersection(x, y):

    distance = math.sqrt(
        x * x
        + y * y
    )

    return distance < INTERSECTION_OPENING_RADIUS


def make_barrier(
    name,
    x,
    y,
    yaw,
    length,
):

    z = BARRIER_HEIGHT / 2.0

    return f"""
    <model name="{name}">

      <static>true</static>

      <pose>
        {x:.4f} {y:.4f} {z:.4f}
        0 0 {yaw:.6f}
      </pose>

      <link name="link">

        <collision name="collision">

          <geometry>

            <box>

              <size>
                {length:.4f}
                {BARRIER_THICKNESS:.4f}
                {BARRIER_HEIGHT:.4f}
              </size>

            </box>

          </geometry>

        </collision>


        <visual name="visual">

          <geometry>

            <box>

              <size>
                {length:.4f}
                {BARRIER_THICKNESS:.4f}
                {BARRIER_HEIGHT:.4f}
              </size>

            </box>

          </geometry>

          <material>

            <ambient>
              0.4 0.4 0.4 1
            </ambient>

            <diffuse>
              0.5 0.5 0.5 1
            </diffuse>

          </material>

        </visual>

      </link>

    </model>
"""


def generate_barriers():

    left_points = []
    right_points = []

    # --------------------------------------------------------
    # Generate offset points
    # --------------------------------------------------------

    for i in range(NUM_SEGMENTS + 1):

        t = (
            2.0
            * math.pi
            * i
            / NUM_SEGMENTS
        )

        x, y = centerline(t)

        tx, ty = tangent(t)

        # Left normal
        nx = -ty
        ny = tx

        # Left barrier
        lx = x + BARRIER_OFFSET * nx
        ly = y + BARRIER_OFFSET * ny

        # Right barrier
        rx = x - BARRIER_OFFSET * nx
        ry = y - BARRIER_OFFSET * ny

        left_points.append(
            (lx, ly)
        )

        right_points.append(
            (rx, ry)
        )

    models = []

    skipped = 0

    # --------------------------------------------------------
    # Create left and right barrier segments
    # --------------------------------------------------------

    for i in range(NUM_SEGMENTS):

        # ====================================================
        # LEFT
        # ====================================================

        x1, y1 = left_points[i]
        x2, y2 = left_points[i + 1]

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        if near_intersection(cx, cy):

            skipped += 1

        else:

            dx = x2 - x1
            dy = y2 - y1

            length = math.sqrt(
                dx * dx
                + dy * dy
            )

            yaw = math.atan2(
                dy,
                dx
            )

            models.append(
                make_barrier(
                    f"slam_barrier_left_{i:03d}",
                    cx,
                    cy,
                    yaw,
                    length,
                )
            )

        # ====================================================
        # RIGHT
        # ====================================================

        x1, y1 = right_points[i]
        x2, y2 = right_points[i + 1]

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        if near_intersection(cx, cy):

            skipped += 1

        else:

            dx = x2 - x1
            dy = y2 - y1

            length = math.sqrt(
                dx * dx
                + dy * dy
            )

            yaw = math.atan2(
                dy,
                dx
            )

            models.append(
                make_barrier(
                    f"slam_barrier_right_{i:03d}",
                    cx,
                    cy,
                    yaw,
                    length,
                )
            )

    print(
        f"Skipped barrier segments: {skipped}"
    )

    return "\n".join(models)


def main():

    print()
    print(
        "================================================"
    )
    print(
        "FIGURE-8 SLAM BARRIER GENERATOR"
    )
    print(
        "================================================"
    )
    print()

    if not SOURCE_WORLD.exists():

        print(
            "ERROR: Source world does not exist:"
        )

        print(
            SOURCE_WORLD
        )

        raise SystemExit(1)

    # --------------------------------------------------------
    # Backup original
    # --------------------------------------------------------

    if not BACKUP_WORLD.exists():

        shutil.copy2(
            SOURCE_WORLD,
            BACKUP_WORLD
        )

        print(
            "Original world backed up:"
        )

        print(
            BACKUP_WORLD
        )

    # --------------------------------------------------------
    # Read original world
    # --------------------------------------------------------

    world = SOURCE_WORLD.read_text()

    # --------------------------------------------------------
    # Generate barriers
    # --------------------------------------------------------

    barriers = generate_barriers()

    # --------------------------------------------------------
    # Insert before </world>
    # --------------------------------------------------------

    end = world.rfind(
        "</world>"
    )

    if end == -1:

        print(
            "ERROR: </world> not found."
        )

        raise SystemExit(1)

    barrier_text = f"""

    <!-- ================================================= -->
    <!-- SLAM BARRIERS                                    -->
    <!--                                                     -->
    <!-- NOTE: Barrier opening around figure-8 crossing. -->
    <!-- ================================================= -->

{barriers}

"""

    result = (
        world[:end]
        + barrier_text
        + world[end:]
    )

    OUTPUT_WORLD.write_text(
        result
    )

    print()
    print(
        "SLAM world created:"
    )

    print(
        OUTPUT_WORLD
    )

    print()
    print(
        f"Intersection opening radius: "
        f"{INTERSECTION_OPENING_RADIUS} m"
    )

    print()
    print(
        "Done."
    )


if __name__ == "__main__":
    main()