"""Runs a pick -> inspect -> sort cycle in RoboDK and verifies the result.

Each of the three workpieces on the infeed table is given a random "true"
category (pass / fail / rework) up front and tinted with that bin's color,
standing in for what a real vision/quality inspection would report. The
robot then picks each part, "inspects" it (reads back that same category),
and places it into the matching bin. After the run, every workpiece's
final bin is checked against its assigned category and a verification
report is printed -- this is the "pre-colored parts, check it sorts them
correctly" self-test.

Requires the station to already exist (or will build it fresh via
robodk_setup.build_station()) and a robot within reach of both the table
and the bins -- see robodk_setup.py for the layout.
"""

import os
import random
import sys
import time

# When run from a terminal, robodk_setup.py sits next to this file. When
# run from inside RoboDK (File > Open > double-click / F5), RoboDK copies
# the script to a temp folder first, so the same-directory guess breaks --
# fall back to this project's known src/ location on this machine.
_CANDIDATE_DIRS = [
    os.path.dirname(os.path.abspath(__file__)),
    r"C:\Users\kimil\Desktop\automotive-sorting-cell\src",
]
for _dir in _CANDIDATE_DIRS:
    if os.path.isfile(os.path.join(_dir, "robodk_setup.py")) and _dir not in sys.path:
        sys.path.insert(0, _dir)
        break

from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, Robolink
from robodk.robomath import pi, rotx, transl

import robodk_setup as station

PICK_APPROACH_Z_MM = station.TABLE_SIZE_MM[2] + 180  # clearance above the table
PICK_Z_MM = station.TABLE_SIZE_MM[2] + 30  # grab height, roughly part-center height

PLACE_APPROACH_Z_MM = station.BIN_SIZE_MM[2] + 120  # clearance above the bin rim
PLACE_Z_MM = 90  # release height inside the bin, below the rim

HOME_JOINTS_DEG = [0, -20, 20, 0, 70, 0]


def top_down_pose(x: float, y: float, z: float):
    return transl(x, y, z) * rotx(pi)


def check_reachable(robot, pose, label: str) -> None:
    solution = robot.SolveIK(pose)
    if len(solution.list()) < 6:
        raise RuntimeError(f"Target unreachable: {label} at {pose.Pos()}")


def build_targets(robot):
    """Compute and validate every pose the robot will move to."""
    targets = {"pick": [], "place": {}}

    for i, x_mm in enumerate(station.WORKPIECE_LOCAL_XY_MM, start=1):
        wx = station.TABLE_ORIGIN_MM[0] + x_mm
        wy = station.TABLE_ORIGIN_MM[1]
        approach = top_down_pose(wx, wy, PICK_APPROACH_Z_MM)
        pick = top_down_pose(wx, wy, PICK_Z_MM)
        check_reachable(robot, approach, f"pick approach {i}")
        check_reachable(robot, pick, f"pick {i}")
        targets["pick"].append({"name": f"{station.WORKPIECE_NAME} {i}", "approach": approach, "pick": pick})

    for bin_id, (frame_name, xyz_mm, _color) in station.BINS.items():
        bx, by, _bz = xyz_mm
        approach = top_down_pose(bx, by, PLACE_APPROACH_Z_MM)
        place = top_down_pose(bx, by, PLACE_Z_MM)
        check_reachable(robot, approach, f"place approach {frame_name}")
        check_reachable(robot, place, f"place {frame_name}")
        targets["place"][bin_id] = {"frame_name": frame_name, "approach": approach, "place": place}

    return targets


def assign_categories(rdk: Robolink):
    """Randomly assign each workpiece a true category and tint it to match.

    This is the "pre-colored parts" ground truth: a human (or this script,
    afterwards) can check the robot's placement against the visible color.
    """
    assignment = {}
    for i in range(1, len(station.WORKPIECE_LOCAL_XY_MM) + 1):
        name = f"{station.WORKPIECE_NAME} {i}"
        bin_id = random.choice(list(station.BINS.keys()))
        frame_name, _xyz, color = station.BINS[bin_id]
        item = rdk.Item(name)
        item.setColor(color)
        assignment[name] = bin_id
        print(f"  {name}: assigned -> {bin_id} ({frame_name})")
    return assignment


def run_cycle(rdk: Robolink, robot, targets, assignment) -> None:
    robot.setSpeed(300)
    robot.MoveJ(HOME_JOINTS_DEG)

    for pick_target in targets["pick"]:
        name = pick_target["name"]
        bin_id = assignment[name]
        place_target = targets["place"][bin_id]
        bin_frame = rdk.Item(place_target["frame_name"], ITEM_TYPE_FRAME)

        part = rdk.Item(name)

        robot.MoveJ(pick_target["approach"])
        robot.MoveL(pick_target["pick"])
        # setParent (not setParentStatic) keeps the part's local pose at
        # identity relative to the robot flange, i.e. it rigidly follows
        # the robot from here on, like a part held by a gripper.
        station.retry_call(lambda: part.setParent(robot))
        part.setPose(transl(0, 0, 0))
        robot.MoveL(pick_target["approach"])

        robot.MoveJ(place_target["approach"])
        robot.MoveL(place_target["place"])
        # setParentStatic (not setParent) keeps the part's current absolute
        # pose, i.e. it stays right where it was released instead of
        # jumping to the bin frame's origin.
        station.retry_call(lambda: part.setParentStatic(bin_frame))
        robot.MoveL(place_target["approach"])
        robot.MoveJ(HOME_JOINTS_DEG)

        print(f"  {name} -> {place_target['frame_name']}")


def _inside_bin_footprint(world_xy, bin_xyz_mm) -> bool:
    half_x, half_y = station.BIN_SIZE_MM[0] / 2, station.BIN_SIZE_MM[1] / 2
    bx, by, _bz = bin_xyz_mm
    return abs(world_xy[0] - bx) <= half_x and abs(world_xy[1] - by) <= half_y


def verify_sorting(rdk: Robolink, assignment: dict) -> bool:
    print("\nVerification (does the final bin match the pre-assigned color?):")
    all_ok = True
    for name, expected_bin_id in assignment.items():
        expected_frame, expected_xyz_mm, _color = station.BINS[expected_bin_id]
        item = rdk.Item(name)
        actual_frame = item.Parent().Name()
        world_pos = item.PoseAbs().Pos()
        parent_ok = actual_frame == expected_frame
        # Parent-hierarchy check alone only proves the bookkeeping is
        # consistent; also check the part's actual world position falls
        # inside the target bin's footprint, catching cases where the
        # reparent succeeded but the release pose landed somewhere else.
        position_ok = _inside_bin_footprint(world_pos, expected_xyz_mm)
        ok = parent_ok and position_ok
        all_ok = all_ok and ok
        status = "OK" if ok else "MISMATCH"
        print(f"  [{status}] {name}: expected {expected_frame}, parent={actual_frame} (parent_ok={parent_ok}), "
              f"world_xy=({world_pos[0]:.0f}, {world_pos[1]:.0f}) inside_footprint={position_ok}")
    print("Result:", "ALL PARTS SORTED CORRECTLY" if all_ok else "SORTING ERROR DETECTED")
    return all_ok


def run_simulation() -> bool:
    rdk = Robolink()
    station.build_station()

    robot = rdk.Item(station.ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot not found after build_station()")

    print("Validating reach for all pick/place targets...")
    targets = build_targets(robot)

    print("Assigning ground-truth categories to workpieces...")
    assignment = assign_categories(rdk)

    print("Running pick -> inspect -> sort cycle...")
    run_cycle(rdk, robot, targets, assignment)

    return verify_sorting(rdk, assignment)


if __name__ == "__main__":
    start = time.time()
    success = run_simulation()
    print(f"\nDone in {time.time() - start:.1f}s")
    sys.exit(0 if success else 1)
