"""Builds the automotive sorting cell station in RoboDK.

Connects to a running RoboDK instance (starting it if needed), loads an
ABB IRB 120 robot from RoboDK's local example library, and lays out three
output pallets (pass / fail / rework) plus a placeholder workpiece at the
infeed position. Bin identifiers match the "robot.bins" section of
config.json so robot_control.py refers to the same station.

Re-running this script rebuilds the cell from scratch (existing frames and
shapes are replaced), so it is safe to run repeatedly.
"""

import os

from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, Robolink
from robodk.robomath import transl

ROBOT_LIBRARY_FILE = r"C:\RoboDK\Library\06-Pick and Place\Parts\ABB-IRB-120-3-0-6.robot"

CELL_FRAME_NAME = "Sorting Cell Frame"
INFEED_FRAME_NAME = "Infeed Frame"
INFEED_POSITION_MM = (300, 0, 0)

PALLET_SIZE_MM = (300, 200, 15)
WORKPIECE_SIZE_MM = (50, 50, 50)

# bin id (see config.json -> robot.bins) -> (frame name, position relative to cell frame, RGBA color)
PALLETS = {
    "BIN_A": ("Pallet Pass", (600, -300, 0), [0.2, 0.75, 0.2, 1.0]),
    "BIN_B": ("Pallet Fail", (600, 0, 0), [0.8, 0.2, 0.2, 1.0]),
    "BIN_C": ("Pallet Rework", (600, 300, 0), [0.9, 0.75, 0.1, 1.0]),
}


def _box_triangles(sx: float, sy: float, sz: float) -> list:
    """Triangle vertices (grouped by 3) for a box centered in XY, sitting on Z=0."""
    x0, y0, z0 = -sx / 2, -sy / 2, 0
    x1, y1, z1 = sx / 2, sy / 2, sz
    p = {
        0: [x0, y0, z0], 1: [x1, y0, z0], 2: [x1, y1, z0], 3: [x0, y1, z0],
        4: [x0, y0, z1], 5: [x1, y0, z1], 6: [x1, y1, z1], 7: [x0, y1, z1],
    }
    faces = [
        (0, 2, 1), (0, 3, 2),  # bottom
        (4, 5, 6), (4, 6, 7),  # top
        (0, 1, 5), (0, 5, 4),  # front
        (1, 2, 6), (1, 6, 5),  # right
        (2, 3, 7), (2, 7, 6),  # back
        (3, 0, 4), (3, 4, 7),  # left
    ]
    triangles = []
    for a, b, c in faces:
        triangles.extend([p[a], p[b], p[c]])
    return triangles


def get_or_add_robot(rdk: Robolink):
    robot = rdk.Item("", ITEM_TYPE_ROBOT)
    if robot.Valid():
        return robot
    if not os.path.exists(ROBOT_LIBRARY_FILE):
        raise FileNotFoundError(f"Robot library file not found: {ROBOT_LIBRARY_FILE}")
    return rdk.AddFile(ROBOT_LIBRARY_FILE)


def add_frame(rdk: Robolink, name: str, xyz_mm: tuple, parent=0):
    existing = rdk.Item(name, ITEM_TYPE_FRAME)
    if existing.Valid():
        existing.Delete()
    frame = rdk.AddFrame(name, parent)
    frame.setPose(transl(*xyz_mm))
    return frame


def add_box(rdk: Robolink, name: str, size_mm: tuple, color_rgba: list, parent):
    existing = rdk.Item(name)
    if existing.Valid():
        existing.Delete()
    # AddShape rejects a Frame passed directly as add_to, so attach the
    # parent afterwards via setParentStatic instead.
    box = rdk.AddShape(_box_triangles(*size_mm))
    box.setName(name)
    box.setColor(color_rgba)
    box.setParentStatic(parent)
    return box


def build_station() -> None:
    rdk = Robolink()

    robot = get_or_add_robot(rdk)
    robot.setVisible(True)

    cell_frame = add_frame(rdk, CELL_FRAME_NAME, (0, 0, 0))

    infeed_frame = add_frame(rdk, INFEED_FRAME_NAME, INFEED_POSITION_MM, cell_frame)
    add_box(rdk, "Workpiece", WORKPIECE_SIZE_MM, [0.6, 0.6, 0.65, 1.0], infeed_frame)

    for bin_id, (frame_name, xyz_mm, color) in PALLETS.items():
        pallet_frame = add_frame(rdk, frame_name, xyz_mm, cell_frame)
        add_box(rdk, f"{frame_name} Surface", PALLET_SIZE_MM, color, pallet_frame)

    print(f"Station built: robot={robot.Name()}, pallets={[p[0] for p in PALLETS.values()]}")


if __name__ == "__main__":
    build_station()
