"""Builds the automotive sorting cell station in RoboDK.

Connects to a running RoboDK instance (starting it if needed) and lays out:
  - an ABB IRB 120 robot
  - a simple infeed table holding three workpieces for the robot to pick,
    inspect and sort
  - three open-top sorting bins (pass / fail / rework) that the robot
    places inspected parts into

Bin identifiers match the "robot.bins" section of config.json so
robot_control.py refers to the same station.

Re-running this script rebuilds the cell from scratch (existing frames
and shapes are replaced), so it is safe to run repeatedly.
"""

import os
import time

from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, VISIBLE_ROBOT_DEFAULT, VISIBLE_ROBOT_FLANGE, Robolink
from robodk.robomath import transl

ROBOT_LIBRARY_FILE = r"C:\RoboDK\Library\06-Pick and Place\Parts\ABB-IRB-120-3-0-6.robot"
ROBOT_NAME = "ABB IRB 120-3/0.6"
BOX_FILE = r"C:\RoboDK\Library\17-Conveyor Swap\Parts\Box.stl"

CELL_FRAME_NAME = "Sorting Cell Frame"

TABLE_FRAME_NAME = "Infeed Table Frame"
TABLE_ORIGIN_MM = (0, -400, 0)  # in front of the robot, opposite the bins
TABLE_SIZE_MM = (500, 400, 30)  # tabletop slab: width x depth x thickness
TABLE_COLOR = [0.55, 0.55, 0.58, 1.0]

WORKPIECE_NAME = "Workpiece"
WORKPIECE_SCALE = 0.4
# Three parts laid out on the table, spread along local X, sitting on top
# of the slab (Z = table thickness).
WORKPIECE_LOCAL_XY_MM = [-150, 0, 150]

BIN_SIZE_MM = (350, 350, 180)  # open-top sorting bin: footprint + wall height

# bin id (see config.json -> robot.bins) -> (frame name, position relative to cell frame, RGBA color)
# On the opposite side of the robot from the table, fanned out in a row.
BINS = {
    "BIN_A": ("Bin Pass", (-400, 400, 0), [0.25, 0.7, 0.3, 1.0]),
    "BIN_B": ("Bin Fail", (0, 400, 0), [0.75, 0.25, 0.25, 1.0]),
    "BIN_C": ("Bin Rework", (400, 400, 0), [0.85, 0.7, 0.15, 1.0]),
}


def _box_triangles(sx: float, sy: float, sz: float) -> list:
    """Triangle vertices for a closed box, centered in XY, sitting on Z=0."""
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


def _open_box_triangles(sx: float, sy: float, sz: float) -> list:
    """Triangle vertices for an open-top box: bottom + 4 walls, no lid.

    Centered in XY, sitting on Z=0, so it reads as a container you can
    place parts into rather than a solid block.
    """
    x0, y0, z0 = -sx / 2, -sy / 2, 0
    x1, y1, z1 = sx / 2, sy / 2, sz
    p = {
        0: [x0, y0, z0], 1: [x1, y0, z0], 2: [x1, y1, z0], 3: [x0, y1, z0],
        4: [x0, y0, z1], 5: [x1, y0, z1], 6: [x1, y1, z1], 7: [x0, y1, z1],
    }
    faces = [
        (0, 2, 1), (0, 3, 2),  # bottom
        (0, 1, 5), (0, 5, 4),  # front wall
        (1, 2, 6), (1, 6, 5),  # right wall
        (2, 3, 7), (2, 7, 6),  # back wall
        (3, 0, 4), (3, 4, 7),  # left wall
        # top intentionally omitted: open box
    ]
    triangles = []
    for a, b, c in faces:
        triangles.extend([p[a], p[b], p[c]])
    return triangles


def _retry(fn, attempts: int = 5, delay: float = 0.3):
    """Retry a RoboDK API call a few times.

    RoboDK's API occasionally rejects a command with "Invalid item
    provided" when a previous command (AddFile/AddFrame/AddShape) hasn't
    fully registered yet on the server side, even with no delete involved.
    A short retry loop is the pragmatic fix since it's a timing issue in
    RoboDK itself, not something controllable from the client side.
    """
    last_exc = None
    for _ in range(attempts):
        try:
            return fn()
        except Exception as exc:
            last_exc = exc
            time.sleep(delay)
    raise last_exc


def get_or_add_robot(rdk: Robolink):
    robot = rdk.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if robot.Valid():
        return robot
    if not os.path.exists(ROBOT_LIBRARY_FILE):
        raise FileNotFoundError(f"Robot library file not found: {ROBOT_LIBRARY_FILE}")
    return rdk.AddFile(ROBOT_LIBRARY_FILE)


def add_frame(rdk: Robolink, name: str, xyz_mm: tuple, parent=0, show_axes: bool = False):
    # Reuse the frame if it already exists instead of deleting and
    # recreating it. Deleting a frame cascades through every descendant
    # (all pallet/bin meshes etc.), and RoboDK's GUI can lag behind that
    # cascade, causing the very next AddShape/setParent call to intermittently
    # fail with "Invalid item provided". Reusing frames avoids that entirely;
    # the individual mesh children are still safely replaced by add_mesh/add_box.
    frame = rdk.Item(name, ITEM_TYPE_FRAME)
    if not frame.Valid():
        frame = rdk.AddFrame(name, parent)
    elif not isinstance(parent, int):
        # setParent rejects a plain 0 (unlike AddFrame, where 0 means
        # "attach to station root"), so only reparent when a real parent
        # Item was given. Also, Item.__ne__ doesn't handle comparison
        # against a bare int, so check the type instead of "!= 0".
        _retry(lambda: frame.setParent(parent))
    frame.setPose(transl(*xyz_mm))
    frame.setVisible(show_axes)
    return frame


def add_mesh(rdk: Robolink, file_path: str, name: str, scale, color_rgba, parent, local_xyz_mm=(0, 0, 0)):
    existing = rdk.Item(name)
    if existing.Valid():
        existing.Delete()
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Asset file not found: {file_path}")
    item = rdk.AddFile(file_path)
    item.setName(name)
    if scale is not None:
        item.Scale(scale)
    if color_rgba is not None:
        item.setColor(color_rgba)
    # setParent (not setParentStatic) keeps the item's local pose at
    # identity, i.e. sitting right at the parent frame's origin instead
    # of jumping back to wherever AddFile happened to place it.
    _retry(lambda: item.setParent(parent))
    item.setPose(transl(*local_xyz_mm))
    return item


def add_box(rdk: Robolink, name: str, size_mm: tuple, color_rgba: list, parent, open_top: bool = False):
    existing = rdk.Item(name)
    if existing.Valid():
        existing.Delete()
    triangles = _open_box_triangles(*size_mm) if open_top else _box_triangles(*size_mm)
    box = rdk.AddShape(triangles)
    box.setName(name)
    box.setColor(color_rgba)
    _retry(lambda: box.setParent(parent))
    box.setPose(transl(0, 0, 0))
    return box


def build_station() -> None:
    rdk = Robolink()
    rdk.Render(False)  # batch the rebuild; re-enabled at the end

    robot = get_or_add_robot(rdk)
    robot.setVisible(True, VISIBLE_ROBOT_DEFAULT & ~VISIBLE_ROBOT_FLANGE)
    robot_base = rdk.Item(f"{ROBOT_NAME} Base", ITEM_TYPE_FRAME)
    if robot_base.Valid():
        robot_base.setVisible(False)

    cell_frame = add_frame(rdk, CELL_FRAME_NAME, (0, 0, 0))

    table_frame = add_frame(rdk, TABLE_FRAME_NAME, TABLE_ORIGIN_MM, cell_frame)
    add_box(rdk, "Infeed Table", TABLE_SIZE_MM, TABLE_COLOR, table_frame)
    for i, x_mm in enumerate(WORKPIECE_LOCAL_XY_MM, start=1):
        add_mesh(rdk, BOX_FILE, f"{WORKPIECE_NAME} {i}", WORKPIECE_SCALE, None, table_frame, (x_mm, 0, TABLE_SIZE_MM[2]))

    for bin_id, (frame_name, xyz_mm, color) in BINS.items():
        bin_frame = add_frame(rdk, frame_name, xyz_mm, cell_frame)
        add_box(rdk, f"{frame_name} Container", BIN_SIZE_MM, color, bin_frame, open_top=True)

    rdk.Render(True)
    print(f"Station built: robot={robot.Name()}, workpieces={len(WORKPIECE_LOCAL_XY_MM)}, bins={[b[0] for b in BINS.values()]}")


if __name__ == "__main__":
    build_station()
