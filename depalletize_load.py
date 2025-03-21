from robolink import *
from robodk import *
import time
from enum import Enum
from enum import auto


RDK = Robolink()


class States(Enum):
    APPROACH_BEARING = auto()
    GRIP_BEARING = auto()
    LIFT_BEARING = auto()
    FLIP_VACUUM = auto()
    APPROACH_LOAD = auto()
    LOAD = auto()


state = States.APPROACH_BEARING
entry_flag = 0
flip_flag = 0
motion_time = 0

BEARING_APPROACH = [-57.44, -142.73, -90.89, 48.27, 15.04, -85.94]
BEARING_CONTACT = [-63.83, -143.65, -92.02, 54.12, 20.74, -90.06]
BEARING_LEAVE = [-62.74, -125.31, -94.15, 39.45, 19.04, -89.97]

FLIPPED_BEARING_APPROACH = [-64.14, -157.24, -58.14, 37.36, -67.16, -273.06]
FLIPPED_BEARING_CONTACT = [-68.61, -157.20, -58.14, 38.39, -64.44, -272.70]
FLIPPED_BEARING_LEAVE = [-68.86, -150.41, -58.14, 32.05, -64.60, -272.57]

LOAD = [-58.10, -101.58, -111.03, -97.66, -66.67, -200.53]
FLIPPED_LOAD = [-62.64, -108.24, -115.58, -7.20, 72.80, -164.01]


def establish_connection() -> RDK.Item:
    robot = RDK.Item("", robolink.ITEM_TYPE_ROBOT)
    robot.setTool(robomath.Pose(0, 0, 75, 0, 0, 0))
    reference_frame = RDK.Item('UR5e Base')
    robot.setFrame(reference_frame)

    RDK.ShowMessage("Trying to connect to %s..." % robot.Name())
    robot.Connect()
    time.sleep(3)
    return robot


def go_home(robot: RDK.Item):
    robot.MoveJ([-90, -90, -90, 0, 90, 0])
    print('Moved to Home Position')
    time.sleep(5)


def non_blocking_move(robot: RDK.Item, offset: list, start_pos: list):
    global motion_time, entry_flag
    pos = [a + b for a, b in zip(start_pos, offset)]
    robot.MoveJ(pos, blocking=False)
    motion_time = time.perf_counter()
    entry_flag = 1


def state_machine(robot: RDK.Item, num_unloads: int):
    global entry_flag, state, motion_time, flip_flag

    # Determine joint offsets for de-palletization
    offset_x = num_unloads % 2
    offset_y = (num_unloads // 3) % 13

    # Flip end effector every other unload
    if offset_x:
        bearing_offset_x = [0 * offset_x, -6.67 * offset_x, 15.14 * offset_x, -10.84 * offset_x, 1.19 * offset_x,
                            0.78 * offset_x]
        bearing_offset_y = [-2.72 * offset_y, -0.16 * offset_y, 0 * offset_y, 0 * offset_y, 1.78 * offset_y,
                            -0.20 * offset_y]
        state = States.FLIP_VACUUM if not flip_flag else state
    else:
        bearing_offset_x = [-2.35 * offset_x, -0.71 * offset_x, 5.72 * offset_x, 9.25 * offset_x, 1.47 * offset_x,
                            7.26 * offset_x]
        bearing_offset_y = [-2.72 * offset_y, -0.16 * offset_y, 0 * offset_y, 0 * offset_y, 1.78 * offset_y,
                            -0.20 * offset_y]

    bearing_offset = [x + y for x, y in zip(bearing_offset_x, bearing_offset_y)]

    match state:
        case States.FLIP_VACUUM:
            if not entry_flag:
                flip_flag = 1
                non_blocking_move(robot, [10, 0, 0, 0, -90, -180], BEARING_LEAVE)

            if (time.perf_counter() - motion_time) >= 10:
                state = States.APPROACH_BEARING
                entry_flag = 0

        case States.APPROACH_BEARING:
            if not entry_flag:
                if offset_x:
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_APPROACH)
                else:
                    non_blocking_move(robot, bearing_offset, BEARING_APPROACH)

            if (time.perf_counter() - motion_time) >= 10:
                state = States.GRIP_BEARING
                entry_flag = 0

        case States.GRIP_BEARING:
            if entry_flag == 0:
                if offset_x:
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_CONTACT)
                else:
                    non_blocking_move(robot, bearing_offset, BEARING_CONTACT)

            if (time.perf_counter() - motion_time) >= 5:
                state = States.LIFT_BEARING
                entry_flag = 0

        case States.LIFT_BEARING:
            if not entry_flag:
                if offset_x:
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_LEAVE)
                else:
                    non_blocking_move(robot, bearing_offset, BEARING_LEAVE)

            if (time.perf_counter() - motion_time) >= 5:
                state = States.APPROACH_LOAD if flip_flag else States.APPROACH_BEARING
                num_unloads += 1
                entry_flag = 0
                flip_flag = 0

        case States.APPROACH_LOAD:
            if not entry_flag:
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], LOAD)

            if (time.perf_counter() - motion_time) >= 5:
                state = States.APPROACH_BEARING
                entry_flag = 0

    return num_unloads


def main():
    robot = establish_connection()
    go_home(robot)
    num_unloads = 0
    while True:
        num_unloads = state_machine(robot, num_unloads)


if __name__ == '__main__':
    main()
