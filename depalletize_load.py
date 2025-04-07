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

BEARING_APPROACH = [-74.24, -141.68, -106.94, 72.65, 28.82, -94.27]
BEARING_CONTACT = [-79.18, -141.68, -106.94, 72.61, 34.51, -94.27]
BEARING_LEAVE = [-79.18, -128.60, -106.94, 60., 34.51, -93.40]

FLIPPED_BEARING_APPROACH = [-76.95, -149.72, -81.01, 48.41, -54.55, -268.35]
FLIPPED_BEARING_CONTACT = [-80.43, -149.72, -81.01, 48.41, -51.23, -268.35]
FLIPPED_BEARING_LEAVE = [-80.43, -144.30, -81.08, 45.47, -55.04, -268.26]

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
    offset = num_unloads % 6

    # Flip end effector every other unload
    if num_unloads % 2:
        bearing_offset = [-2.21 * offset, 0 * offset, 0 * offset, 0 * offset, 1.73 * offset, 0.35 * offset]
        state = States.FLIP_VACUUM if not flip_flag else state
    else:
        bearing_offset = [-2.55 * offset, 0 * offset, -0.39 * offset, 0.13 * offset, 3.23 * offset, -0.07 * offset]

    match state:
        case States.FLIP_VACUUM:
            if not entry_flag:
                flip_flag = 1
                non_blocking_move(robot, [10, 0, 0, 0, -90, -180], BEARING_LEAVE)

            if not robot.Busy():
                state = States.APPROACH_BEARING
                entry_flag = 0

        case States.APPROACH_BEARING:
            if not entry_flag:
                if flip_flag:
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_APPROACH)
                else:
                    non_blocking_move(robot, bearing_offset, BEARING_APPROACH)

            if not robot.Busy():
                state = States.GRIP_BEARING
                entry_flag = 0

        case States.GRIP_BEARING:
            if not entry_flag:
                if flip_flag:
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_CONTACT)
                else:
                    non_blocking_move(robot, bearing_offset, BEARING_CONTACT)

            if not robot.Busy():
                robot.setDO(io_value=0, io_var=1)  # First, set D_O0 to low (activate vacuum)
                robot.setDO(io_value=0, io_var=0)
                if (time.perf_counter() - motion_time) >= 3:
                    state = States.LIFT_BEARING
                    entry_flag = 0

        case States.LIFT_BEARING:
            if not entry_flag:
                if flip_flag:
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_LEAVE)
                else:
                    non_blocking_move(robot, bearing_offset, BEARING_LEAVE)

            if not robot.Busy():
                state = States.APPROACH_LOAD if flip_flag else States.APPROACH_BEARING
                num_unloads += 1
                entry_flag = 0
                flip_flag = 0

        case States.APPROACH_LOAD:
            if not entry_flag:
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], LOAD)

            if not robot.Busy():
                state = States.APPROACH_BEARING
                entry_flag = 0

    return num_unloads


def main():
    robot = establish_connection()

    # Initialize IO
    robot.setDO(io_value=1, io_var=0)
    robot.setDO(io_value=1, io_var=1)
    robot.setDO(io_value=0, io_var=2)

    go_home(robot)
    num_unloads = 0
    while True:
        num_unloads = state_machine(robot, num_unloads)

    # IO LOGIC
    # robot.setDO(io_value=0, io_var=1)  # First, set D_O0 to low (activate vacuum)
    # robot.setDO(io_value=0, io_var=0)  # Then D_O1 can be set to low (activate vacuum)

    # robot.setDO(io_value=1, io_var=1)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=0)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=2)  # Activate blow off nozzle


if __name__ == '__main__':
    main()
