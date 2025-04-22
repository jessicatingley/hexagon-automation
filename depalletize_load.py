from robolink import *
from robodk import *
import time
from enum import Enum
from enum import auto

# Initialize connection to RoboDK
RDK = Robolink()


# Define robot operation states using an enum
class States(Enum):
    APPROACH_BEARING = auto()
    GRIP_BEARING = auto()
    LIFT_BEARING = auto()
    FLIP_VACUUM = auto()
    APPROACH_LOAD = auto()
    LOAD = auto()
    EXIT_LOAD = auto()
    IDLE = auto()
    SCREW_TOP = auto()
    SCREW_BOTTOM = auto()
    BLOW_OFF = auto()


# Constants and state variables
NUM_TRAY_ROWS = 6
state = States.APPROACH_BEARING
entry_flag = 0  # Used to trigger one-time actions in each state
flip_flag = 0     # Used to determine if the vacuum tool should be flipped
motion_time = 0   # Used for timing transitions
num_load_moves = 1
substep_flag = 0
num_loads = 0

# Predefined joint positions for the robot
BEARING_APPROACH = [-72.49, -136.82, -107.21, 70.10, 27.81, -96.38]
FLIPPED_BEARING_APPROACH = [-75.80, -146.80, -81.78, 48.47, -56.85, -268.48]
FLIPPED_BEARING_LEAVE = [-81.04, -120.42, -81.23, 30.41, -52.31, -270.96]

APPROACH_TOMB_FIRST = [-37.14, -119.54, -90.06, -91.28, -52.96, -221.38]
APPROACH_TOMB = [-50.96, -117.67, -112.97, -77.70, -60.86, -207.82]

EXIT_LOAD_FIRST = [-20.42, -108.52, -107.46, -71.78, -45.99, -241.48]
EXIT_LOAD = [-29.34, -106.71, -134.93, -53.71, -49.28, -230.25]

TOP_SCREW = [-36.61, -125.34, -57.59, -231.14, -52.65, -321.14]
BLOW_TOP = [-40.14, -109.44, -95.32, -97.97, -54.55, -218.23]


# Initialize the robot arm, tool, frame, and IO
def init_robot() -> RDK.Item:
    robot = RDK.Item("", robolink.ITEM_TYPE_ROBOT)
    robot.setTool(robomath.Pose(0, 70, 70, -45, 0, 0))

    RDK.ShowMessage("Trying to connect to %s..." % robot.Name())
    robot.Connect()
    time.sleep(3)  # Wait for robot to connect

    # Set digital outputs: 0 and 1 control vacuum, 2 is blow-off nozzle
    robot.setDO(io_value=1, io_var=0)
    robot.setDO(io_value=1, io_var=1)
    robot.setDO(io_value=0, io_var=2)
    return robot


# Move the robot to a known safe home position
def go_home(robot: RDK.Item):
    robot.MoveJ([-90, -90, -90, 0, 90, 0])
    print('Moved to Home Position')
    time.sleep(5)


# Move robot to a position with a specified offset, non-blocking
def non_blocking_move(robot: RDK.Item, offset: list, start_pos: list):
    global motion_time, entry_flag
    pos = [a + b for a, b in zip(start_pos, offset)]  # Calculate target joint position
    robot.MoveJ(pos, blocking=False)
    motion_time = time.perf_counter()
    entry_flag = 1  # Signal that the motion was initiated


# Main state machine for managing robot tasks
def state_machine(robot: RDK.Item, num_unloads: int):
    global entry_flag, state, motion_time, flip_flag, num_load_moves, substep_flag, num_loads

    # Determine joint offsets for de-palletization
    offset = num_unloads % 2  # TODO: CHANGE 2 BACK TO 6 WHEN GRIPPER FIXED (NUM_TRAY_ROWS)

    # Decide the offset for gripping positions and whether to flip the tool
    if num_unloads % 2:
        bearing_offset = [-2.32 * offset, -1.0 * offset, 0.70 * offset, -0.60 * offset, 2.44 * offset, -0.30 * offset]
        state = States.FLIP_VACUUM if not flip_flag else state
    else:
        bearing_offset = [-2.55 * offset, 0 * offset, -0.39 * offset, 0.13 * offset, 3.23 * offset, -0.07 * offset]

    # Handle behavior for each state
    match state:
        case States.FLIP_VACUUM:
            if not entry_flag:
                flip_flag = 1
                entry_flag = 1
                robot.MoveJ(FLIPPED_BEARING_LEAVE, blocking=False)

            if not robot.Busy():
                state = States.APPROACH_BEARING
                entry_flag = 0

        case States.APPROACH_BEARING:
            if not entry_flag:
                if flip_flag:
                    robot.MoveJ(FLIPPED_BEARING_APPROACH, blocking=False)
                else:
                    robot.MoveJ(BEARING_APPROACH, blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not robot.Busy():
                if not substep_flag:
                    # robot.MoveJ(robot.Pose() * robomath.transl(0, 0, 25 * offset), blocking=False)
                    substep_flag = 1
                if substep_flag and not robot.Busy():
                    state = States.GRIP_BEARING
                    entry_flag = 0
                    substep_flag = 0

        case States.GRIP_BEARING:
            if not entry_flag:
                robot.MoveJ(robot.Pose() * robomath.transl(0, 0, (53 + (25 * offset))), blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not robot.Busy():
                robot.setDO(io_value=0, io_var=1)
                if flip_flag:
                    robot.setDO(io_value=0, io_var=0)
                if (time.perf_counter() - motion_time) >= 2:
                    state = States.LIFT_BEARING
                    entry_flag = 0

        case States.LIFT_BEARING:
            if not entry_flag:
                if flip_flag:
                    robot.MoveJ(robot.Pose() * robomath.transl(-180, 0, 0), blocking=False)
                else:
                    robot.MoveJ(robot.Pose() * robomath.transl(180, 0, 0), blocking=False)
                entry_flag = 1

            if not robot.Busy():
                # If flipped, go to load; else, repeat the pick
                state = States.APPROACH_LOAD if flip_flag else States.APPROACH_BEARING
                num_unloads += 1
                entry_flag = 0
                flip_flag = 0

        case States.APPROACH_LOAD:
            if not entry_flag:
                if not num_loads:
                    robot.MoveJ(EXIT_LOAD_FIRST, blocking=False)
                else:
                    robot.MoveJ(EXIT_LOAD, blocking=False)
                entry_flag = 1

            if not robot.Busy():
                state = States.LOAD
                entry_flag = 0

        case States.LOAD:
            if not entry_flag:
                if not num_loads:
                    robot.MoveJ(APPROACH_TOMB_FIRST, blocking=False)
                else:
                    robot.MoveJ(APPROACH_TOMB, blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 1:
                robot.setSpeed(speed_joints=5, speed_linear=0.0001)
                robot.MoveJ(robot.Pose() * robomath.transl(-5.5, 0, 0), blocking=False)
                motion_time = time.perf_counter()
                num_load_moves += 1

                if num_load_moves > 9:
                    state = States.EXIT_LOAD
                    entry_flag = 0
                    num_load_moves = 0

        case States.EXIT_LOAD:
            if not entry_flag:
                if not num_loads:
                    robot.setDO(io_value=1, io_var=0)
                    robot.MoveJ(EXIT_LOAD_FIRST, blocking=False)
                else:
                    robot.setDO(io_value=1, io_var=1)
                    robot.MoveJ(EXIT_LOAD, blocking=False)
                entry_flag = 1

            if not robot.Busy():
                robot.setSpeed(speed_joints=40, speed_linear=0.06)
                if not num_loads:
                    state = States.APPROACH_LOAD
                    num_loads = 1
                else:
                    state = States.SCREW_TOP
                entry_flag = 0

        case States.SCREW_TOP:
            if not entry_flag:
                robot.MoveJ(TOP_SCREW, blocking=False)
                entry_flag = 1
                motion_time = time.perf_counter()
                robot.setDO(io_value=1, io_var=6)

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 6:
                robot.setDO(io_value=0, io_var=6)
                entry_flag = 0
                state = States.SCREW_BOTTOM

        case States.SCREW_BOTTOM:
            if not entry_flag:
                robot.MoveJ(robot.Pose() * robomath.transl(0, 114, 0), blocking=False)
                entry_flag = 1
                motion_time = time.perf_counter()
                robot.setDO(io_value=1, io_var=6)

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 3:
                robot.setDO(io_value=0, io_var=6)
                entry_flag = 0
                state = States.IDLE

        case States.IDLE:
            if not entry_flag:
                robot.MoveJ([-90, -90, -90, 0, 90, 0], blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 10:
                state = States.BLOW_OFF
                entry_flag = 0

        case States.BLOW_OFF:
            if not entry_flag:
                robot.MoveJ(BLOW_TOP, blocking=False)
                robot.setDO(io_value=1, io_var=2)
                entry_flag = 1

            if not robot.Busy():
                if not substep_flag:
                    robot.setSpeed(speed_joints=5, speed_linear=0.0001)
                    robot.MoveJ(robot.Pose() * robomath.transl(0, 200, 0), blocking=False)
                    substep_flag = 1
                if substep_flag and not robot.Busy():
                    robot.setSpeed(speed_joints=40, speed_linear=0.06)
                    state = States.IDLE
                    robot.setDO(io_value=0, io_var=2)
                    entry_flag = 0
                    substep_flag = 0

    return num_unloads


# Entry point of the script
def main():
    robot = init_robot()
    robot.setSpeed(speed_joints=40, speed_linear=0.07)
    go_home(robot)
    num_unloads = 0

    # robot.MoveJ(APPROACH_TOMB_FIRST)
    # robot.MoveJ(robot.Pose() * robomath.transl(0, 2, 0))

    # robot.MoveJ(APPROACH_TOMB_FIRST)
    # robot.MoveJ(robot.Pose() * robomath.transl(-10, 0, 0))
    # robot.setDO(io_value=0, io_var=1)
    # robot.setDO(io_value=0, io_var=0)
    # robot.MoveJ(APPROACH_TOMB_FIRST)
    # robot.MoveJ(robot.Pose() * robomath.transl(0, 0, -5))

    # robot.MoveJ(robot.Pose() * robomath.transl(10, 0, 0))
    # robot.MoveJ(robot.Pose() * robomath.transl(0, 0, 53))
    # robot.setDO(io_value=0, io_var=1)
    # time.sleep(3)
    # robot.MoveJ(robot.Pose() * robomath.transl(180, 0, 0))
    # robot.setDO(io_value=1, io_var=1)

    # Continuously run state machine to perform task sequence
    while True:
        num_unloads = state_machine(robot, num_unloads)


    # IO LOGIC
    # robot.setDO(io_value=0, io_var=1)  # First, set D_O0 to low (activate vacuum)
    # robot.setDO(io_value=0, io_var=0)  # Then D_O1 can be set to low (activate vacuum)

    # robot.setDO(io_value=1, io_var=1)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=0)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=2)  # Activate blow off nozzle


# Run main if script is executed directly
if __name__ == '__main__':
    main()
