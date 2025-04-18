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
    LOAD_STAGE_1 = auto()
    LOAD_STAGE_2 = auto()
    LOAD_STAGE_3 = auto()
    EXIT_LOAD = auto()


# Constants and state variables
NUM_TRAY_ROWS = 6
state = States.APPROACH_BEARING
entry_flag = 0  # Used to trigger one-time actions in each state
flip_flag = 0     # Used to determine if the vacuum tool should be flipped
motion_time = 0   # Used for timing transitions

# Predefined joint positions for the robot
# BEARING_APPROACH = [-72.08, -136.94, -107.35, 72.62, 28.90, -101.10]
# BEARING_CONTACT = [-76.86, -138.17, -107.35, 72.01, 33.81, -101.37]
# BEARING_LEAVE = [-78.65, -116.62, -107.35, 45.52, 34.38, -92.97]

# BEARING_APPROACH = robomath.xyzrpw_2_pose([-150.174, -565.479, 214.394, 0.932, -84.477, 1.286])
BEARING_CONTACT = robomath.xyzrpw_2_pose([-189.766, -561.344, 205.956, 2.439, -83.195, 0.019])
BEARING_APPROACH = BEARING_CONTACT * robomath.transl(0, 0, -50)
BEARING_LEAVE = BEARING_CONTACT * robomath.transl(180, 0, 0)

# FLIPPED_BEARING_APPROACH = robomath.xyzrpw_2_pose([-150.025, -565.479, 214.394, 124.553, 1.528, -126.685])
# FLIPPED_BEARING_CONTACT = robomath.xyzrpw_2_pose([-201.302, -564.701, 206.736, 128.401, 1.997, -122.804])
# FLIPPED_BEARING_APPROACH = FLIPPED_BEARING_CONTACT * robomath.transl(0, 0, -50)
# FLIPPED_BEARING_LEAVE = FLIPPED_BEARING_CONTACT * robomath.transl(180, 0, 0)
FLIPPED_BEARING_APPROACH = [-75.78, -145.76, -82.02, 47.66, -56.88, -268.48]
FLIPPED_BEARING_CONTACT = [-80.42, -145.67, -81.23, 49.79, -52.09, -272.93]
FLIPPED_BEARING_LEAVE = [-81.04, -120.42, -81.23, 30.41, -52.31, -270.96]

APPROACH_TOMB = [-55.80, -122.43, -112.67, -76.41, -70.98, -204.38]
LOAD1 = [-52.23, -123.73, -110.89, -76.51, -66.93, -205.80]
LOAD2 = [-48.45, -124.34, -109.89, -74.35, -61.02, -209.74]
EXIT_LOAD = [-43.43, -116.53, -123.78, -68.05, -59.07, -212.17]

FLIPPED_APPROACH_TOMB = [-54.98, -124.79, -77.34, -207.92, -67.75, 21.85]
FLIPPED_EXIT_LOAD = [-41.64, -117.36, -90.60, -207.93, -57.33, 34.36]

APPROACH_TOP_SCREW = [-39.86, -108.00, -107.55, -15.31, 61.30, -145.73]
TOP_SCREW = [-44.62, -118.14, -92.73, -19.66, 63.11, -148.56]


# Initialize the robot arm, tool, frame, and IO
def init_robot() -> RDK.Item:
    robot = RDK.Item("", robolink.ITEM_TYPE_ROBOT)
    robot.setTool(robomath.Pose(0, 70, 70, -45, 0, 0))
    robot.setSpeed(speed_joints=40, speed_linear=0.06)

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
    global entry_flag, state, motion_time, flip_flag

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
                robot.MoveJ(FLIPPED_BEARING_LEAVE)

            if not robot.Busy():
                state = States.APPROACH_BEARING
                entry_flag = 0

        case States.APPROACH_BEARING:
            if not entry_flag:
                if flip_flag:
                    # robot.MoveL((FLIPPED_BEARING_APPROACH * robomath.transl(0, 0, 25 * offset)), blocking=False)
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_APPROACH)
                else:
                    robot.MoveL((BEARING_APPROACH * robomath.transl(0, 0, 25 * offset)), blocking=False)
                    # non_blocking_move(robot, bearing_offset, BEARING_APPROACH)

            if not robot.Busy():
                state = States.GRIP_BEARING
                entry_flag = 0

        case States.GRIP_BEARING:
            if not entry_flag:
                if flip_flag:
                    # robot.MoveL((FLIPPED_BEARING_CONTACT * robomath.transl(0, 0, 25 * offset)), blocking=False)
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_CONTACT)
                else:
                    robot.MoveL((BEARING_CONTACT * robomath.transl(0, 0, 25 * offset)), blocking=False)
                    # non_blocking_move(robot, bearing_offset, BEARING_CONTACT)

            if not robot.Busy():
                robot.setDO(io_value=0, io_var=1)
                if flip_flag:
                    robot.setDO(io_value=0, io_var=0)
                if (time.perf_counter() - motion_time) >= 3:
                    state = States.LIFT_BEARING
                    entry_flag = 0

        case States.LIFT_BEARING:
            if not entry_flag:
                if flip_flag:
                    # robot.MoveL((FLIPPED_BEARING_LEAVE * robomath.transl(0, 0, 25 * offset)), blocking=False)
                    non_blocking_move(robot, bearing_offset, FLIPPED_BEARING_LEAVE)
                else:
                    robot.MoveL((BEARING_LEAVE * robomath.transl(0, 0, 25 * offset)), blocking=False)
                    # non_blocking_move(robot, bearing_offset, BEARING_LEAVE)

            if not robot.Busy():
                # If flipped, go to load; else, repeat the pick
                state = States.APPROACH_LOAD if flip_flag else States.APPROACH_BEARING
                num_unloads += 1
                entry_flag = 0
                flip_flag = 0

        case States.APPROACH_LOAD:
            if not entry_flag:
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], EXIT_LOAD)

            if not robot.Busy():
                state = States.LOAD_STAGE_1
                entry_flag = 0

        case States.LOAD_STAGE_1:
            if not entry_flag:
                robot.setSpeed(speed_joints=5, speed_linear=0.0001)
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], APPROACH_TOMB)

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 2:
                state = States.LOAD_STAGE_2
                entry_flag = 0

        case States.LOAD_STAGE_2:
            if not entry_flag:
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], LOAD1)

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 2:
                state = States.LOAD_STAGE_3
                entry_flag = 0

        case States.LOAD_STAGE_3:
            if not entry_flag:
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], LOAD2)

            if not robot.Busy():
                robot.setDO(io_value=1, io_var=1)
                # if flip_flag:
                #     robot.setDO(io_value=1, io_var=0)
                state = States.EXIT_LOAD
                entry_flag = 0

        case States.EXIT_LOAD:
            if not entry_flag:
                non_blocking_move(robot, [0, 0, 0, 0, 0, 0], EXIT_LOAD)

            if not robot.Busy():
                robot.setSpeed(speed_joints=40, speed_linear=0.06)
                state = States.EXIT_LOAD
                entry_flag = 0

    return num_unloads


# Entry point of the script
def main():
    robot = init_robot()
    go_home(robot)
    num_unloads = 0

    # Continuously run state machine to perform task sequence
    while True:
        num_unloads = state_machine(robot, num_unloads)

    # relative_pose = robomath.transl(0, 0, 100)
    # robot.MoveL(robot.Pose() * relative_pose)
    #
    # print("Current tool pose:", robot.PoseTool())

    # IO LOGIC
    # robot.setDO(io_value=0, io_var=1)  # First, set D_O0 to low (activate vacuum)
    # robot.setDO(io_value=0, io_var=0)  # Then D_O1 can be set to low (activate vacuum)

    # robot.setDO(io_value=1, io_var=1)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=0)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=2)  # Activate blow off nozzle


# Run main if script is executed directly
if __name__ == '__main__':
    main()
