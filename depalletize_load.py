import csv
import os.path

from robolink import *
from robodk import *
import time
from enum import Enum
from enum import auto
from datetime import datetime

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
    APPROACH_UNLOAD = auto()
    UNLOAD = auto()
    EXIT_UNLOAD = auto()
    CYCLE_RESET = auto()


# Constants and state variables
NUM_TRAY_ROWS = 6
LOG_FILE = "robot_log.csv"
CHECKPOINT_FILE = "last_state.csv"
state = States.APPROACH_BEARING
entry_flag = 0  # Used to trigger one-time actions in each state
flip_flag = 0     # Used to determine if the vacuum tool should be flipped
motion_time = 0   # Used for timing transitions
num_load_moves = 1
substep_flag = 0
num_loads = 0
num_unloads = 0

# Predefined joint positions for the robot
BEARING_APPROACH = [-72.49, -136.82, -107.21, 70.10, 27.81, -96.38]
FLIPPED_BEARING_APPROACH = [-75.79, -146.28, -81.90, 48.07, -56.86, -268.48]
FLIPPED_BEARING_LEAVE = [-81.04, -120.42, -81.23, 30.41, -52.31, -270.96]

APPROACH_TOMB_FIRST = [-37.14, -119.54, -90.06, -91.28, -52.96, -221.38]
APPROACH_TOMB = [-51.30, -118.13, -112.14, -78.23, -61.07, -207.52]

EXIT_LOAD_FIRST = [-20.42, -108.52, -107.46, -71.78, -45.99, -241.48]
EXIT_LOAD = [-29.34, -106.71, -134.93, -53.71, -49.28, -230.25]

APPROACH_SCREW = [-37.40, -126.55, -55.38, -231.65, -53.12, -321.94]
TOP_SCREW = [-36.61, -125.34, -57.59, -231.14, -52.65, -321.14]
BLOW_TOP = [-40.14, -109.44, -95.32, -97.97, -54.55, -218.23]

APPROACH_LOADED_TOP = [-31.12, -118.80, -91.73, -86.17, -50.06, -228.12]
GRIP_LOADED_TOP = [-34.83, -123.10, -84.25, -92.00, -51.80, -223.90]

GRIP_LOADED_BOTTOM = [-47.30, -120.16, -108.48, -78.00, -58.63, -211.19]


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


def log_state(state, num_unloads, flip_flag, num_load_moves, substep_flag, num_loads):
    timestamp = datetime.utcnow().isoformat()
    data = [timestamp, state.name, num_unloads, flip_flag, num_load_moves, substep_flag, num_loads]

    write_header = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, 'a') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp", "state", "num_unloads", "flip_flag", "num_load_moves", "substep_flag", "num_loads"])
        writer.writerow(data)


def load_last_checkpoint():
    if not os.path.exists(CHECKPOINT_FILE):
        return None
    with open(CHECKPOINT_FILE, newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows:
            return None
        row = rows[0]
        return row


# Main state machine for managing robot tasks
def state_machine(robot: RDK.Item, num_tray_unloads: int):
    global entry_flag, state, motion_time, flip_flag, num_load_moves, substep_flag, num_loads, num_unloads

    # Determine joint offsets for de-palletization
    offset = num_tray_unloads % 4  # TODO: CHANGE BACK TO 6 WHEN USING FULL TRAY (NUM_TRAY_ROWS)

    # Decide the offset for gripping positions and whether to flip the tool
    if num_tray_unloads % 2:
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
            num_loads = 0
            num_unloads = 0
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
                num_tray_unloads += 1
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
                robot.MoveJ(APPROACH_SCREW, blocking=False)
                entry_flag = 1

            if not substep_flag and entry_flag and not robot.Busy():
                robot.MoveJ(robot.Pose() * robomath.transl(0, 0, 20), blocking=False)
                robot.setDO(io_value=1, io_var=6)
                motion_time = time.perf_counter()
                substep_flag = 1

            if not robot.Busy() and substep_flag and (time.perf_counter() - motion_time) >= 3:
                robot.setDO(io_value=0, io_var=6)
                robot.MoveJ(robot.Pose() * robomath.transl(0, 0, -20), blocking=False)
                entry_flag = 0
                substep_flag = 0
                state = States.SCREW_BOTTOM

        case States.SCREW_BOTTOM:
            if not entry_flag and not robot.Busy():
                robot.MoveJ(robot.Pose() * robomath.transl(0, 114, 0), blocking=False)
                entry_flag = 1

            if not substep_flag and entry_flag and not robot.Busy():
                robot.MoveJ(robot.Pose() * robomath.transl(0, 0, 20), blocking=False)
                robot.setDO(io_value=1, io_var=6)
                motion_time = time.perf_counter()
                substep_flag = 1

            if not robot.Busy() and substep_flag and (time.perf_counter() - motion_time) >= 3:
                robot.setDO(io_value=0, io_var=6)
                entry_flag = 0
                substep_flag = 0
                state = States.IDLE

        case States.IDLE:
            if not entry_flag:
                # First move back to avoid collision with tombstone
                robot.MoveJ(robot.Pose() * robomath.transl(0, 0, -60), blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not substep_flag and not robot.Busy():
                # Move to safe home position
                robot.MoveJ([-90, -90, -90, 0, 90, 0], blocking=False)
                motion_time = time.perf_counter()
                substep_flag = 1

            # Simulate waiting for CNC machining to be done
            if substep_flag and entry_flag and not robot.Busy() and (time.perf_counter() - motion_time) >= 15:
                state = States.BLOW_OFF
                entry_flag = 0
                substep_flag = 0

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
                    state = States.APPROACH_UNLOAD
                    robot.setDO(io_value=0, io_var=2)
                    entry_flag = 0
                    substep_flag = 0

        case States.APPROACH_UNLOAD:
            if not entry_flag:
                if not num_unloads:
                    robot.MoveJ(EXIT_LOAD, blocking=False)
                else:
                    robot.MoveJ(EXIT_LOAD_FIRST, blocking=False)
                entry_flag = 1

            if not robot.Busy():
                state = States.UNLOAD
                robot.setDO(io_value=0, io_var=1)
                if num_unloads:
                    robot.setDO(io_value=0, io_var=0)
                entry_flag = 0

        case States.UNLOAD:
            if not entry_flag:
                if not num_unloads:
                    robot.MoveJ(GRIP_LOADED_BOTTOM, blocking=False)
                else:
                    robot.MoveJ(GRIP_LOADED_TOP, blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not robot.Busy() and (time.perf_counter() - motion_time) >= 1:
                robot.setSpeed(speed_joints=5, speed_linear=0.0001)
                robot.MoveJ(robot.Pose() * robomath.transl(-5.5, 0, 0), blocking=False)
                motion_time = time.perf_counter()
                num_load_moves += 1

                if num_load_moves > 11:
                    state = States.EXIT_UNLOAD
                    entry_flag = 0
                    num_load_moves = 0

        case States.EXIT_UNLOAD:
            if not entry_flag:
                robot.MoveJ(robot.Pose() * robomath.transl(0, 0, -50), blocking=False)
                entry_flag = 1

            if not robot.Busy():
                robot.setSpeed(speed_joints=40, speed_linear=0.06)
                if not num_unloads:
                    state = States.APPROACH_UNLOAD
                    num_unloads = 1
                else:
                    state = States.CYCLE_RESET
                entry_flag = 0

        case States.CYCLE_RESET:
            if not entry_flag:
                # First move back to avoid collision with tombstone
                robot.MoveJ(robot.Pose() * robomath.transl(0, -50, -200), blocking=False)
                motion_time = time.perf_counter()
                entry_flag = 1

            if not substep_flag and not robot.Busy():
                # Move to safe home position
                robot.MoveJ([-90, -90, -90, 0, 90, 0], blocking=False)
                motion_time = time.perf_counter()
                substep_flag = 1

            if substep_flag and entry_flag and not robot.Busy() and (time.perf_counter() - motion_time) >= 10:
                state = States.APPROACH_BEARING
                robot.setDO(io_value=1, io_var=0)
                robot.setDO(io_value=1, io_var=1)
                robot.setDO(io_value=0, io_var=2)
                entry_flag = 0
                substep_flag = 0

    return num_tray_unloads


# Entry point of the script
def main():
    global state, entry_flag, flip_flag, num_load_moves, substep_flag, num_loads
    robot = init_robot()
    robot.setSpeed(speed_joints=40, speed_linear=0.07)

    # UNCOMMENT TO ENABLE CSV LOGGING (DISABLED FOR CAPSTONE DEMO)
    # if robot.Pose() != xyzrpw_2_pose([-133.301, -561.854, 617.195, 0.000, -166.299, 68.882]):
    #     checkpoint = load_last_checkpoint()
    #     if checkpoint:
    #         state = States[checkpoint["state"]]
    #         num_unloads = int(checkpoint["num_unloads"])
    #         flip_flag = int(checkpoint["flip_flag"])
    #         num_load_moves = int(checkpoint["num_load_moves"])
    #         substep_flag = int(checkpoint["substep_flag"])
    #         num_loads = int(checkpoint["num_loads"])
    #         print(f"Resuming from state: {state.name}")
    #     else:
    #         num_unloads = 0
    # else:
    #     num_unloads = 0

    go_home(robot)
    num_tray_unloads = 0

    # Continuously run state machine to perform task sequence
    while True:
        num_tray_unloads = state_machine(robot, num_tray_unloads)
        # log_state(state, num_unloads, flip_flag, num_load_moves, substep_flag, num_loads)


    # IO LOGIC
    # robot.setDO(io_value=0, io_var=1)  # First, set D_O0 to low (activate vacuum)
    # robot.setDO(io_value=0, io_var=0)  # Then D_O1 can be set to low (activate vacuum)

    # robot.setDO(io_value=1, io_var=1)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=0)  # Deactivate vacuum
    # robot.setDO(io_value=1, io_var=2)  # Activate blow off nozzle


# Run main if script is executed directly
if __name__ == '__main__':
    main()
