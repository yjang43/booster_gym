import pickle
from scipy.interpolate import interp1d, CubicSpline
from loguru import logger
import numpy as np
import time
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    LowState,
    B1JointCnt,
    RobotMode,
    LowCmdType,
    MotorCmd,
)

from const import *


def init_motor_cmd(low_cmd: LowCmd):
    low_cmd.cmd_type = LowCmdType.SERIAL
    motorCmds = [MotorCmd() for _ in range(B1JointCnt)]
    low_cmd.motor_cmd = motorCmds

    for i in range(B1JointCnt):
        low_cmd.motor_cmd[i].q = 0.0
        low_cmd.motor_cmd[i].dq = 0.0
        low_cmd.motor_cmd[i].tau = 0.0
        low_cmd.motor_cmd[i].kp = 0.0
        low_cmd.motor_cmd[i].kd = 0.0
        # weight is not effective in custom mode
        low_cmd.motor_cmd[i].weight = 0.0

class Puppeteer:
    # MASK = (
    #     False, False,
    #     True, True, True, True,
    #     True, True, True, True,
    #     False,
    #     False, False, False, False, False, False,
    #     False, False, False, False, False, False
    # )
    STIFFNESS = (
        20, 20,
        20, 20, 20, 20,
        20, 20, 20, 20,
        # 50, 50, 50, 50,
        # 50, 50, 50, 50,
        200,
        200, 200, 200, 200, 50, 50,
        200, 200, 200, 200, 50, 50
    )
    # STIFFNESS = (
    #     0, 0,
    #     0, 0, 0, 0,
    #     0, 0, 0, 0,
    #     0,
    #     200, 200, 200, 200, 50, 50,
    #     200, 200, 200, 200, 50, 50
    # )
    # DAMPING = (
    #     2, 2,
    #     20, 20, 20, 20,
    #     20, 20, 20, 20,
    #     5,
    #     5, 5, 5, 5, 3, 3,
    #     5, 5, 5, 5, 3, 3
    # )
    # DAMPING = (
    #     0, 0,
    #     0, 0, 0, 0,
    #     0, 0, 0, 0,
    #     0,
    #     0, 0, 0, 0, 0, 0,
    #     0, 0, 0, 0, 0, 0
    # )
    # DAMPING = (
    #     2, 2,
    #     4, 4, 4, 4,
    #     4, 4, 4, 4,
    #     5,
    #     5, 5, 5, 5, 3, 3,
    #     5, 5, 5, 5, 3, 3
    # )
    DAMPING = (
        0.2, 0.2,
        0.5, 0.5, 0.5, 0.5,
        0.5, 0.5, 0.5, 0.5,
        5,
        5, 5, 5, 5, 3, 3,
        5, 5, 5, 5, 3, 3
    )
    LOW_STIFFNESS = (
        10, 10,
        10, 10, 10, 10,
        10, 10, 10, 10,
        200,
        200, 200, 200, 200, 50, 50,
        200, 200, 200, 200, 50, 50
    )
    HIGH_DAMPING = (
        2, 2,
        5, 5, 5, 5,
        5, 5, 5, 5,
        5,
        5, 5, 5, 5, 3, 3,
        5, 5, 5, 5, 3, 3
    )
    DEFAULT_QPOS = [
        0.0,  0.0,
        0.25, -1.4, 0.0, -0.5,
        0.25, 1.4, 0.0, 0.5,
        0.0,
        -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
        -0.1, 0.0, 0.0, 0.2, -0.1, 0.0,
    ]


    def __init__(self, ckpt_path="") -> None:

        self._init_puppet()

        # Simple stiffness toggle
        self.stiffness_enabled = True

        # Body part to mask mapping
        self.body_part_masks = {
            'la': LEFT_ARM_MASK,    # left arm
            'ra': RIGHT_ARM_MASK,   # right arm
            'h': HEAD_MASK,         # head
            'w': WAIST_MASK,        # waist
            'll': LEFT_LEG_MASK,    # left leg
            'rl': RIGHT_LEG_MASK    # right leg
        }

        # Keyframes, a list of tuples of (time, dof_pos)
        if ckpt_path:
            import pickle
            with open(ckpt_path, 'rb') as f:
                self.keyframes = pickle.load(f)["puppeteer"]["keyframes"]
        else:
            self.keyframes = [(0.0, self.DEFAULT_QPOS)]

    def _init_puppet(self):
        try:
            # Initialize low state values
            self.dof_pos = np.zeros(B1JointCnt, dtype=np.float32)

            # Define low state handler
            def _low_state_handler(low_state_msg: LowState):
                for i, motor in enumerate(low_state_msg.motor_state_serial):
                    self.dof_pos[i] = motor.q

            ChannelFactory.Instance().Init(0)
            self.low_cmd = LowCmd()
            init_motor_cmd(self.low_cmd)
            self.low_state_subscriber = B1LowStateSubscriber(_low_state_handler)
            self.low_cmd_publisher = B1LowCmdPublisher()
            self.client = B1LocoClient()

            self.low_state_subscriber.InitChannel()
            self.low_cmd_publisher.InitChannel()
            self.client.Init()

            # Wait for communication to establish
            time.sleep(2)

            # Enter custom mode
            # Initialize motor commands with current positions
            for i in range(B1JointCnt):
                self.low_cmd.motor_cmd[i].q = self.dof_pos[i]
                self.low_cmd.motor_cmd[i].dq = 0.0
                self.low_cmd.motor_cmd[i].tau = 0.0
                self.low_cmd.motor_cmd[i].kp = self.STIFFNESS[i]
                self.low_cmd.motor_cmd[i].kd = self.DAMPING[i]

            # Send initial command
            self.low_cmd_publisher.Write(self.low_cmd)

            # Change to custom mode
            self.client.ChangeMode(RobotMode.kCustom)

            # Wait until joints reach desired positions
            # logger.info("Waiting for joints to reach initial positions...")
            # while True:
            #     max_error = 0.0
            #     for i in range(B1JointCnt):
            #         error = abs(self.dof_pos[i] - self.DEFAULT_QPOS[i])
            #         max_error = max(max_error, error)

            #     if max_error < 0.1:  # Within 0.1 radians
            #         break

            #     time.sleep(0.1)
            # logger.info("Joints reached initial positions")
        except Exception as e:
            logger.error(f"Failed to initialize communication: {e}")
            raise

    def _toggle_stiffness(self, body_part=None):
        """Toggle stiffness for all joints or enable only specific body part.

        Args:
            body_part (str, optional): Body part code ('la', 'ra', 'h', 'w', 'll', 'rl')
                                     If None, enables stiffness for all joints
        """
        if body_part is None:
            # Enable stiffness for all joints
            for i in range(B1JointCnt):
                self.low_cmd.motor_cmd[i].q = self.dof_pos[i]
                self.low_cmd.motor_cmd[i].kp = self.STIFFNESS[i]
                self.low_cmd.motor_cmd[i].kd = self.DAMPING[i]

            self.stiffness_enabled = True
            logger.info("Stiffness ENABLED for ALL joints - Robot holding position")

        else:
            # Disable all joints first, then enable only the specified body part
            if body_part not in self.body_part_masks:
                logger.error(f"Unknown body part: {body_part}")
                return False

            mask = self.body_part_masks[body_part]

            for i in range(B1JointCnt):
                if not mask[i]:
                    # Enable stiffness for this body part
                    self.low_cmd.motor_cmd[i].q = self.dof_pos[i]
                    self.low_cmd.motor_cmd[i].kp = self.STIFFNESS[i]
                    self.low_cmd.motor_cmd[i].kd = self.DAMPING[i]
                else:
                    # Disable stiffness for all other joints
                    self.low_cmd.motor_cmd[i].kp = 0.0
                    self.low_cmd.motor_cmd[i].kd = 0.5

            self.stiffness_enabled = False
            logger.info(f"Stiffness ENABLED only for {body_part} - other joints are compliant")

        self.low_cmd_publisher.Write(self.low_cmd)
        return True

    def goto_keyframe(self, i, safe=False):
        if i < 0 or i >= len(self.keyframes):
            raise IndexError(f"Keyframe index {i} out of range [0, {len(self.keyframes)-1}]")

        # Get the keyframe (time, dof_pos)
        t, target_dof_pos = self.keyframes[i]
        logger.debug(f"at\t{i}th-keyframe\t{t}sec")

        # Set motor commands to the keyframe positions
        for j in range(B1JointCnt):
            self.low_cmd.motor_cmd[j].q = target_dof_pos[j]
            # NOTE: corl
            # self.low_cmd.motor_cmd[j].q = target_dof_pos[j] * UPPER_BODY_MASK[j]
            self.low_cmd.motor_cmd[j].dq = 0.0
            self.low_cmd.motor_cmd[j].tau = 0.0
            self.low_cmd.motor_cmd[j].kp = self.STIFFNESS[j] if not safe else self.LOW_STIFFNESS[j]
            self.low_cmd.motor_cmd[j].kd = self.DAMPING[j] if not safe else self.HIGH_DAMPING[j]

        # Send the command
        self.low_cmd_publisher.Write(self.low_cmd)

    def add_keyframe(self, t):
        # Create a keyframe with current dof positions
        keyframe = (t, self.dof_pos.copy())

        # Binary search to find insertion point
        left, right = 0, len(self.keyframes)
        while left < right:
            mid = (left + right) // 2
            if self.keyframes[mid][0] < t:
                left = mid + 1
            else:
                right = mid

        # Insert at the correct position to maintain sorted order
        self.keyframes.insert(left, keyframe)
        logger.debug(f"added\t{left}th-keyframe\t{t}sec")
        return left

    def remove_keyframe(self, i):
        if i < 0 or i >= len(self.keyframes):
            raise IndexError(f"Keyframe index {i} out of range [0, {len(self.keyframes)-1}]")

        t, _ = self.keyframes[i]

        # Prevent removing the first keyframe at time 0
        if i == 0 and t == 0.0:
            logger.warning("Cannot remove the initial keyframe at time 0")
            return

        self.keyframes.pop(i)
        logger.debug(f"removed\t{i}th-keyframe\t{t}sec")

    def edit_keyframe(self, i):
        if i < 0 or i >= len(self.keyframes):
            raise IndexError(f"Keyframe index {i} out of range [0, {len(self.keyframes)-1}]")

        t, _ = self.keyframes[i]
        # Update the keyframe with current robot position
        self.keyframes[i] = (t, self.dof_pos.copy())
        logger.debug(f"edited\t{i}th-keyframe\t{t}sec")

    def search(self, t, safe=False):
        """Go to interpolated position at given time."""
        if len(self.keyframes) < 2:
            logger.warning("Need at least 2 keyframes to interpolate")
            return

        # Extract times and positions from keyframes
        times, positions = zip(*self.keyframes)
        times = np.array(times)
        positions = np.array(positions)

        # Check if time is within range
        if t < times[0] or t > times[-1]:
            logger.warning(f"Time {t} is outside keyframe range [{times[0]}, {times[-1]}]")
            return

        # Create interpolation function
        interp_func = interp1d(times, positions, axis=0)

        # Get interpolated position at time t
        target_dof_pos = interp_func(t)

        logger.debug(f"goto time\t{t}sec")

        # Set motor commands to the interpolated positions
        for j in range(B1JointCnt):
            self.low_cmd.motor_cmd[j].q = target_dof_pos[j]
            self.low_cmd.motor_cmd[j].dq = 0.0
            self.low_cmd.motor_cmd[j].tau = 0.0
            self.low_cmd.motor_cmd[j].kp = self.STIFFNESS[j] if not safe else self.LOW_STIFFNESS[j]
            self.low_cmd.motor_cmd[j].kd = self.DAMPING[j] if not safe else self.HIGH_DAMPING[j]

        # Send the command
        self.low_cmd_publisher.Write(self.low_cmd)

    def play(self, fps, speed=1.0):
        """Play the interpolated keyframe trajectory at given fps. Press 'q' to quit."""
        import select
        import sys

        result = self._interp_traj(fps)
        if result is None:
            logger.warning("Need at least 2 keyframes to play")
            return

        time_points, interpolated_positions = result
        dt = (1.0 / fps) / speed  # Adjust timing based on speed

        # Go to first frame and wait
        logger.info(f"Going to first frame...")
        # first_positions = interpolated_positions[0]
        self.goto_keyframe(0, safe=True)
        time.sleep(1.0)  # Wait 1 second

        logger.info(f"Playing {len(time_points)} frames at {fps} fps (press 'q' to quit)")

        for i, positions in enumerate(interpolated_positions):
            start_time = time.perf_counter()

            # Check for user input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.read(1)
                if user_input.lower() == 'q':
                    logger.info("Playback interrupted by user")
                    break

            # Set motor commands to interpolated positions
            for j in range(B1JointCnt):
                # self.low_cmd.motor_cmd[j].q = positions[j]
                # NOTE: safety
                self.low_cmd.motor_cmd[j].q = positions[j] * UPPER_BODY_MASK[j]
                self.low_cmd.motor_cmd[j].dq = 0.0
                self.low_cmd.motor_cmd[j].tau = 0.0
                self.low_cmd.motor_cmd[j].kp = self.STIFFNESS[j]
                self.low_cmd.motor_cmd[j].kd = self.DAMPING[j]

            # Send the command
            self.low_cmd_publisher.Write(self.low_cmd)

            # Sleep to maintain fps
            elapsed = time.perf_counter() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Playback completed")

    def export(self, filepath, fps):
        """Export interpolated trajectory and keyframes to a pickle file.

        Format:
        {
          "puppeteer": {
            "keyframes": [(time, dof_pos), ...],
            "dof": np.array(T, B1JointCnt)
          }
        }
        """

        result = self._interp_traj(fps)
        if result is None:
            logger.error("Need at least 2 keyframes to export")
            return

        time_points, interpolated_positions = result

        # Create the export dictionary
        export_data = {
            "puppeteer": {
                "keyframes": self.keyframes,
                "dof": interpolated_positions.astype(np.float32)
            }
        }

        # Save to pickle file
        try:
            with open(filepath, 'wb') as f:
                pickle.dump(export_data, f)
            logger.info(f"Exported {len(self.keyframes)} keyframes and {len(interpolated_positions)} frames to {filepath}")
        except Exception as e:
            logger.error(f"Failed to export to {filepath}: {e}")
            raise

    # def timecode_to_seconds(self, timecode: str) -> float:
    #     parts = timecode.split(":")
    #     if len(parts) != 2:
    #         raise ValueError(f"Invalid timecode format: {timecode}. Expected 'SS:FF'")

    #     try:
    #         seconds = int(parts[0])
    #         frames = int(parts[1])
    #     except ValueError:
    #         raise ValueError(f"Invalid timecode format: {timecode}. Seconds and frames must be integers")

    #     if frames < 0 or frames > 29:
    #         raise ValueError(f"Invalid frame number: {frames}. Frames must be between 0-29 for 30fps")

    #     if seconds < 0:
    #         raise ValueError(f"Invalid seconds: {seconds}. Seconds must be non-negative")

    #     frame_duration = 1.0 / 30
    #     total_seconds = seconds + (frames * frame_duration)

    #     return total_seconds

    def record(self, i):
        if i < len(self.keyframes):
            self.goto_keyframe(i, safe=True)

        while True:
            try:
                user_input = input(f"[{i}] (a <time> | r | g <index> | p <fps> [speed] | x <fps> <filepath> | e | s <time> | t [body_part] | q): ").strip().lower()

                if not user_input:
                    continue

                parts = user_input.split()
                command = parts[0]

                if command == 'q':
                    logger.debug("Quitting record session")
                    break

                elif command == 't':
                    if len(parts) == 1:
                        # Toggle all joints to stiff
                        self._toggle_stiffness()
                    elif len(parts) == 2:
                        # Toggle specific body part
                        body_part = parts[1]
                        if body_part in self.body_part_masks:
                            self._toggle_stiffness(body_part)
                            print(f"Move the {body_part} to desired position, then use 't' to enable all joints")
                        else:
                            print(f"Unknown body part: {body_part}. Valid options: la, ra, h, w, ll, rl")
                    else:
                        print("Invalid toggle command. Use 't' for all joints or 't <body_part>' for specific part")

                elif command == 'a' and len(parts) == 2:
                    time_val = float(parts[1])
                    i = self.add_keyframe(time_val)
                    # self.goto_keyframe(i)

                elif command == 'r':
                    t, _ = self.keyframes[i]
                    if i == 0 and t == 0.0:
                        print("Cannot remove the initial keyframe at time 0")
                    else:
                        self.remove_keyframe(i)
                        # Go to previous keyframe or stay at 0
                        i = max(0, i - 1)
                        self.goto_keyframe(i, safe=True)

                elif command == 'g' and len(parts) == 2:
                    target_i = int(parts[1])
                    if 0 <= target_i < len(self.keyframes):
                        i = target_i
                        self.goto_keyframe(i, safe=True)
                    else:
                        print(f"Index {target_i} out of range [0, {len(self.keyframes)-1}]")

                elif command == 'p' and len(parts) >= 2:
                    fps = float(parts[1])
                    speed = float(parts[2]) if len(parts) == 3 else 1.0
                    self.play(fps, speed)

                elif command == 'x' and len(parts) == 3:
                    fps = float(parts[1])
                    filepath = parts[2]
                    self.export(filepath, fps)

                elif command == 'e':
                    self.edit_keyframe(i)

                elif command == 's' and len(parts) == 2:
                    time_val = float(parts[1])
                    self.search(time_val, safe=True)

                else:
                    print("Invalid command. Use 'a <time>', 'r', 'g <index>', 'p <fps> [speed]', 'x <fps> <filepath>', 'e', 's <time>', 't [body_part]', or 'q'")
                    print("Body parts: la (left arm), ra (right arm), h (head), w (waist), ll (left leg), rl (right leg)")

            except KeyboardInterrupt:
                logger.debug("Record session interrupted")
                break
            except Exception as e:
                print(f"Error: {e}")


    def _interp_traj(self, fps):
        """Create interpolated trajectory from keyframes at given fps."""
        if len(self.keyframes) < 2:
            return None

        # Extract times and positions from keyframes using zip
        times, positions = zip(*self.keyframes)
        times = np.array(times)
        positions = np.array(positions)

        # Create cubic spline interpolation for all joints at once
        interp_func = interp1d(times, positions, axis=0)

        # Generate time points based on fps
        start_time = times[0]
        end_time = times[-1]
        num_points = int((end_time - start_time) * fps) + 1
        time_points = np.linspace(start_time, end_time, num_points)

        # Interpolate positions for all time points
        interpolated_positions = interp_func(time_points)

        return time_points, interpolated_positions


if __name__ == "__main__":
    # puppeteer = Puppeteer("ys.pkl")
    puppeteer = Puppeteer("merged_fixed.pkl")
    # puppeteer = Puppeteer()
    puppeteer.record(0)
