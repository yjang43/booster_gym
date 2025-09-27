import numpy as np
import time
import yaml
import logging
import threading
import pickle
import torch

from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    LowState,
    B1JointCnt,
    RobotMode,
)

from utils.command import create_prepare_cmd, create_first_frame_rl_cmd
from utils.remote_control_service import RemoteControlService
from utils.rotate import rotate_vector_inverse_rpy
from utils.timer import TimerConfig, Timer
from utils.policy import Policy


UPPER_BODY_INDICES_HIP = [ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10]
UPPER_BODY_INDICES_KNEE = [ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 17, 14, 20]
UPPER_BODY_INDICES_ANKLE =  [0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 17, 14, 20, 12, 13, 18, 19]

class Controller:
    def __init__(self, cfg_file) -> None:
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Load config
        with open(cfg_file, "r", encoding="utf-8") as f:
            self.cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

        # Initialize components
        self.remoteControlService = RemoteControlService()
        self.policy = Policy(cfg=self.cfg)

        self._init_timer()
        self._init_low_state_values()
        self._init_motion()
        self._init_communication()
        self.publish_runner = None
        self.running = True

        self.publish_lock = threading.Lock()

    def _init_timer(self):
        self.timer = Timer(TimerConfig(time_step=self.cfg["common"]["dt"]))
        self.next_publish_time = self.timer.get_time()
        self.next_inference_time = self.timer.get_time()

    def _init_low_state_values(self):
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.dof_pos = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_vel = np.zeros(B1JointCnt, dtype=np.float32)

        self.dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.filtered_dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_pos_latest = np.zeros(B1JointCnt, dtype=np.float32)

    def _init_motion(self):
        motion_file = self.cfg["motion"]["file"]
        self.motion_fps = self.cfg["motion"]["fps"]
        with open(motion_file, "rb") as f:
            motion = pickle.load(f)
        motion = motion[next(iter(motion))]

        # Keep motion data on CPU initially, will move to CUDA when accessed
        # for k in motion:
        #     if isinstance(motion[k], np.ndarray):
        #         motion[k] = torch.from_numpy(motion[k]).float()

        # Motion data is already in unnormalized absolute positions (same format as policy output)
        self.motion = motion
        self.motion_fps = self.cfg["motion"]["fps"]
        # Calculate motion time scale to handle frequency mismatch
        control_dt = self.cfg["common"]["dt"] * self.cfg["policy"]["control"]["decimation"]
        motion_dt = 1.0 / self.motion_fps
        self.motion_time_scale = motion_dt / control_dt

        self.motion_step = 0
        # self.upper_body_dof_indices = list(range(11))
        self.upper_body_dof_indices = self.cfg["policy"]["upper_body_dof_indices"]

    def _init_communication(self) -> None:
        try:
            self.low_cmd = LowCmd()
            self.low_state_subscriber = B1LowStateSubscriber(self._low_state_handler)
            self.low_cmd_publisher = B1LowCmdPublisher()
            self.client = B1LocoClient()

            self.low_state_subscriber.InitChannel()
            self.low_cmd_publisher.InitChannel()
            self.client.Init()
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise

    def _low_state_handler(self, low_state_msg: LowState):
        # if abs(low_state_msg.imu_state.rpy[0]) > 1.0 or abs(low_state_msg.imu_state.rpy[1]) > 1.0:
        #     self.logger.warning("IMU base rpy values are too large: {}".format(low_state_msg.imu_state.rpy))
        #     self.running = False
        self.timer.tick_timer_if_sim()
        time_now = self.timer.get_time()
        for i, motor in enumerate(low_state_msg.motor_state_serial):
            self.dof_pos_latest[i] = motor.q
        if time_now >= self.next_inference_time:
            self.projected_gravity[:] = rotate_vector_inverse_rpy(
                low_state_msg.imu_state.rpy[0],
                low_state_msg.imu_state.rpy[1],
                low_state_msg.imu_state.rpy[2],
                np.array([0.0, 0.0, -1.0]),
            )
            self.base_ang_vel[:] = low_state_msg.imu_state.gyro
            for i, motor in enumerate(low_state_msg.motor_state_serial):
                self.dof_pos[i] = motor.q
                self.dof_vel[i] = motor.dq

    def _send_cmd(self, cmd: LowCmd):
        self.low_cmd_publisher.Write(cmd)

    def cleanup(self) -> None:
        """Cleanup resources."""
        self.remoteControlService.close()
        if hasattr(self, "low_cmd_publisher"):
            self.low_cmd_publisher.CloseChannel()
        if hasattr(self, "low_state_subscriber"):
            self.low_state_subscriber.CloseChannel()
        if hasattr(self, "publish_runner") and getattr(self, "publish_runner") != None:
            self.publish_runner.join(timeout=1.0)

    def start_custom_mode_conditionally(self):
        print(f"{self.remoteControlService.get_custom_mode_operation_hint()}")
        while True:
            if self.remoteControlService.start_custom_mode():
                break
            time.sleep(0.1)
        method_start = time.perf_counter()

        # Time create_prepare_cmd
        t0 = time.perf_counter()
        create_prepare_cmd(self.low_cmd, self.cfg)
        t1 = time.perf_counter()
        print(f"  create_prepare_cmd took {(t1 - t0)*1000:.4f} ms")

        # Time target initialization
        t0 = time.perf_counter()
        for i in range(B1JointCnt):
            self.dof_target[i] = self.low_cmd.motor_cmd[i].q
            self.filtered_dof_target[i] = self.low_cmd.motor_cmd[i].q
        t1 = time.perf_counter()
        print(f"  target initialization took {(t1 - t0)*1000:.4f} ms")

        # policy_targets[self.upper_body_dof_indices] = self.motion["dof"][0][self.upper_body_dof_indices]
        # for i in range(B1JointCnt):
        # for i in self.upper_body_dof_indices:
        #     self.dof_target[i] = self.motion["dof"][0][i]
        #     self.filtered_dof_target[i] = self.motion["dof"][0][i]
        #     self.low_cmd.motor_cmd[i].q = self.motion["dof"][0][i]

        # Time _send_cmd
        t0 = time.perf_counter()
        self._send_cmd(self.low_cmd)
        t1 = time.perf_counter()
        print(f"  _send_cmd took {(t1 - t0)*1000:.4f} ms")

        # Time ChangeMode
        t0 = time.perf_counter()
        self.client.ChangeMode(RobotMode.kCustom)
        t1 = time.perf_counter()
        print(f"  ChangeMode(kCustom) took {(t1 - t0)*1000:.4f} ms")

        print(f"  TOTAL start_custom_mode took {(time.perf_counter() - method_start)*1000:.4f} ms")

    def start_rl_gait_conditionally(self):
        print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
        while True:
            if self.remoteControlService.start_rl_gait():
                break
            time.sleep(0.1)
        method_start = time.perf_counter()

        # Time create_first_frame_rl_cmd
        t0 = time.perf_counter()
        create_first_frame_rl_cmd(self.low_cmd, self.cfg)
        t1 = time.perf_counter()
        print(f"  create_first_frame_rl_cmd took {(t1 - t0)*1000:.4f} ms")

        # Time _send_cmd
        t0 = time.perf_counter()
        self._send_cmd(self.low_cmd)
        t1 = time.perf_counter()
        print(f"  _send_cmd took {(t1 - t0)*1000:.4f} ms")

        # Time timer updates
        t0 = time.perf_counter()
        self.next_inference_time = self.timer.get_time()
        self.next_publish_time = self.timer.get_time()
        t1 = time.perf_counter()
        print(f"  timer updates took {(t1 - t0)*1000:.4f} ms")

        # Time thread creation and start
        t0 = time.perf_counter()
        self.publish_runner = threading.Thread(target=self._publish_cmd)
        self.publish_runner.daemon = True
        self.publish_runner.start()
        t1 = time.perf_counter()
        print(f"  thread creation and start took {(t1 - t0)*1000:.4f} ms")

        print(f"  TOTAL start_rl_gait took {(time.perf_counter() - method_start)*1000:.4f} ms")
        # print(f"{self.remoteControlService.get_operation_hint()}")

    def run(self):
        time_now = self.timer.get_time()
        if time_now < self.next_inference_time:
            time.sleep(0.001)
            return
        self.logger.debug("-----------------------------------------------------")
        self.next_inference_time += self.policy.get_policy_interval()
        self.logger.debug(f"Next start time: {self.next_inference_time}")
        start_time = time.perf_counter()

        policy_targets = self.policy.inference(
            time_now=time_now,
            dof_pos=self.dof_pos,
            dof_vel=self.dof_vel,
            base_ang_vel=self.base_ang_vel,
            projected_gravity=self.projected_gravity,
            # vx=self.remoteControlService.get_vx_cmd(),
            # vy=self.remoteControlService.get_vy_cmd(),
            # vyaw=self.remoteControlService.get_vyaw_cmd(),
            vx=0.0,
            vy=0.0,
            vyaw=0.0,
            last_dof_target=self.dof_target.copy()
        )

        # Override upper body joints with motion data
        motion_frame = np.clip(
            int(self.motion_step / self.motion_time_scale),
            0,
            len(self.motion["dof"]) - 1
        )

        if motion_frame >= len(self.motion["dof"]) - 1:
            self.running = False
            # self.client.ChangeMode(RobotMode.kPrepare)

        # policy_targets = np.zeros_like(policy_targets)
        policy_targets[self.upper_body_dof_indices] = self.motion["dof"][motion_frame][self.upper_body_dof_indices]
        # policy_targets[self.upper_body_dof_indices] = self.motion["dof"][0][self.upper_body_dof_indices]
        # policy_targets[self.upper_body_dof_indices] = np.array(self.cfg["common"]["default_qpos"], dtype=np.float32)[self.upper_body_dof_indices]
        self.motion_step += 1

        self.dof_target[:] = policy_targets

        inference_time = time.perf_counter()
        self.logger.debug(f"Inference took {(inference_time - start_time)*1000:.4f} ms")
        time.sleep(0.001)

    def _publish_cmd(self):
        while self.running:
            time_now = self.timer.get_time()
            if time_now < self.next_publish_time:
                time.sleep(0.001)
                continue
            self.next_publish_time += self.cfg["common"]["dt"]
            self.logger.debug(f"Next publish time: {self.next_publish_time}")

            self.filtered_dof_target = self.filtered_dof_target * 0.8 + self.dof_target * 0.2

            for i in range(B1JointCnt):
                self.low_cmd.motor_cmd[i].q = self.filtered_dof_target[i]

            # Use series-parallel conversion for torque to avoid non-linearity
            for i in self.cfg["mech"]["parallel_mech_indexes"]:
                self.low_cmd.motor_cmd[i].q = self.dof_pos_latest[i]
                self.low_cmd.motor_cmd[i].tau = np.clip(
                    (self.filtered_dof_target[i] - self.dof_pos_latest[i]) * self.cfg["common"]["stiffness"][i],
                    -self.cfg["common"]["torque_limit"][i],
                    self.cfg["common"]["torque_limit"][i],
                )
                self.low_cmd.motor_cmd[i].kp = 0.0

            start_time = time.perf_counter()
            self._send_cmd(self.low_cmd)
            publish_time = time.perf_counter()
            self.logger.debug(f"Publish took {(publish_time - start_time)*1000:.4f} ms")
            time.sleep(0.001)

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()


if __name__ == "__main__":
    import argparse
    import signal
    import sys
    import os

    def signal_handler(sig, frame):
        print("\nShutting down...")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=str, help="Name of the configuration file.")
    parser.add_argument("--net", type=str, default="127.0.0.1", help="Network interface for SDK communication.")
    args = parser.parse_args()
    cfg_file = os.path.join("configs", args.config)

    print(f"Starting custom controller, connecting to {args.net} ...")
    ChannelFactory.Instance().Init(0, args.net)

    with Controller(cfg_file) as controller:
        time.sleep(2)  # Wait for channels to initialize
        print("Initialization complete.")
        debug_t1 = time.perf_counter()
        controller.start_custom_mode_conditionally()
        print(f"Start custom mode took {(time.perf_counter() - debug_t1)*1000:.4f} ms")
        input("Press enter to start")
        print("start")
        debug_t2 = time.perf_counter()
        controller.start_rl_gait_conditionally()
        print(f"Start custom mode took {(time.perf_counter() - debug_t2)*1000:.4f} ms")
        

        try:
            while controller.running:
                controller.run()
            controller.client.ChangeMode(RobotMode.kWalking)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Cleaning up...")
            controller.cleanup()
