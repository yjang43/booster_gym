import time
import threading
import functools
import numpy as np
import evdev
from dataclasses import dataclass
from utils.remote_control_service import RemoteControlService, JoystickConfig
from utils.command import create_prepare_cmd, create_first_frame_rl_cmd
from deploy import Controller
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    LowState,
    B1JointCnt,
    RobotMode,
    GetModeResponse
)


lock = threading.Lock()
gm = GetModeResponse()


def comm_thread(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        with lock:
            return func(*args, **kwargs)
    return wrapper


class ControllerForDance(Controller):

    def start_custom_mode_conditionally(self):
        create_prepare_cmd(self.low_cmd, self.cfg)
        for i in range(B1JointCnt):
            self.dof_target[i] = self.low_cmd.motor_cmd[i].q
            self.filtered_dof_target[i] = self.low_cmd.motor_cmd[i].q
        self._send_cmd(self.low_cmd)
        self.client.ChangeMode(RobotMode.kCustom)

    def start_rl_gait_conditionally(self):
        create_first_frame_rl_cmd(self.low_cmd, self.cfg)
        self._send_cmd(self.low_cmd)
        self.publish_runner = threading.Thread(target=self._publish_cmd)
        self.publish_runner.daemon = True
        self.publish_runner.start()


@dataclass
class JoystickConfigForDance(JoystickConfig):
    dance_button: evdev.ecodes = evdev.ecodes.BTN_B # TODO: change to different button.

class RemoteControlServiceForDance(RemoteControlService):

    def __init__(self, config=None):
        self.released = True

    def _init_keyboard_control(self, remote_control_service):
        super()._init_keyboard_control()
        self.keybaord_dance = False

    def _handle_keyboard_press(self, key):
        super()._handle_keyboard_press(key)
        if key == "z":
            self.keybaord_dance = True

    def start_dance(self) -> bool:
        if hasattr(self, "joystick") and getattr(self, "joystick") != None:
            pressed = self.joystick.active_keys() == [self.config.dance_button]
            return pressed
            if pressed:
                res = self.released
                self.released = False
                return res
            else:
                self.released = True

        # return self.keybaord_dance
        input()

def ensure_mode_change(client, mode):
    while not client.ChangeMode(mode):
        time.sleep(0.01)
        pass
    # client.GetMode(gm)
    # while gm.mode != mode:
    #     client.ChangeMode(mode)
    #     client.GetMode(gm)

def get_elapsed_time():
    return time.perf_counter() - start_time

def collect_timing_cues():
    num_cues = len(delay_times)
    cue_times = []

    for i in range(num_cues):
        input(f"Press Enter for cue {i+1}/{num_cues} (delay: {delay_times[i]}s): ")
        # print(f"Press Enter for cue {i+1}/{num_cues} (delay: {delay_times[i]}s): ")
        # while True:
        #     if remote_control_service.start_dance():
        #         remote_control_service.keybaord_dance = False
        #         break
        #     time.sleep(0.1)
        current_time = get_elapsed_time()
        cue_time = current_time + delay_times[i]
        cue_times.append(cue_time)

    return cue_times

def calculate_consensus_timing(cue_times):
    if len(cue_times) == 1:
        return cue_times[0]

    accum_intervals = delay_times[0] - np.array(delay_times)
    cue_times = np.array(cue_times)

    errors = []
    for ref_idx, ref_time in enumerate(cue_times):
        offsets = accum_intervals[ref_idx] - accum_intervals
        expected_times = ref_time + offsets
        total_error = np.sum(np.abs(cue_times - expected_times))
        errors.append(total_error)

    best_idx = np.argmin(errors)
    return cue_times[best_idx]


@comm_thread
def comm_dance():
    print("current time:", get_elapsed_time())
    print("🕺 DANCING... 🕺")
    controller.next_inference_time = controller.timer.get_time()
    controller.start_rl_gait_conditionally()
    while controller.running:
        controller.run()
    print("Dance done")
    ensure_mode_change(controller.client, RobotMode.kPrepare)
    ensure_mode_change(controller.client, RobotMode.kWalking)

@comm_thread
def comm_get_up():
    print("current time:", get_elapsed_time())
    print("🚶 GETTING UP... 🚶")
    controller.client.GetUp()

@comm_thread
def comm_prepare_mode():
    print("current time:", get_elapsed_time())
    print("Prepare mode")
    # controller.client.ChangeMode(RobotMode.kPrepare)
    ensure_mode_change(controller.client, RobotMode.kPrepare)

@comm_thread
def comm_custom_mode():
    print("current time:", get_elapsed_time())
    print("Custom mode")
    controller.start_custom_mode_conditionally()

def schedule_task(target_time, task_func, *task_func_args):
    current_time = get_elapsed_time()
    delay = target_time - current_time

    print(f"Scheduling {task_func.__name__} at {target_time:.3f}s (current: {current_time:.3f}s, delay: {delay:.3f}s)")

    if delay > 0:
        timer = threading.Timer(delay, task_func, args=task_func_args)
        timer.start()
        timers.append(timer)
    else:
        task_func(*task_func_args)


if __name__ == "__main__":
    ChannelFactory.Instance().Init(0)
    controller = ControllerForDance("configs/T1.yaml")
    remote_control_service = RemoteControlServiceForDance(JoystickConfigForDance)
    start_time = time.perf_counter()

    dance_time = 3*60 + 8.415
    key_marks = [52.932, 55.244, 57.142, 59.165] + [1*60 + 42.5] + [2*60 + 1.953, 2*60 + 3.907, 2*60 + 5.931, 2*60 + 8.08]
    delay_times = [dance_time - km for km in key_marks]
    # delay_times = [3]
    timers = []

    cue_times = collect_timing_cues()
    consensus_time = calculate_consensus_timing(cue_times)
    schedule_task(consensus_time - 18, comm_prepare_mode)
    schedule_task(consensus_time - 12, comm_get_up)
    schedule_task(consensus_time - 3, comm_prepare_mode)
    schedule_task(consensus_time - 2, comm_custom_mode)
    schedule_task(consensus_time, comm_dance)

    for timer in timers:
        timer.join()

    
