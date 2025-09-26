import time
import threading
import functools
import numpy as np
import evdev
from dataclasses import dataclass
from utils.remote_control_service import RemoteControlService, JoystickConfig
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
)


ChannelFactory.Instance().Init(0)

controller = Controller("configs/T1.yaml")


lock = threading.Lock()

start_time = time.perf_counter()

dance_time = 60*3 + 8.415
# key_marks = [52.932, 55.244, 57.142, 59.165]
key_marks = [52.932, 55.244, 57.142, 59.165]
delay_times = [dance_time - km for km in key_marks]
delay_times = [35, 30, 25]

timers = []


def comm_thread(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        with lock:
            return func(*args, **kwargs)
    return wrapper


@dataclass
class JoystickConfigForDance(JoystickConfig):
    dance_button: evdev.ecodes = evdev.ecodes.BTN_0 # TODO: change to different button.

class RemoteControlServiceForDance(RemoteControlService):

    def _init_keyboard_control(self):
        super()._init_keyboard_control()
        self.keybaord_dance = False

    def _handle_keyboard_press(self, key):
        super()._handle_keyboard_press(key)
        if key == "z":
            self.keybaord_dance = True

    def start_dance(self) -> bool:
        if hasattr(self, "joystick") and getattr(self, "joystick") != None:
            return self.joystick.active_keys() == [self.config.dance_button]
        return self.keybaord_dance


remote_control_service = RemoteControlServiceForDance()


def get_elapsed_time():
    return time.perf_counter() - start_time

def collect_timing_cues():
    num_cues = len(delay_times)
    cue_times = []

    for i in range(num_cues):
        print(f"Press Enter for cue {i+1}/{num_cues} (delay: {delay_times[i]}s): ")
        while True:
            if remote_control_service.start_dance():
                remote_control_service.keybaord_dance = False
                break
            time.sleep(0.1)
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
    controller.start_rl_gait_conditionally()
    while controller.running:
        controller.run()

@comm_thread
def comm_get_up():
    print("current time:", get_elapsed_time())
    print("🚶 GETTING UP... 🚶")
    controller.client.GetUp()

@comm_thread
def comm_prepare_mode():
    print("current time:", get_elapsed_time())
    print("Prepare mode")
    controller.client.ChangeMode(RobotMode.kPrepare)

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

    cue_times = collect_timing_cues()
    consensus_time = calculate_consensus_timing(cue_times)
    schedule_task(consensus_time - 22, comm_prepare_mode)
    schedule_task(consensus_time - 20, comm_get_up)
    schedule_task(consensus_time - 2, comm_custom_mode)
    schedule_task(consensus_time, comm_dance)

    for timer in timers:
        timer.join()

    