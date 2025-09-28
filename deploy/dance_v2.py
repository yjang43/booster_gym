# pip install textual rich
from __future__ import annotations
import time
from statistics import median
from typing import Dict, TypedDict, Optional

from textual.app import App, ComposeResult
from textual.widgets import Static, ProgressBar, Digits
from textual import events
from rich.table import Table
from rich.text import Text

# ---- Update rate (FPS) ----
UPDATE_FPS = 60.0
UPDATE_INTERVAL = 1.0 / UPDATE_FPS

class KeyConfig(TypedDict):
    desc: str
    delay: float

KeySpec = Dict[str, KeyConfig]


def fmt_eta(seconds: float) -> str:
    s = max(0.0, seconds)
    m = int(s // 60)
    sec = int(s % 60)
    hs = int((s - int(s)) * 100)  # hundredths
    return f"{m:02d}:{sec:02d}.{hs:02d}"


class MedianTimer(App):
    """Median timer: big centered Digits, full-width bar, subtle row-tinted table."""
    CSS = """
    Screen { layout: vertical; padding: 1 2; }   /* no 'align' here so bars can fill width */

    #big {
        width: 100%;
        content-align: center middle;            /* center the digits text */
        text-style: bold;
        margin: 1 0;
    }

    #mainbar {
        height: 4;                               /* thick bar */
        width: 100%;                             /* truly full width */
        margin: 0 0 1 0;
    }

    #title  { padding: 0 1; }
    #status { padding: 0 1; color: $text-muted; }

    /* Render the Rich table container at content width */
    #table_holder { width: 100%; }
    """

    def __init__(self, delays: KeySpec):
        super().__init__()
        # config
        self.delays: KeySpec = {k: {"desc": v["desc"], "delay": float(v["delay"])} for k, v in delays.items()}
        # state
        self.press_times: Dict[str, float] = {}
        self.use_key: Dict[str, bool] = {k: True for k in self.delays}
        self.start_time: Optional[float] = None
        self.running: bool = False
        # widgets
        self.big: Digits
        self.bar: ProgressBar
        self.status: Static
        self.table_holder: Static

    # ----- UI -----
    def compose(self) -> ComposeResult:
        self.big = Digits(id="big"); yield self.big
        yield Static("Overall (median end):", id="title")
        self.bar = ProgressBar(total=1, show_eta=False, id="mainbar"); yield self.bar
        self.status = Static("Active (included): —  •  lowercase=press  •  UPPERCASE=toggle  •  Ctrl+C=quit", id="status")
        yield self.status
        self.table_holder = Static(id="table_holder")
        yield self.table_holder

    # ----- helpers -----
    def _active_end_times(self) -> Dict[str, float]:
        return {
            k: self.press_times[k] + self.delays[k]["delay"]
            for k in self.press_times
            if self.use_key.get(k, False)
        }

    def _median_end(self) -> Optional[float]:
        vals = self._active_end_times().values()
        return median(vals) if vals else None

    def _update_big(self, seconds_left: float) -> None:
        self.big.update(fmt_eta(seconds_left))

    def _build_table(self) -> Table:
        tbl = Table(show_header=True, expand=True)
        # Right-justify Description, Delay, and Δ from median columns
        tbl.add_column("Key")
        tbl.add_column("Description", justify="right")
        tbl.add_column("Delay (s)", justify="right")
        tbl.add_column("Use?")
        tbl.add_column("ETA")
        tbl.add_column("Δ from median", justify="right")

        now = time.time()
        mend = self._median_end()

        # >>> SORT BY DELAY (ascending) instead of by key <<<
        for k, cfg in sorted(self.delays.items(), key=lambda kv: kv[1]["delay"], reverse=True):
            delay = cfg["delay"]
            used = self.use_key.get(k, False)
            t = self.press_times.get(k)

            # Per-key ETA (even if toggled off, if pressed we show its ETA)
            eta = fmt_eta((t + delay) - now) if t is not None else "—"

            # Δ from current median (signed, 0.01)
            delta_txt = (
                f"{((t + delay) - mend):+0.2f}s"
                if (t is not None and mend is not None)
                else "—"
            )

            # Row tinting:
            # - toggled Off  -> subtle red tint
            # - toggled On & pressed -> subtle green tint
            # - toggled On & never pressed -> neutral (dim text)
            if not used:
                row_style = "on rgb(40,26,26)"      # subtle red
                dim = False
            elif t is None:
                row_style = None
                dim = True
            else:
                row_style = "on rgb(26,40,26)"      # subtle green
                dim = False

            def cell(x: str) -> Text:
                return Text(x, style=("dim" if dim else ""))

            tbl.add_row(
                cell(k),
                cell(cfg["desc"]),
                cell(f"{delay:.2f}"),
                cell("On" if used else "Off"),
                cell(eta),
                cell(delta_txt),
                style=row_style,
            )
        return tbl

    def _refresh_status(self, done: bool = False) -> None:
        active = ", ".join(sorted(self._active_end_times())) or "—"
        self.status.update(
            f"{'DONE! ' if done else ''}Active (included): {active}  •  lowercase=press  •  UPPERCASE=toggle  •  Ctrl+C=quit"
        )

    def _refresh_all(self) -> None:
        self.table_holder.update(self._build_table())
        mend = self._median_end()
        self._update_big((mend - time.time()) if mend else 0.0)
        self.bar.update(progress=0, total=1)
        self._refresh_status()

    # ----- lifecycle -----
    def on_mount(self) -> None:
        self.set_interval(UPDATE_INTERVAL, self._tick)   # configurable FPS
        self._refresh_all()

    # ----- input -----
    async def on_key(self, event: events.Key) -> None:
        k = event.key
        if not (len(k) == 1 and k.isalpha()):   # Esc ignored; Ctrl+C quits the process
            return
        base = k.lower()
        if base not in self.delays:
            return

        if k.isupper():
            self.use_key[base] = not self.use_key.get(base, True)
            # stopping run if nothing currently included
            if not self._active_end_times():
                self.running = False
                self.bar.update(progress=0, total=1)
            self.table_holder.update(self._build_table())
            self._refresh_status()
        else:
            now = time.time()
            self.press_times[base] = now
            if self.start_time is None:
                self.start_time = now
            self.running = True
            self.table_holder.update(self._build_table())
            self._refresh_status()

    # ----- ticking -----
    def _tick(self) -> None:
        if not self.running:
            return
        now = time.time()
        mend = self._median_end()
        if mend is None:
            self.bar.update(progress=0, total=1)
            self._update_big(0.0)
            self._refresh_status()
            return

        remain = mend - now
        if remain <= 0:
            # IMMEDIATE exit at the exact moment the timer completes.
            self.exit()
            return

        # Update big digits and progress
        self._update_big(remain)
        t0 = self.start_time or now
        denom = max(mend - t0, 1e-6)
        self.bar.update(progress=min(max((now - t0) / denom, 0.0), 1.0), total=1)

        # Update table ETAs / deltas continuously
        self.table_holder.update(self._build_table())


def wait_for_dance(delays: KeySpec) -> None:
    MedianTimer(delays).run()

# ---- SOURCE TIMES (s) ----
# Snaps
snap_1 = 52.95
snap_2 = 55.02
snap_3 = 57.12
snap_4 = 59.19

# Mids
mid_1  = 99.90
mid_2  = 100.92

# Pops
pop_1  = 121.79
pop_2  = 123.88
pop_3  = 125.95
pop_4  = 128.03

# ---- CONSTANTS (s) ----
clip = 188.411 #Audacity mark of the start of the clip
# prep = 12.0    #Extra time the robot needs to get up etc
prep = 18.0    #Extra time the robot needs to get up etc

def delay_from(timepoint: float, rehearse_start: float) -> float:
    """Compute delay = clip - (timepoint - rehearse_start) - prep."""
    return round(clip - (timepoint - rehearse_start) - prep, 3)

# ---- build delays for a given rehearsal start ----
def get_dance_delays(start_time: float) -> KeySpec:
    """Return {key: {desc, delay}} computed with a given rehearsal start time (s)."""
    return {
        "q": {"desc": "Snap 1", "delay": delay_from(snap_1, start_time)},   # 120.461 at start=0
        "w": {"desc": "Snap 2", "delay": delay_from(snap_2, start_time)},   # 118.391
        "e": {"desc": "Snap 3", "delay": delay_from(snap_3, start_time)},   # 116.291
        "r": {"desc": "Snap 4", "delay": delay_from(snap_4, start_time)},   # 114.221

        "a": {"desc": "Mid 1",  "delay": delay_from(mid_1,  start_time)},   # 73.511
        "s": {"desc": "Mid 2",  "delay": delay_from(mid_2,  start_time)},   # 72.491

        "z": {"desc": "Pop 1",  "delay": delay_from(pop_1,  start_time)},   # 51.621
        "x": {"desc": "Pop 2",  "delay": delay_from(pop_2,  start_time)},   # 49.531
        "c": {"desc": "Pop 3",  "delay": delay_from(pop_3,  start_time)},   # 47.461
        "v": {"desc": "Pop 4",  "delay": delay_from(pop_4,  start_time)},   # 45.381
    }

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
    start_time = 0
    """Run the app using delays computed from a given rehearsal start time (seconds)."""
    ChannelFactory.Instance().Init(0)
    controller = ControllerForDance("configs/T1.yaml")
    remote_control_service = RemoteControlServiceForDance(JoystickConfigForDance)

    delays = get_dance_delays(start_time)
    wait_for_dance(delays)

    timers = []
    print("Hi robot")

    # schedule_task(0, comm_prepare_mode)
    # schedule_task(6, comm_get_up)
    # schedule_task(15, comm_prepare_mode)
    # schedule_task(16, comm_custom_mode)
    # schedule_task(18, comm_dance)
    
    # for timer in timers:
    #     timer.join()
