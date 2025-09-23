import booster_robotics_sdk_python as B
from functools import partial
from time import sleep
from easydict import EasyDict
import rp
import math


def set_global(name, value):
    globals()[name] = value


@rp.globalize_locals
def _init_communication() -> None:
    if not "INITIALIZED" in globals():
        B.ChannelFactory.Instance().Init(0)

        # low_cmd_publisher = B1LowCmdPublisher()
        # low_cmd_publisher.InitChannel()
        # low_cmd = LowCmd()

        client = B.B1LocoClient()
        client.Init()

        handler = partial(set_global, "low_state")  # Low Level State
        low_state_subscriber = B.B1LowStateSubscriber(handler)
        low_state_subscriber.InitChannel()

        # Low-level joint control setup
        low_cmd_publisher = B.B1LowCmdPublisher()
        low_cmd_publisher.InitChannel()
        motor_cmds = [B.MotorCmd() for _ in range(B.B1JointCnt)]
        set_global("low_cmd_publisher", low_cmd_publisher)
        set_global("motor_cmds", motor_cmds)

        INITIALIZED = True
    else:
        rp.fansi_print("_init_communication: Already initialized", 'yellow italic')

def tick(): sleep(.01)

# Mode control
def set_mode_custom (): return client.ChangeMode(B.RobotMode.kCustom ) # Goes limp, allows low level control
def set_mode_prepare(): return client.ChangeMode(B.RobotMode.kPrepare) # Stand ready position
def set_mode_walking(): return client.ChangeMode(B.RobotMode.kWalking) # For movement and dancing. WARNING: Never do this when the robot is on the stand!
def set_mode_damping(): return client.ChangeMode(B.RobotMode.kDamping) # Damped/compliant mode

# Dance moves - must be in walking mode
def do_new_years_dance(): return client.Dance(B.DanceId.kNewYear      ) # New Year dance
def do_nezha_dance    (): return client.Dance(B.DanceId.kNezha        ) # Nezha dance
def do_future_dance   (): return client.Dance(B.DanceId.kTowardsFuture) # Towards Future dance
def stop_dance        (): return client.Dance(B.DanceId.kStop         ) # Stop dancing

# Movement control - must be in walking mode
def move(x=0.0, y=0.0, z=0.0): return client.Move(x, y, z) # x=forward/back, y=left/right, z=rotate
def stop_movement(): return move(0, 0, 0)
def walk_forward (speed=0.8): return move(speed, 0, 0 )
def walk_backward(speed=0.2): return move(-speed, 0, 0)
def walk_left    (speed=0.2): return move(0, speed, 0 )
def walk_right   (speed=0.2): return move(0, -speed, 0)
def rotate_left  (speed=0.2): return move(0, 0, speed )
def rotate_right (speed=0.2): return move(0, 0, -speed)

# Head control - must be in walking mode
def rotate_head(pitch=0.0, yaw=0.0): return client.RotateHead(pitch, yaw)
def head_look_down (): return rotate_head(1.0, 0.0   )
def head_look_up   (): return rotate_head(-0.3, 0.0  )
def head_look_left (): return rotate_head(0.0, 0.785 )  # 45 degrees
def head_look_right(): return rotate_head(0.0, -0.785)  # 45 degrees
def head_center    (): return rotate_head(0.0, 0.0   )

# Arm/hand control - must be in walking mode
def handwave      (): return client.WaveHand (B.kHandOpen ) # Wave hand open
def stop_handwave (): return client.WaveHand (B.kHandClose) # Wave hand close
def handshake     (): return client.Handshake(B.kHandOpen ) # Start handshake motion
def stop_handshake(): return client.Handshake(B.kHandClose) # End handshake motion

# Get up / lie down - must be in prepare mode. WARNING: It's very clumsy!
def lie_down(): return client.LieDown()  # Makes robot lie down
def get_up  (): return client.GetUp  ()  # Makes robot stand up from lying position

# Pose control - bare bones
_init_communication()
