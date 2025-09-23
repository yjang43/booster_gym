import time

from deploy import Controller
from booster_robotics_sdk_python import RobotMode

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

# Timing constants (in seconds)
# START_DELAY = 87.425
START_DELAY = 85.   # minus the delay to set up mode.
# START_DELAY = 10
STAND_DELAY = 15
DANCE_DELAY = 25


ChannelFactory.Instance().Init(0)

def main():
    """Execute robot dance sequence."""
    with Controller("configs/T1.yaml") as controller:
        controller.client.ChangeMode(RobotMode.kPrepare)
        # Wait for user confirmation
        input("Press enter to start")
        print("Starting dance sequence...")
        time.sleep(START_DELAY)

        # # Stand up sequence
        # controller.client.GetUp()
        # time.sleep(STAND_DELAY)

        # Start dance mode
        controller.start_custom_mode_conditionally()
        controller.start_rl_gait_conditionally()

        # Main control loop
        while controller.running:
            controller.run()

        # Transition to damping mode
        controller.client.ChangeMode(RobotMode.kPrepare)

        # Return to walking mode
        controller.client.ChangeMode(RobotMode.kWalking)


if __name__ == "__main__":
    main()
