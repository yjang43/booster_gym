import time
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    RobotMode,
)
from deploy import Controller


def set_prepare_mode(controller):
    """Set the robot to prepare mode"""
    print("Setting robot to prepare mode...")
    controller.client.ChangeMode(RobotMode.kPrepare)
    print("Robot is now in prepare mode")


if __name__ == "__main__":
    # Initialize communication
    ChannelFactory.Instance().Init(0)
    controller = Controller("configs/T1.yaml")

    print("Robot prepare mode script")
    print("Press Enter to set prepare mode, or 'q' + Enter to quit")

    while True:
        try:
            user_input = input("\nPress Enter for prepare mode (or 'q' to quit): ").strip().lower()

            if user_input == 'q':
                print("Exiting...")
                break

            # Set prepare mode
            set_prepare_mode(controller)

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            continue

    print("Script finished")