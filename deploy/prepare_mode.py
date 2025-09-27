import time
from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    RobotMode,
)
from deploy import Controller


def set_prepare_mode(client):
    """Set the robot to prepare mode"""
    print("Setting robot to prepare mode...")
    client.ChangeMode(RobotMode.kPrepare)
    print("Robot is now in prepare mode")

def set_damp_mode(client):
    """Set the robot to damping mode"""
    print("Setting robot to damping mode...")
    client.ChangeMode(RobotMode.kDamping)
    print("Robot is now in damping mode")


if __name__ == "__main__":
    # Initialize communication
    ChannelFactory.Instance().Init(0)
    time.sleep(2)
    client = B1LocoClient()
    client.Init()

    print("Robot prepare mode script")
    print("Press Enter to set prepare mode, or 'q' + Enter to quit")

    while True:
        try:
            user_input = input("\nPress Enter for prepare mode (or 'q' to quit): ").strip().lower()

            if user_input == 'q':
                print("Exiting...")
                break

            if user_input == "d":
                set_damp_mode(client)

            else:
                # Set prepare mode
                set_prepare_mode(client)

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            continue

    print("Script finished")