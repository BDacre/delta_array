
import time

from delta_control import DeltaArrayEnv
from delta_control.constants import NUM_MOTORS

SETTLE_TIME = 30.0
TARGET_POS = 0.005

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_ROBOT_ID = 9


def run(port: str, robot_id: int) -> None:
    print(f"opening {port} for robot id {robot_id}")
    env = DeltaArrayEnv(port, active_ids=(robot_id,))
    agent = env.agents[robot_id]

    try:
        print("homing...")
        env.reset()
        time.sleep(1)

        print(f"moving all motors to {TARGET_POS}")
        agent.move_joint_position([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06, 0.05, 0.05, 0.05])
        #agent.move_joint_position([TARGET_POS] * NUM_MOTORS)
        time.sleep(SETTLE_TIME)

        print("returning to home")
        env.reset()
        time.sleep(SETTLE_TIME)
    finally:
        print("closing port")
        agent.close()



if __name__ == "__main__":
    run(DEFAULT_PORT, DEFAULT_ROBOT_ID) 
