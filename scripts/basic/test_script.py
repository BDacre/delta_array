
import time

from delta_control import open_board
from delta_control.constants import NUM_MOTORS

SETTLE_TIME = 30.0
TARGET_POS = 0.005

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip


def run(port: str, board=None) -> None:
    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")

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
    run(DEFAULT_PORT, DEFAULT_BOARD) 
