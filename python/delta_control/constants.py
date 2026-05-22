NUM_MOTORS = 12

# Joint position limits (meters of prismatic travel).
MIN_JOINT_POS = 0.005
MAX_JOINT_POS = 0.0985

# Delta robot geometry (meters).
SIDE_LENGTH_PLATFORM = 0.015
SIDE_LENGTH_BASE = 0.043
LEG_LENGTH = 0.045

# Default home pose for the EE: centered (x=0, y=0), mid-workspace (z=0.08).
HOME_POSITION = (0.0, 0.0, 0.08)

# Physical layout of robots in the 8x8 array (meters).
# Triangular grid: every other column is offset by half the row pitch.
ROBOT_PITCH_X = 0.043301
ROBOT_PITCH_Y = 0.0375
ROBOT_PITCH_X_HALF = ROBOT_PITCH_X / 2

# Magnitude of the EE xy offset used when steering robots toward a focus
# point (meters). 5 mm matches the original cm-era behavior.
EE_OFFSET_MAGNITUDE = 0.005

ALL_AGENT_IDS = tuple(range(1, 17))
DEFAULT_ACTIVE_AGENT_IDS = (9,)

MAX_TRAJECTORY_ROWS = 20

FRAME_START = b"\xa6"
FRAME_END = b"\xa7"

DEFAULT_BAUD = 57600
