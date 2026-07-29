NUM_MOTORS = 12
MOTORS_PER_DELTA = 3
DELTAS_PER_BOARD = NUM_MOTORS // MOTORS_PER_DELTA  # 4

# Joint position limits (meters of prismatic travel).
MIN_JOINT_POS = 0.005
MAX_JOINT_POS = 0.0985

# Delta robot geometry (meters).
END_EFFECTOR_TRIANGLE_SIDE_LEN = 0.024
# Triangle formed by the upper leg connection points. Legs connect at the
# midpoints of the end-effector triangle's sides, so by the midsegment theorem
# this side length is half the end-effector triangle's. This is the platform
# triangle the kinematics actually solves against.
PLATFORM_TRIANGLE_SIDE_LEN = END_EFFECTOR_TRIANGLE_SIDE_LEN / 2
# Distance between the linear actuator rails (the base triangle).
BASE_TRIANGLE_SIDE_LEN = 0.0213908
LEG_LENGTH = 0.045 + 2*0.0055 # leg plus hinges

# Vertical offset (+z, along the platform normal) from the platform (leg-
# connection) triangle plane up to the end-effector reference point that IK/FK
# report. The tip sits TIP_HEIGHT above the platform, which is itself
# PLATFORM_TRIANGLE_HEIGHT tall. IK subtracts EE_Z_OFFSET to recover the
# platform center it solves against; FK adds it back so it returns the tip.
PLATFORM_TRIANGLE_HEIGHT = 0.005
TIP_HEIGHT = 0.019  # 19 mm
TIP_RADIUS = 0.0075
EE_Z_OFFSET = TIP_HEIGHT + PLATFORM_TRIANGLE_HEIGHT

# Default home pose for the EE tip: FK of all three actuators at the center of
# their travel (0.05675 m). Recompute if the geometry or EE offset changes.
HOME_POSITION = (0.0, 0.0, 0.13648692036511054)

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

# Backdoor / broadcast id. Every board also answers to this id, so the host can
# talk to a board before knowing its real (chip-derived) id — used for whoami
# discovery. Must match BROADCAST_ID in the firmware's variables_and_parameters.h.
BROADCAST_ID = 0

# Registry mapping a friendly label to a board's chip-derived id. Firmware now
# derives each board's id from its SAMD21 serial number, so ids are large and
# not human-chosen. Populate this once per board using scripts/identify_board.py
BOARD_REGISTRY: dict[str, int] = {
    "board0": 855203507,
    "board1": 1183344710,
    "board2": 2018580162,
    "board3": 1159118204,
}

# Reverse lookup: chip id -> label. Rebuilt from BOARD_REGISTRY; unknown ids
# simply won't be present.
BOARD_LABELS: dict[int, str] = {v: k for k, v in BOARD_REGISTRY.items()}

MAX_TRAJECTORY_ROWS = 20

FRAME_START = b"\xa6"
FRAME_END = b"\xa7"

# Max bytes read_frame() will discard while hunting for FRAME_START before
# giving up. Bounds the hunt so a wrong/chatty serial port (another CDC device
# streaming non-protocol bytes) fails cleanly instead of looping forever. Set
# well above one max frame so legitimate leading noise (boot text) is tolerated.
MAX_HUNT_BYTES = 4096

DEFAULT_BAUD = 57600

# Acknowledgement read timeout for command sends.
# Bound by firmware MOVE_TIMEOUT_MS (5000) since the firmware can't service
# new frames mid-move; small headroom for round-trip latency.
ACK_TIMEOUT_S = 6.0

# Short read timeout used when probing candidate ports for a board (whoami only,
# no motion). Kept small so scanning several ports is quick; a board replies to
# a whoami in milliseconds.
DISCOVERY_TIMEOUT_S = 1.0

# Glob for candidate serial ports, used as a fallback when USB VID:PID metadata
# is unavailable (VID:PID filtering is the primary, faster path — see below).
BOARD_PORT_GLOB = "/dev/ttyACM*"

# USB (vid, pid) pairs treated as candidate delta boards. Delta boards are
# Adafruit Feather M0s, which enumerate as 239A:800B when running a sketch.
# Ports that don't match are skipped WITHOUT being opened, so unrelated CDC
# devices (e.g. an Arduino Uno sharing the /dev/ttyACM* namespace) never slow
# discovery. Add more (vid, pid) pairs here if you introduce other board types.
FEATHER_M0_USB_ID = (0x239A, 0x800B)
BOARD_USB_IDS = frozenset({FEATHER_M0_USB_ID})
