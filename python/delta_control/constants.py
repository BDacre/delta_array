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

# Backdoor / broadcast id. Every board also answers to this id, so the host can
# talk to a board before knowing its real (chip-derived) id — used for whoami
# discovery. Must match BROADCAST_ID in the firmware's variables_and_parameters.h.
BROADCAST_ID = 0

# Registry mapping a friendly label to a board's chip-derived id. Firmware now
# derives each board's id from its SAMD21 serial number, so ids are large and
# not human-chosen. Populate this once per board using scripts/identify_board.py
# (which prints the discovered id), then address boards by label in host code.
# Example: BOARD_REGISTRY = {"corner_a": 123456789, "corner_b": 987654321}
BOARD_REGISTRY: dict[str, int] = {}

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

# Glob for candidate serial ports scanned during port auto-detection.
BOARD_PORT_GLOB = "/dev/ttyACM*"
