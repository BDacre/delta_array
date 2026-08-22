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
# their travel, (MIN_JOINT_POS + MAX_JOINT_POS) / 2 = 0.05175 m. Recompute if the
# geometry, the joint limits or the EE offset changes.
#
# This was 0.13648692036511054 (joint 0.05675 m) until 2026-08-18, which sat 5.00 mm
# high: 0.05675 = 0.05175 + 0.005, i.e. mid-travel with MIN_JOINT_POS added a second
# time. The effect was a home pose 5 mm off centre, leaving 41.75 mm of travel above
# it and 51.75 mm below. Anything that opens symmetrically about home (the paraboloid
# in delta_array_connected_manipulation, tilt, saddle) was clipped by the short side
# while the long side went unused.
HOME_POSITION = (0.0, 0.0, 0.13148692036511053)

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

# Which physical boards exist (BOARD_REGISTRY / BOARD_LABELS) lives in boards.py
# — that's an inventory of one bench's hardware, not a property of the protocol,
# and it changes whenever a board is swapped. Everything here is code-versioned.

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
