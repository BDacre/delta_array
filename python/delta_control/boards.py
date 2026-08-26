"""Inventory of the physical delta boards in this array.

Separate from constants.py on purpose. constants.py holds properties of the
firmware *protocol* (frame bytes, baud, USB ids) — they change only when the code
changes, and every copy of this repo shares them. This file is a record of the
hardware sitting on one bench: it changes when a board is swapped or added, and a
different array would need different numbers. Keeping the two apart means a
hardware swap touches this file and nothing else.

Firmware derives each board's id from its SAMD21 serial number, so ids are large
and not human-chosen. To register a new board, run
``scripts/maintenance/identify_board.py`` to read its id, add a line below, then
run ``python -m delta_control.boards`` to check it against the calibration files.
"""

# Friendly label -> chip-derived id. Insertion order is board0..boardN and is
# what board_order() iterates; plain sorted() on these labels is lexicographic
# (board0, board1, board10, board2, ...), which is rarely what you want.
BOARD_REGISTRY: dict[str, int] = {
    "board0": 2018580162,
    "board1": 1183344710,
    "board2": 655311829,
    "board3": 649458237,
    "board4": 351007952,
    "board5": 855203507,
    "board6": 1159118204,
    "board7": 1846180647,
    "board8": 1874185319,
    "board9": 1018140047,
    "board10": 310932672,
    "board11": 1879946509,
    "board12": 917820413,
    "board13": 158645278,
    "board14": 2043512705,
    "board15": 1900500898,
}

# Reverse lookup: chip id -> label. Unknown ids simply won't be present.
BOARD_LABELS: dict[int, str] = {v: k for k, v in BOARD_REGISTRY.items()}

# Two labels sharing one chip id silently collapses in BOARD_LABELS and makes one
# board unaddressable by name — the likely outcome of pasting an id into the wrong
# line. Cheap enough (no I/O) to check at import; the cross-check against the
# calibration files needs the filesystem and lives in check_calibration_ids().
_DUPLICATE_IDS = [
    f"{label} and {BOARD_LABELS[bid]} both use id {bid}"
    for label, bid in BOARD_REGISTRY.items()
    if BOARD_LABELS[bid] != label
]
assert not _DUPLICATE_IDS, (
    "duplicate board id in BOARD_REGISTRY: " + "; ".join(_DUPLICATE_IDS)
)


def board_order() -> list[str]:
    """Registry labels in board0, board1, ... board10 order (not lexicographic)."""
    return list(BOARD_REGISTRY)


def check_calibration_ids(calib_dir=None) -> list[str]:
    """Cross-check the registry against ``config/calibration/board_*.json``.

    Each calibration file already carries its own ``board_id``, so the registry
    restates data that exists elsewhere; this is what keeps the two honest.
    Returns a list of human-readable problems — empty means they agree. A
    registered board with no calibration file is NOT a problem (an uncalibrated
    board runs on firmware defaults), but a file whose ``board_id`` contradicts
    the registry, or that claims an id no label owns, is.
    """
    # Imported here rather than at module scope so this stays a data module that
    # can be imported without pulling in the calibration loader.
    import glob
    import os

    from .calibration import DEFAULT_CALIBRATION_DIR, load_calibration_file

    calib_dir = calib_dir or DEFAULT_CALIBRATION_DIR
    problems = []
    for path in sorted(glob.glob(os.path.join(calib_dir, "board_*.json"))):
        name = os.path.basename(path)
        try:
            calib = load_calibration_file(path)
        except (ValueError, OSError) as exc:
            problems.append(f"{name}: unreadable ({exc})")
            continue
        cfg_id = calib.get("board_id")
        if cfg_id is None:
            problems.append(f"{name}: no board_id field")
            continue
        # Files are named board_<N>.json for the human board number, so
        # board_15.json describes label "board15". A file named by chip id
        # instead (board_855203507.json, what calibration_path() writes) yields
        # no known label and falls through to the membership check below.
        label = "board" + os.path.splitext(name)[0][len("board_"):]
        if label in BOARD_REGISTRY:
            if BOARD_REGISTRY[label] != cfg_id:
                problems.append(
                    f"{name}: board_id {cfg_id} != BOARD_REGISTRY[{label!r}] "
                    f"{BOARD_REGISTRY[label]}"
                )
        elif cfg_id not in BOARD_LABELS:
            problems.append(f"{name}: board_id {cfg_id} is not in BOARD_REGISTRY")
    return problems


if __name__ == "__main__":
    import sys

    found = check_calibration_ids()
    for line in found:
        print(f"  {line}")
    print(
        f"{len(BOARD_REGISTRY)} boards registered; "
        + ("calibration files agree" if not found else f"{len(found)} problem(s)")
    )
    sys.exit(1 if found else 0)
