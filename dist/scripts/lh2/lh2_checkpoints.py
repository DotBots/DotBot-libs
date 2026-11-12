#!/usr/bin/env python3
"""Generate the static checkpoints of the LH2 LFSR search.

drv/lh2_decoder/lh2_checkpoints.c holds, per polynomial, 64 checkpoints: the
one state whose top 6 bits are i, for each i, packed as index << 11 | the
state's low 11 bits. The search walks the LFSR both ways from a capture's
state until it meets one, so its cost is the distance to the nearest. This
picks, for every polynomial, the 64 states (one per top-6-bit value) spread
as evenly as an assignment allows over the counts a revolution can give, and
rewrites the table. Needs numpy and scipy.

    python3 dist/scripts/lh2/lh2_checkpoints.py           # rewrite the table
    python3 dist/scripts/lh2/lh2_checkpoints.py --check   # verify the table, report the worst search
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

PERIOD = 131071  # 2^17 - 1
CHECKPOINTS = 64
REVOLUTION = 959000 // 8  # counts in the slowest channel's revolution, the most any capture gives
TABLE = Path(__file__).resolve().parents[3] / "drv" / "lh2_decoder" / "lh2_checkpoints.c"


def polynomials(text: str) -> list[int]:
    block = text[text.index("db_lh2_polynomials") : text.index("};")]
    return [int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]{8})", block)]


def table(text: str) -> list[list[int]]:
    block = text[text.index("db_lh2_checkpoints[") :]
    words = [int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]{8})", block)]
    return [words[i * CHECKPOINTS : (i + 1) * CHECKPOINTS] for i in range(len(words) // CHECKPOINTS)]


def sequence(poly: int) -> np.ndarray:
    """The state at every index, from state 1 at index 0, as the decoder's LFSR runs forward."""
    states = np.empty(PERIOD, dtype=np.uint32)
    s = 1
    for n in range(PERIOD):
        states[n] = s
        s = ((s << 1) | (bin(s & poly).count("1") & 1)) & 0x1FFFF
    return states


def worst_gap(indices: list[int]) -> int:
    """The longest search from any count of a revolution: its distance to the nearest checkpoint, either way round."""
    pts = np.array(sorted(indices))
    n = np.arange(REVOLUTION)
    d = np.abs(n[:, None] - pts[None, :])
    return int(np.minimum(d, PERIOD - d).min(axis=1).max())


def choose(states: np.ndarray) -> list[int]:
    """One index per top-6-bit value, spread evenly over 0..REVOLUTION."""
    buckets = [np.nonzero((states[:REVOLUTION] >> 11) == h)[0] for h in range(CHECKPOINTS)]
    targets = (np.arange(CHECKPOINTS) + 0.5) * REVOLUTION / CHECKPOINTS
    cost = np.empty((CHECKPOINTS, CHECKPOINTS))
    near = np.empty((CHECKPOINTS, CHECKPOINTS), dtype=np.int64)
    for h, b in enumerate(buckets):
        d = np.abs(b[None, :] - targets[:, None])
        k = d.argmin(axis=1)
        cost[h] = d[np.arange(CHECKPOINTS), k]
        near[h] = b[k]
    rows, cols = linear_sum_assignment(cost)
    chosen = [0] * CHECKPOINTS
    for h, t in zip(rows, cols):
        chosen[h] = int(near[h, t])
    return chosen


def render(text: str, rows: list[list[int]]) -> str:
    head = text[: text.index("const uint32_t db_lh2_checkpoints[")]
    out = [head + "const uint32_t db_lh2_checkpoints[DB_LH2_POLYNOMIALS][DB_LH2_CHECKPOINTS] = {"]
    for p, row in enumerate(rows):
        out.append("    {")
        out.append(f"        // polynomial {p}")
        out += [f"        0x{w:08X}," for w in row]
        out.append("    },")
    out.append("};")
    return "\n".join(out) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    text = TABLE.read_text()
    polys = polynomials(text)
    current = table(text)
    rows = []
    worst = 0
    for p, poly in enumerate(polys):
        states = sequence(poly)
        for h, w in enumerate(current[p]):
            index, low = w >> 11, w & 0x7FF
            if states[index] != (h << 11) | low:
                print(f"polynomial {p} slot {h}: the table's word does not match the LFSR")
                return 1
        if args.check:
            worst = max(worst, worst_gap([w >> 11 for w in current[p]]))
            continue
        chosen = choose(states)
        rows.append([(n << 11) | (int(states[n]) & 0x7FF) for n in chosen])
        worst = max(worst, worst_gap(chosen))
    if args.check:
        print(f"table verified; the longest search from a revolution's count is {worst} steps")
        return 0
    TABLE.write_text(render(text, rows))
    print(f"wrote {TABLE.relative_to(TABLE.parents[2])}; the longest search from a revolution's count is {worst} steps")
    return 0


if __name__ == "__main__":
    sys.exit(main())
