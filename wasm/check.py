"""Check build/wasm/dotbot_control.wasm: no imports, the ABI the header declares,
and a waypoint batch replayed to a known trace.

    make wasm
    python wasm/check.py            # checks; exit status 1 on any failure
    python wasm/check.py --update   # prints the trace hash to paste into GOLDEN
    python wasm/check.py --bench 1000

Needs wasmtime and numpy (pip install wasmtime numpy).

The replay drives one robot from boot through three waypoints against a small
plant written here in Python, and hashes every tick's output and report. The
plant feeds the core whole counts and whole millimetres, so the hash moves
only when the core's behaviour does, or the plant's.
"""

import argparse
import hashlib
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np
import wasmtime

WASM = Path(__file__).resolve().parent.parent / "build" / "wasm" / "dotbot_control.wasm"
ABI_VERSION = 1
GOLDEN = "3707b44e8c974d16575b6a19d3cb48d268bd295df3c8a4b827405b76716faa78"

INPUT = np.dtype(
    [("counts_left", "<i4"), ("counts_right", "<i4"), ("fix_sequence", "<u4"),
     ("fix_x", "<u4"), ("fix_y", "<u4"), ("elapsed_ticks", "<u4")]
)
OUTPUT = np.dtype(
    [("pwm_left", "i1"), ("pwm_right", "i1"), ("brake_left", "u1"), ("brake_right", "u1"),
     ("write", "u1"), ("advertise", "u1"), ("reserved", "u1", 2)]
)
REPORT = np.dtype(
    [("axle_x_mm", "<f4"), ("axle_y_mm", "<f4"), ("heading_deg", "<f4"), ("max_speed_mm_s", "<f4"),
     ("sensor_x", "<u4"), ("sensor_y", "<u4"), ("waypoint_x", "<u4"), ("waypoint_y", "<u4"),
     ("encoder_left", "<u4"), ("encoder_right", "<u4"), ("fix_sequence", "<u4"),
     ("direction", "<i2"), ("axle_x", "<u2"), ("axle_y", "<u2"),
     ("pwm_left", "i1"), ("pwm_right", "i1"), ("brake_left", "u1"), ("brake_right", "u1"),
     ("control_mode", "u1"), ("drive_mode", "u1"), ("steering_state", "u1"), ("estimator_status", "u1"),
     ("waypoint_index", "u1"), ("waypoint_count", "u1"), ("batch_id", "u1"), ("status", "u1"),
     ("reason", "u1"), ("max_speed_10mm", "u1")]
)

STATES = ["IDLE", "NO_HEADING", "ALIGN", "DRIVE", "FINAL_TURN", "ARRIVED", "HOLD",
          "FAILED", "RECOVER", "SETTLE", "NUDGE"]
POINTS = [(1000, 1500), (1500, 1500), (1500, 1000)]
THRESHOLD_MM = 30
START = (1000.0, 1000.0, 0.0)

# v3 geometry, drv/geometry.h
MM_PER_COUNT = math.pi * 43.0 / (28.0 * 51.0)
LEVER_MM = 51.5
TRACK, TRACK_ARC, TRACK_RATIO = 81.0, 85.0, 2.35


class Core:
    def __init__(self, path=WASM):
        self.engine = wasmtime.Engine()
        self.module = wasmtime.Module.from_file(self.engine, str(path))
        self.store = wasmtime.Store(self.engine)
        self.instance = wasmtime.Instance(self.store, self.module, [])
        ex = self.instance.exports(self.store)
        self.memory = ex["memory"]
        self._fn = {e.name: ex[e.name] for e in self.module.exports if isinstance(ex[e.name], wasmtime.Func)}
        self("_initialize")

    def __call__(self, name, *args):
        return self._fn[name](self.store, *args)

    def write(self, ptr, data):
        self.memory.write(self.store, data, ptr)

    def read(self, ptr, length):
        return self.memory.read(self.store, ptr, ptr + length)

    def init(self, count):
        if self("fleet_init", count) != 0:
            raise MemoryError("fleet_init")
        self.count = count
        self.inputs = self("fleet_inputs")
        self.outputs = self("fleet_outputs")
        self.reports = self("fleet_report_buffer")
        self.rx_buffer = self("fleet_rx_buffer")

    def rx(self, index, packet):
        self.write(self.rx_buffer, packet)
        self("fleet_rx", index, self.rx_buffer, len(packet))

    def step(self, inputs):
        self.write(self.inputs, inputs.tobytes())
        self("fleet_step", self.inputs, self.outputs)
        return np.frombuffer(self.read(self.outputs, OUTPUT.itemsize * self.count), OUTPUT)

    def report(self):
        self("fleet_reports", self.reports)
        return np.frombuffer(self.read(self.reports, REPORT.itemsize * self.count), REPORT)


def waypoints_packet(points, threshold, batch_id):
    packet = struct.pack("<BHB", 8, threshold, len(points))
    packet += b"".join(struct.pack("<II", x, y) for x, y in points)
    packet += struct.pack("<BBH", batch_id, 0, 0)
    packet += b"".join(struct.pack("<h", 0x7FFF) for _ in points)
    return packet


class Plant:
    """Duty in, counts and 2-tick-old 10 Hz fixes with 1 mm noise out."""

    TAU_S, DT_S, U_RUN, K_RUN, U_BREAK, FIX_AGE = 0.05, 0.01, 32.0, 0.097, 40.0, 2

    def __init__(self, x, y, heading_deg, seed=12345):
        self.x, self.y, self.h = x, y, heading_deg
        self.v = [0.0, 0.0]
        self.frac = [0.0, 0.0]
        self.pwm, self.brake = [0, 0], [False, False]
        self.rng = seed
        self.tick = 0
        self.history = [self._photodiode()] * (self.FIX_AGE + 1)
        self.fix = (0, 0, 0)

    def _uniform(self):
        self.rng = (self.rng * 1664525 + 1013904223) & 0xFFFFFFFF
        return ((self.rng >> 8) + 0.5) / 16777216.0

    def _gauss(self):
        u1, u2 = self._uniform(), self._uniform()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

    def _photodiode(self):
        h = math.radians(self.h)
        return self.x - LEVER_MM * math.sin(h), self.y + LEVER_MM * math.cos(h)

    def _target(self, pwm, v):
        a = abs(pwm)
        if a <= self.U_RUN or (abs(v) < 1.0 and a < self.U_BREAK):
            return 0.0
        return math.copysign((a - self.U_RUN) / self.K_RUN, pwm)

    def apply(self, out):
        if out["write"]:
            self.pwm = [int(out["pwm_left"]), int(out["pwm_right"])]
            self.brake = [bool(out["brake_left"]), bool(out["brake_right"])]

    def step(self):
        gain = 1.0 - math.exp(-self.DT_S / self.TAU_S)
        d = []
        for w in (0, 1):
            target = 0.0 if self.brake[w] else self._target(self.pwm[w], self.v[w])
            v = self.v[w] + gain * (target - self.v[w])
            d.append(0.5 * (self.v[w] + v) * self.DT_S)
            self.v[w] = v
        diff, total = abs(d[1] - d[0]), abs(d[1] + d[0])
        track = TRACK_ARC if total >= TRACK_RATIO * diff else TRACK + (TRACK_ARC - TRACK) * total / (TRACK_RATIO * diff)
        dtheta = math.degrees((d[0] - d[1]) / track)
        h = math.radians(self.h + dtheta / 2.0)
        self.x += 0.5 * (d[0] + d[1]) * -math.sin(h)
        self.y += 0.5 * (d[0] + d[1]) * math.cos(h)
        self.h = (self.h + dtheta + 180.0) % 360.0 - 180.0
        counts = []
        for w in (0, 1):
            self.frac[w] += d[w] / MM_PER_COUNT
            c = math.trunc(self.frac[w])
            self.frac[w] -= c
            counts.append(c)
        self.history = [self._photodiode()] + self.history[:-1]
        self.tick += 1
        if self.tick % 10 == 0:
            px, py = self.history[-1]
            self.fix = (self.fix[0] + 1, round(max(0.0, px + self._gauss())), round(max(0.0, py + self._gauss())))
        return (counts[0], counts[1], self.fix[0], self.fix[1], self.fix[2], 1)


def replay(core, ticks=1500):
    core.init(1)
    core.rx(0, waypoints_packet(POINTS, THRESHOLD_MM, 1))
    plant = Plant(*START)
    trace = hashlib.sha256()
    states, arrived = [], None
    inputs = np.zeros(1, INPUT)
    for _ in range(ticks):
        inputs[0] = plant.step()
        out = core.step(inputs)
        plant.apply(out[0])
        report = core.report()
        trace.update(out.tobytes())
        trace.update(report.tobytes())
        state = STATES[report[0]["steering_state"]]
        if state != "IDLE" and (not states or states[-1] != state):
            states.append(state)
        if state == "ARRIVED" and arrived is None:
            arrived = plant.tick
    miss = math.hypot(plant.x - POINTS[-1][0], plant.y - POINTS[-1][1])
    return trace.hexdigest(), states, arrived, miss, report[0]


def check(core, update):
    failures = []

    def expect(ok, what):
        print(("ok   " if ok else "FAIL ") + what)
        if not ok:
            failures.append(what)

    expect(len(core.module.imports) == 0, f"no imports ({[(i.module, i.name) for i in core.module.imports]})")
    expect(core("abi_version") == ABI_VERSION, f"ABI version {core('abi_version')}")
    for name, dtype in (("input", INPUT), ("output", OUTPUT), ("report", REPORT)):
        expect(core(f"sizeof_{name}") == dtype.itemsize, f"sizeof {name} {core(f'sizeof_{name}')} == {dtype.itemsize}")

    digest, states, arrived, miss, report = replay(core)
    print(f"     states {'-'.join(states)}, arrived at tick {arrived}, missed by {miss:.1f} mm")
    expect(states[0] == "NO_HEADING" and states[-1] == "ARRIVED", "spins for a heading, then arrives")
    expect(arrived is not None and arrived < 1000, "arrives within 10 s")
    expect(miss < THRESHOLD_MM + 5, "stops within the threshold")
    expect(report["status"] == 2 and report["waypoint_index"] == 3, "reports ARRIVED at index 3")
    if update:
        print(f"GOLDEN = \"{digest}\"")
    else:
        expect(digest == GOLDEN, f"trace hash {digest}")
    return not failures


def bench(core, count, ticks=500):
    """Every robot drives a batch against a vectorised plant; times fleet_step alone."""
    core.init(count)
    for i in range(count):
        core.rx(i, waypoints_packet([(1000, 1600), (1600, 1600)], THRESHOLD_MM, 1))
    x = np.full(count, 1000.0)
    y = np.full(count, 1000.0)
    h = np.zeros(count)
    v = np.zeros((2, count))
    frac = np.zeros((2, count))
    pwm = np.zeros((2, count))
    brake = np.zeros((2, count), bool)
    inputs = np.zeros(count, INPUT)
    inputs["elapsed_ticks"] = 1
    gain = 1.0 - math.exp(-0.01 / 0.05)
    spent = 0.0
    for tick in range(1, ticks + 1):
        a = np.abs(pwm)
        target = np.where((a <= 32.0) | ((np.abs(v) < 1.0) & (a < 40.0)) | brake, 0.0, np.sign(pwm) * (a - 32.0) / 0.097)
        v_new = v + gain * (target - v)
        d = 0.5 * (v + v_new) * 0.01
        v = v_new
        dtheta = np.degrees((d[0] - d[1]) / TRACK)
        rad = np.radians(h + dtheta / 2)
        x -= 0.5 * (d[0] + d[1]) * np.sin(rad)
        y += 0.5 * (d[0] + d[1]) * np.cos(rad)
        h += dtheta
        frac += d / MM_PER_COUNT
        counts = np.trunc(frac)
        frac -= counts
        inputs["counts_left"], inputs["counts_right"] = counts[0], counts[1]
        if tick % 10 == 0:
            rad = np.radians(h)
            inputs["fix_sequence"] += 1
            inputs["fix_x"] = np.rint(x - LEVER_MM * np.sin(rad))
            inputs["fix_y"] = np.rint(y + LEVER_MM * np.cos(rad))
        start = time.perf_counter()
        out = core.step(inputs)
        spent += time.perf_counter() - start
        w = out["write"].astype(bool)
        pwm[0, w], pwm[1, w] = out["pwm_left"][w], out["pwm_right"][w]
        brake[0, w], brake[1, w] = out["brake_left"][w].astype(bool), out["brake_right"][w].astype(bool)
    start = time.perf_counter()
    for _ in range(100):
        reports = core.report()
    report_us = (time.perf_counter() - start) / 100 * 1e6
    per = spent / (ticks * count) * 1e6
    arrived = int(np.sum(reports["status"] == 2))
    print(f"N={count}: fleet_step {per:.3f} us per robot-tick ({per * count * 100 / 1e4:.1f}% of a core at 100 Hz), "
          f"reports {report_us:.0f} us per call, {arrived}/{count} arrived after {ticks / 100:.0f} s")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--update", action="store_true", help="print the trace hash instead of checking it")
    parser.add_argument("--bench", type=int, metavar="N", help="time fleet_step for N robots")
    parser.add_argument("--wasm", type=Path, default=WASM)
    args = parser.parse_args()
    core = Core(args.wasm)
    if args.bench:
        bench(core, args.bench)
        return 0
    return 0 if check(core, args.update) else 1


if __name__ == "__main__":
    sys.exit(main())
