LH2 localization, from a capture to a position
==============================================

A DotBot finds its position from the Lighthouse 2 (LH2) base stations in four
stages, each a module with its own host test. This page follows one synthetic
sweep pair through all of them, with the numbers the host build prints. The
station is channel 0; the photodiode is at (1000, 1000) mm on the floor.

| Stage | Module | Runs on |
|---|---|---|
| Capture | `bsp/lh2.h`: `db_lh2_capture_init()`, `db_lh2_capture_pop()` | the robot only (TS4231, SPIM, (D)PPI) |
| Decode | `drv/lh2_decoder.h`: `db_lh2_decoder_feed()`, `db_lh2_decoder_take_pair()` | robot and host |
| Geometry | `drv/lh2_geometry.h`: `db_lh2_camera_point()`, `db_lh2_solve()`, `db_lh2_sweep_lines()` | robot and host |
| Station choice | `drv/lh2_select.h`: `db_lh2_select_offer_pair()`, `db_lh2_lines_drain()` | robot and host |

The host tests build the same sources with `make test`; `tests/lh2_synth.c`
and `tests/lh2_scene.c` generate the captures a station would produce.

## 1. Capture

A station's rotor restarts its LFSR at angle 0 of every revolution and emits
one bit per count, a count being 8 periods of 48 MHz. When a sweep crosses the
photodiode, the TS4231 drops its envelope line and the SPIM samples the data
line at 32 MHz for 64 bytes. A capture is those bytes plus the time the
handler stamped it with:

```text
sweep 0: stamp 7398 us, spi 1f1ce303e3031f031f1ce31ce31c1f03...
sweep 1: stamp 14342 us, spi 8c7c0f83f07c0c7c0f838c738c73f383...
```

## 2. Decode

`db_lh2_demodulate()` undoes the biphase-mark code into 64 bits,
`db_lh2_polynomial()` finds which of the 32 LFSR polynomials (two per station)
they belong to, and `db_lh2_lfsr_index()` finds where in that polynomial's
sequence they start, from the static checkpoints and the decoder's own:

```text
sweep 0: bits 7297f0da91c3a47d, polynomial 0 (station 0), offset 8, index 38303 -> count 38295
sweep 1: bits c043f514cb5a7405, polynomial 0 (station 0), offset 8, index 79968 -> count 79960
```

`db_lh2_decoder_feed()` does all of it, checks the count, and stores it in one
of the station's two sweep slots, chosen by time against the rotor period.
`db_lh2_decoder_take_pair()` hands both slots over once both are fresh: the
pair (38295, 79960). The scene put the sweeps at 38294.5 and 79959.4 counts;
the decoder reads the count of the first whole bit.

## 3. Geometry

The two counts are two rotor angles, and `db_lh2_camera_point()` turns them into
the station's pinhole image point, (0.042481, -0.077513). The calibration's
homography maps it to the floor: `db_lh2_solve()` gives (999.960, 999.979) mm.
`db_lh2_sweep_lines()` turns each sweep alone into a line on the floor, with a
variance along its normal; both pass within 0.05 mm of the photodiode.

## 4. Station choice

With several stations in view, `db_lh2_select_offer_pair()` solves each fresh
pair, drops a solve that is not finite or lies outside its station's
rectangle, and keeps the one nearest its own rectangle's centre. Every solve
inside its rectangle adds its two floor lines to a ring the control core
drains with `db_lh2_lines_drain()`; the fix and the lines then go to
`db_control_tick()`, which fuses the lines into its pose estimate.

## Where each stage is tested

- `tests/test_lh2_baseline.c`: today's decode behaviour, defects included, against
  a frozen copy of the original decoder.
- `tests/test_lh2_decoder.c`: `drv/lh2_decoder` against that copy, capture by capture.
- `tests/test_lh2_select.c`: `drv/lh2_select` against a frozen copy of the
  original station choice, and the station-rule cases PyDotBot's site editor writes.
- `tests/test_lh2_scene.c`: a robot driven by the control core's plant, seen
  by two stations, through every stage into the estimator.
- `tests/fuzz_lh2_decoder.c`: random input to the decoder and the choice, `make fuzz`.
