# DotBot-libs

## Purpose

Low-level peripheral drivers and BSP libraries for the DotBot platform on Nordic nRF52833 / nRF52840 / nRF5340 MCUs. Pure-C library; consumed by `DotBot-firmware`, `swarmit`, and `dotbot-lh2-calibration` via git submodule. The repo is structurally standalone — nothing it imports comes from the other DotBots repos.

## Tech stack

- **Language**: C
- **Targets**: nRF52833, nRF52840, nRF5340 (app + net cores)
- **Build**: SEGGER Embedded Studio (`.emProject` files at root, one per target) driven by a top-level `Makefile`. A Docker wrapper (`aabadie/dotbot:latest`) wraps SES for CI.
- **Style**: `clang-format`
- **Docs**: Doxygen + Sphinx
- **Vendored**: `nRF/` (Nordic startup/linker, ~124K), `bsp/nrf/` (~228K), `crypto/nrf_cc310/` (~1.1M precompiled lib)

## Entry points

- `README.md` — overview, SES setup, list of supported boards.
- `Makefile` — per-target project lists, Docker wrapper, format/doc targets.
- `drv/protocol.h` — the legacy DotBot remote-control wire protocol (the API flagged as outdated; ~234 lines, monolithic enum from 2022).
- `drv/control_loop/` — EKF + pure-pursuit + waypoint transitions; the most active area.

## Build / run / test

```bash
# Build a target
make BUILD_TARGET=<dotbot-v2|dotbot-v3|nrf52833dk|nrf52840dk|nrf5340dk-app|nrf5340dk-net> BUILD_CONFIG=<Debug|Release>

# Same, in container (CI path; SES Docker image build currently commented out)
make docker BUILD_TARGET=... BUILD_CONFIG=...

# Other
make list-projects     # what's buildable
make artifacts         # collect ELF/HEX into artifacts/
make format            # clang-format in-place
make check-format      # clang-format --dry-run
make doc               # Sphinx + Doxygen
make clean / distclean
```

CI: `.github/workflows/build.yml` runs the matrix above plus style + doc. **No unit tests; no host-side test runner.** CI only verifies it compiles.

## Cross-repo dependencies

- **Consumed by** (this repo doesn't depend on them; they pull this in as a submodule):
  - `DotBot-firmware` — submodule at `dotbot-libs/`
  - `swarmit` — submodule at `dotbot-libs/`
  - `dotbot-lh2-calibration` — submodule at `dotbot-libs/`
  - `PyDotBot` — checked out in CI to build `utils/control_loop`
- **Depends on**: nothing internal. Only vendored Nordic content.

## State of repo (snapshot 2026-05-05)

- Last commit on `main`: 2026-04-29
- Total commits on `main`: 77 (young repo)
- Commits in last 90 days: **41** (very active)
- Branches: only `main` (no stale feature branches)
- TODO/FIXME/XXX/HACK markers: ~17 across `bsp drv crypto projects`

## Hot spots and known gaps

- **`drv/control_loop/`** (EKF, pure-pursuit, waypoint transitions) — single author (`aabadie`); bus-factor risk. Departure summer 2026.
- **`drv/protocol.h`** — outdated remote-control API. Replacing it touches many sites; the corresponding Python mirror lives in `PyDotBot/dotbot/protocol.py`.
- **TDMA stack is still alive**: `drv/tdma_client/`, `drv/tdma_server/` (with `_default` and `_nrf5340_app` variants) plus `01drv_tdma_client` / `01drv_tdma_server` projects. The plan is to replace TDMA with Mari integration. **Greenfield**: no half-merged refactor in progress.
- **`Makefile` references projects that don't live here** (`03app_dotbot`, `03app_dotbot_gateway*`, `03app_sailbot`, `03app_xgo`, `03app_freebot`, `03app_nrf5340_net`, `03app_lh2_mini_mote*`). They live in `DotBot-firmware`. Either dead config or out-of-sync filter.
- **No host-side unit tests at all.** Every change is validated only by "does it compile across 6 targets × 2 configs."
- **Docker CI build disabled** (commented out, awaiting SEGGER auth).

## Branch policy

- Default: `main`
- New work: feature branches off `main`, PRs even for solo work.
- Stale branches (>6 months untouched) should be deleted or, if salvageable, rebased and merged.

## Agent-task ideas (concrete, bounded)

- **Add host-side unit tests** for protocol parsing, PID, control_loop, lz4/uzlib wrappers, and crypto soft implementations. Pure-logic modules separable from HAL. High value (closes the test-debt gap).
- **TDMA → Mari driver swap**: replace `drv/tdma_*` with a mari driver. Greenfield, no half-merged state.
- **Replace `drv/protocol.h`** with a versioned message format; coordinate with `PyDotBot/dotbot/protocol.py`.
- **Reconcile `Makefile` project list** with what actually lives in `DotBot-firmware` (or move artifacts here).
- **SES → CMake/GCC migration** (cross-cutting; biggest long-term unblock for new contributors).

## Don't

- **Don't break the SES build** until a working CMake/GCC equivalent has been validated for at least one target. The team still uses SES daily.
- **Don't refactor `crypto/nrf_cc310/`** — vendored upstream binary; track for license/updates only.
- **Don't touch `Makefile` project lists** without checking `DotBot-firmware/Makefile` first; the two should drift toward consistency, not apart.
