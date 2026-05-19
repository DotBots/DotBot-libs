# dotbot-libs — opt-in CMake build.
#
# Additive: SES (.emProject) projects continue to work unchanged. Only
# consumers that opt into CMake see the target below.
#
# Usage from a downstream CMake project:
#
#   set(DOTBOT_LIBS /path/to/dotbot-libs)
#   add_subdirectory(${DOTBOT_LIBS} dotbot-libs-build)
#
#   add_executable(my_app main.c ...)
#   target_link_libraries(my_app PRIVATE dotbot_libs)
#   target_compile_definitions(my_app PRIVATE
#       NRF5340_XXAA NRF_APPLICATION BOARD_DOTBOT_V3
#       # NRF_TRUSTZONE_NONSECURE  # add only for sandbox / non-secure apps
#   )
#
# Consumer responsibilities (not enforced here):
#   * Cross-compiling toolchain (gcc-arm-none-eabi recommended).
#   * Board / chip / TrustZone-mode defines (see usage example).
#   * Linker script.
# Headers + startup are handled here:
#   * Nordic MDK + ARM CMSIS-Core are fetched via FetchContent at
#     configure time and exported via the `dotbot_nordic` INTERFACE
#     target. Linking `dotbot_libs` brings them in transitively.
#   * Pure-C startup (`nRF/System/cpu.c`) is part of `dotbot_libs`.
#     Consumers that want headers ONLY (e.g. ship their own startup)
#     should link `dotbot_nordic` instead of `dotbot_libs`.
#
# Why INTERFACE + target_sources (not a STATIC library):
#   dotbot-libs sources need different defines per consumer — most
#   notably NRF_TRUSTZONE_NONSECURE flips bsp/gpio.h between NRF_P0_S
#   and NRF_P0_NS aliases. INTERFACE + target_sources lets each
#   consumer compile the same sources under its own define set, which
#   matters when bare and sandbox apps coexist in one build.
#
# Scope of this first cut: sources known to compile under
# gcc-arm-none-eabi for nRF5340 app core (validated in proto-dotbot-
# firmware against DotBot v3 hardware). Subsequent PRs will add
# per-chip variants (lh2_default vs lh2_nrf5340_net), more drivers
# (control_loop, lh2, hdlc), and the vendored crypto blob.

cmake_minimum_required(VERSION 3.20)

project(dotbot-libs LANGUAGES C ASM)

# ── Nordic MDK + ARM CMSIS-Core via FetchContent ─────────────────────
# dotbot-libs sources `#include <nrf.h>` and `#include "core_cm33.h"`
# but the repo does not vendor the Nordic device headers (SES builds
# pick them up from installed CMSIS packs). To make standalone CMake
# builds reproducible — and to give every downstream consumer ONE
# canonical pin instead of each repo redeclaring its own — fetch
# them here and export the include paths via the dotbot_nordic
# INTERFACE target below.
include(FetchContent)

FetchContent_Declare(nrfx
    GIT_REPOSITORY https://github.com/NordicSemiconductor/nrfx.git
    GIT_TAG        v3.5.0
    GIT_SHALLOW    TRUE
)
FetchContent_Declare(cmsis_core
    GIT_REPOSITORY https://github.com/ARM-software/CMSIS_5.git
    GIT_TAG        5.9.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nrfx cmsis_core)

# Lightweight INTERFACE target: ONLY the Nordic + ARM device headers.
# Link this from targets that want the headers but supply their own
# startup/BSP (e.g. swarmit's secure bootloader and netcore, which
# ship their own System/startup.c and intentionally do NOT link
# dotbot_libs to avoid double-startup with nRF/System/cpu.c).
add_library(dotbot_nordic INTERFACE)
target_include_directories(dotbot_nordic INTERFACE
    ${nrfx_SOURCE_DIR}/mdk                       # nrf.h + Nordic peripheral defs
    ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include  # core_cm33.h, __WFE, etc.
)

# Full dotbot-libs INTERFACE: BSP + drivers + pure-C startup, with the
# Nordic headers propagated transitively via dotbot_nordic.
add_library(dotbot_libs INTERFACE)

target_sources(dotbot_libs INTERFACE
    # Pure-C startup: cpu.c defines reset_handler() and #include's the
    # per-chip _cpu.c file (vector table, weak ISR aliases, IPC glue)
    # based on NRF5340_XXAA / NRF_APPLICATION defines from the consumer.
    ${CMAKE_CURRENT_SOURCE_DIR}/nRF/System/cpu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/nRF/System/nRF5340_xxAA_Application_system_init.c

    # BSP
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/nrf/board.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/nrf/clock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/nrf/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/nrf/pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/nrf/timer.c

    # Drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/drv/motors/motors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drv/rgbled/rgbled.c
)

target_include_directories(dotbot_libs INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/conf
    ${CMAKE_CURRENT_SOURCE_DIR}/drv
    ${CMAKE_CURRENT_SOURCE_DIR}/nRF/System  # so cpu.c can #include the per-chip _cpu.c
)

# Propagate Nordic + CMSIS headers to dotbot_libs consumers for free.
target_link_libraries(dotbot_libs INTERFACE dotbot_nordic)
