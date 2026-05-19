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
#   * Nordic MDK device headers (nrf.h, ...) and ARM CMSIS-Core
#     (core_cm33.h) on the include path. Easy path: FetchContent
#     `NordicSemiconductor/nrfx` and `ARM-software/CMSIS_5`.
#   * Board / chip / TrustZone-mode defines (see usage example).
#   * Startup code, linker script, Reset_Handler.
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
