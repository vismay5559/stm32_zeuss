cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
#
# RelWithDebInfo, not Debug: this firmware has a hard 1 ms budget and the
# estimator's predict step alone is two 21x21 matrix products. An unoptimised
# build is not a slower version of the robot, it is a different one. -O2 with
# full debug symbols keeps the debugger usable without giving that up.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# What this firmware does, and what it insists on being healthy.
# See Appli/App/nexus_mode.h and Appli/App/health.h.
#
# Both are also read from the ENVIRONMENT, and that is not redundancy.
# The top-level project builds this one through ExternalProject_Add, whose
# CMAKE_ARGS are fixed in the CubeMX-generated mx-generated.cmake - so a
# -D on `cmake --preset ...` never reaches this file, while an environment
# variable passes straight through. Editing the generated file instead would
# work until the next CubeMX run silently reverted it.
#
#     NEXUS_MODE=NEXUS_MODE_LEG_CAN cmake --preset Debug     # whole project
#     cmake -S Appli -B Appli/build -DNEXUS_MODE=...         # this one alone
if(NOT NEXUS_MODE AND DEFINED ENV{NEXUS_MODE})
    set(NEXUS_MODE "$ENV{NEXUS_MODE}")
endif()

set(NEXUS_MODE "${NEXUS_MODE}" CACHE STRING
    "Firmware mode: NEXUS_MODE_ROBOT, NEXUS_MODE_LEG_CAN or NEXUS_MODE_IMU")
if(NOT NEXUS_MODE)
    set(NEXUS_MODE "NEXUS_MODE_ROBOT" CACHE STRING "" FORCE)
endif()
set_property(CACHE NEXUS_MODE PROPERTY STRINGS
    NEXUS_MODE_ROBOT NEXUS_MODE_LEG_CAN NEXUS_MODE_IMU)

if(NOT HEALTH_EXPECTED_MASK AND DEFINED ENV{HEALTH_EXPECTED_MASK})
    set(HEALTH_EXPECTED_MASK "$ENV{HEALTH_EXPECTED_MASK}")
endif()
set(HEALTH_EXPECTED_MASK "${HEALTH_EXPECTED_MASK}" CACHE STRING
    "Subsystems that must be healthy before the robot may arm. Empty = all.")

message("Build type: " ${CMAKE_BUILD_TYPE})
message("Nexus mode: " ${NEXUS_MODE})

if(NOT NEXUS_MODE STREQUAL "NEXUS_MODE_ROBOT")
    message(WARNING
        "Building a BRING-UP TEST binary (${NEXUS_MODE}), not the robot loop. "
        "It will not answer the Pi. Unset NEXUS_MODE for flight firmware.")
endif()

if(HEALTH_EXPECTED_MASK)
    message(WARNING
        "HEALTH_EXPECTED_MASK is narrowed to ${HEALTH_EXPECTED_MASK}. "
        "Subsystems outside it cannot fault, so the failsafe will not act on "
        "them. Bring-up only - unset it before anything walks.")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard " )

#Linker options
set(STM32_LINKER_SCRIPT stm32h7rsxx_ROMxspi2.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME nexus_first_Appli)
project(${CMAKE_PROJECT_NAME})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include mx-generated
include("mx-generated.cmake")


# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    NEXUS_MODE=${NEXUS_MODE}
    $<$<BOOL:${HEALTH_EXPECTED_MASK}>:HEALTH_EXPECTED_MASK=${HEALTH_EXPECTED_MASK}>
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/app.c
    App/contact.c
    App/health.c
    App/safety.c
    App/watchdog.c
    App/robot_config.c
    App/lie_group.c
    App/kinematics.c
    App/inekf.c
    App/fusion.c
    App/gait_ref.c
    App/test_leg_can.c
    App/test_leg_torque.c
    App/test_imu.c
    App/enc_as5048a.c
    App/imu_bno085.c
    App/act_odrive.c
    App/link_usb.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)

# Pull in newlib-nano's floating-point printf support.
#
# nano.specs links a cut-down printf that silently prints NOTHING for %f - so
# "pos=%.4f" comes out as "pos=" with no warning at compile or run time. Any
# diagnostic that reports a float (encoder position, velocity, torque, IMU
# quaternions) needs this or it quietly reports blanks.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()

# Produce a .hex next to the .elf - FLASH THE .hex, NOT THE .elf.
#
# STM32CubeProgrammer walks every LOAD segment in an ELF and tries to erase at
# each segment address. This ELF has two segments that hold no data at all:
#   0x20000000  heap + stack in DTCM   (FileSiz 0)
#   0x24070000  non-cacheable buffers  (FileSiz 0)
# With the external loader attached - which only knows how to drive flash at
# 0x70000000 - erasing 0x20000000 fails and the whole download aborts with
# "Error: failed to download Sector[0]".
#
# A .hex only records sections that actually contain data, at their load
# addresses, so it programs cleanly (0x70000000-0x70011600 and nothing else).
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex (flash this, not the .elf)"
)









