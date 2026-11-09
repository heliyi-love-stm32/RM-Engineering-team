cmake_minimum_required(VERSION 3.22)

# Board 2 is intentionally self-contained.  Do not add a parent project or a
# cross-board library here: the two boards are deployed and debugged separately.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME RM2027HeavyBoard2)
include("cmake/gcc-arm-none-eabi.cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME} C ASM)
message(STATUS "Board 2 build type: ${CMAKE_BUILD_TYPE}")

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# Keep this list explicit.  The inherited Engineer project used a recursive
# source glob; doing that here would silently link board-1 chassis/leg code and
# duplicate task globals into the board-2 image.
set(BOARD2_USER_SOURCES
    source/bsp/can/can_api.c
    source/bsp/can/can_bsp.c
    source/bsp/crc/crc_api.c
    source/bsp/dwt/dwt_bsp.c
    source/bsp/uart/uart_api.c
    source/bsp/uart/uart_bsp.c
    source/library/uint_float_convert.c
    source/module/control/PID/PIDtool.c
    source/module/control/toolBox/tool.c
    source/module/motor/motor_Dji/motor.DJI.c
    source/module/motor/motor_DM/motor_DM.c
    source/module/refereeSys/referee_api.c
    source/task/Heavy_Task/heavy_board2.c
    source/task/Heavy_Task/heavy_interboard.c
    source/task/Heavy_Task/heavy_referee.c
    source/task/Heavy_Task/heavy_sbus.c
    source/task/Heavy_Task/heavy_shoot.c
    source/task/Heavy_Task/heavy_watchdog.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${BOARD2_USER_SOURCES})

# The inherited framework uses headers in many small directories.  Headers do
# not create a build dependency; this only exposes the H7 BSP/module API to the
# explicitly listed sources above.
file(GLOB_RECURSE BOARD2_HEADER_FILES CONFIGURE_DEPENDS
    "${PROJECT_SOURCE_DIR}/source/*.h")
set(BOARD2_INCLUDE_DIRS)
foreach(BOARD2_HEADER_FILE IN LISTS BOARD2_HEADER_FILES)
    get_filename_component(BOARD2_HEADER_DIR "${BOARD2_HEADER_FILE}" DIRECTORY)
    list(APPEND BOARD2_INCLUDE_DIRS "${BOARD2_HEADER_DIR}")
endforeach()
list(REMOVE_DUPLICATES BOARD2_INCLUDE_DIRS)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${BOARD2_INCLUDE_DIRS}
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/DSP/PrivateInclude
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    HEAVY_BOARD2=1
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    stm32cubemx
    m
    ${PROJECT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM7lfdp_math.a
)
