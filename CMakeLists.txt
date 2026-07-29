cmake_minimum_required(VERSION 3.22)

project(dexslide_infra LANGUAGES NONE)

set(DEXSLIDE_BOARD "" CACHE STRING "Firmware board to build")
set_property(CACHE DEXSLIDE_BOARD PROPERTY STRINGS
    joints_f103
    tactile_h562
    tactile_h562_raw
)

if(NOT DEXSLIDE_BOARD)
    message(FATAL_ERROR
        "DEXSLIDE_BOARD is not set.\n"
        "Use one of:\n"
        "  cmake -DDEXSLIDE_BOARD=joints_f103 ...\n"
        "  cmake -DDEXSLIDE_BOARD=tactile_h562 ...\n"
        "  cmake -DDEXSLIDE_BOARD=tactile_h562_raw ..."
    )
endif()

if(DEXSLIDE_BOARD STREQUAL "joints_f103")
    add_subdirectory(firmware/joints_f103)
elseif(DEXSLIDE_BOARD STREQUAL "tactile_h562")
    if(EXISTS "${CMAKE_SOURCE_DIR}/firmware/tactile_h562/CMakeLists.txt")
        add_subdirectory(firmware/tactile_h562)
    else()
        message(FATAL_ERROR
            "firmware/tactile_h562 has no generated CubeMX CMake project yet.\n"
            "Generate the STM32H562VGT6 project in firmware/tactile_h562 first."
        )
    endif()
elseif(DEXSLIDE_BOARD STREQUAL "tactile_h562_raw")
    add_subdirectory(firmware/tactile_h562_raw)
else()
    message(FATAL_ERROR "Unknown DEXSLIDE_BOARD: ${DEXSLIDE_BOARD}")
endif()
