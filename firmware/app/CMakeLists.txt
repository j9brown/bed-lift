cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "${CMAKE_CURRENT_SOURCE_DIR}/../modules/drv8434s")
set(BOARD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(BOARD "brownstudios_bedlift")
set(BOARD_DEBUG_RUNNER openocd)
set(BOARD_FLASH_RUNNER openocd)

if(FEATURE_DEBUG)
    set(CONFIG_DEBUG y)
    set(CONFIG_DEBUG_THREAD_INFO y)
endif()

if(FEATURE_LOG)
    list(APPEND EXTRA_CONF_FILE log-deferred.conf)
    #list(APPEND EXTRA_CONF_FILE log-minimal.conf)
else()
    list(APPEND EXTRA_CONF_FILE log-none.conf)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bedlift)

target_sources(app PRIVATE
    src/bed/bed.c
    src/bed/lift.c
    src/bed/span.c
    src/control.c
    src/indicator.c
    src/main.c
    src/melody.c
    src/monitor.c
    src/power.c
)

target_include_directories(app PRIVATE
    src
)
