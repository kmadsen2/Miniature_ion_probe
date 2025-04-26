# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Pi_Stat C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(libs/ad5940)
include_directories(./libs/ad5940)

# Add executable. Default name is the project name, version 0.1

add_executable(Pi_Stat Pi_Stat.c server_common.c)

pico_set_program_name(Pi_Stat "Pi_Stat")
pico_set_program_version(Pi_Stat "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Pi_Stat 0)
pico_enable_stdio_usb(Pi_Stat 1)

# Add the standard library to the build
target_link_libraries(Pi_Stat
        pico_stdlib
        pico_btstack_ble
        pico_btstack_cyw43
        pico_cyw43_arch_none
        hardware_adc)

# Add the standard include files to the build
target_include_directories(Pi_Stat PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_btstack_make_gatt_header(Pi_Stat PRIVATE "${CMAKE_CURRENT_LIST_DIR}/temp_sensor.gatt")

# Add any user requested libraries
target_link_libraries(Pi_Stat 
        hardware_spi
        ad5940
        )

pico_add_extra_outputs(Pi_Stat)

