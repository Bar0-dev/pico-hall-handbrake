cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico-handbrake)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project/home/bartosz/projects/pico-hall-handbrake
add_executable(main
    main.c
)

target_sources(main PUBLIC ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c)

target_include_directories(main PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(main pico_stdlib hardware_adc tinyusb_device tinyusb_board)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(main)