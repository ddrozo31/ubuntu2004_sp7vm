cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(flashprint C CXX ASM)

set(CMAKE_C STANDARD 11)
set(CMAKE_CXX STANDARD 17)

pico_sdk_init()

add_executable(flashprint
        pico_c.c
        )

target_link_libraries(flashprint
        pico_stdlib              # for core functionality
        pico_cyw43_arch_lwip_threadsafe_background     # we need Wifi to access the GPIO, but we don't need anything else
        )

pico_enable_stdio_usb(flashprint 1)
pico_enable_stdio_uart(flashprint 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(flashprint)

target_include_directories(flashprint PRIVATE ${CMAKE_CURRENT_LIST_DIR})

