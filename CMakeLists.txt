cmake_minimum_required(VERSION 3.1)
project(uart LANGUAGES C ASM)
file(GLOB APP_SOURCES 
    "${CMAKE_SOURCE_DIR}/*.c"
    "${CMAKE_SOURCE_DIR}/*.s"
)
add_executable(uart.elf ${APP_SOURCES})
# Add include directories
target_link_options(uart.elf PRIVATE -T ${CMAKE_SOURCE_DIR}/linker.ld)
