cmake_minimum_required(VERSION 3.13)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(robot_pwm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(${PROJECT_NAME} 
    main.cpp 
    src/motor_control.cpp 
    include/motor_control.h 
    include/pins.h
    include/config.h
)

pico_set_program_name(${PROJECT_NAME} "robot_pwm")
pico_set_program_version(${PROJECT_NAME} "0.1")

target_include_directories(${PROJECT_NAME} PRIVATE include)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_adc)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME}
        hardware_timer
        hardware_pwm
)

pico_add_extra_outputs(${PROJECT_NAME})

# Enable USB serial
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
