add_library(pico-stepper INTERFACE)

target_sources(pico-stepper INTERFACE 
    stepper.hpp
    stepper.cpp
)

target_link_libraries(pico-stepper
    INTERFACE 
        hardware_gpio
        hardware_pwm
        hardware_clocks
        pico_time
        pico_platform
        pico_stdlib
)

target_include_directories(pico-stepper
    INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}  
)   