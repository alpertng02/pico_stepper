# Create an interface library for pico-stepper
add_library(pico-stepper INTERFACE)

# Specify the source files for the pico-stepper library
target_sources(pico-stepper INTERFACE 
    stepper.hpp
    stepper.cpp
)

# Link the necessary libraries to pico-stepper
target_link_libraries(pico-stepper
    INTERFACE 
        hardware_gpio
        hardware_pwm
        hardware_clocks
        pico_time
        pico_platform
        pico_stdlib
)

# Include the current source directory for header files
target_include_directories(pico-stepper
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)