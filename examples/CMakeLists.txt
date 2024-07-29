# Add the executable for the stepper example
add_executable(stepper_example stepper_example.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(stepper_example 
    pico_stdlib
    pico-stepper
)

# Set compile options for the stepper example
target_compile_options(stepper_example PRIVATE -Wall -Wextra -Werror)

# Add extra outputs for the stepper example
pico_add_extra_outputs(stepper_example)

# Enable USB output, disable UART output
pico_enable_stdio_usb(stepper_example 1)
pico_enable_stdio_uart(stepper_example 0)


