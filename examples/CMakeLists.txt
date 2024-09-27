
# Add the executable for the stepper example
add_executable(stepper_example stepper_example.cpp)

target_compile_options(stepper_example PRIVATE) 

# Link the necessary libraries to the stepper example
target_link_libraries(stepper_example 
    pico_stdlib
    pico-stepper
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(stepper_example)

# Enable USB output, disable UART output
pico_enable_stdio_usb(stepper_example 1)
pico_enable_stdio_uart(stepper_example 0)

#--------------------------------------------------------------

# Add the executable for the stepper example
add_executable(arm_test arm_test.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(arm_test 
    pico_stdlib
    pico-stepper
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(arm_test)

# Enable USB output, disable UART output
pico_enable_stdio_usb(arm_test 1)
pico_enable_stdio_uart(arm_test 0)

#--------------------------------------------------------------

# Add the executable for the stepper example
add_executable(multicore_example multicore_example.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(multicore_example 
    pico_stdlib
    pico_multicore
    pico-stepper
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(multicore_example)

# Enable USB output, disable UART output
pico_enable_stdio_usb(multicore_example 1)
pico_enable_stdio_uart(multicore_example 0)

#--------------------------------------------------------------

# Add the executable for the stepper example
add_executable(same_stepper_both_cores same_stepper_both_cores.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(same_stepper_both_cores 
    pico_stdlib
    pico_multicore
    pico-stepper
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(same_stepper_both_cores)

# Enable USB output, disable UART output
pico_enable_stdio_usb(same_stepper_both_cores 1)
pico_enable_stdio_uart(same_stepper_both_cores 0)



