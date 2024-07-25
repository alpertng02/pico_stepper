add_executable(stepper_example stepper_example.cpp)

target_link_libraries(stepper_example 
    pico_stdlib
    pico-stepper
)

pico_add_extra_outputs(stepper_example)

# enable usb output, disable uart output
pico_enable_stdio_usb(stepper_example 1)
pico_enable_stdio_uart(stepper_example 0)


