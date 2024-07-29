cmake_minimum_required(VERSION 3.13)  # Specify the minimum CMake version required

# Include the Pico SDK
include(pico_sdk_import.cmake)

# Define the project name, version, and supported languages
project(pico_stepper_dm860h VERSION 1.0 LANGUAGES C CXX ASM)

# Set the C and C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-variable)

# Initialize the Pico SDK
pico_sdk_init()

# Add subdirectories for libraries and examples
add_subdirectory(lib/Stepper)
add_subdirectory(examples)

