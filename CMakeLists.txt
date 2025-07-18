cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME AD9833_Demo)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # CMSIS/DSP/Src/BasicMathFunctions/BasicMathFunctions.c
    # CMSIS/DSP/Src/BayesFunctions/BayesFunctions.c
    # CMSIS/DSP/Src/CommonTables/CommonTables.c
    # CMSIS/DSP/Src/ComplexMathFunctions/ComplexMathFunctions.c
    # CMSIS/DSP/Src/ControllerFunctions/ControllerFunctions.c
    # CMSIS/DSP/Src/DistanceFunctions/DistanceFunctions.c
    # CMSIS/DSP/Src/FastMathFunctions/FastMathFunctions.c
    # CMSIS/DSP/Src/FilteringFunctions/FilteringFunctions.c
    # CMSIS/DSP/Src/InterpolationFunctions/InterpolationFunctions.c
    # CMSIS/DSP/Src/MatrixFunctions/MatrixFunctions.c
    # CMSIS/DSP/Src/QuaternionMathFunctions/QuaternionMathFunctions.c
    # CMSIS/DSP/Src/StatisticsFunctions/StatisticsFunctions.c
    # CMSIS/DSP/Src/SupportFunctions/SupportFunctions.c
    # CMSIS/DSP/Src/SVMFunctions/SVMFunctions.c
    # CMSIS/DSP/Src/TransformFunctions/TransformFunctions.c
    # CMSIS/DSP/Src/WindowFunctions/WindowFunctions.c
    Drivers/System/Delay/delay.c
    Drivers/System/usart_printf/usart_printf.c
#    Drivers/AD9833_HAL/AD9833_HAL.c
    Drivers/AD9833_Soft/AD9833_Soft.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    CMSIS/Core/Inc
    CMSIS/DSP/Inc
    CMSIS/DSP/PrivateInclude
    Drivers/System/Delay
    Drivers/System/usart_printf
#    Drivers/AD9833_HAL
    Drivers/AD9833_Soft
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -mfpu=fpv4-sp-d16 -mfloat-abi=hard -u _printf_float")
