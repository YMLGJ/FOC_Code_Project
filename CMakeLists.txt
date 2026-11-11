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
set(CMAKE_PROJECT_NAME FOC_Code_Project)

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
    Drivers/CMSIS/DSP/Lib/GCC
)

# STM32Cube G4 firmware package (contains the ADC HAL driver, which is not
# copied into this project's Drivers folder because ADC was not enabled in CubeMX).
set(STM32CUBE_G4_HAL_DIR
    "C:/Users/YMLGJ/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.3/Drivers/STM32G4xx_HAL_Driver")
if(NOT EXISTS "${STM32CUBE_G4_HAL_DIR}/Inc/stm32g4xx_hal_adc.h")
    message(FATAL_ERROR
        "STM32CubeG4 HAL ADC not found at ${STM32CUBE_G4_HAL_DIR}. "
        "Install STM32Cube_FW_G4_V1.6.3 or update STM32CUBE_G4_HAL_DIR.")
endif()

# The generated STM32_Drivers/USB_Device_Library targets also include
# stm32g4xx_hal.h, so they need the ADC header from the Cube package too.
target_include_directories(stm32cubemx INTERFACE ${STM32CUBE_G4_HAL_DIR}/Inc)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/custom_lib/bsp/bsp.c
    Core/custom_lib/debug/debug.c
    Core/custom_lib/system_m4/sys_m4.c
    Core/custom_lib/foc/foc.c
    Core/custom_lib/foc/foc_current.c
    Core/custom_lib/foc/foc_obs.c
    ${STM32CUBE_G4_HAL_DIR}/Src/stm32g4xx_hal_adc.c
    ${STM32CUBE_G4_HAL_DIR}/Src/stm32g4xx_hal_adc_ex.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Drivers/CMSIS/DSP/Include
    Core/custom_lib/bsp
    Core/custom_lib/debug
    Core/custom_lib/system_m4
    Core/custom_lib/foc
    ${STM32CUBE_G4_HAL_DIR}/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    arm_cortexM4lf_math

    # Add user defined libraries    rm -rf build/Debug
)
