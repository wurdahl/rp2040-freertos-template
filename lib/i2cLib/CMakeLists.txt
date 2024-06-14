# lib/i2cLib/CMakeLists.txt

# Define the common library for BMI08X
add_library(I2CLIB STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/i2cLib.c
)

# Specify the include directories for BMP3
target_include_directories(I2CLIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(I2CLIB PUBLIC lib/FreeRTOS-Kernel/include)
include_directories(I2CLIB PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Link the BMP3 library with the common library
target_link_libraries(I2CLIB pico_stdlib hardware_i2c FreeRTOS-Kernel-Heap4)