# lib/commonBMI08X/CMakeLists.txt

# Define the common library for BMI08X
add_library(COMMONBMI08X STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/commonBMI08X.c
)

# Specify the include directories for BMP3
target_include_directories(COMMONBMI08X PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(COMMONBMI08X PUBLIC lib/i2cLib/include)
include_directories(COMMONBMI08X PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Link the BMP3 library with the common library
target_link_libraries(COMMONBMI08X BMI08X pico_stdlib hardware_i2c I2CLIB)