# lib/commonBMM350/CMakeLists.txt

# Define the common library for BMI08X
add_library(COMMONBMM350 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/commonBMM350.c
)

# Specify the include directories for BMP3
target_include_directories(COMMONBMM350 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(COMMONBMM350 PUBLIC lib/i2cLib/include)
include_directories(COMMONBMM350 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Link the BMP3 library with the common library
target_link_libraries(COMMONBMM350 BMM350 pico_stdlib hardware_i2c I2CLIB)