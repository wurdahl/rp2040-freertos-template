# lib/BMP5/CMakeLists.txt

# Define the BMP3 library
add_library(COMMONBMP5 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.c
)

# Specify the include directories for BMP3
target_include_directories(COMMONBMP5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(COMMONBMP5 PUBLIC lib/i2cLib/include)
include_directories(COMMONBMP5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Link the BMP3 library with the common library
target_link_libraries(COMMONBMP5 BMP5 pico_stdlib hardware_i2c I2CLIB)