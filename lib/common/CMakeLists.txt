# lib/BMP3/CMakeLists.txt

# Define the BMP3 library
add_library(COMMON STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common.c
)

# Specify the include directories for BMP3
target_include_directories(COMMON PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link the BMP3 library with the common library
target_link_libraries(COMMON BMP3 pico_stdlib hardware_i2c)