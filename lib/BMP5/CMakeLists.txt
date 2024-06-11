# lib/BMP5/CMakeLists.txt

# Define the BMP3 library
add_library(BMP5 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bmp5.c
)

# Specify the include directories for BMP3
target_include_directories(BMP5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
