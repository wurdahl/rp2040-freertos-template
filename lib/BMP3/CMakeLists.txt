# lib/BMP3/CMakeLists.txt

# Define the BMP3 library
add_library(BMP3 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bmp3.c
)

# Specify the include directories for BMP3
target_include_directories(BMP3 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
