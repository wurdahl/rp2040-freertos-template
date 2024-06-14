# lib/BMM350/CMakeLists.txt

# Define the BMM350 library
add_library(BMM350 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bmm350.c
)

# Specify the include directories for BMP3
target_include_directories(BMM350 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)