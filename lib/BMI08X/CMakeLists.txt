# lib/BMI08X/CMakeLists.txt

# Define the BMI08X library
add_library(BMI08X STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bmi08a.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bmi08g.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bmi08xa.c
)

# Specify the include directories for BMP3
target_include_directories(BMI08X PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
