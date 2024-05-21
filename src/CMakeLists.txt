add_executable(${ProjectName}
    main.c
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    FreeRTOS-Kernel-Heap4
    BMP3
    COMMON 
    )

pico_add_extra_outputs(${ProjectName})

# add_executable(MyExecutable main.c)

# # Link the BMP3 library and FreeRTOS with the executable
# target_link_libraries(MyExecutable BMP3 FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

# # Include directories for the BMP3 library
# target_include_directories(MyExecutable PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../lib/BMP3/include)