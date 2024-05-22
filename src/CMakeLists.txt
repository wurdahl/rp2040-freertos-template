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

# when using debug probe with uart cables, leave commented out
#   debug probe allows you to see serial output best
# if debug probe cannot connect to uart pins, uncomment so that you get direct output from usb
# pico_enable_stdio_usb(${ProjectName} 1)
# pico_enable_stdio_uart(${ProjectName} 0)

