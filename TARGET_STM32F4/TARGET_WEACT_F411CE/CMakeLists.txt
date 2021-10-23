add_library(mbed-weact-f411ce INTERFACE)

target_sources(mbed-weact-f411ce 
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-weact-f411ce 
    INTERFACE
        .
)

target_link_libraries(mbed-weact-f411ce INTERFACE mbed-stm32f411xe)
