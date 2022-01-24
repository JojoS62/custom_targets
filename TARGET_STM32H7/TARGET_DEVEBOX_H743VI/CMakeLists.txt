add_library(mbed-devebox-h743vi INTERFACE)

target_sources(mbed-devebox-h743vi
    INTERFACE
        PeripheralPins.c
        system_clock.c
)

target_include_directories(mbed-devebox-h743vi
    INTERFACE
        .
)

target_link_libraries(mbed-devebox-h743vi INTERFACE mbed-stm32h743xi)

