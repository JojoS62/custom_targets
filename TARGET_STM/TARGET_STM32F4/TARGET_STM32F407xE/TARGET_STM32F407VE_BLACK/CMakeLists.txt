add_library(mbed-stm32f407ve-black INTERFACE)

target_sources(mbed-stm32f407ve-black
    INTERFACE
        PeripheralPins.c
        system_clock_custom.c
        stm32f4_eth_conf.c
        stm32f4_eth_init.c
)

target_include_directories(mbed-stm32f407ve-black
    INTERFACE
        .
)

target_link_libraries(mbed-stm32f407ve-black INTERFACE mbed-stm32f407xe)
