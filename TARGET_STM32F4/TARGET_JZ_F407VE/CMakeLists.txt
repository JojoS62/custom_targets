add_library(mbed-jz-f407ve INTERFACE)

target_sources(mbed-jz-f407ve
    INTERFACE
        PeripheralPins.c
        stm32f4_eth_conf.c
        stm32f4_eth_init.c
)

target_include_directories(mbed-jz-f407ve
    INTERFACE
        .
)

target_link_libraries(mbed-jz-f407ve INTERFACE mbed-stm32f407xe)
