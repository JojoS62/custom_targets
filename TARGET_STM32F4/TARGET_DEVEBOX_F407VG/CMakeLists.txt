add_library(mbed-devebox-f407vg INTERFACE)

if(${MBED_TOOLCHAIN} STREQUAL "GCC_ARM")
    set(STARTUP_FILE TOOLCHAIN_GCC_ARM/startup_stm32f407xx.S)
    set(LINKER_FILE TOOLCHAIN_GCC_ARM/stm32f407xg.ld)
elseif(${MBED_TOOLCHAIN} STREQUAL "ARM")
    set(STARTUP_FILE TOOLCHAIN_ARM/startup_stm32f407xx.S)
    set(LINKER_FILE TOOLCHAIN_ARM/stm32f407xg.sct)
endif()

target_sources(mbed-devebox-f407vg
    INTERFACE
        ${STARTUP_FILE}
        system_clock.c
        PeripheralPins.c
        stm32f4_eth_conf.c
        stm32f4_eth_init.c
)

target_include_directories(mbed-devebox-f407vg
    INTERFACE
        .
)

mbed_set_linker_script(mbed-devebox-f407vg ${CMAKE_CURRENT_SOURCE_DIR}/${LINKER_FILE})

target_link_libraries(mbed-devebox-f407vg INTERFACE mbed-stm32f4)
