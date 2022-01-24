add_library(mbed-weact-f411ce INTERFACE)

if(${MBED_TOOLCHAIN} STREQUAL "GCC_ARM")
    set(STARTUP_FILE TOOLCHAIN_GCC_ARM/startup_stm32f411xe.S)
    set(LINKER_FILE TOOLCHAIN_GCC_ARM/stm32f411xe.ld)
elseif(${MBED_TOOLCHAIN} STREQUAL "ARM")
    set(STARTUP_FILE TOOLCHAIN_ARM/startup_stm32f411xe.S)
    set(LINKER_FILE TOOLCHAIN_ARM/stm32f411xe.sct)
endif()

target_sources(mbed-weact-f411ce 
    INTERFACE
        ${STARTUP_FILE}
        system_clock.c
        PeripheralPins.c
)

target_include_directories(mbed-weact-f411ce 
    INTERFACE
        .
)


mbed_set_linker_script(mbed-weact-f411ce ${CMAKE_CURRENT_SOURCE_DIR}/${LINKER_FILE})

target_link_libraries(mbed-weact-f411ce INTERFACE mbed-stm32f4)
