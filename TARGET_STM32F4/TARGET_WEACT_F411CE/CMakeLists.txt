add_library(mbed-weact-f411ce INTERFACE)

set(STM32CUBE_UPLOAD_ENABLED TRUE)
set(STM32CUBE_CONNECT_COMMAND "-c port=SWD reset=HWrst")
set(STM32CUBE_GDBSERVER_ARGS "--swd")

if(${MBED_TOOLCHAIN} STREQUAL "GCC_ARM")
    set(STARTUP_FILE TOOLCHAIN_GCC_ARM/startup_stm32f411xe.S)
    set(LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/TOOLCHAIN_GCC_ARM/stm32f411xe.ld)
elseif(${MBED_TOOLCHAIN} STREQUAL "ARM")
    set(STARTUP_FILE TOOLCHAIN_ARM/startup_stm32f411xe.S)
    set(LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/TOOLCHAIN_ARM/stm32f411xe.sct)
endif()

target_sources(mbed-weact-f411ce 
    INTERFACE
        # ${STARTUP_FILE}
        system_clock.c
        PeripheralPins.c
)

target_include_directories(mbed-weact-f411ce 
    INTERFACE
        .
)

mbed_disable_mcu_target_file(mbed-stm32f411xe system_clock.c)
target_link_libraries(mbed-weact-f411ce INTERFACE mbed-stm32f411xe)

# mbed_set_linker_script(mbed-weact-f411ce ${LINKER_FILE})
# target_link_libraries(mbed-weact-f411ce INTERFACE mbed-stm32f4)
