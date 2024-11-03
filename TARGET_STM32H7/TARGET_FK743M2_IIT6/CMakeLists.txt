# Copyright (c) 2024 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-fk743m2-iit6 INTERFACE)

target_sources(mbed-fk743m2-iit6
    INTERFACE
        PeripheralPins.c
)

target_include_directories(mbed-fk743m2-iit6
    INTERFACE
        .
)

target_link_libraries(mbed-fk743m2-iit6 INTERFACE mbed-stm32h743xi)
