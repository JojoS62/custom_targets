# Copyright (c) 2024 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

add_library(mbed-fk743m5-xih6 INTERFACE)

target_sources(mbed-fk743m5-xih6
    INTERFACE
        PeripheralPins.c
        fmc.cpp
)

target_include_directories(mbed-fk743m5-xih6
    INTERFACE
        .
)

target_link_libraries(mbed-fk743m5-xih6 INTERFACE mbed-stm32h743xi)
