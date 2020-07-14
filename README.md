# custom_targets
Custom target definitions for Mbed5

Currently supported:
- STM32F407VE_BLACK
- BLUEPILL_F103RB
  same as BLUEPILL_F103C8, but uses 128 kB flash and newlib-nano for bare_metal profile
- WEACT_F411CE
  WeACT MiniF4 F411CE
- NODE_LPC812 and NODE_LPC824 
  from Mbed-os-5.15, targets were removed in Mbed6
  custom hardware derived from LPC812/824


For usage with mbed-cli, simply clone the repo into your project. The build system will use it automatically.

For usage with MbedStudio, clone this repo into your project and move custom_targets.json to the project root. This is neccessary for the target detection in the target list box.
