#!/bin/bash

$HOME/.platformio/packages/tool-mkspiffs/mkspiffs_espressif32_arduino -c $PWD/emptyffs/ -s 0x20000 $PWD/.pio/build/saola/spiffsvars.bin
$HOME/.platformio/packages/tool-mkspiffs/mkspiffs_espressif32_arduino -c $PWD/data/ -s 0x20000 $PWD/.pio/build/saola/spiffs.bin

$HOME/.platformio/packages/tool-esptoolpy/esptool.py --port /dev/ttyUSB0 write_flash 0x3B0000 $PWD/.pio/build/saola/spiffs.bin
$HOME/.platformio/packages/tool-esptoolpy/esptool.py --port /dev/ttyUSB0 write_flash 0x3D0000 $PWD/.pio/build/saola/spiffsvars.bin