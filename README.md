freertos-avr-test
=============

A basic example of running FreeRTOS on an ATmega328p

I *highly* recommend using the MinSizeRel configuration in CMake, otherwise the binary is huge.

Building
--------
```
git clone https://github.com/teknoman117/freertos-avr-test
cd freertos-avr-test
# Fetch FreeRTOS kernel
git submodule update --init
cmake -GNinja -DCMAKE_BUILD_TYPE=MinSizeRel -B build .
cmake --build build
```

Upload
------
Uploading to an Arduino Uno R3
```
avrdude -v -p atmega328p -c arduino -P /dev/ttyACM0 -b 115200 -D -U flash:w:build/src/test.hex:i
```