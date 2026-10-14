#!/bin/sh
# Flash the pre-built Calypso image to an ESP32 over USB, then open the serial monitor.
set -e
cd "$(dirname "$0")"
espflash write-bin --chip esp32 0x0 calypso.bin
espflash monitor
