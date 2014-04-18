#!/bin/bash -e

mkdir -p build
cd build
rm stm32-rfid-clone.elf || echo "Cannot remove. stm32-rfid-clone.elf not build?"
make stm32-rfid-clone.bin && \
make stm32-rfid-clone.list
