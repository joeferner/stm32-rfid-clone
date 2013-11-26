#!/bin/bash -e

mkdir -p build
cd build
rm stm32-ft312d-test.elf || echo "Cannot remove. stm32-ft312d-test.elf not build?"
make stm32-ft312d-test.bin && \
make stm32-ft312d-test.list
