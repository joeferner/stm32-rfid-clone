#!/bin/bash -e

rm -rf build
mkdir -p build
cd build
cmake -DTOOLCHAIN_PREFIX=/opt/arm-linaro-eabi-4.6 -DCMAKE_TOOLCHAIN_FILE=../gcc_stm32.cmake -DCMAKE_INSTALL_PREFIX=/opt/arm-linaro-eabi-4.6/ -DCMAKE_BUILD_TYPE=Release -DSTM32_CHIP_TYPE=LD ../
