#!/bin/bash -e

rm -rf build
mkdir -p build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../gcc_stm32.cmake -DCMAKE_BUILD_TYPE=Release -DSTM32_CHIP_TYPE=LD ../

