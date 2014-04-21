#!/bin/bash

cd tests
mkdir -p build
gcc -DTEST -o build/test ../em4x05.c ../ring_buffer.c CuTest.c tests.c em4x05_test.c ring_buffer_test.c && ./build/test

