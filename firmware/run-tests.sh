#!/bin/bash

cd tests
mkdir -p build
gcc -DTEST -o build/test ../em4x05.c CuTest.c tests.c em4x05_test.c && ./build/test

