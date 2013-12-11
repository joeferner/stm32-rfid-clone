#!/bin/bash

kicad-update -i stm32-48pin-devBoard.mod.list -o stm32-48pin-devBoard.mod --basedir ./kicad-library
kicad-update -i stm32-48pin-devBoard.lib.list -o stm32-48pin-devBoard.lib --basedir ./kicad-library
