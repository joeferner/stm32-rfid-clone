#!/bin/bash

kicad-split --yes -i stm32-48pin-devBoard.mod -o ./kicad-library/mods/
kicad-split --yes -i stm32-48pin-devBoard.lib -o ./kicad-library/libs/
