#!/bin/bash

kicad-split --yes -i hardware/stm32-rfid-clone.mod -o ./kicad-library/mods/
kicad-split --yes -i hardware/stm32-rfid-clone.lib -o ./kicad-library/libs/
