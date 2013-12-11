#!/bin/bash

kicad-update -i hardware/stm32-rfid-clone.mod.list -o hardware/stm32-rfid-clone.mod --basedir ./kicad-library
kicad-update -i hardware/stm32-rfid-clone.lib.list -o hardware/stm32-rfid-clone.lib --basedir ./kicad-library
