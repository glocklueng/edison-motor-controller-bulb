#!/bin/bash

cp mods/* ../../kicad-library/mods/
kicad-split --yes -i motor-controller.lib -o ../../kicad-library/libs/
