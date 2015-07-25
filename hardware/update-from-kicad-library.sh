#!/bin/bash

echo "Sync'ing mods"
for f in mods/*
do
  FILENAME=$(basename $f)
  KICADLIB_FILENAME=../../kicad-library/mods/${FILENAME}
  LOCAL_FILENAME=./mods/${FILENAME}
  if [ -f ${KICADLIB_FILENAME} ]; then
    cp ${KICADLIB_FILENAME} ${LOCAL_FILENAME}
  else
    echo "Could not find ${KICADLIB_FILENAME}"
  fi
done

kicad-update -i motor-controller.lib.list -o motor-controller.lib --basedir ../../kicad-library
