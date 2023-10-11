#!/usr/bin/bash

# set your Arduino user path here
ARDUINO_USER_DIR=$HOME/Arduino

TARGET_DIR=$ARDUINO_USER_DIR/libraries/Interstitial_Quadrant
mkdir $TARGET_DIR
cp -r ../src ../examples ../LICENSE ../README.md ../library.properties $TARGET_DIR
