#!/usr/bin/bash

# set your Arduino user path here
ARDUINO_USER_DIR=$HOME/Arduino

TARGET_DIR=$ARDUINO_USER_DIR/libraries/Interstitial_Quadrant
if [[ $* == *-f* ]]; then
    rm -r $TARGET_DIR 2> /dev/null
fi
mkdir $TARGET_DIR
if [[ $? == 1 ]]; then
    echo "To overwrite existing library, use $0 -f"
    exit
fi
cp -r ../src ../examples ../LICENSE ../README.md ../library.properties ../keywords.txt $TARGET_DIR
