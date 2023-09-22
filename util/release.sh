#!/usr/bin/bash

if [ ! -z "$(git status --porcelain)" ]; then 
    echo "You have un-committed changes. Please commit them and create a tag before releasing."
    exit
fi

if [[ $1 = "--tag" ]]; then
    tag=$2
else
    tag=$(git describe --exact-match --tags $(git log -n1 --pretty='%h'))
    if [[ $? != 0 ]]; then
        echo "You are not currently on a tagged commit. Please create a tag before releasing."
        exit
    fi
fi

TLD=Interstitial_Quadrant-"$tag"
mkdir $TLD
cp -r ../src ../examples ../LICENSE ../README.md ../library.properties $TLD

zip -r "$TLD".zip $TLD
rm -rf $TLD
echo "Created $TLD.zip"
