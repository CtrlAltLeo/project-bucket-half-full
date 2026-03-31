#!/bin/bash

# This script copies the stero images created in Blender to the stereoLeft and stereoRight folders in this directory.

count=$( ls ./images/stereoLeft | wc -l)

mv /tmp/0001.png ./images/stereoRight/$count.png
mv /tmp/0002.png ./images/stereoLeft/$count.png
