#!/bin/bash

leftCount=$(ls ./images/stereoLeft/ | wc -l)
rightCount=$(ls ./images/stereoRight/ | wc -l)

echo $leftCount
echo $rightCount

cp /tmp/0001.png ./images/stereoLeft/imageL${leftCount}.png
cp /tmp/0002.png ./images/stereoRight/imageR${rightCount}.png
