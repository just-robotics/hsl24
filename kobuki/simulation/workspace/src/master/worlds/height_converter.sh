#!/bin/bash

if [[ $# -eq 0 ]]
then
    echo "Error: height required"
    exit 1
fi

height=$1
pose=$(bc<<<"scale=3;$height/2")
pose="0$pose"
y=0.045

echo "height = $height"
echo "pose = $pose"
echo "y = $y"

cp maze.world.template maze.world
sed -i -r "s/###/$height/g" maze.world
sed -i -r "s/&&&/$pose/g" maze.world
sed -i -r "s/&#&/$y/g" maze.world
