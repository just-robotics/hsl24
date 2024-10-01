#!/bin/bash


name=$(cat ./name.txt)

if [[ $name == "" ]]
then
    echo "container name in name.txt is empty"
    exit 1
fi

docker stop $name > /dev/null
