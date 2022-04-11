#!/bin/sh

docker build src/scripts/linux
docker run -i --user $(id -u):$(id -g) -v $PWD:/src/ $(docker build -q src/scripts/linux/) "./src/scripts/linux/compile.sh"
