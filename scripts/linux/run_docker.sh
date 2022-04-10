docker build scripts/linux
docker run -it -v $PWD:/src/ $(docker build -q scripts/linux/) /bin/bash
