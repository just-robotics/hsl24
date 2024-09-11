ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=${PWD}

cd ${ROOT_DIR}

#xhost +local:docker > /dev/null || true

SOURCE_IMG_NAME="nickodema/hsl_kobuki:galactic-20.04-20102023"
GPU_FLAG=""

### Docker build -------------------------------------------------------- #

container_name="hsl24_kobuki_sim"
echo $container_name > $EXEC_PATH/name.txt

docker build -t $container_name -f ${ROOT_DIR}/docker/Dockerfile ${ROOT_DIR} \
                            --network=host  \
                            --build-arg from=$SOURCE_IMG_NAME

cd ${EXEC_PATH}
