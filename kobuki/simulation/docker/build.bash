ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=${PWD}

cd ${ROOT_DIR}

#xhost +local:docker > /dev/null || true

SOURCE_IMG_NAME="osrf/ros:humble-desktop-full"
GPU_FLAG=""

### Check if NVIDIA GPU flag is needed ----------------------------------- #

# if [ -n "$(which nvidia-smi)" ] && [ -n "$(nvidia-smi)" ]; then
#     GPU_FLAG=(--gpus all)
#     SOURCE_IMG_NAME="${SOURCE_IMG_NAME}:nvidia"
# else
#     SOURCE_IMG_NAME="${SOURCE_IMG_NAME}:general"
# fi

### Docker build -------------------------------------------------------- #

container_name="hsl24_kobuki_sim"
echo $container_name > $EXEC_PATH/name.txt

docker build -t $container_name -f ${ROOT_DIR}/docker/Dockerfile ${ROOT_DIR} \
                            --network=host  \
                            --build-arg from=$SOURCE_IMG_NAME

cd ${EXEC_PATH}
