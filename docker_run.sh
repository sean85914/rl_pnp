#!/bin/bash
CONTAINER_NAME="ddqn_bin_picking"

# Check if given input arguments
if [ $# -eq 0 ]
then
  echo -e "\033[1;31mNo input arguments given, exiting...\033[0m"
  return
fi

xhost +local:docker

if [ "$1" == "same" ]
then
  docker exec -it --privileged ${CONTAINER_NAME} bash
  exit
fi

# Check if valid cuda version
if [ `echo $LD_LIBRARY_PATH | grep -c "cuda-9.*" ` -gt 0 ]
then
  echo -e "\033[1;35mFind Cuda version 9\033[0m"
  CUDA_TAG="cuda9.0"
elif [ `echo $LD_LIBRARY_PATH | grep -c "cuda-10.*" ` -gt 0 ]
then
  echo -e "\033[1;35mFind Cuda version 10\033[0m"
  CUDA_TAG="cuda10.0"
fi

# Check if $CUDA_TAG set
if [ -z "$CUDA_TAG" ]
then
  echo -e "\033[1;31mInvalid Cuda version (we only provided with version 9.* and 10.*) or no Cuda Installed\033[0m"
  return
fi

if [[ "$1" == "run" ]]
then
  echo -e "\033[1;33mOpening container with name ${CONTAINER_NAME}\033[0m"
  docker run --name ${CONTAINER_NAME} --rm -it --net=host --privileged -v /dev:/dev \
  --env="DISPLAY"="$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /etc/localtime:/etc/localtime:ro \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/.X11-unix/:/tmp/.X11-unix \
  -v $PWD/:/home/$USER/rl_pnp \
  --env "XAUTHORITY=$XAUTH" \
  -v "XAUTHORITY=$XAUTH" \
  --runtime=nvidia \
  --device=/dev/dri:/dev/dri \
  --device=/dev/nvhost-gpu \
  --device=/dev/nvhost-as-gpu \
  --device=/dev/nvhost-ctrl \
  --device=/dev/nvhost-ctrl-gpu \
  --device=/dev/nvhost-prof-gpu \
  --device=/dev/nvmap \
  --device=/dev/ttyUSB* \
  --device=/dev/ttyACM* \
  -v /dev/bus/usb:/dev/bus/usb \
  -w /home/$USER/rl_pnp \
  sean85914/ddqn_bin_picking:${CUDA_TAG} bash
fi
