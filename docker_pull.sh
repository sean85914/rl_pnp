# Get Cuda version
if [ `echo $LD_LIBRARY_PATH | grep -c "cuda-9.*" ` -gt 0 ]
then
  echo -e "\033[1;35mFind Cuda version 9.0\033[0m"
  CUDA_TAG="cuda9.0"
elif [ `echo $LD_LIBRARY_PATH | grep -c "cuda-10.*" ` -gt 0 ]
then
  echo -e "\033[1;35mFind Cuda version 10.0\033[0m"
  CUDA_TAG="cuda10.0"
fi

# Check if $CUDA_TAG set
if [ -z "$CUDA_TAG" ]
then
  echo "\033[1;31mInvalid Cuda version (we only provided with version 9.* and 10.*) or no Cuda Installed\033[0m"
  return
fi

docker pull sean85914/ddqn_bin_picking:$CUDA_TAG
