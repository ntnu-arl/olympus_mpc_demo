SCRIPT_PATH=$(dirname $(readlink -f "$0"))
echo $SCRIPT_PATH
cd $SCRIPT_PATH

#package root
cd ../../
sim_pkg_root=$(realpath .)


echo "This script assumes the ROS workspace itself is not inside a 'src' folder."
read -p "Is your workspace inside a 'src' folder? (y/n): " response

#go to ros workspace root
if [[ "$response" =~ ^[Yy]$ ]]; then

  while [ $(pwd | grep -c 'src') -gt 0 ]; do
    # Move up one directory
    cd ../
  done

  ws_root=$(realpath .)
  echo "ROS workspace: $ws_root"

  #build & install
  mkdir -p $sim_pkg_root/olympus_drake/build
  cd $sim_pkg_root/olympus_drake/build

  cmake -DCMAKE_INSTALL_PREFIX=$ws_root/install ..
  make
  make install
  cd $ws_root

fi
echo "Finishing installing library"
