#!/bin/bash

if [ $# -eq 0 ]; then
  echo "No installation path provided for ACADOS. It will be installed in $HOME/acados by default if it doesn't exist."
  acados_installation_path=$HOME/acados
elif [ $# -eq 1 ]; then 
    acados_installation_path=$1 
fi

# Check if ACADOS is already installed
if [ ! -d "$HOME/acados" ] || [ ! -d "$acados_installation_path" ]; then
    echo "ACADOS not found. Installing ACADOS..."
    
    git clone https://github.com/acados/acados.git $acados_installation_path
    cd $acados_installation_path
    git submodule update --recursive --init

    # Build ACADOS
    mkdir -p build
    cd build
    cmake -DACADOS_WITH_QPOASES=OFF ..
    make install

    echo "ACADOS installation completed."
else
    echo "ACADOS is already installed."
fi