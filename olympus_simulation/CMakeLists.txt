cmake_minimum_required(VERSION 3.12)
project(vscode_drake_sim)

#This cmake lists is used to interface with vscode
add_subdirectory(olympus_drake)


# This file allows for the use of `CMake Tools` extension in vscode, as it allows building the simulation from the current directory.
# Correct  definition of launch procedures allows for debugging and running the simulation as well. 