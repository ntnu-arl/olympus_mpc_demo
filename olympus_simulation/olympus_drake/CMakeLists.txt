# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.10)
# set(CMAKE_FIND_DEBUG_MODE ON) //to enable debugger is commented out

#Ros required PIC 
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 

# make cache variables for install destinations
include(GNUInstallDirs)

enable_language(CXX) #for find threads -> dependency of drake

find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(Eigen3 REQUIRED PATHS /usr/lib/)
find_package(yaml-cpp REQUIRED)

# Set the project name and version
project(olympus_drake VERSION 1.0)

# Targets:
# 1. Add the library target
add_library(drake_helpers           STATIC   src/drake_helpers.cpp) #probably a library
add_library(olympus_drake           STATIC   src/olympus_drake.cpp) #probably a library

# 2. Add the executable target
add_executable(olympus_drake_sim              src/olympus_drake_sim.cpp) #probably a library

# Linking:
# Add dependencies
add_dependencies(olympus_drake drake_helpers)
add_dependencies(olympus_drake_sim olympus_drake)

# add header files -> PUBLIC to be available for all targets that link againts olympus_drake
target_include_directories(olympus_drake_sim           PRIVATE include) 
target_include_directories(olympus_drake               PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>" )
target_include_directories(drake_helpers               PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>" ) 

# Link the target with the necessary libraries
target_link_libraries(drake_helpers drake::drake)
target_link_libraries(olympus_drake drake::drake drake_helpers yaml-cpp)
target_link_libraries(olympus_drake_sim olympus_drake drake::drake)

##-----------------
## INSTALL LIBRARY:
##-----------------

#1.  Install static archives and define export target set
install(TARGETS olympus_drake drake_helpers
        EXPORT olympus_drakeTargets 
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})


#2. Install the include directory -> Makes the public headers available to users of the library.
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#3. Exports the target definitions to a file -> makes the **exported** targets known when using cmake
#This file is the `olympus_drakeTargets.cmake`. CMAKE CANNOT FIND THE EXPORTED TARGETS -> used to import package
install(EXPORT olympus_drakeTargets
        FILE olympus_drakeTargets.cmake
        NAMESPACE olympus_drake::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/olympus_drake)

#4a. Create config file = Allows other projects to use find_package to locate and use `olympus_drake`.
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/olympus_drakeConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/olympus_drake)

#4b. Install the config file 
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/olympus_drakeConfig.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/olympus_drake
)