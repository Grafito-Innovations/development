# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/abel/Desktop/hello_world/components/micro_ros_espidf_component"
  "/home/abel/Desktop/hello_world/components/micro_ros_espidf_component"
  "/home/abel/Desktop/hello_world/build/libmicroros-prefix"
  "/home/abel/Desktop/hello_world/build/libmicroros-prefix/tmp"
  "/home/abel/Desktop/hello_world/build/libmicroros-prefix/src/libmicroros_project-stamp"
  "/home/abel/Desktop/hello_world/build/libmicroros-prefix/src"
  "/home/abel/Desktop/hello_world/build/libmicroros-prefix/src/libmicroros_project-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/abel/Desktop/hello_world/build/libmicroros-prefix/src/libmicroros_project-stamp/${subDir}")
endforeach()
