# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/abel/esp/esp-idf/components/bootloader/subproject"
  "/home/abel/Desktop/hello_world/build/bootloader"
  "/home/abel/Desktop/hello_world/build/bootloader-prefix"
  "/home/abel/Desktop/hello_world/build/bootloader-prefix/tmp"
  "/home/abel/Desktop/hello_world/build/bootloader-prefix/src/bootloader-stamp"
  "/home/abel/Desktop/hello_world/build/bootloader-prefix/src"
  "/home/abel/Desktop/hello_world/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/abel/Desktop/hello_world/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
