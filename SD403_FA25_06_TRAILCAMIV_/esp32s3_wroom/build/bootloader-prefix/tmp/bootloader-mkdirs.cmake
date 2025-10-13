# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/cole-zenk/esp/esp-idf/components/bootloader/subproject"
  "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader"
  "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix"
  "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix/tmp"
  "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix/src/bootloader-stamp"
  "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix/src"
  "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/cole-zenk/esp/SD403_TrailCamera_NDSU/SD403_FA25_06_TRAILCAMIV_/esp32s3_wroom/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
