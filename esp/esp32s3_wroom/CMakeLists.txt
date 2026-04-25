cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_master)

spiffs_create_partition_image(www ${CMAKE_CURRENT_SOURCE_DIR}/www FLASH_IN_PROJECT)
