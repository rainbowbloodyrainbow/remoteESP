cmake_minimum_required(VERSION 3.16)

# Pin the default target without overriding an explicit idf.py set-target.
set(IDF_TARGET "esp32" CACHE STRING "Classic ESP32-D0WD")
# The application owns STA reconnects; do not compile unused example netifs.
set(BUILD_WLAN_INTERFACE OFF CACHE BOOL "Use application STA implementation" FORCE)
set(BUILD_ETHERNET_INTERFACE OFF CACHE BOOL "WLAN chassis only" FORCE)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(remoteESP)

if(NOT IDF_TARGET STREQUAL "esp32")
    message(FATAL_ERROR "remoteESP pin map supports classic ESP32 only")
endif()
if(IDF_VERSION_MAJOR LESS 5 OR
   (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR LESS 2))
    message(FATAL_ERROR "Use ESP-IDF >= 5.2; this project was inspected against v5.2.7")
endif()
