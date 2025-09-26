cmake_minimum_required(VERSION 3.16)

# Tell ESP-IDF to treat sys/ as the main component
set(MAIN_COMPONENT_DIR "sys")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_TCD1304)
