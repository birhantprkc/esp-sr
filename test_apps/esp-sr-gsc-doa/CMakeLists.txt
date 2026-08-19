cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET esp32p4)

set(EXTRA_COMPONENT_DIRS "../../../esp-sr")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_sr_gsc_doa_test)
