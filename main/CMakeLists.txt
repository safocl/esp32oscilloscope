# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(SRCS main.cpp adc.cpp esrMeter.cpp PRIV_REQUIRES esp_wifi esp_netif nvs_flash esp_adc esp_driver_gpio esp_driver_uart esp_driver_dac)
# idf_component_register(SRCS ${app_sources} )
