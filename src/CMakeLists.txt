# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

# FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp)

idf_component_register(SRCS main.cpp PRIV_REQUIRES esp_wifi esp_netif nvs_flash esp_adc)
# idf_component_register(SRCS ${app_sources} )
