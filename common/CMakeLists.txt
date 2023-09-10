cmake_minimum_required(VERSION 3.16.0)

add_library(zhelpers STATIC zhelpers.h)
add_library(cJSON STATIC cJSON.h cJSON.c)
add_library(utils STATIC utils.h utils.c)
# Cmake moment
set_target_properties(zhelpers PROPERTIES LINKER_LANGUAGE C)
