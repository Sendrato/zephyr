

cmake_minimum_required(VERSION 3.13.1)

#include(../../../awareable.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(pm)

target_sources(app PRIVATE src/main.c)
