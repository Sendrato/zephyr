#
# Copyright (c) 2021, Vincent van der Locht
#
# SPDX-License-Identifier: Apache-2.0
#

add_subdirectory(${SOC_SERIES})

zephyr_include_directories_ifdef(CONFIG_SOC_FAMILY_K32 ${CMAKE_CURRENT_LIST_DIR})
zephyr_sources_ifdef(CONFIG_SOC_FAMILY_K32 board_utility.c)

# Sign binary
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands

        #Sign secure binary image with public key
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/dk6_image_tool.py
        -b1
        -i0
        -s 294912
        ${CMAKE_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}

        )