#
# Copyright (c) 2021 metraTec GmbH
# Copyright 2024 NXP
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(soc.c)
zephyr_library_sources(power.c)


#While not required, including PWR_setjmp.S is to prevent warnings of software
#that needs to be kept because it could be of use later on.

zephyr_library_sources(PWR_setjmp.S)

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
  )

zephyr_include_directories(.)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
