#
# Copyright (c) 2021, Vincent van der Locht
# Copyright (c) 2022, Hessel van der Molen, sendrato.com
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources(soc.c)

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
  )

