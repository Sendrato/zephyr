#
# Copyright (c) 2021, Vincent van der Locht
# Copyright (c) 2022, Hessel van der Molen, sendrato.com
#
# SPDX-License-Identifier: Apache-2.0
#

# Suppress "simple_bus_reg" on QN9090 boards as all GPIO ports use the same register.
list(APPEND EXTRA_DTC_FLAGS "-Wno-simple_bus_reg")
list(APPEND EXTRA_DTC_FLAGS "-Wno-spi_bus_bridge")
