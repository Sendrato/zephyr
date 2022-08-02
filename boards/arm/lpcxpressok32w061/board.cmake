#
# Copyright (c) 2021, Vincent van der Locht
# SPDX-License-Identifier: Apache-2.0
#

#board_runner_args(jlink "--device=K32W061")
board_runner_args(jlink "--device=QN9090T")
#board_runner_args(pyocd "--target=k32w061")

#include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
