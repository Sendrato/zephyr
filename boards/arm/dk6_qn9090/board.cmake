#
# Copyright (c) 2021, Vincent van der Locht
# Copyright (c) 2022, Hessel van der Molen, sendrato.com
#
# SPDX-License-Identifier: Apache-2.0
#

board_runner_args(jlink "--device=QN9090T")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
