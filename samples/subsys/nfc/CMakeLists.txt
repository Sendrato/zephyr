# Copyright (c) 2021, Sendrato.com, All rights reserved.
#
# This software is the confidential and proprietary information of Sendrato
# and is subject to the terms and conditions defined in file 'LICENSE.txt',
# which is part of this source code package. You shall not disclose such
# Confidential Information and shall use it only in accordance with the terms
# of the license agreement you entered into with Sendrato.
#
# 2021, Sendrato, Hessel van der Molen
#
# NFC: test application
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nfc)

target_sources(app PRIVATE src/main.c)

