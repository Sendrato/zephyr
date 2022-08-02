# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_BT_ESP32       hci_esp32.c)
zephyr_library_sources_ifdef(CONFIG_BT_H4          h4.c)
zephyr_library_sources_ifdef(CONFIG_BT_H5          h5.c)
zephyr_library_sources_ifdef(CONFIG_BT_RPMSG       rpmsg.c)
zephyr_library_sources_ifdef(CONFIG_BT_SPI         spi.c)
zephyr_library_sources_ifdef(CONFIG_BT_STM32_IPM   ipm_stm32wb.c)
zephyr_library_sources_ifdef(CONFIG_BT_NXP_IPM     ipm_nxp.c)
zephyr_library_sources_ifdef(CONFIG_BT_USERCHAN    userchan.c)

if ((CONFIG_CODE_DATA_RELOCATION) AND ("${MCUX_DEVICE}" STREQUAL "QN9090"))
    zephyr_code_relocate(ipm_nxp.c SRAM1_BSS)
endif()
