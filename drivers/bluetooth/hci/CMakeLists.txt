# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_BT_B91         hci_b91.c)
zephyr_library_sources_ifdef(CONFIG_BT_CYW43XXX    cyw43xxx.c)
zephyr_library_sources_ifdef(CONFIG_BT_ESP32       hci_esp32.c)
zephyr_library_sources_ifdef(CONFIG_BT_H4          h4.c)
zephyr_library_sources_ifdef(CONFIG_BT_H5          h5.c)
zephyr_library_sources_ifdef(CONFIG_BT_RPMSG       rpmsg.c)
zephyr_library_sources_ifdef(CONFIG_BT_SPI         spi.c)
zephyr_library_sources_ifdef(CONFIG_BT_STM32_IPM   ipm_stm32wb.c)
zephyr_library_sources_ifdef(CONFIG_BT_USERCHAN    userchan.c)
zephyr_library_sources_ifdef(CONFIG_BT_SILABS_HCI  slz_hci.c)
zephyr_library_sources_ifdef(CONFIG_BT_PSOC6_BLESS hci_psoc6_bless.c)
zephyr_library_sources_ifdef(CONFIG_SOC_NRF5340_CPUAPP nrf53_support.c)
zephyr_library_sources_ifdef(CONFIG_BT_NXP_IPM     ipm_nxp.c)

if ((CONFIG_CODE_DATA_RELOCATION) AND (CONFIG_SOC_QN9090))
    zephyr_code_relocate(FILES ipm_nxp.c LOCATION SRAM1_BSS)
endif()
