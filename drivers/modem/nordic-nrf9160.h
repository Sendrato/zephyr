/*
 * Copyright (c) Sendrato B.V. 2023
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef NORDIC_NRF9160_H
#define NORDIC_NRF9160_H

#include <zephyr/kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/init.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/offloaded_netdev.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>
#include <zephyr/drivers/modem/nordic-nrf9160.h>

#include "modem_context.h"
#include "modem_socket.h"
#include "modem_cmd_handler.h"
#include "modem_iface_uart.h"

#define MDM_UART_NODE             DT_INST_BUS(0)
#define MDM_UART_DEV              DEVICE_DT_GET(MDM_UART_NODE)
#define MDM_AWAKE_CMD_TIMEOUT     K_SECONDS(1)
#define MDM_CMD_TIMEOUT           K_SECONDS(5)
#define MDM_CMD_CONN_TIMEOUT      K_SECONDS(120)
#define MDM_REGISTRATION_TIMEOUT  K_SECONDS(180)
#define MDM_SENDMSG_SLEEP         K_MSEC(1)
#define MDM_RECV_DATA_TIMEOUT_SEC 1

#define MDM_MAC_ADDR_LENGTH     6
#define MDM_MAX_DATA_LENGTH     1024
#define MDM_RECV_MAX_BUF        5
#define MDM_RECV_BUF_SIZE       512
#define MDM_MAX_SOCKETS         3
#define MDM_BASE_SOCKET_NUM     0
#define MDM_INIT_RETRY_COUNT    5
#define MDM_AWAKE_RETRY_COUNT   5
#define MDM_AWAKE_SLEEP_TIME_MS K_MSEC(100)
#define BUF_ALLOC_TIMEOUT       K_SECONDS(1)
#define MDM_DNS_TIMEOUT         K_SECONDS(210)

/* Default lengths of certain things. */
#define MDM_MANUFACTURER_LENGTH 30
#define MDM_MODEL_LENGTH        16
#define MDM_REVISION_LENGTH     64
#define MDM_IMEI_LENGTH         16

/* Setup AT commands */
/* System mode */
#if IS_ENABLED(CONFIG_MODEM_NRF9160_MODE_LTE_ONLY)
#define MDM_SETUP_CMD_SYSTEM_MODE "AT%XSYSTEMMODE=1,0,1,1"
#elif IS_ENABLED(CONFIG_MODEM_NRF9160_MODE_DUAL)
#define MDM_SETUP_CMD_SYSTEM_MODE "AT%XSYSTEMMODE=1,1,1,0"
#elif IS_ENABLED(CONFIG_MODEM_NRF9160_MODE_DUAL_LTE_PREF)
#define MDM_SETUP_CMD_SYSTEM_MODE "AT%XSYSTEMMODE=1,1,1,1"
#endif
/* PDP context */
#define MDM_SETUP_CMD_PDP_CTX "AT+CGDCONT=0,\"IP\",\"" CONFIG_MODEM_NRF9160_APN "\""

/* Default SLM data mode terminator command */
#define MDM_DATA_MODE_TERMINATOR "!~>&}@%"

/* Modem ATOI routine. */
#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)

enum nrf9160_state {
	NRF9160_STATE_UNKNOWN,
	NRF9160_STATE_INIT,
	NRF9160_STATE_REGISTERING,
	NRF9160_STATE_REGISTERED,
	NRF9160_STATE_NOT_REGISTERED,
	NRF9160_STATE_OFF,
};

struct modem_data {
	struct net_if *net_iface;
	uint8_t mac_addr[MDM_MAC_ADDR_LENGTH];

	/* modem interface */
	struct modem_iface_uart_data iface_data;
	uint8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];

	/* modem cmds */
	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_match_buf[MDM_RECV_BUF_SIZE + 1];

	/* socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	/* Semaphore(s) */
	struct k_sem sem_response;
	struct k_sem sem_modem_state;
	struct k_sem sem_sock;
	struct k_sem sem_rx_ringbuf;

	/* modem data */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];

	/* Socket that is currently selected */
	int sock_fd;

	/* Modem state */
	enum nrf9160_state state;
};

/* Socket read callback data */
struct socket_read_data {
	char *recv_buf;
	size_t recv_buf_len;
	struct sockaddr *recv_addr;
	uint16_t recv_read_len;
};

#endif // NORDIC_NRF9160_H
