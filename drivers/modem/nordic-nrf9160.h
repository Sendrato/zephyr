/*
 * Copyright (c) 2023 Sendrato
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MODEM_NRF9160_H_
#define MODEM_NRF9160_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/modem/chat.h>
#include <zephyr/modem/pipe.h>
#include <zephyr/modem/backend/uart.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/offloaded_netdev.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include "modem_socket.h"

/* Factory almanac generated on 2024-08-01 08:33:13.
*
* Note, that the almanac gets more inaccurate with time and it should be updated periodically.
 */

#define FACTORY_ALMANAC_DATA_V2 \
       "f0ea020031150900000000000000000000000000000000000000000000000000" \
       "00000000000000000000317a837b1509251049fd002d0da10041e32000d609d1" \
       "ff54ae4c006afe020031e92e7b1509da1b4cfd001f0da100a22f4f000c502e00" \
       "f0c1bbfffc01040031e6177b15093e0f6cfd00710da1005c1b7b00567a86ff49" \
       "bd3700b501020031d2307b150952133ffd00d30ca100bb0e4d00c6fb340070c7" \
       "520041ff00003124187b1509c21e54fd00ff0ca1001af42400c664e2ff3b2af6" \
       "ff8100f9ff3117997b150913054dfd007c0ca1004357a4ff982baaff8c4cd1ff" \
       "c3ff020031184e7b15097a065efd00f40ca100663cf8ff72fa0e00d62e45003b" \
       "0104003117187b1509180b65fd00a40ca1005fa97800da225000033d59003901" \
       "0400319d4d7b15099f1b4bfd00220da100f8144f00c9179fffd07d95ffa9fffd" \
       "ff31920c7b15097a0f40fd009a0da100623d2600b1a696ff9881270012fdfeff" \
       "31f3467b1509760c55fd00710ca100cfd1d1ff107b3b00ad71b3ffd7fdffff31" \
       "e6447b1509751371fd00ef0ca100a9757f00045b2600f1853900bd020100311d" \
       "277b1509950142fd00110da1001bf7cfff00478bffcc1eb0ffe601020031b082" \
       "7b15090dfc4dfd001e0da100e3617300ab5a360074861d00c50001003165717b" \
       "15091b0c54fd00500ca1006b8bd2ff9f3b22009e716b001aff030031116e7b15" \
       "09cb106efd00db0da100c222fcffb280caff94234300b802feff317d247b1509" \
       "e21447fd00760ca1009a142500cf6784ff137bf5ff5efdffff31f5517b150948" \
       "106efd00780da10055f0fdff4e686a0085b892ff20020100314a1e7b1509d207" \
       "2bfd00270ca10024c24700080897ff1d8b0500860100003118cf7b1509670c49" \
       "fd00f50ca1001d9420008501eaff63e13d006e00ffff3153747b1509ac0b52fd" \
       "00150da100cfadd2ffbc08d2ff588a5900cfffffff3195247b1509711845fd00" \
       "6d0ca1001fe34d00182d87ffad65c4ff2e01020031c97f7b150984fa46fd0023" \
       "0da100cf56a0ffac722800d72df9ff03fe000031565f7b15099b0446fd00860d" \
       "a100a442ceff3b742c001c03b0ff0b02000031c1497b15090ef831fd00d90ca1" \
       "0089a9cbffbf2b1600c13691ff7200feff31b7667b15095f0c67fd001e0da100" \
       "5256f9ff7e862000f1494700deff0000318f027b1509520c58fd00120da10051" \
       "06a3ffa97e3b00abbda1ff8afefcff31ce187b150965126dfd00250ca100d9c0" \
       "fcff178e6b00e1b4430099fd010031da3a7b150964fb3efd00500ca1000153a4" \
       "ffb3599cffc910cbff94fe02003143567b1509a40753fd009f0da100a33aa5ff" \
       "a5941c00d69daaff13ff000031d93f7b15094c0d68fd00550da1008c44790097" \
       "27a9ff1311620082fd0100000000000000000000000000000000000000000000" \
       "0000000000"
#define FACTORY_ALMANAC_CHECKSUM_V2 \
       "dc6e1ee50b4f0cec12f6ee1c039de4926accb054dd74e150db0e622638830b4d"

#define MDM_INIT_SCRIPT_TIMEOUT_SECONDS  10
#define MDM_DYNAMIC_SCRIPT_TIMEOUT_SEC   5
#define MDM_RECV_DATA_SCRIPT_TIMEOUT_SEC 2
#define MDM_SCRIPT_DONE_TIMEOUT_SEC      (MDM_DYNAMIC_SCRIPT_TIMEOUT_SEC + 2)

#define MDM_SENDMSG_SLEEP         K_MSEC(1)
#define MDM_RECV_DATA_TIMEOUT_SEC 1
#define MDM_INIT_TIMEOUT_SEC      (MDM_INIT_SCRIPT_TIMEOUT_SECONDS + 2)
#define MDM_RESET_TIMEOUT_SEC     10

#define MDM_REQUEST_SCHED_DELAY_MSEC    500
#define MDM_REQUEST_WAIT_EXEC_SEM_MSEC  10
#define MDM_REQUEST_DISPATCH_DELAY_MSEC 10

#define MDM_MAC_ADDR_LENGTH 6
#define MDM_MAX_DATA_LENGTH 1024
#define MDM_MAX_SOCKETS     3
#define MDM_BASE_SOCKET_NUM 0

/* Default lengths of modem info */
#define MDM_IMEI_LENGTH         15
#define MDM_MANUFACTURER_LENGTH 30
#define MDM_MODEL_LENGTH        24
#define MDM_REVISION_LENGTH     64

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
#define MDM_SETUP_CMD_ALMANAC_DATA                                                                 \
	"AT%XFILEWRITE=1,\"" FACTORY_ALMANAC_DATA_V2 "\",\"" FACTORY_ALMANAC_CHECKSUM_V2 "\""

/* Default SLM data mode terminator command */
#define MDM_DATA_MODE_TERMINATOR "!~>&}@%"

/* Modem ATOI routine. */
#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)
/* Modem ATOL routine. */
#define ATOL(s_, desc_, res_)   modem_atol(s_, desc_, res_, __func__)

enum modem_event {
	MODEM_EVENT_RESUME = 0,
	MODEM_EVENT_SUSPEND,
	MODEM_EVENT_SCRIPT_SUCCESS,
	MODEM_EVENT_SCRIPT_FAILED,
	MODEM_EVENT_BUS_OPENED,
	MODEM_EVENT_BUS_CLOSED,
};

enum modem_request {
	MODEM_REQ_RESET,
	/* NetIF related requests */
	MODEM_REQ_IFACE_ENABLE,
	MODEM_REQ_IFACE_DISABLE,
	/* GNSS related requests */
	MODEM_REQ_GNSS_RESUME,
	MODEM_REQ_GNSS_SUSPEND,
	/* Sockets related requests */
	MODEM_REQ_OPEN_SOCK,
	MODEM_REQ_CLOSE_SOCK,
	MODEM_REQ_CONNECT_SOCK,
	MODEM_REQ_DATA_MODE,
	MODEM_REQ_SEND_DATA,
	MODEM_REQ_RECV_DATA,
	MODEM_REQ_SELECT_SOCK,
	MODEM_REQ_GET_ACTIVE_SOCK,
	/* DNS related requests */
	MODEM_REQ_GET_ADDRINFO,
};

enum modem_state {
	MODEM_STATE_IDLE = 0,
	MODEM_STATE_INIT,
	MODEM_STATE_READY,
};

struct net_if_data {
	const struct device *modem_dev;
};

struct offload_if {
	struct net_if *net_iface;
	uint8_t mac_addr[MDM_MAC_ADDR_LENGTH];
};

#define OFFLOADED_NETDEV_L2_CTX_TYPE struct offload_if

struct open_sock_t {
	int family;
	int type;
};

struct connect_sock_t {
	char ip_str[NET_IPV6_ADDR_LEN];
	uint16_t dst_port;
};

struct socket_send_t {
	struct modem_socket *sock;
	const struct sockaddr *dst_addr;
	uint8_t *buf;
	size_t len;
	int sent;
};

struct get_addrinfo_t {
	const char *node;
};

struct recv_sock_t {
	struct modem_socket *sock;
	int flags;
	/* Amount of bytes received */
	uint16_t nbytes;
};

struct select_sock_t {
	int sock_fd;
};

struct modem_data {
	/* Child node net_if */
	struct offload_if iface;
	/* Child node gnss device */
	const struct device *gnss_dev;

	/* UART backend */
	struct modem_pipe *uart_pipe;
	struct modem_backend_uart uart_backend;
	uint8_t uart_backend_receive_buf[CONFIG_MODEM_NORDIC_NRF9160_UART_RX_BUF_SIZE];
	uint8_t uart_backend_transmit_buf[CONFIG_MODEM_NORDIC_NRF9160_UART_TX_BUF_SIZE];

	/* Modem chat */
	struct modem_chat chat;
	uint8_t chat_receive_buf[128];
	uint8_t chat_delimiter[2];
	uint8_t *chat_argv[32];

	/* Modem info */
	uint8_t imei[MDM_IMEI_LENGTH];
	uint8_t manufacturer[MDM_MANUFACTURER_LENGTH];
	uint8_t model[MDM_MODEL_LENGTH];
	uint8_t revision[MDM_REVISION_LENGTH];

	/* Device node */
	const struct device *dev;
	enum modem_state state;
	bool connected;

	/* Event dispatcher */
	struct k_work event_dispatch_work;
	uint8_t event_buf[8];
	struct ring_buf event_rb;
	struct k_mutex event_rb_lock;

	/* Request dispatcher */
	struct k_work_delayable request_dispatch_work;
	uint8_t request_buf[8];
	struct ring_buf request_rb;
	struct k_mutex request_rb_lock;

	/* Dynamic chat script */
	uint8_t dynamic_match_buf[32];
	uint8_t dynamic_separators_buf[2];
	uint8_t dynamic_request_buf[64];
	struct modem_chat_match dynamic_match;
	struct modem_chat_script_chat dynamic_script_chat;
	struct modem_chat_script dynamic_script;
	int dynamic_script_res;

	/* Socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];
	/* Active socket fd */
	int sock_fd;

	/* State semaphore */
	struct k_sem sem_state;
	/* Script exec semaphore */
	struct k_sem sem_script_exec;
	/* Script done semaphore */
	struct k_sem sem_script_done;
	/* Script sync semaphore */
	struct k_sem sem_script_sync;

	/* GNSS data */
	uint16_t gnss_interval;
	uint16_t gnss_timeout;

	/* Structs to offload socket operations */
	struct open_sock_t open_sock;
	struct connect_sock_t connect_sock;
	struct recv_sock_t recv_sock;
	struct socket_send_t send_sock;
	struct select_sock_t select_sock;
	/* Structs to offload DNS operations */
	struct get_addrinfo_t get_addrinfo;
};

struct modem_config {
	const struct device *uart;
	const struct gpio_dt_spec power_gpio;
	const struct gpio_dt_spec reset_gpio;
	const struct modem_chat_script *init_chat_script;
	/* Offload DNS ops */
	const struct socket_dns_offload dns_ops;
	/* Socket create API */
	net_socket_create_t sock_create;
};

#endif /* MODEM_NRF9160_H_ */
