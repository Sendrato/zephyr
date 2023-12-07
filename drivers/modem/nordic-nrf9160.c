/*
 * Copyright (c) Sendrato B.V. 2023
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nordic_nrf9160

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modem_nordic_nrf9160, CONFIG_MODEM_LOG_LEVEL);

#include "nordic-nrf9160.h"

static struct k_thread modem_rx_thread;
/* Modem data */
static struct modem_data mdata;
/* Modem context */
static struct modem_context mctx;
/* Static structs for offloaded APIs */
static const struct socket_op_vtable offload_socket_fd_op_vtable;
static const struct socket_dns_offload offload_dns_ops;

/* Static DNS buffers */
static struct zsock_addrinfo dns_result;
static struct sockaddr dns_result_addr;
static char dns_result_canonname[DNS_MAX_NAME_SIZE + 1];

static K_KERNEL_STACK_DEFINE(modem_rx_stack, CONFIG_MODEM_NORDIC_NRF9160_RX_STACK_SIZE);
NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE, 0, NULL);

/*
 * Modem RX ringbuffer
 * TODO: We'll need a ring buffer for each socket
 */
RING_BUF_DECLARE(rx_ringbuf, CONFIG_MODEM_NORDIC_NRF9160_RX_RINGBUF_SIZE);

#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios)
static const struct gpio_dt_spec power_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_power_gpios);
#endif
#if DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_INST_GET(0, mdm_reset_gpios);
#endif

static inline uint32_t hash32(char *str, int len)
{
#define HASH_MULTIPLIER 37

	uint32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static inline uint8_t *modem_get_mac(const struct device *dev)
{
	struct modem_data *data = dev->data;
	uint32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use IMEI for mac_addr */
	hash_value = hash32(mdata.mdm_imei, strlen(mdata.mdm_imei));

	UNALIGNED_PUT(hash_value, (uint32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

/*
 * Func: modem_atoi
 * Desc: Convert string to long integer handling errors
 */
static int modem_atoi(const char *s, const int err_value, const char *desc, const char *func)
{
	int ret;
	char *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", s, desc, func);
		return err_value;
	}

	return ret;
}

/* Function to convert a string containing floating point number to float */
static int str_to_float(char *str, float *res)
{
    int ret = 0;
    size_t len = strlen(str);
    int idx = 0;

    /* Look for the dot */
    for (; idx < len ; idx++) {
        if (str[idx] == '.') {
            break;
        }
    }
    /* Check if we actually found it, can't be the last char */
    if (idx == (len - 1)) {
        ret = -1;
        goto error;
    }

    /* Calculate number of  decimal digits */
    int dec_digits = (int)strlen(str) - (idx + 1);
    /* Override the dot */
    memcpy(&str[idx], &str[idx+1], dec_digits);
    /* Override last char with string terminator */
    str[len-1] = '\0';

    /* Cast string to integer*/
    int tmp = ATOI(str, -1, "tmp");
    if (tmp >= 0) {
        /*
         * No pow function, so calculate the divider
         * base on the number of decimal digits
         */
        float divider = 1;
        for (int i = 0; i<dec_digits ; i++) {
            divider *= 10;
        }
        /* Divide by calculated divider */
        *res = (float)tmp / divider;
    } else {
        ret = -1;
    }

error:
    return ret;
}

/*
 * Thread safe function to set modem state
 */
static void set_modem_state(enum nrf9160_state state)
{
	k_sem_take(&mdata.sem_modem_state, K_FOREVER);
	mdata.state = state;
	LOG_DBG("Entered state %d", state);
	k_sem_give(&mdata.sem_modem_state);
}

/*
 * Thread safe function to get modem state
 */
static enum nrf9160_state get_modem_state(void)
{
	enum nrf9160_state state;

	k_sem_take(&mdata.sem_modem_state, K_FOREVER);
	state = mdata.state;
	k_sem_give(&mdata.sem_modem_state);

	return state;
}

/*
 * Handler: Data received from socket.
 * After #XRECV (in a new line) and no specific pattern to find
 * Received data will be pushed to a ring buffer, this because
 * the Serial LTE Modem application doesn't allow to specify the
 * number of bytes to receive, but just returns all data available
 * in the socket.
 * The amount of data requested by the application will be popped
 * from the ring buffer instead.
 */
MODEM_CMD_DEFINE(on_cmd_datarecv)
{
	int ret = 0;
	int available = 0;
	uint8_t *ringbuf_ptr;
	uint32_t claimed_len = 0;
    size_t offset = 0;

	k_sem_take(&mdata.sem_rx_ringbuf, K_FOREVER);

	/* Retrieve available space in rx_ringbuf */
	available = (int)ring_buf_space_get(&rx_ringbuf);
	if (available < len) {
		LOG_ERR("Not enough space available in ring buf (%d < %d)", available, len);
		ret = -ENOMEM;
		goto error;
	}

    /*
     * It's possible that we can't claim all bytes at once
     * if we are close to the end of the ringbuf
     */
    while (len > 0) {
        /* Claim tbc bytes in rx_ringbuf */
        claimed_len = (int) ring_buf_put_claim(&rx_ringbuf, &ringbuf_ptr, len);
        if (claimed_len != len) {
            LOG_DBG("Couldn't claim enough bytes, %d instead of %d", claimed_len, len);
        }

        /* Update len to the number of bytes that we still need to claim */
        len -= claimed_len;

        /* Linearize received data and copy it to rx_ringbuf */
        ret = (int) net_buf_linearize(ringbuf_ptr, claimed_len, data->rx_buf, offset, (uint16_t) claimed_len);

        /* Update offset in case we couldn't claim bytes all at once */
        offset += claimed_len;

        /*
         * Finalize copying bytes to rx_ringbuf
         * ret at this point contains the number of bytes we actually copied
         */
        ret = ring_buf_put_finish(&rx_ringbuf, ret);
        if (ret) {
            LOG_ERR("Failed to copy all data to ringbuf");
            ret = -ENOMEM;
            goto error;
        }
    }

error:
	k_sem_give(&mdata.sem_rx_ringbuf);

	return ret;
}

/* Handler: #XRECV: <size>[0] */
MODEM_CMD_DEFINE(on_cmd_sockrecv)
{
	int ret = 0;
	int recv_len = 0;

	if (!len) {
		LOG_ERR("Invalid length, Aborting!");
		ret = -EAGAIN;
		goto error;
	}

	/* Make sure we still have buf data */
	if (!data->rx_buf) {
		LOG_ERR("Incorrect format! Ignoring data!");
		ret = -EINVAL;
		goto error;
	}

	recv_len = ATOI(argv[0], -1, "size");
	LOG_DBG("Received %d bytes", recv_len);

	if (recv_len <= 0) {
		LOG_ERR("Received data len not valid (%d)", recv_len);
		ret = -EAGAIN;
		goto error;
	}

error:
	return 0;
}

/* Handler: #XSEND: <size>[0] */
MODEM_CMD_DEFINE(on_cmd_sockwrite)
{
	int ret = 0;

	ret = ATOI(argv[0], -1, "sent");
	if (ret == 0) {
		/* Need to give semaphore, no OK to follow */
		modem_cmd_handler_set_error(data, 0);
		k_sem_give(&mdata.sem_response);
	}

	return ret;
}

/* Send binary data via the AT#XSEND commands using SLM Data Mode */
static ssize_t send_socket_data(struct modem_socket *sock, const struct sockaddr *dst_addr,
				const struct modem_cmd *handler_cmds, size_t handler_cmds_len,
				const char *buf, size_t buf_len, k_timeout_t timeout)
{
	int ret;
	char send_buf[sizeof("AT#XSENDTO=\"####.####.####.####.####.####.####.####\",####\t")];
	uint16_t dst_port = 0U;

	if (!sock) {
		return -EINVAL;
	}

	if (!dst_addr && sock->ip_proto == IPPROTO_UDP) {
		dst_addr = &sock->dst;
	}

	/*
	 * Data mode allows sending MDM_MAX_DATA_LENGTH bytes to
	 * the socket in one command
	 */
	if (buf_len > MDM_MAX_DATA_LENGTH) {
		buf_len = MDM_MAX_DATA_LENGTH;
	}

	if (sock->ip_proto == IPPROTO_UDP) {
		char ip_str[NET_IPV6_ADDR_LEN];

		ret = modem_context_sprint_ip_addr(dst_addr, ip_str, sizeof(ip_str));
		if (ret != 0) {
			LOG_ERR("Error formatting IP string %d", ret);
			goto exit;
		}

		ret = modem_context_get_addr_port(dst_addr, &dst_port);
		if (ret != 0) {
			LOG_ERR("Error getting port from IP address %d", ret);
			goto exit;
		}

		snprintk(send_buf, sizeof(send_buf), "AT#XSENDTO=\"%s\",%d", ip_str, dst_port);
	} else {
		snprintk(send_buf, sizeof(send_buf), "AT#XSEND");
	}

	/* Send command that will trigger entering SLM Data Mode */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, send_buf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Error entering data mode, timed out");
		goto exit;
	}

	/*
	 * Write all data to the iface and
	 * send MDM_DATA_MODE_TERMINATOR to exit SLM Data Mode
	 */
    LOG_HEXDUMP_DBG(buf, buf_len, "XMODE_DATA");
	mctx.iface.write(&mctx.iface, buf, buf_len);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, handler_cmds, handler_cmds_len,
			     MDM_DATA_MODE_TERMINATOR, &mdata.sem_response, timeout);

	if (ret < 0) {
		LOG_ERR("Data mode error");
		ret = -1;
	}

exit:
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Written %d bytes", buf_len);
	return buf_len;
}

/* Handler: OK */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: ERROR */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CME Error: <err>[0] */
MODEM_CMD_DEFINE(on_cmd_exterror)
{
	int err = 0;

	err = ATOI(argv[0], 0, "err");
	LOG_ERR("CME Error: %d", err);

	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CEREG: <reg_status>[0],<tac>[1],<ci>[2],<AcT>[3] */
MODEM_CMD_DEFINE(on_cmd_unsol_cereg)
{
	int registration_status = 0;

	registration_status = ATOI(argv[0], 0, "reg_status");

	/* Update modem state based on registration status */
	switch (registration_status) {
	case 0:
		// Not registered, not trying
		set_modem_state(NRF9160_STATE_OFF);
		break;
	case 1:
		// Registered, home network
	case 5:
		// Registered, roaming
		set_modem_state(NRF9160_STATE_REGISTERED);
		break;
	case 2:
		// Not registered, but trying
		set_modem_state(NRF9160_STATE_REGISTERING);
		break;
	default:
		set_modem_state(NRF9160_STATE_NOT_REGISTERED);
	}

	LOG_DBG("Network registration status %d", registration_status);
	if (argc == 4) {
		int act = 0;

		act = ATOI(argv[3], 0, "AcT");
		LOG_DBG("tac %s ci %s AcT %d", argv[1], argv[2], act);
	}
	return 0;
}

/*
 * Handler: #XGPS: <gnss_service>[0],<gnss_status>[1]
 * Handler: #XGPS: <latitude>[0],<longitude>[1],<altitude>[2],<accuracy>[3],<speed>[4],<heading>[5],<datetime>[6]
 */
MODEM_CMD_DEFINE(on_cmd_unsol_gnss)
{
    int ret = 0;
    int service = 0;
    int status = 0;

    if (argc == 2) {
        /* Status notification*/
        service = ATOI(argv[0], -1, "gnss_service");
        status = ATOI(argv[1], -1, "gnss_status");

        LOG_INF("Received GNSS service:%d status:%d", service, status);
    } else if (argc >= 6) {
        /* PVT data */
        LOG_INF("Received PVT data:");
        float latitude = 0;
        if (str_to_float(argv[0], &latitude) < 0) {
            LOG_ERR("Failed to convert latitude");
        }
        float longitude = 0;
        if (str_to_float(argv[1], &longitude) < 0) {
            LOG_ERR("Failed to convert longitude");
        }
        float altitude = 0;
        if (str_to_float(argv[2], &altitude) < 0) {
            LOG_ERR("Failed to convert altitude");
        }
        float accuracy = 0;
        if (str_to_float(argv[3], &accuracy) < 0) {
            LOG_ERR("Failed to convert accuracy");
        }
        float speed = 0;
        if (str_to_float(argv[4], &speed) < 0) {
            LOG_ERR("Failed to convert speed");
        }
        float heading = 0;
        if (str_to_float(argv[5], &heading) < 0) {
            LOG_ERR("Failed to convert heading");
        }
        LOG_INF("latitude:%f longitude:%f altitude:%f", latitude, longitude, altitude);
        LOG_INF("accuracy:%f speed:%f heading:%f", accuracy, speed, heading);
        /* TODO: why does it fail to parse datetime? */
        if (argc == 7) {
            LOG_INF("datetime:%s", argv[6]);
        }
    } else {
        LOG_WRN("Parsed %d args", argc);
        ret = -EIO;
    }

    return ret;
}

/*
 * Handler: +CEDRXP: <AcT-type>[0],<requested_edrx>[1],<provided_edrx>[2],<time_window>[3]
 * Params 1-2-3 are optional
 */
MODEM_CMD_DEFINE(on_cmd_unsol_edrx)
{
    int AcT_type = ATOI(argv[0], -1, "AcT_type");

    switch (argc) {
        case 1:
            LOG_INF("%s AcT:%d", __func__, AcT_type);
            break;
        case 2:
            LOG_INF("%s AcT:%d, req_eDRX:%s", __func__, AcT_type, argv[1]);
            break;
        case 3:
            LOG_INF("%s AcT:%d req_eDRX:%s provided_eDRX:%s", __func__, AcT_type, argv[1], argv[2]);
            break;
        case 4:
            LOG_INF("%s AcT:%d req_eDRX:%s provided_eDRX:%s time_window:%s", __func__, AcT_type, argv[1], argv[2], argv[3]);
            break;
        default:
            LOG_WRN("Received %d args", argc);
    }

    return 0;
}

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len = net_buf_linearize(
		mdata.mdm_manufacturer, sizeof(mdata.mdm_manufacturer) - 1, data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_DBG("Manufacturer: %s", mdata.mdm_manufacturer);
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len = net_buf_linearize(mdata.mdm_model, sizeof(mdata.mdm_model) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';

	/* Log the received information. */
	LOG_DBG("Model: %s", mdata.mdm_model);
	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len = net_buf_linearize(mdata.mdm_revision, sizeof(mdata.mdm_revision) - 1,
					   data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';

	/* Log the received information. */
	LOG_DBG("Revision: %s", mdata.mdm_revision);
	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len = 0;

	out_len = net_buf_linearize(mdata.mdm_imei, sizeof(mdata.mdm_imei) - 1,
								data->rx_buf, 0, len);
	mdata.mdm_imei[out_len] = '\0';

	/* Log the received information. */
	LOG_DBG("IMEI: %s", mdata.mdm_imei);
	return 0;
}

/* Handler: #XSOCKET: <handle>[0], <type>[1], <protocol>[2] */
MODEM_CMD_DEFINE(on_cmd_sockcreate)
{
	int ret = 0;
	int id;
	struct modem_socket *sock = NULL;

	k_sem_take(&mdata.sem_sock, K_FOREVER);

	/* Get socket from fd */
	sock = modem_socket_from_fd(&mdata.socket_config, mdata.sock_fd);
	if (sock) {
		id = ATOI(argv[0], -1, "handle");

		LOG_DBG("Created socket with id %d", id);

		/* Use received handle as socket ID, on error give up modem socket */
		ret = modem_socket_id_assign(&mdata.socket_config, sock, id);
		if (ret < 0) {
			LOG_ERR("Failed to assign socket ID %d", ret);
			modem_socket_put(&mdata.socket_config, sock->sock_fd);
		}
	}

	k_sem_give(&mdata.sem_sock);

	return 0;
}

/* Handler: #XSOCKET: <result>[0], <result_str>[1] */
MODEM_CMD_DEFINE(on_cmd_sockclose)
{
	int result = 0;

	result = ATOI(argv[0], -1, "result");

	/* Check if the socket was closed successfully */
	if (result == 0) {
		LOG_DBG("Socket closed");
	}

	return 0;
}

/* Handler: #XGETADDRINFO: <hostname>[0] */
MODEM_CMD_DEFINE(on_cmd_getaddrinfo)
{
	int ret = 0;
	char ips[256];
	char *ip;
	size_t out_len;

	/* Offset to skip the leading " */
	out_len = net_buf_linearize(ips, sizeof(ips) - 1, data->rx_buf, 1, len);
	ips[out_len] = '\0';

	/* find trailing " */
	ip = strstr(ips, "\"");

	if (!ip) {
		LOG_ERR("Malformed DNS response!!");
		ret = -EIO;
		goto exit;
	}

	/* Remove trailing " */
	*ip = '\0';

	/* Set addr family type based on str len */
	if (strlen(ips) > INET_ADDRSTRLEN) {
		/* IPV6 */
		dns_result.ai_family = AF_INET6;
		dns_result_addr.sa_family = AF_INET6;
	} else {
		/* IPV4 */
		dns_result.ai_family = AF_INET;
		dns_result_addr.sa_family = AF_INET;
	}

	LOG_DBG("Resolved ip addr %s", ips);

	ret = net_addr_pton(dns_result.ai_family, ips,
			    &((struct sockaddr_in *)&dns_result_addr)->sin_addr);
	if (ret < 0) {
		LOG_ERR("Failed to convert string to ip addr %d", ret);
	}
exit:
	return ret;
}

/* Handler: #XCONNECT: <status>[0] */
MODEM_CMD_DEFINE(on_cmd_connect)
{
	int ret = 0;
	int status = 0;
	struct modem_socket *sock = NULL;

	status = ATOI(argv[0], 0, "status");

	k_sem_take(&mdata.sem_sock, K_FOREVER);

	/* Retrieve socket */
	sock = modem_socket_from_fd(&mdata.socket_config, mdata.sock_fd);
	if (!sock) {
		LOG_ERR("Socket %d not found", mdata.sock_fd);
		ret = -EINVAL;
		goto error;
	}

	switch (status) {
	case 0:
		/* Disconnected */
		LOG_DBG("Disconnected");
		sock->is_connected = false;
		break;
	case 1:
		/* Connected */
		LOG_DBG("Connected");
		sock->is_connected = true;
		break;
	default:
		LOG_WRN("Received unknown status from XCONNECT %d", status);
	}

error:
	k_sem_give(&mdata.sem_sock);
	return ret;
}

/* Handler: #XSOCKETSELECT:
 * <handle>[0],<family>[1],<role>[2],<type>[3],<sec_tag>[4],<ranking>[5],<cid>[6] Handler:
 * #XSOCKETSELECT: <handle_active>[0]
 */
MODEM_CMD_DEFINE(on_cmd_sockselect)
{
	int ret = 0;

	if (argc == 7) {
		/* Nothing to do here really, just log */
		int handle = 0;

		handle = ATOI(argv[0], -1, "handle");
		if (handle >= 0) {
			LOG_DBG("Socket %d exists", handle);
		}
	} else {
		/* Received handle of the active socket, handle is used as sock id */
		int handle_active = 0;

		handle_active = ATOI(argv[0], -1, "handle_active");
		if (handle_active >= 0) {
			LOG_DBG("Socket %d is active", handle_active);

			k_sem_take(&mdata.sem_sock, K_FOREVER);

			/* Retrieve socket using received ID (active_handle) */
			struct modem_socket *sock =
				modem_socket_from_id(&mdata.socket_config, handle_active);
			if (!sock) {
				LOG_ERR("Failed to get socket from id %d", handle_active);
				ret = -EINVAL;
			} else {
				/* Update selected socket handle */
				mdata.sock_fd = sock->sock_fd;
			}

			k_sem_give(&mdata.sem_sock);
		} else {
			LOG_ERR("Failed to parse active socket handle");
			ret = -EIO;
		}
	}

	return ret;
}

MODEM_CMD_DEFINE(on_cmd_xcoex)
{
    LOG_DBG("%s", argv[0]);
    return 0;
}

MODEM_CMD_DEFINE(on_cmd_xmagpio)
{
    LOG_DBG("%s", argv[0]);
    return 0;
}

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("ERROR", on_cmd_error, 0U, ""),
	MODEM_CMD("+CME ERROR: ", on_cmd_exterror, 1U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD_ARGS_MAX("+CEREG: ", on_cmd_unsol_cereg, 1U, 4U, ","),
    MODEM_CMD_ARGS_MAX("#XGPS: ", on_cmd_unsol_gnss, 2U, 7U, ","),
    MODEM_CMD_ARGS_MAX("+CEDRXP: ", on_cmd_unsol_edrx, 1U, 4U, ","),
};

/* Commands sent to the modem to set it up at boot time. */
static const struct setup_cmd setup_cmds[] = {
	SETUP_CMD_NOHANDLE("AT+CFUN=0"),
	SETUP_CMD_NOHANDLE(MDM_SETUP_CMD_SYSTEM_MODE),
	SETUP_CMD_NOHANDLE("AT%XBANDLOCK=0"),
	SETUP_CMD_NOHANDLE("AT+COPS=0"),
	SETUP_CMD_NOHANDLE(MDM_SETUP_CMD_PDP_CTX),
	SETUP_CMD_NOHANDLE("AT+CEREG=2"),

    SETUP_CMD("AT%XCOEX0?", "%XCOEX0: ", on_cmd_xcoex, 1U, ""),
    SETUP_CMD("AT%XMAGPIO?", "%XMAGPIO: ", on_cmd_xmagpio, 1U, ""),
	/* Commands to read info from the modem (things like IMEI, Model etc). */
	SETUP_CMD("AT+CGMI", "", on_cmd_atcmdinfo_manufacturer, 0U, ""),
	SETUP_CMD("AT+CGMM", "", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+CGMR", "", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+CGSN", "", on_cmd_atcmdinfo_imei, 0U, ""),
};

/* Func: offload_reset
 * Desc: Function to send reset command to the modem
 */
static int offload_reset(void)
{
    int ret = 0;

    ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
                         NULL, 0U, "AT#XRESET",
                         &mdata.sem_response, MDM_CMD_TIMEOUT);
    if (ret) {
        LOG_ERR("Failed to reset modem, error %d", ret);
    }

    return ret;
}

/* Func: offload_gnss
 * Desc: Function to enable/disable GNSS in various modes
 */
static int offload_gnss(bool enable, uint16_t interval, uint16_t timeout)
{
    int ret = 0;
    char sendbuf[sizeof("AT#XGPS=#,#,#####,#####\t")];
    bool cloud_assistance = false; /* Do not use cloud assistance */

    if (enable) {
        /* Start GNSS */
        if (interval == 1) {
            /* Continous mode, omit timeout param */
            snprintk(sendbuf, sizeof(sendbuf), "AT#XGPS=%d,%d,%d", enable, cloud_assistance, interval);
        } else {
            /* One-shot or periodic */
            snprintk(sendbuf, sizeof(sendbuf), "AT#XGPS=%d,%d,%d,%d", enable, cloud_assistance, interval, timeout);
        }
    } else {
        /* Stop GNSS */
        snprintk(sendbuf, sizeof(sendbuf), "AT#XGPS=%d", enable);
    }

    ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
                         NULL, 0, sendbuf,
                         &mdata.sem_response, MDM_CMD_TIMEOUT);
    if (ret) {
        LOG_ERR("Failed to send GNSS command, error %d", ret);
    }

    return ret;
}

/* Func: modem_rx
 * Desc: Thread to process all messages received from the Modem.
 */
static void modem_rx(void)
{
	k_thread_name_set(NULL, "Modem_rx");

	while (true) {
		/* Wait for incoming data */
		modem_iface_uart_rx_wait(&mctx.iface, K_FOREVER);

		modem_cmd_handler_process(&mctx.cmd_handler, &mctx.iface);
	}
}

/* Func: pin_init
 * Desc: Power up the modem if mdm_power_gpios is defined
 * or reset it if mdm_reset_gpios is defined
 */
static int pin_init(void)
{
	int ret = 0;

#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios)
	ret = gpio_pin_set_dt(&power_gpio, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set power gpio high, error %d", ret);
		goto error;
	}
	k_sleep(K_MSEC(100));
	ret = gpio_pin_set_dt(&power_gpio, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set power gpio high, error %d", ret);
		goto error;
	}
	k_sleep(K_MSEC(100));
#elif DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	ret = gpio_pin_set_dt(&reset_gpio, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set reset gpio low, error %d", ret);
		goto error;
	}

	k_sleep(K_MSEC(100));

	ret = gpio_pin_set_dt(&reset_gpio, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set reset gpio high, error %d", ret);
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios) || DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
error:
#endif
	return ret;
}

/* Func: modem_setup
 * Desc: This function is used to setup the modem from zero. The idea
 * is that this function will be called right after the modem is
 * powered on to do the stuff necessary to talk to the modem.
 */
static int modem_setup(void)
{
	int ret = 0, counter;
	int init_retry_count = 0;

	/* Make sure the modem is being powered or reset */
	ret = pin_init();
	if (ret < 0) {
		goto error;
	}

restart:

	counter = 0;

	/* Make sure the modem is up, wait for a response to simple AT command */
	ret = -1;
	while (counter++ < MDM_AWAKE_RETRY_COUNT && ret < 0) {
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0, "AT",
				     &mdata.sem_response, MDM_AWAKE_CMD_TIMEOUT);
		if (ret < 0 && ret != -ETIMEDOUT) {
			break;
		}
		k_sleep(MDM_AWAKE_SLEEP_TIME_MS);
	}

	if (ret < 0) {
		LOG_ERR("Modem failed to reply to simple AT command: %d", ret);
		goto error;
	}

	/* Run setup commands on the modem. */
	LOG_DBG("Running modem setup commands");
	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler, setup_cmds,
					   ARRAY_SIZE(setup_cmds), &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);

	if (ret < 0 && init_retry_count < MDM_INIT_RETRY_COUNT) {
		init_retry_count++;
		goto restart;
	} else if (ret < 0) {
		LOG_ERR("Modem setup timeout");
		goto error;
	}

	/* Network is ready */
	LOG_DBG("Modem is configured");

error:
	return ret;
}

/* Func: get_active_socket
 * Desc: This function queries the modem to know which socket
 * is the active one (if any).
 * Called by offload_close() as when a socket is closed the
 * modem automatically selects another open one if available
 * without any notification.
 */
static int get_active_socket(void)
{
	int ret = 0;
	static const struct modem_cmd cmds[] = {
		MODEM_CMD_ARGS_MAX("#XSOCKETSELECT: ", on_cmd_sockselect, 1U, 7U, ","),
	};

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmds, ARRAY_SIZE(cmds),
			     "AT#XSOCKETSELECT?", &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Error retrieving active socket %d", ret);
	}

	return ret;
}

/* Func: select_socket
 * Desc: This function is called by every offloaded API
 * that uses socket to check that the active socket is
 * the one the application wants to use.
 * If the given socket is not the active one, sends a
 * command to the modem to select it.
 */
static int select_socket(int sock_fd)
{
	int ret = 0;
	int active_sock = 0;
	static const struct modem_cmd cmds[] = {
		MODEM_CMD("#XSOCKETSELECT: ", on_cmd_sockselect, 1U, ""),
	};
	char sendbuf[sizeof("AT#XSOCKETSELECT=#\t")];

	if ((sock_fd < 0) || (sock_fd >= MDM_MAX_SOCKETS)) {
		LOG_INF("Socket id %d out of range", sock_fd);
		ret = -EINVAL;
		goto error;
	}

	k_sem_take(&mdata.sem_sock, K_FOREVER);
	active_sock = mdata.sock_fd;
	k_sem_give(&mdata.sem_sock);

	// Check if the socket is already the active one
	if (sock_fd == active_sock) {
		LOG_DBG("Socket %d is already active", sock_fd);
		goto error;
	}

	struct modem_socket *sock = modem_socket_from_fd(&mdata.socket_config, sock_fd);

	snprintk(sendbuf, sizeof(sendbuf), "AT#XSOCKETSELECT=%d", sock->id);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmds, ARRAY_SIZE(cmds), sendbuf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Error selecting socket %d", ret);
	}

error:
	return ret;
}

/*
 * Func: offload_socket
 * Desc: This function will reserve a file descriptor from the fdtable
 * and send the command to the modem to open a new socket with the given options.
 */
static int offload_socket(int family, int type, int proto)
{
	int ret;
	static struct modem_cmd cmd = MODEM_CMD("#XSOCKET: ", on_cmd_sockcreate, 3U, ",");
	char sendbuf[sizeof("AT#XSOCKET=#,#,#\t")];
	int role = 0; /* Default: Client */
	int sock_fd;

	sock_fd = modem_socket_get(&mdata.socket_config, family, type, proto);
	if (sock_fd < 0) {
		ret = sock_fd;
		errno = -ret;
		goto error;
	}

	k_sem_take(&mdata.sem_sock, K_FOREVER);
	/* Store sock_fd */
	mdata.sock_fd = sock_fd;
	k_sem_give(&mdata.sem_sock);

	snprintk(sendbuf, sizeof(sendbuf), "AT#XSOCKET=%d,%d,%d", family, type, role);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, &cmd, 1U, sendbuf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		goto error;
	}

	/* Return socket fd */
	ret = sock_fd;
	errno = 0;
error:
	return ret;
}

/*
 * Func: offload_close
 * Desc: This function closes the connection with the remote client and
 * frees the socket.
 */
static int offload_close(void *obj)
{
	static const struct modem_cmd cmd = MODEM_CMD("#XSOCKET: ", on_cmd_sockclose, 2U, ",");
	struct modem_socket *sock = (struct modem_socket *)obj;
	char buf[] = "AT#XSOCKET=0";
	int ret = 0;

	/* Make sure socket is allocated and assigned an id */
	if (modem_socket_id_is_assigned(&mdata.socket_config, sock) == false) {
		goto exit;
	}

	/* Make sure the given socket is the one selected by the modem */
	ret = select_socket(sock->sock_fd);
	if (ret) {
		LOG_ERR("Failed to select socket %d, error %d", sock->id, ret);
		goto exit;
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, &cmd, 1U, buf, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", buf, ret);
		/* TODO: Socket was most likely closed by the server, what to do in this case? */
	}

	/* Invalidate reference to selected socket */
	k_sem_take(&mdata.sem_sock, K_FOREVER);
	mdata.sock_fd = -1;
	k_sem_give(&mdata.sem_sock);

	/* Query modem to know if another socket has been selected */
	ret = get_active_socket();
	if (ret) {
		LOG_ERR("Error querying active socket %d", ret);
	}

	/* Close socket */
	modem_socket_put(&mdata.socket_config, sock->sock_fd);
exit:
	return ret;
}

/*
 * Func: offload_bind
 * Desc: This function binds the provided socket to the provided address.
 */
static int offload_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* Make sure we've created the socket */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
		LOG_ERR("Need to create a socket first!");
		return -1;
	}

	/* Save bind address information */
	memcpy(&sock->src, addr, sizeof(*addr));

	return 0;
}

/*
 * Func: offload_connect
 * Desc: This function will connect with a provided address.
 */
static int offload_connect(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	struct modem_socket *sock = (struct modem_socket *)obj;
	int ret;
	static const struct modem_cmd cmd = MODEM_CMD("#XCONNECT: ", on_cmd_connect, 1U, ",");
	char buf[sizeof("AT#XCONNECT=\"####.####.####.####.####.####.####.####\",#####\t")];
	uint16_t dst_port = 0U;
	char ip_str[NET_IPV6_ADDR_LEN];

	/* Modem is not attached to the network. */
	if (get_modem_state() != NRF9160_STATE_REGISTERED) {
		LOG_ERR("Modem not attached to the network, can't connect");
		return -EAGAIN;
	}

	if (!addr) {
		errno = EINVAL;
		ret = -1;
		goto error;
	}

	/* Make sure socket has been allocated */
	if (modem_socket_is_allocated(&mdata.socket_config, sock) == false) {
		LOG_ERR("Invalid socket_id(%d) from fd:%d", sock->id, sock->sock_fd);
		errno = EINVAL;
		ret = -1;
		goto error;
	}

	/* Make sure we've created the socket */
	if (modem_socket_id_is_assigned(&mdata.socket_config, sock) == false) {
		LOG_ERR("Need to create a socket first!");
		ret = -1;
		goto error;
	}

	memcpy(&sock->dst, addr, sizeof(*addr));
	if (addr->sa_family == AF_INET6) {
		dst_port = ntohs(net_sin6(addr)->sin6_port);
	} else if (addr->sa_family == AF_INET) {
		dst_port = ntohs(net_sin(addr)->sin_port);
	} else {
		errno = EAFNOSUPPORT;
		ret = -1;
		goto error;
	}

	/* Skip socket connect if UDP */
	if (sock->ip_proto == IPPROTO_UDP) {
		errno = 0;
		ret = 0;
		goto error;
	}

	ret = modem_context_sprint_ip_addr(addr, ip_str, sizeof(ip_str));
	if (ret != 0) {
		errno = -ret;
		LOG_ERR("Error formatting IP string %d", ret);
		goto error;
	}

	/* Make sure the given socket is the one selected by the modem */
	ret = select_socket(sock->sock_fd);
	if (ret) {
		LOG_ERR("Failed to select socket %d, error %d", sock->id, ret);
		goto error;
	}

	snprintk(buf, sizeof(buf), "AT#XCONNECT=\"%s\",%d", ip_str, dst_port);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, &cmd, 1U, buf, &mdata.sem_response,
			     MDM_CMD_CONN_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("%s ret:%d", buf, ret);
		errno = -ret;
		goto error;
	}

	errno = 0;

error:
	return ret;
}

/*
 * Func: offload_sendto
 * Desc: This function will send data on the socket object.
 */
static ssize_t offload_sendto(void *obj, const void *buf, size_t len, int flags,
			      const struct sockaddr *to, socklen_t tolen)
{
	int ret;
	const struct modem_cmd handler_cmds[] = {
		MODEM_CMD("#XDATAMODE: ", on_cmd_sockwrite, 1U, "")};
	struct modem_socket *sock = (struct modem_socket *)obj;

	/* Modem is not attached to the network. */
	if (get_modem_state() != NRF9160_STATE_REGISTERED) {
		LOG_ERR("Modem not attached to the network, can't send");
		ret = -EAGAIN;
		goto error;
	}

	/* Ensure that valid parameters are passed. */
	if (!buf || len == 0) {
		LOG_ERR("Invalid buf or len");
		errno = EINVAL;
		ret = -1;
		goto error;
	}

	if (!sock->is_connected && sock->ip_proto != IPPROTO_UDP) {
		LOG_ERR("Socket is not connected");
		errno = ENOTCONN;
		ret = -1;
		goto error;
	}

	/* Make sure the given socket is the one selected by the modem */
	ret = select_socket(sock->sock_fd);
	if (ret) {
		LOG_ERR("Failed to select socket %d, error %d", sock->id, ret);
		goto error;
	}

	ret = send_socket_data(sock, to, handler_cmds, ARRAY_SIZE(handler_cmds), buf, len,
			       MDM_CMD_TIMEOUT);
	if (ret < 0) {
		errno = -ret;
		ret = -1;
		goto error;
	}

	/* Data was written successfully. */
	errno = 0;
error:
	return ret;
}

/*
 * Func: offload_recvfrom
 * Desc: This function will receive data on the socket object.
 */
static ssize_t offload_recvfrom(void *obj, void *buf, size_t len, int flags, struct sockaddr *from,
				socklen_t *fromlen)
{
	int ret = 0;
	char sendbuf[sizeof("AT#XRECV=####,###\t")];
	struct modem_socket *sock = (struct modem_socket *)obj;
	static const struct modem_cmd cmd[] = {
		MODEM_CMD("#XRECV: ", on_cmd_sockrecv, 1U, ""), /* Response to AT#XRECV */
		MODEM_CMD("", on_cmd_datarecv, 1U, ""),         /* Actual data */
	};

	/* Modem is not attached to the network. */
	if (get_modem_state() != NRF9160_STATE_REGISTERED) {
		LOG_ERR("Modem not attached to the network, can't receive");
		return -EAGAIN;
	}

	if (!buf || len == 0) {
		errno = EINVAL;
		ret = -1;
		goto error;
	}

	/* Make sure the given socket is the one selected by the modem */
	ret = select_socket(sock->sock_fd);
	if (ret) {
		LOG_ERR("Failed to select socket %d, error %d", sock->id, ret);
		goto error;
	}

	if ((flags & ZSOCK_MSG_DONTWAIT) || (flags & ZSOCK_MSG_WAITALL) ||
	    (flags & ZSOCK_MSG_PEEK)) {
		/* Create message to start receiving data, using provided flags */
		snprintk(sendbuf, sizeof(sendbuf), "AT#XRECV=%d,%d", MDM_RECV_DATA_TIMEOUT_SEC,
			 flags);
	} else {
		/* Create message to start receiving data */
		snprintk(sendbuf, sizeof(sendbuf), "AT#XRECV=%d", MDM_RECV_DATA_TIMEOUT_SEC);
	}

	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), sendbuf,
			     &mdata.sem_response, MDM_CMD_TIMEOUT);
	/*
	 * If -EIO is returned we didn't receive anything from the socket,
	 * but we might still have data in the ringbuf to return
	 */
	if ((ret < 0) && (ret != -EIO)) {
		errno = -ret;
		goto error;
	}

	k_sem_take(&mdata.sem_rx_ringbuf, K_FOREVER);
	/*
	 * Check if ringbuf is empty, not data has been received
	 * If empty we actually do not have any data to return
	 */
	if (ring_buf_is_empty(&rx_ringbuf)) {
		LOG_DBG("No data received");
		ret = -EAGAIN;
		errno = -ret;
		goto exit;
	}

	/* Get data from rx_ringbuf and copy it to caller's buf */
	ret = (int)ring_buf_get(&rx_ringbuf, buf, len);
	if (ret != len) {
		LOG_DBG("Received data smaller than buffer, %d < %d", ret, len);
	}

	errno = 0;
exit:
	k_sem_give(&mdata.sem_rx_ringbuf);
error:
	return ret;
}

/*
 * Func: offload_read
 * Desc: This function reads data from the given socket object.
 */
static ssize_t offload_read(void *obj, void *buffer, size_t count)
{
	return offload_recvfrom(obj, buffer, count, 0, NULL, 0);
}

/*
 * Func: offload_write
 * Desc: This function writes data to the given socket object.
 */
static ssize_t offload_write(void *obj, const void *buffer, size_t count)
{
	return offload_sendto(obj, buffer, count, 0, NULL, 0);
}

/*
 * Func: offload_sendmsg
 * Desc: This function sends messages to the modem.
 */
static ssize_t offload_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	ssize_t sent = 0;
	int rc;

	LOG_DBG("msg_iovlen:%zd flags:%d", msg->msg_iovlen, flags);

	for (int i = 0; i < msg->msg_iovlen; i++) {
		const char *buf = msg->msg_iov[i].iov_base;
		size_t len = msg->msg_iov[i].iov_len;

		while (len > 0) {
			rc = offload_sendto(obj, buf, len, flags, msg->msg_name, msg->msg_namelen);
			if (rc < 0) {
				if (rc == -EAGAIN) {
					k_sleep(MDM_SENDMSG_SLEEP);
				} else {
					sent = rc;
					break;
				}
			} else {
				sent += rc;
				buf += rc;
				len -= rc;
			}
		}
	}

	return (ssize_t)sent;
}

/* Func: offload_ioctl
 * Desc: Function call to handle various misc requests.
 */
static int offload_ioctl(void *obj, unsigned int request, va_list args)
{
	switch (request) {
	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;
		struct k_poll_event *pev_end;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);
		pev_end = va_arg(args, struct k_poll_event *);

		return modem_socket_poll_prepare(&mdata.socket_config, obj, pfd, pev, pev_end);
	}
	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd;
		struct k_poll_event **pev;

		pfd = va_arg(args, struct zsock_pollfd *);
		pev = va_arg(args, struct k_poll_event **);

		return modem_socket_poll_update(obj, pfd, pev);
	}

	default:
		errno = EINVAL;
		return -1;
	}
}

/*
 * Perform a dns lookup.
 */
static int offload_getaddrinfo(const char *node, const char *service,
			       const struct zsock_addrinfo *hints, struct zsock_addrinfo **res)
{
	struct modem_cmd cmd[] = {MODEM_CMD("#XGETADDRINFO: ", on_cmd_getaddrinfo, 1U, "")};
	char sendbuf[sizeof("AT#XGETADDRINFO=") + 128];
	uint32_t port = 0;
	int ret;

	/* Modem is not attached to the network. */
	if (get_modem_state() != NRF9160_STATE_REGISTERED) {
		LOG_ERR("Modem currently not attached to the network!");
		return DNS_EAI_AGAIN;
	}

	/* init result */
	(void)memset(&dns_result, 0, sizeof(dns_result));
	(void)memset(&dns_result_addr, 0, sizeof(dns_result_addr));

	dns_result.ai_addr = &dns_result_addr;
	dns_result.ai_addrlen = sizeof(dns_result_addr);
	dns_result.ai_canonname = dns_result_canonname;
	dns_result_canonname[0] = '\0';

	if (service) {
		port = ATOI(service, -1, "port");
		if (port < 1 || port > USHRT_MAX) {
			LOG_ERR("Port number is out of range %d", port);
			return DNS_EAI_SERVICE;
		}
	}

	if (port > 0U) {
		if (dns_result.ai_family == AF_INET) {
			net_sin(&dns_result_addr)->sin_port = htons(port);
		}
	}

	/* Check if node is an IP address */
	if (net_addr_pton(dns_result.ai_family, node,
			  &((struct sockaddr_in *)&dns_result_addr)->sin_addr) == 0) {
		*res = &dns_result;
		LOG_INF("Already an IP address, returning");
		return 0;
	}

	/* user flagged node as numeric host, but we failed net_addr_pton */
	if (hints && hints->ai_flags & AI_NUMERICHOST) {
		LOG_ERR("Numeric host flag, but failed to convert address");
		return DNS_EAI_NONAME;
	}

	snprintk(sendbuf, sizeof(sendbuf), "AT#XGETADDRINFO=\"%s\"", node);
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, cmd, ARRAY_SIZE(cmd), sendbuf,
			     &mdata.sem_response, MDM_DNS_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	*res = (struct zsock_addrinfo *)&dns_result;
	return 0;
}

/*
 * Free addrinfo structure.
 */
static void offload_freeaddrinfo(struct zsock_addrinfo *res)
{
	/* No need to free static memory. */
	res = NULL;
}

/*
 * Setup the Modem NET Interface
 */
static void modem_net_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct modem_data *data = dev->data;

	/* Direct socket offload used instead of net offload: */
	net_if_set_link_addr(iface, modem_get_mac(dev), sizeof(data->mac_addr), NET_LINK_ETHERNET);
	data->net_iface = iface;

	socket_offload_dns_register(&offload_dns_ops);

	net_if_socket_offload_set(iface, offload_socket);
}

/*
 * Enable or disable modem using AT+CFUN
 * when net_if_up/down() is called
 */
static int modem_net_iface_enable(const struct net_if *iface, bool state)
{
	char buf[sizeof("AT+CFUN=#\t")];

	LOG_DBG("Received iface %s", state ? "enable" : "disable");

	snprintk(buf, sizeof(buf), "AT+CFUN=%d", state ? 1 : 0);

	int ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler, NULL, 0U, buf, &mdata.sem_response,
				 MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("Failed to set modem mode, err %d", ret);
	}

	return ret;
}

/* Public APIs exposed in zephyr/include/drivers/modem/nordic-nrf9160.h */

/*
 * Function used to reset the modem.
 * Runs modem_setup() to either power cycle or trigger
 * reset pin base on what is defined in DTS, run
 * setup commands and enable interface.
 */
int mdm_nrf9160_reset(void)
{
	int ret = 0;

    /* Make sure modem is disconnected */
    ret = modem_net_iface_enable(NULL, false);
    if (ret < 0) {
        LOG_ERR("Error disabling modem %d", ret);
        goto error;
    }

    /* Reset modem */
    ret = offload_reset();
    if (ret) {
        LOG_ERR("Error resetting modem %d", ret);
        goto error;
    }

    /* Configure modem */
	ret = modem_setup();
	if (ret < 0) {
		LOG_ERR("Modem setup error %d", ret);
		goto error;
	}

	/* Enable the modem again */
	ret = modem_net_iface_enable(NULL, true);
	if (ret < 0) {
		LOG_ERR("Error enabling modem %d", ret);
	}
error:
	return ret;
}

/*
 * Function used to start the GNSS in oneshot mode
 * specifying a timeout in seconds.
 * Passing a timeout of 0 seconds will make the GNSS
 * run until it's able to get a fix.
 */
int mdm_nrf9160_gnss_start_oneshot(uint16_t timeout)
{
    int ret = 0;

    ret = offload_gnss(true, 0, timeout);

    return ret;
}

/*
 * Function used to start the GNSS in continuous mode.
 * In this mode the fix interval is set to 1 second.
 */
int mdm_nrf9160_gnss_start_continuous(void)
{
    int ret = 0;

    ret = offload_gnss(true, 1, 0);

    return ret;
}

/*
 * Function used to start the GNSS in periodic mode
 * specifying an interval and a timeout in seconds.
 * Passing a timeout of 0 seconds will result in the GNSS
 * running until it's able to get a fix.
 */
int mdm_nrf9160_gnss_start_periodic(uint16_t interval, uint16_t timeout)
{
    int ret = 0;

    ret = offload_gnss(1, interval, timeout);

    return ret;
}

/*
 * Function used to stop the GNSS
 */
int mdm_nrf9160_gnss_stop(void)
{
    int ret = 0;

    ret = offload_gnss(false, 0, 0);

    return ret;
}

/*
 * DNS vtable.
 */
static const struct socket_dns_offload offload_dns_ops = {
	.getaddrinfo = offload_getaddrinfo,
	.freeaddrinfo = offload_freeaddrinfo,
};

/*
 * Offloaded API funcs
 */
static struct offloaded_if_api api_funcs = {
	.iface_api.init = modem_net_iface_init,
	.enable = modem_net_iface_enable,
};

/*
 * Socket vtable.
 */
static const struct socket_op_vtable offload_socket_fd_op_vtable = {
	.fd_vtable = {
		.read = offload_read,
		.write = offload_write,
		.close = offload_close,
		.ioctl = offload_ioctl,
	},
	.shutdown = NULL,
	.bind = offload_bind,
	.connect = offload_connect,
	.listen = NULL,
	.accept = NULL,
	.sendto = offload_sendto,
	.recvfrom = offload_recvfrom,
	.getsockopt = NULL,
	.setsockopt = NULL,
	.sendmsg = offload_sendmsg,
	.getpeername = NULL,
	.getsockname = NULL,
};

static bool offload_is_supported(int family, int type, int proto)
{
	if (family != AF_INET && family != AF_INET6) {
		LOG_DBG("Offload not supported, family %d", family);
		return false;
	}

	if (type != SOCK_STREAM) {
		LOG_DBG("Offload not supported, type %d", type);
		return false;
	}

	if (proto != IPPROTO_TCP) {
		LOG_DBG("Offload not supported, proto %d", proto);
		return false;
	}

	return true;
}

static int modem_init(const struct device *dev)
{
	int ret;

	ARG_UNUSED(dev);

	/* Semaphores */
	k_sem_init(&mdata.sem_response, 0, 1);
	k_sem_init(&mdata.sem_modem_state, 1, 1);
	k_sem_init(&mdata.sem_sock, 1, 1);
	k_sem_init(&mdata.sem_rx_ringbuf, 1, 1);

	/* Socket config */
	ret = modem_socket_init(&mdata.socket_config, &mdata.sockets[0], ARRAY_SIZE(mdata.sockets),
				MDM_BASE_SOCKET_NUM, false, &offload_socket_fd_op_vtable);
	if (ret < 0) {
		LOG_ERR("Socket init error %d", ret);
		goto error;
	}

	/* Cmd handler setup */
	const struct modem_cmd_handler_config cmd_handler_config = {
		.match_buf = &mdata.cmd_match_buf[0],
		.match_buf_len = sizeof(mdata.cmd_match_buf),
		.buf_pool = &mdm_recv_pool,
		.alloc_timeout = BUF_ALLOC_TIMEOUT,
		.eol = "\r\n",
		.user_data = NULL,
		.response_cmds = response_cmds,
		.response_cmds_len = ARRAY_SIZE(response_cmds),
		.unsol_cmds = unsol_cmds,
		.unsol_cmds_len = ARRAY_SIZE(unsol_cmds),
	};

	ret = modem_cmd_handler_init(&mctx.cmd_handler, &mdata.cmd_handler_data,
				     &cmd_handler_config);
	if (ret < 0) {
		LOG_ERR("Cmd handler init error %d", ret);
		goto error;
	}

	/* Modem interface */
	const struct modem_iface_uart_config uart_config = {
		.rx_rb_buf = &mdata.iface_rb_buf[0],
		.rx_rb_buf_len = sizeof(mdata.iface_rb_buf),
		.dev = MDM_UART_DEV,
		.hw_flow_control = DT_PROP(MDM_UART_NODE, hw_flow_control),
	};

	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data, &uart_config);
	if (ret < 0) {
		LOG_ERR("Modem iface init error %d", ret);
		goto error;
	}

	/* Modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_imei;

	/* Pin setup */
#if DT_INST_NODE_HAS_PROP(0, mdm_power_gpios)
	ret = gpio_pin_configure_dt(&power_gpio, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "power");
		goto error;
	}
#endif

#if DT_INST_NODE_HAS_PROP(0, mdm_reset_gpios)
	ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		LOG_ERR("Failed to configure %s pin", "reset");
		goto error;
	}
#endif

	/* Modem context setup */
	mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Modem context registration error %d", ret);
		goto error;
	}

	/* Start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack, K_KERNEL_STACK_SIZEOF(modem_rx_stack),
			(k_thread_entry_t)modem_rx, NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	ret = modem_setup();
	if (ret < 0) {
		LOG_ERR("Modem setup error %d", ret);
	} else {
		/* Update modem state */
		set_modem_state(NRF9160_STATE_INIT);
	}

error:
	return ret;
}

/* Register the device with the Networking stack. */
NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0, modem_init, NULL, &mdata, NULL,
				  CONFIG_MODEM_NORDIC_NRF9160_INIT_PRIORITY, &api_funcs,
				  MDM_MAX_DATA_LENGTH);

/* Register NET sockets. */
NET_SOCKET_OFFLOAD_REGISTER(nordic_nrf9160, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, AF_UNSPEC,
			    offload_is_supported, offload_socket);
