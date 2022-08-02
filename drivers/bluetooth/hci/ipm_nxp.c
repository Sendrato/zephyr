/**
 * Copyright 2021 Sendrato - https://sendrato.com
 * All rights reserved.
 *
 * \file
 *
 * Bluetooth driver for NXP module
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <init.h>
#include <sys/util.h>
#include <kernel.h>
#include <bluetooth/hci.h>
#include <drivers/bluetooth/hci_driver.h>

#include "radio.h"
#include "fsl_os_abstraction.h"
#include "controller_interface.h"
#include "EmbeddedTypes.h" // TODO not necessary?

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME hci_ipm
#include "common/log.h"

#define HCI_CMD                 0x01
#define HCI_ACL                 0x02
#define HCI_SCO                 0x03
#define HCI_EVT                 0x04

#define ACI_WRITE_SET_TX_POWER_LEVEL       BT_OP(BT_OGF_VS, 0xFC0F)

/* TODO: get proper size from bt_controller_task_config.h */
#define BLE_CONTROLLER_STACK_SIZE	(4096)

/* TODO: map this to config */
#if defined gBleUseHSClock2MbpsPhy_c && (gBleUseHSClock2MbpsPhy_c == 1)
#define DATA_RATE DR_2MBPS
#else
#define DATA_RATE DR_1MBPS
#endif

/**
 * @brief BT Controller_TaskHandler stack and thread definitions
 */
static K_KERNEL_STACK_DEFINE(bt_controller_stack, BLE_CONTROLLER_STACK_SIZE);
static struct k_thread _bt_controller_thread_data;


/**
 * @brief OSA event object definition for (BT) Controller_TaskHandler
 */
static osaTaskId_t _bt_controller_task_event;


/**
 * @brief Determine if an HCI event is discardable.
 *
 * @param[in] packet    Pointer to HCI-Event Packet
 * @return              true if discardable, false otherwise
 */
static bool is_hci_event_discardable(const uint8_t *packet)
{
    struct bt_hci_evt_hdr *evt_hdr;
    struct bt_hci_evt_le_meta_event *evt_data;

    evt_hdr  = (struct bt_hci_evt_hdr *) &packet[0];
    evt_data = (struct bt_hci_evt_le_meta_event *) &packet[BT_HCI_EVT_HDR_SIZE];

    switch (evt_hdr->evt) {
#if defined(CONFIG_BT_BREDR)
        case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
	    case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
		    return true;
#endif
        case BT_HCI_EVT_LE_META_EVENT:
            switch (evt_data->subevent) {
                case BT_HCI_EVT_LE_ADVERTISING_REPORT:
                case BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT:
                    return true;
                default:
                    return false;
            }

        default:
            return false;
    }
}


/**
 * @brief Packet callback from Controller_TaskHandler.
 *
 * Packet specification can be found in:
 *      Bluetooth Core v5.0, Part E, Section 5.4, HCI Data Formats
 *
 * Note that this callback does not queue/dequeue the received packets from the
 * TaskHandler. The TaskHandler is expected to be an endless-looping function
 * within the controller-archive. As such, it is encapsulated in a dedicated
 * Zephyr thread in bt_ipm_init(). Since this TaskHandler acts as a thread, it
 * is expected that the controller-archive adds messages and radio interrupts
 * to an internal queue, which is dequeued by the TaskHandler and pushed to this
 * callback. With the internal queue, it is expected that no additional queue
 * is required for this callback.
 *
 * @param[in] packet_type  Packet-type of HCI packet
 * @param[in] packet       Pointer to HCI Packet (without type-byte)
 * @param[in] size         Length of packet
 * @return                 NXP bleResult value, gBleSuccess_c on succes.
 */
static bleResult_t bt_controller_cb( hciPacketType_t packet_type,
                                    void *packet, uint16_t size )
{
	k_timeout_t timeout = K_FOREVER;
	bool discardable    = false;
	struct net_buf *buf = NULL;

    struct bt_hci_acl_hdr *acl_hdr;
    struct bt_hci_evt_hdr *evt_hdr;

	switch (packet_type) {
        case HCI_EVT:
            evt_hdr = (struct bt_hci_evt_hdr *) packet;
            BT_DBG("EVT: evt_code: 0x%02x", evt_hdr->evt);

            discardable = is_hci_event_discardable(packet);
            if (discardable == true) {
                timeout = K_NO_WAIT;
            }

            /* Allocate buf for EVT data */
            buf = bt_buf_get_evt(evt_hdr->evt, discardable, timeout);
            if (!buf) {
                if (discardable) {
                    BT_DBG("Discard EVT due to insufficient buf, ignoring event");
                } else {
                    BT_ERR("Discard EVT due to insufficient buf");
                }
                return gBleOutOfMemory_c;
            }

            /* Add Event-header + Event-data */
            net_buf_add_mem(buf, evt_hdr, BT_HCI_EVT_HDR_SIZE);
            net_buf_add_mem(buf, &((uint8_t*)packet)[BT_HCI_EVT_HDR_SIZE], evt_hdr->len);
            break;

        case HCI_ACL:
            acl_hdr = (struct bt_hci_acl_hdr *) packet;
            BT_DBG("ACL: handle %x, len %x", acl_hdr->handle, acl_hdr->len);

            /* Allocate buf for ACL data */
            buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
            if (!buf) {
                BT_ERR("Discard ACL due to insufficient buf");
                return gBleOutOfMemory_c;
            }

            /* Add ACL-header + ACL-data */
            net_buf_add_mem(buf, acl_hdr, BT_HCI_ACL_HDR_SIZE);
            net_buf_add_mem(buf, &((uint8_t*)packet)[BT_HCI_ACL_HDR_SIZE], acl_hdr->len);
            break;

        default:
            BT_ERR("Unknown BT buf type %d", packet_type);
            return gBleInvalidParameter_c;
	}

	/* Push HCI packet to Zephyr-host stack */
	bt_recv(buf);

	return gBleSuccess_c;
}


/**
 * @brief Sent packet from Zephyr-Host to Controller.
 *
 * @param[in] buf  Pointer to Buffer containing HCI Packet
 * @return         0 on success, negative otherwise.
 */
static int bt_ipm_send(struct net_buf *buf)
{
    bleResult_t rv;
    hciPacketType_t packet_type;

    /*
     * Determine packet-type of HCI-packet as Controller has different
     * typedef's when compared with Zephy
     */
    uint8_t buf_type = bt_buf_get_type(buf);
	switch (buf_type) {
        case BT_BUF_CMD:
            packet_type = gHciCommandPacket_c;
            break;
        case BT_BUF_EVT:
            packet_type = gHciEventPacket_c;
            break;
        case BT_BUF_ACL_OUT:
            packet_type = gHciDataPacket_c;
            break;
        default:
            BT_DBG("HCI Send packet invalid packet type: %d", buf_type);
            return -EINVAL;
	}

    /* Sent packet to Controller */
    rv = Hci_SendPacketToController(packet_type, (void*)buf->data, buf->len);
    net_buf_unref(buf);

    /* bleResult mapping to Zephyr error */
	switch (rv) {
	    case gBleSuccess_c:
            return 0;
	    case gBleOutOfMemory_c:
            return -ENOMEM;
	    default:
            return -EINVAL;
	}
}


/**
 * @brief Init Bluetooth Radio.
 *
 * @return 0 on success, xcvrStatus_t otherwise.
 */
static xcvrStatus_t bt_radio_init(void)
{
	int32_t temperature = 0;

    /* TODO: Initial radio calibration function of temperature */
    /* temperature = BOARD_GetTemperature(); */
	XCVR_TemperatureUpdate(temperature);

	return XCVR_Init(BLE_MODE, DATA_RATE);
}


/**
 * @brief Open IPM channel with Controller
 *
 * @return 0 on success, negative otherwise.
 */
static int bt_ipm_open(void)
{
	int rv = 0;
    struct net_buf *rsp = NULL;

    BT_DBG("IPM Opening Channel...\n");

    /* Send HCI_RESET */
    rv = bt_hci_cmd_send_sync(BT_HCI_OP_RESET, NULL, &rsp);
    if (rv != 0) {
        BT_DBG("IPM Channel Open Failed: HCI Send Initial RESET has error %04x\n", rv);
        return rv;
    }

    /* TBD: Something to do on reset complete? */
    net_buf_unref(rsp);

    BT_DBG("IPM Channel Open Completed\n");
	return 0;
}


static const struct bt_hci_driver _bt_controller_drv = {
	.name           = "BT HCI NXP",
	.bus            = BT_HCI_DRIVER_BUS_IPM,
	.quirks         = BT_QUIRK_NO_RESET,
	.open           = bt_ipm_open,
	.send           = bt_ipm_send,
};

/**
 * @brief Init IPM and Controller
 *
 * @return 0 on success, negative otherwise.
 */
static int bt_ipm_init(const struct device *unused)
{
    ARG_UNUSED(unused);

    bt_hci_driver_register(&_bt_controller_drv);

    /* Init Radio Hardware */
    if (bt_radio_init() != gXcvrSuccess_c) {
        return -EIO;
    }

    /* TODO: move to proper location */
    extern void BLE_LL_ALL_IRQHandler();
    extern void RFP_TMU_IRQHandler();

    IRQ_DIRECT_CONNECT(BLE_LL_ALL_IRQn, 1, BLE_LL_ALL_IRQHandler, 0);
    IRQ_DIRECT_CONNECT(RFP_TMU_IRQn, 1, RFP_TMU_IRQHandler, 0);

    irq_enable(BLE_LL_ALL_IRQn);
    irq_enable(RFP_TMU_IRQn);

    /* BLE */
    if (NVIC_GetPendingIRQ(BLE_WAKE_UP_TIMER_IRQn)) {
        NVIC_ClearPendingIRQ(BLE_WAKE_UP_TIMER_IRQn);
    }

    /* Enable BLE Interrupt */
    NVIC_EnableIRQ(BLE_LL_ALL_IRQn);
    NVIC_EnableIRQ(RFP_TMU_IRQn);

    /* Setup Controller */
	_bt_controller_task_event = OSA_EventCreate(TRUE);
	Controller_TaskEventInit(_bt_controller_task_event, true);
	if (Controller_Init(bt_controller_cb) != osaStatus_Success) {
        BT_ERR("NXP controller failed to start");
        return -EIO;
    }
    /* Start Controller-RX / taskhandler-thread */
    k_thread_create(&_bt_controller_thread_data, bt_controller_stack,
                    K_KERNEL_STACK_SIZEOF(bt_controller_stack),
                    (k_thread_entry_t)Controller_TaskHandler, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(CONFIG_BT_DRIVER_RX_HIGH_PRIO),
                    0, K_NO_WAIT);

    k_thread_name_set(&_bt_controller_thread_data, "nxp hci controller");

    BT_DBG("NXP controller started");
	return 0;
}

SYS_INIT(bt_ipm_init, POST_KERNEL, CONFIG_BT_NXP_IPM_INIT_PRIORITY);
