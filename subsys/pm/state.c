/*
 * Copyright (c) 2018 Intel Corporation.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/state.h>
#include <zephyr/toolchain.h>

BUILD_ASSERT(DT_NODE_EXISTS(DT_PATH(cpus)),
	     "cpus node not defined in Devicetree");

/**
 * Check CPU power state consistency.
 *
 * @param i Power state index.
 * @param node_id CPU node identifier.
 */
#define CHECK_POWER_STATE_CONSISTENCY(i, node_id)			       \
	BUILD_ASSERT(							       \
		DT_PROP_BY_PHANDLE_IDX_OR(node_id, cpu_power_states, i,	       \
					  min_residency_us, 0U) >=	       \
		DT_PROP_BY_PHANDLE_IDX_OR(node_id, cpu_power_states, i,	       \
					  exit_latency_us, 0U),		       \
		"Found CPU power state with min_residency < exit_latency")

/**
 * @brief Check CPU power states consistency
 *
 * All states should have a minimum residency >= than the exit latency.
 *
 * @param node_id A CPU node identifier.
 */
#define CHECK_POWER_STATES_CONSISTENCY(node_id)				       \
	LISTIFY(DT_PROP_LEN_OR(node_id, cpu_power_states, 0),		       \
		CHECK_POWER_STATE_CONSISTENCY, (;), node_id);		       \

/* Check that all power states are consistent */
DT_FOREACH_CHILD(DT_PATH(cpus), CHECK_POWER_STATES_CONSISTENCY)


//struct qn9090_config {
//    uint8_t state;
//    uint8_t substate_id;
//    pm_power_config_t config;
//};
///*
//if(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false)){
//config.pm_wakeup_src |= POWER_WAKEUPSRC_IO;
//config.pm_wakeup_io |= 1 << BOARD_SW1_GPIO_PIN;
//}
//        if(DT_PROP_OR(NODE_ID_SUSTORAM, retain_radio_device, false)){
//                config.pm_config |= PM_CFG_RADIO_RET;           //Behouden radio instellingen
//            }
//*/
//
//#define PM_STATE_INFO_DT_INIT2(node_id)					       \
//	{								       \
//		.state = PM_STATE_DT_INIT(node_id),			       \
//		.substate_id = DT_PROP_OR(node_id, substate_id, 0),	       \
//		.config.pm_config = (COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, retain_radio_device, false), PM_CFG_RADIO_RET, 0)),\
//		.config.pm_wakeup_src = (COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false), POWER_WAKEUPSRC_IO, 0) \
//                                 | COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false), POWER_WAKEUPSRC_IO, 0)   \
//                                 | COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false), POWER_WAKEUPSRC_IO, 0)),\
//		.config.pm_wakeup_io = (COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false), 1 << BOARD_SW1_GPIO_PIN, 0) \
//                                 | COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false), POWER_WAKEUPSRC_IO, 0)   \
//                                 | COND_CODE_1(DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false), POWER_WAKEUPSRC_IO, 0)),\
//                                 }
//
//#define Z_PM_STATE_INFO_FROM_DT_CPU2(i, node_id)                                                   \
//	COND_CODE_1(DT_NODE_HAS_STATUS(DT_PHANDLE_BY_IDX(node_id, cpu_power_states, i), okay),    \
//		    (PM_STATE_INFO_DT_INIT2(DT_PHANDLE_BY_IDX(node_id, cpu_power_states, i)),), ())
//
//
//#define PM_STATE_INFO_LIST_FROM_DT_CPU2(node_id)				       \
//	{								       \
//		LISTIFY(DT_PROP_LEN_OR(node_id, cpu_power_states, 0),	       \
//			Z_PM_STATE_INFO_FROM_DT_CPU2, (), node_id)	       \
//	}
//
//#define DEFINE_CPU_STATES(n) \
//	static const struct qn9090_config qn9090_config_##n[] \
//		= PM_STATE_INFO_LIST_FROM_DT_CPU2(n);
//#define CPU_STATE_REF(n) pmstates_##n
/**
 * @brief Check CPU power states consistency
 *
 * All states should have a minimum residency >= than the exit latency.
 *
 * @param node_id A CPU node identifier.
 */

#define DEFINE_CPU_STATES(n) \
	static const struct pm_state_info pmstates_##n[] \
		= PM_STATE_INFO_LIST_FROM_DT_CPU(n);
#define CPU_STATE_REF(n) pmstates_##n

DT_FOREACH_CHILD(DT_PATH(cpus), DEFINE_CPU_STATES);

/** CPU power states information for each CPU */
static const struct pm_state_info *cpus_states[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_PATH(cpus), CPU_STATE_REF, (,))
};

/** Number of states for each CPU */
static const uint8_t states_per_cpu[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_PATH(cpus), DT_NUM_CPU_POWER_STATES, (,))
};

uint8_t pm_state_cpu_get_all(uint8_t cpu, const struct pm_state_info **states)
{
	if (cpu >= ARRAY_SIZE(cpus_states)) {
		return 0;
	}

	*states = cpus_states[cpu];

	return states_per_cpu[cpu];
}
