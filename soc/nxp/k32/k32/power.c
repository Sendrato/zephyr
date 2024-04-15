#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>

#include <fsl_power.h>
#include <board.h>
#include <rom_api.h>
#include <fsl_clock.h>

/*
 * Struct for storing all power-management information relating to a single given state.
 * This struct is passed to low-level API's to set configs.
 */

struct qn9090_pm_config {
    uint8_t state;
    uint8_t substate_id;
    pm_power_config_t config;
};

/*
 * Reads and sets the properties, which describe sleep-configurations
 * set in the .overlay file of the compiled devicetree as struct members.
 */

#define PM_STATE_INFO_DT_INIT2(node_id)					\
									\
	{								\
	.state = PM_STATE_DT_INIT(node_id),				\
	.substate_id = DT_PROP_OR(node_id, substate_id, 0),		\
	.config.pm_config =						\
	(COND_CODE_1(DT_PROP_OR(node_id, retain_radio_device, false),	\
	(PM_CFG_RADIO_RET), (0))|					\
	COND_CODE_1(DT_PROP_OR(node_id, retain_radio_device, false),	\
	(PM_CFG_XTAL32M_AUTOSTART), (0))|				\
	COND_CODE_1(DT_PROP_OR(node_id, retain_ram_domain, false),	\
	(PM_CFG_SRAM_ALL_RETENTION), (0))),				\
	.config.pm_wakeup_src =						\
	(COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false),		\
	(POWER_WAKEUPSRC_IO), (0))|					\
	COND_CODE_1(DT_PROP_OR(node_id, timer_wakeup, false),		\
	(POWER_WAKEUPSRC_WAKE_UP_TIMER0), (0))),			\
	.config.pm_wakeup_io =						\
	(COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false),		\
	(1 << BOARD_SW1_GPIO_PIN), (0))					\
	|COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false),		\
	(1 << BOARD_SW1_GPIO_PIN), (0))					\
	|COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false),		\
	(1 << BOARD_SW1_GPIO_PIN), (0)))				\
	}


#define POWER_LABEL DT_NODELABEL(power_states)

/*
 * The macro DT_FOREACH_CHILD_SEP parses the overlay file and invokes
 * PM_STATE_INFO_DT_INIT2 with the DTS-id of each power state as an argument for each power state.
 * The ID that corresponds to a power state is then used to parse the contents of each power state.
 * These contents consist of properties corresponding with configurations
 * for each power state, which are set in the .overlay file.
 */

static struct qn9090_pm_config qn9090_pm_config[] = {
	DT_FOREACH_CHILD_SEP(POWER_LABEL, PM_STATE_INFO_DT_INIT2, (,))};

/*
 * Global variable used to specify a power state from the qn9090_pm_config
 * struct array to pass to the low-level API. This is set by zephyr during runtime.
 *
 * This is required because the pm_s2ram API requires a wrapper
 * function in which a config-array has to be passed.
 * The variable is set during runtime and passed to the wrapper.
 * For more information, read the comment below.
 */

int set_sleep_config;

/*
 * Separate function for sleep mode used in pm_s2ram API,
 * because arch_pm_s2ram_suspend requires as argument function with no arguments.
 * The power_fsl API does require a function with arguments,
 * so a wrapper function with no arguments is required.
 */

int PowerDownWrapper(void)
{

	int ret;

	ret = POWER_EnterPowerMode(PM_POWER_DOWN, &qn9090_pm_config[set_sleep_config].config);

	return ret;
}

__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{

	/*Converts the amount of ticks the kernel is scheduled to be idle for to microseconds*/
	uint64_t idle_time_us = k_ticks_to_us_near64((uint64_t) _kernel.idle);

	printk("\n Kernel will be idle for: %d \n", _kernel.idle);

	/*Converts idle time from microseconds to seconds*/
	double idle_time_s = (double) idle_time_us / (double) 1000000;

	/*Reset the wakeup timer peripheral. This is required for proper operation.*/
	reset_wkt();

	/*substates start at 1, but config array starts at entry 0*/
	set_sleep_config = substate_id - 1;

	/*
	 * Entering deep sleep or higher sets the system clock to 12mHz after wakeup,
	 * so the clock needs to be restored, which will be done with save_clock
	 */

	/*TODO: Make it so the clock type is remembered instead of hard coding it as 48MHz*/
	enum _clock_name save_clock = CLOCK_GetFreq(kCLOCK_MainClk);

	/*
	* Set BASEPRI to 0 so that an interrupt of any priority
	* can wake the system. BASEPRI should be kept at this
	* value in this function, as interrupts need to be
	* enabled when it exits as described in idle.c line 78.
	*/

	__disable_irq();
	uint8_t basepri = __get_BASEPRI();
	__set_BASEPRI(0U);

	if (state == PM_STATE_RUNTIME_IDLE) {

		/*Enter sleep*/
		POWER_EnterSleep();

	} else if (state == PM_STATE_SUSPEND_TO_IDLE) {

		/*Sets a time for the wakeup timer to cause an interrupt*/

		/*
		 * TODO: If sleep times smaller than 30µs, or a resolution higher
		 * than that are required: use a higher frequency
		 * clock for the wakeup timer.
		 */

		init_config_timer(idle_time_s);

		/*Enter sleep and restore BASEPRI afterwards*/
		bool ret = POWER_EnterPowerMode(PM_DEEP_SLEEP,
			&qn9090_pm_config[set_sleep_config].config);

		/*
		 * Variable ret is part of the API, and should
		 * return false if system couldn't go into sleep mode.
		 * Can be used for debugging.
		 */

		ARG_UNUSED(ret);


		/* will vector to ISR here once if PRIMASK = 0 before sleep call*/

		/*
		 * After wakeup from sleep, a 12 mHz clock is set
		 * as the main system clock.
		 * The two functions below restore it to its previous state.
		 */

		CLOCK_AttachClk(kFRO48M_to_MAIN_CLK);
		CLOCK_AttachClk(kMAIN_CLK_to_ASYNC_APB);


		/*
		 *TODO: Implementable state when zephyr supports
		 * context restoration for armv7-m archs.
		 * This mode seems to break debugging so it's a challenge.
		 */

		/*
		} else if (state == PM_STATE_SUSPEND_TO_RAM) {

			break;

			init_config_timer(idle_time_s);

			arch_pm_s2ram_suspend(PowerDownWrapper);

			CLOCK_AttachClk(kFRO48M_to_MAIN_CLK);
			CLOCK_AttachClk(kMAIN_CLK_to_ASYNC_APB);

		} else {
			break;
		*/
	}
	__set_BASEPRI(basepri);
}

__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	/*uninits wakeup timer*/
	deinit_config_timer();
	SCB->SCR=0;

	__enable_irq();
	irq_unlock(0);
}