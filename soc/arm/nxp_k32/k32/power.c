#include <zephyr/kernel.h>

#include <zephyr/pm/pm.h>
#include <device_power.h>

#include <fsl_wtimer.h>
#include <rom_api.h>

#include <zephyr/logging/log.h>
//LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#define BUTTON 0
#define TIMER 1
#define BUTTON_TIMER 2
uint32_t irq_lock_key;

extern uint32_t                   _end_boot_resume_stack;
#define RESUME_STACK_POINTER      ((uint32_t)&_end_boot_resume_stack)
#define PWR_JUMP_BUF_SIZE         10

typedef unsigned long PWR_Register;
typedef PWR_Register PWR_jmp_buf[PWR_JUMP_BUF_SIZE];
PWR_jmp_buf                   pwr_CPUContext;

#define CONFIG_SUBSTATE_TEST 1

#if CONFIG_SUBSTATE_TEST

struct qn9090_config {
    uint8_t state;
    uint8_t substate_id;
    pm_power_config_t config;
};

#define PM_STATE_INFO_DT_INIT2(node_id)					       \
                                                  \
        {                                       \
        .state = PM_STATE_DT_INIT(node_id),                   \
        .substate_id = DT_PROP_OR(node_id, substate_id, 0),           \
        .config.pm_config = (COND_CODE_1(DT_PROP_OR(node_id, retain_radio_device, false), (PM_CFG_RADIO_RET), (0)) \
                            |COND_CODE_1(DT_PROP_OR(node_id, retain_ram_domain, false), (PM_CFG_SRAM_ALL_RETENTION), (0))),\
        .config.pm_wakeup_src = (COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false), (POWER_WAKEUPSRC_IO), (0)) \
                                 | COND_CODE_1(DT_PROP_OR(node_id, timer_wakeup, false), (POWER_WAKEUPSRC_WAKE_UP_TIMER0), (0))),\
        .config.pm_wakeup_io = (COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false), (1 << BOARD_SW1_GPIO_PIN), (0)) \
                                 | COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false), (1 << BOARD_SW1_GPIO_PIN), (0))   \
                                 | COND_CODE_1(DT_PROP_OR(node_id, gpio_wakeup, false), (1 << BOARD_SW1_GPIO_PIN), (0)))\
                                 }



#define POWER_LABEL DT_NODELABEL(power_states)
static const struct qn9090_config qn9090_config[] = { DT_FOREACH_CHILD_SEP(POWER_LABEL, PM_STATE_INFO_DT_INIT2, (,))};

#else

#define NODE_ID_SUSTORAM DT_NODELABEL(state3)

#endif

#define UNUSED(x)

__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{

    bool ret;
    //irq_lock_key = irq_lock();

    #if CONFIG_SUBSTATE_TEST

    //TODO: substate_id komt niet per se overeen met locatie in array

    int max_state = 0;
    //uint8_t state_count = 0 DT_FOREACH_CHILD_SEP(POWER_LABEL,UNUSED,+1); //zet gelijk aan hoeveelheid power modes (kernel bevat hier iets over?)
    //__disable_irq();

    uint8_t state_count = DT_NUM_CPU_POWER_STATES(DT_PATH(cpus));


    while(1){

        //TODO: failsafe if a state without substate is selected. Substate in config generation defaults to 0, but what happens in pm_next_state?

//        if(qn9090_config[state_count].state == state && qn9090_config[state_count].substate_id == substate_id){
//            ret = POWER_EnterPowerMode(PM_POWER_DOWN, &qn9090_config[state_count].config);
//        }

        if((state == PM_STATE_SUSPEND_TO_RAM || PM_STATE_SUSPEND_TO_DISK) && qn9090_config[state_count].substate_id == substate_id){

            //BOOT_SetResumeStackPointer(RESUME_STACK_POINTER);
            //SYSCON->CPSTACK = RESUME_STACK_POINTER;
            if ( 0 == PWR_setjmp(pwr_CPUContext))
            {
                ret = POWER_EnterPowerMode(PM_POWER_DOWN, &qn9090_config[state_count].config);
            }

            break;

        }else if((state == PM_STATE_STANDBY || PM_STATE_SUSPEND_TO_IDLE) && qn9090_config[state_count].substate_id == substate_id){
            //ret = POWER_EnterPowerMode(PM_DEEP_SLEEP, &qn9090_config[state_count].config);
            break;

        }else if(state == PM_STATE_RUNTIME_IDLE && qn9090_config[state_count].substate_id == substate_id) {
            uint32_t basepri = __get_BASEPRI();
            __set_BASEPRI(0);
            POWER_EnterSleep();
            __set_BASEPRI(basepri);
            break;
        }

        state_count++; //TODO: zorg dat terugtelt, maar ook dat geheugen niet fucky gaat omdat qn9090_config[n], n>child_n niet bestaat
        //zorg ook dat while loop leaved, want nu is het oneindig
    }



//    switch (substate_id) {
//
//        //#define CASE_MACRO(node_id) case DT_PROP(node_id, substate-id): break;
//
//        DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_PATH(cpus), CASE_MACRO, (,))
//

    #else

    ARG_UNUSED(substate_id);
    pm_power_config_t config = {};

    switch (state) {

        case PM_STATE_ACTIVE:

            break;
        case PM_STATE_RUNTIME_IDLE:

            break;
        case PM_STATE_SUSPEND_TO_IDLE:

            break;
        case PM_STATE_STANDBY:

            break;
        case PM_STATE_SUSPEND_TO_RAM:

            if (DT_PROP_OR(NODE_ID_SUSTORAM, retain_ram_domain, false)) {
                config.pm_config |= PM_CFG_SRAM_ALL_RETENTION;
            }
            if (DT_PROP_OR(NODE_ID_SUSTORAM, retain_radio_device, false)) {
                config.pm_config |= PM_CFG_RADIO_RET;           //Behouden radio instellingen
            }
            if (DT_PROP_OR(NODE_ID_SUSTORAM, gpio_wakeup, false)) {
                config.pm_wakeup_src |= POWER_WAKEUPSRC_IO;
                config.pm_wakeup_io |= 1 << BOARD_SW1_GPIO_PIN;
            }
            if (DT_PROP_OR(NODE_ID_SUSTORAM, timer_wakeup, false)) {
                config.pm_wakeup_src |= POWER_WAKEUPSRC_WAKE_UP_TIMER0;
            }

            ret = POWER_EnterPowerMode(PM_POWER_DOWN, &config);
            break;

        case PM_STATE_SOFT_OFF:

            break;
        default:
            //LOG_DBG("Unsupported power state %u", state);
            //printk("\r\n!!Unsupported power state!!\r\n\r\n");
            //printk("Unsupported power state %u", state);
            break;

    }
    #endif
}


__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(substate_id);
    //irq_unlock(irq_lock_key);

    switch (state) {
        case PM_STATE_ACTIVE:

      //      printk("Exiting PM_STATE_ACTIVE\n");
            break;
        case PM_STATE_RUNTIME_IDLE:
        //    printk("Exiting PM_STATE_RUNTIME_IDLE\n");
            break;

        case PM_STATE_STANDBY:

          //  printk("Exiting PM_STATE_STANDBY\n");
            break;

        case PM_STATE_SOFT_OFF:

            break;
        default:
            //LOG_DBG("Unsupported power state %u", state);
            //printk("Unsupported power state %u", state);
            break;
    }


    //irq_unlock(0); //System is now in active mode. Reenable interrupts which were disabled when OS started idling code.
    irq_unlock(irq_lock_key);
    //printk("Exiting post pm ops, ints reenabled\n");
}

void WarmMain(void)
{
    __disable_irq();
    PWR_longjmp(pwr_CPUContext, 1); //voor context restoration. zoek uit waar dit moet. Mogelijk in warmmain
    printk("\n!!warmmain called!!\n");
}