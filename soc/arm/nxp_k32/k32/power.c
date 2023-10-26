
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <fsl_power.h>

#include <zephyr/logging/log.h>
//LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

#include <zephyr/sys/printk.h>
#include <fsl_gpio.h>
extern WEAK void WarmMain(void);

uint32_t irq_lock_key;

#define BOARD_SW1_GPIO_PIN 1U
#define LED_PORT_0 0
#define LED_PORT_1 0
#define LED_PIN_0 0
#define LED_PIN_1 3


void LED_0_ON(){
    GPIO_PinInit(GPIO, LED_PORT_0, LED_PIN_0, &((const gpio_pin_config_t){kGPIO_DigitalOutput, 0}));
}

void LED_1_ON(){
    GPIO_PinInit(GPIO, LED_PORT_1, LED_PIN_1, &((const gpio_pin_config_t){kGPIO_DigitalOutput, 0}));
}

void LED_0_OFF(){
    GPIO_PinInit(GPIO, LED_PORT_0, LED_PIN_0, &((const gpio_pin_config_t){kGPIO_DigitalOutput, 1}));
}

void LED_1_OFF(){
    GPIO_PinInit(GPIO, LED_PORT_1, LED_PIN_1, &((const gpio_pin_config_t){kGPIO_DigitalOutput, 1}));
}

__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    if ((void (*)(void))WarmMain != NULL){
        printk("test");
    }else{
        printk("tset");
    }
    ARG_UNUSED(substate_id);

    bool ret;

    pm_power_config_t config = {0};
    config.pm_config |= PM_CFG_RADIO_RET;           //Behouden radio instellingen
    config.pm_config |= PM_CFG_SRAM_ALL_RETENTION;  //behouden SRAM

    //config.pm_wakeup_src = POWER_WAKEUPSRC_IO || POWER_WAKEUPSRC_WAKE_UP_TIMER0; //TODO: inits?
    config.pm_wakeup_src |= POWER_WAKEUPSRC_IO;

    config.pm_wakeup_io |= 1 << BOARD_SW1_GPIO_PIN;//TODO: wat doet dit? VGM de GPIO die als wakeup gebruikt wordt

    //TODO: disable interrupts? Interrupts are enabled @ line 73 ish.
    //printk("starting case-statement for mode %u\n", state);


    switch (state) {

        case PM_STATE_ACTIVE:

            break;
        case PM_STATE_RUNTIME_IDLE:

            break;
        case  PM_STATE_SUSPEND_TO_IDLE:

            break;
        case PM_STATE_STANDBY:

            irq_lock_key = irq_lock(); //new

            ret = POWER_EnterPowerMode(PM_POWER_DOWN, &config);

            if (ret == false) {
                //printk("\r\n!!ERROR WHEN GOING TO SLEEP!!\r\n\r\n");
            }

            break;
        case PM_STATE_SUSPEND_TO_RAM:

            break;
        case PM_STATE_SOFT_OFF:

            break;
        default:
            //LOG_DBG("Unsupported power state %u", state);
            //printk("\r\n!!Unsupported power state!!\r\n\r\n");
            //printk("Unsupported power state %u", state);
            break;
    }
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

