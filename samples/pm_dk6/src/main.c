
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/sys/printk.h>
#include <fsl_gpio.h>



static struct pm_state_info pm_info_standby = {PM_STATE_STANDBY, 0, 0, 0};

int main(void)
{

    LED_0_ON();
    printk("2 second cooldown\n");
    k_sleep(K_MSEC(2000));
    LED_0_OFF();
    printk("Entering sleep in 4 seconds\n"); //printen vlak voor slaapstand gaat niet, heeft delay nodig
    k_sleep(K_MSEC(2000));
    LED_0_ON();
    k_sleep(K_MSEC(2000));
    pm_state_force(0u, &pm_info_standby); //put delay before forcing state, otherwise tasks arent finished.
    LED_0_ON();

    while(1){
        k_sleep(K_MSEC(2000));
        printk("AWAKE");
    }
}

void WarmMain(void)
{
    main();
}