//
// Created by ijsbrand on 1-11-23.
//

#include <device_power.h>


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
