/*
 * Copyright (c) 2021, Vincent van der Locht
 * Copyright (c) 2022, Hessel van der Molen, sendrato.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <fsl_common.h>
#include <fsl_iocon.h>
#include <soc.h>


static int dk6_qn9090_pinmux_init(const struct device *dev)
{
    ARG_UNUSED(dev);

    const struct device *port0 = DEVICE_DT_GET(DT_NODELABEL(pio0));

    const uint32_t port0_pin12_config = (/* Pin is configured as SWCLK */
            IOCON_PIO_FUNC2 |
            /* Selects pull-up function */
            IOCON_PIO_MODE_PULLUP |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin13_config = (/* Pin is configured as SWDIO */
            IOCON_PIO_FUNC2 |
            /* Selects pull-up function */
            IOCON_PIO_MODE_PULLUP |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    pinmux_pin_set(port0, 12, port0_pin12_config);
    pinmux_pin_set(port0, 13, port0_pin13_config);

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm0), nxp_lpc_usart, okay) && CONFIG_SERIAL

    const uint32_t port0_pin8_config = (/* Pin is configured as USART0_TXD */
            IOCON_PIO_FUNC2 |
            /* Selects pull-up function */
            IOCON_PIO_MODE_PULLUP |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin9_config = (/* Pin is configured as USART0_RXD */
            IOCON_PIO_FUNC2 |
            /* Selects pull-up function */
            IOCON_PIO_MODE_PULLUP |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    pinmux_pin_set(port0, 8, port0_pin8_config);
    pinmux_pin_set(port0, 9, port0_pin9_config);
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm2), nxp_lpc_i2c, okay) && CONFIG_I2C

    const uint32_t port0_pin10_config = (/* Pin is configured as I2C0_SCL */
                                         IOCON_PIO_FUNC5 |
                                         /* I2C mode */
                                         IOCON_PIO_EGP_I2C |
                                         /* IO is an open drain cell */
                                         IOCON_PIO_ECS_DI |
                                         /* High speed IO for GPIO mode, IIC not */
                                         IOCON_PIO_EHS_DI |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* IIC mode:Noise pulses below approximately 50ns are filtered out. GPIO mode:a 3ns filter */
                                         IOCON_PIO_FSEL_DI |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* IO_CLAMP disabled */
                                         IOCON_PIO_IO_CLAMP_DI);

    const uint32_t port0_pin11_config = (/* Pin is configured as I2C0_SDA */
                                         IOCON_PIO_FUNC5 |
                                         /* I2C mode */
                                         IOCON_PIO_EGP_I2C |
                                         /* IO is an open drain cell */
                                         IOCON_PIO_ECS_DI |
                                         /* High speed IO for GPIO mode, IIC not */
                                         IOCON_PIO_EHS_DI |
                                         /* Input function is not inverted */
                                         IOCON_PIO_INV_DI |
                                         /* Enables digital function */
                                         IOCON_PIO_DIGITAL_EN |
                                         /* Input filter disabled */
                                         IOCON_PIO_INPFILT_OFF |
                                         /* IIC mode:Noise pulses below approximately 50ns are filtered out. GPIO mode:a 3ns filter */
                                         IOCON_PIO_FSEL_DI |
                                         /* Open drain is disabled */
                                         IOCON_PIO_OPENDRAIN_DI |
                                         /* IO_CLAMP disabled */
                                         IOCON_PIO_IO_CLAMP_DI);

    pinmux_pin_set(port0, 10, port0_pin10_config);
    pinmux_pin_set(port0, 11, port0_pin11_config);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(qspi), okay) && CONFIG_SPI

    const uint32_t port0_pin16_config = (/* Pin is configured as SPIFI_CSN */
            IOCON_PIO_FUNC7 |
            /* Selects pull-up function */
            IOCON_PIO_MODE_PULLUP |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin17_config = (/* Pin is configured as SPIFI_IO3 */
            IOCON_PIO_FUNC7 |
            /* Selects pull-down function */
            IOCON_PIO_MODE_PULLDOWN |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin18_config = (/* Pin is configured as SPIFI_CLK */
            IOCON_PIO_FUNC7 |
            /* Selects pull-down function */
            IOCON_PIO_MODE_PULLDOWN |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin19_config = (/* Pin is configured as SPIFI_IO0 */
            IOCON_PIO_FUNC7 |
            /* Selects pull-down function */
            IOCON_PIO_MODE_PULLDOWN |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin20_config = (/* Pin is configured as SPIFI_IO2 */
            IOCON_PIO_FUNC7 |
            /* Selects pull-down function */
            IOCON_PIO_MODE_PULLDOWN |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);

    const uint32_t port0_pin21_config = (/* Pin is configured as SPIFI_IO1 */
            IOCON_PIO_FUNC7 |
            /* Selects pull-up function */
            IOCON_PIO_MODE_PULLUP |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW0_STANDARD |
            /* Input function is not inverted */
            IOCON_PIO_INV_DI |
            /* Enables digital function */
            IOCON_PIO_DIGITAL_EN |
            /* Input filter disabled */
            IOCON_PIO_INPFILT_OFF |
            /* Standard mode, output slew rate control is disabled */
            IOCON_PIO_SLEW1_STANDARD |
            /* Open drain is disabled */
            IOCON_PIO_OPENDRAIN_DI |
            /* SSEL is disabled */
            IOCON_PIO_SSEL_DI);


    /* PORT0 PIN16 (coords: 19) is configured as SPIFI_CSN */
    pinmux_pin_set(port0, 16U, port0_pin16_config);
    /* PORT0 PIN17 (coords: 21) is configured as SPIFI_IO3 */
    pinmux_pin_set(port0, 17U, port0_pin17_config);
    /* PORT0 PIN18 (coords: 22) is configured as SPIFI_CLK */
    pinmux_pin_set(port0, 18U, port0_pin18_config);
    /* PORT0 PIN19 (coords: 23) is configured as SPIFI_IO0 */
    pinmux_pin_set(port0, 19U, port0_pin19_config);
    /* PORT0 PIN20 (coords: 24) is configured as SPIFI_IO2 */
    pinmux_pin_set(port0, 20U, port0_pin20_config);
    /* PORT0 PIN21 (coords: 25) is configured as SPIFI_IO1 */
    pinmux_pin_set(port0, 21U, port0_pin21_config);

#endif


    return 0;
}


SYS_INIT(dk6_qn9090_pinmux_init, PRE_KERNEL_1,
         CONFIG_PINMUX_INIT_PRIORITY);
