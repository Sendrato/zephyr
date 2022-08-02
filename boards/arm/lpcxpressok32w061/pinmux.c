/*
 * Copyright (c) 2021, Vincent van der Locht
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <fsl_common.h>
#include <fsl_iocon.h>
#include <soc.h>


static int lpcxpresso_k32w061_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#ifdef CONFIG_PINMUX_MCUX_LPC_PORT0
	const struct device *port0 =
		device_get_binding(CONFIG_PINMUX_MCUX_LPC_PORT0_NAME);
#endif

    const uint32_t port0_pin12_config = ( /* Pin is configured as SWCLK */
            IOCON_PIO_FUNC2 |
            IOCON_PIO_MODE_PLAIN |
            IOCON_PIO_SLEW_STANDARD |
            IOCON_PIO_DIGITAL_EN |
            IOCON_PIO_INPFILT_OFF
            );

    const uint32_t port0_pin13_config = (/* Pin is configured as SWDIO */
            IOCON_PIO_FUNC2 |
            IOCON_PIO_MODE_PLAIN |
            IOCON_PIO_SLEW_STANDARD |
            IOCON_PIO_DIGITAL_EN |
            IOCON_PIO_INPFILT_OFF
            );

	pinmux_pin_set(port0, 12, port0_pin12_config);
	pinmux_pin_set(port0, 13, port0_pin13_config);

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm0), nxp_lpc_usart, okay) && CONFIG_SERIAL
	/* USART0 RX,  TX */
	const uint32_t port0_pin8_config = (
            IOCON_PIO_FUNC2 |
            IOCON_PIO_MODE_PLAIN |
            IOCON_PIO_SLEW_STANDARD |
            IOCON_PIO_DIGITAL_EN |
            IOCON_PIO_INPFILT_OFF
			);

	const uint32_t port0_pin9_config = (
            IOCON_PIO_FUNC2 |
            IOCON_PIO_MODE_PLAIN |
            IOCON_PIO_SLEW_STANDARD |
            IOCON_PIO_DIGITAL_EN |
            IOCON_PIO_INPFILT_OFF
			);

	pinmux_pin_set(port0, 8, port0_pin8_config);
	pinmux_pin_set(port0, 9, port0_pin9_config);

#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm3), nxp_lpc_i2c, okay) && CONFIG_I2C
	/* I2C1 SCL, SDA */
	const uint32_t port0_pin6_config = (
            IOCON_PIO_FUNC5 |
            // IOCON_PIO_MODE_PULLUP |
            // IOCON_PIO_SLEW_STANDARD |
            IOCON_PIO_DIGITAL_EN
            // IOCON_PIO_INPFILT_OFF
			);

	const uint32_t port0_pin7_config = (
            IOCON_PIO_FUNC5 |
            // IOCON_PIO_MODE_PULLUP |
            // IOCON_PIO_SLEW_STANDARD |
            IOCON_PIO_DIGITAL_EN
            // IOCON_PIO_INPFILT_OFF
			);

	pinmux_pin_set(port0, 6, port0_pin6_config);
	pinmux_pin_set(port0, 7, port0_pin7_config);

#endif


// #if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm4), nxp_lpc_i2c, okay) && CONFIG_I2C
// 	/* PORT0 PIN25 is configured as FC4_RTS_SCL_SSEL1 */
// 	pinmux_pin_set(port0, 25, IOCON_PIO_FUNC1 |
// 				  IOCON_PIO_I2CSLEW_I2C |
// 				  IOCON_PIO_INV_DI |
// 				  IOCON_PIO_DIGITAL_EN |
// 				  IOCON_PIO_INPFILT_OFF |
// 				  IOCON_PIO_I2CDRIVE_LOW |
// 				  IOCON_PIO_I2CFILTER_EN);

// 	/* PORT0 PIN26 is configured as FC4_CTS_SDA_SSEL0 */
// 	pinmux_pin_set(port0, 26, IOCON_PIO_FUNC1 |
// 				  IOCON_PIO_I2CSLEW_I2C |
// 				  IOCON_PIO_INV_DI |
// 				  IOCON_PIO_DIGITAL_EN |
// 				  IOCON_PIO_INPFILT_OFF |
// 				  IOCON_PIO_I2CDRIVE_LOW |
// 				  IOCON_PIO_I2CFILTER_EN);
// #endif

// #if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm5), nxp_lpc_spi, okay) && CONFIG_SPI
// 	/* PORT0 PIN18 is configured as FC5_TXD_SCL_MISO */
// 	pinmux_pin_set(port0, 18, IOCON_PIO_FUNC1 |
// 				  IOCON_PIO_MODE_PULLUP |
// 				  IOCON_PIO_INV_DI |
// 				  IOCON_PIO_DIGITAL_EN |
// 				  IOCON_PIO_SLEW_STANDARD |
// 				  IOCON_PIO_OPENDRAIN_DI);

// 	/* PORT0 PIN19 is configured as FC5_SCK-SPIFI_CSn */
// 	pinmux_pin_set(port0, 19, IOCON_PIO_FUNC1 |
// 				  IOCON_PIO_MODE_PULLUP |
// 				  IOCON_PIO_INV_DI |
// 				  IOCON_PIO_DIGITAL_EN |
// 				  IOCON_PIO_SLEW_STANDARD |
// 				  IOCON_PIO_OPENDRAIN_DI);

// 	/* PORT0 PIN20 is configured as FC5_RXD_SDA_MOSI */
// 	pinmux_pin_set(port0, 20, IOCON_PIO_FUNC1 |
// 				  IOCON_PIO_MODE_PULLUP |
// 				  IOCON_PIO_INV_DI |
// 				  IOCON_PIO_DIGITAL_EN |
// 				  IOCON_PIO_SLEW_STANDARD |
// 				  IOCON_PIO_OPENDRAIN_DI);

// 	/* PORT1 PIN1 is configured as FC5_SSEL2 */
// 	pinmux_pin_set(port1,  1, IOCON_PIO_FUNC4 |
// 				  IOCON_PIO_MODE_PULLUP |
// 				  IOCON_PIO_INV_DI |
// 				  IOCON_PIO_DIGITAL_EN |
// 				  IOCON_PIO_SLEW_STANDARD |
// 				  IOCON_PIO_OPENDRAIN_DI);
// #endif

	return 0;
}

SYS_INIT(lpcxpresso_k32w061_pinmux_init,  PRE_KERNEL_1,
	 CONFIG_PINMUX_INIT_PRIORITY);
