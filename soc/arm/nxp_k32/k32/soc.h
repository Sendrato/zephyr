/*
 * Copyright (c) 2021, Vincent van der Locht
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifndef _ASMLANGUAGE
#include <sys/util.h>


#include <fsl_common.h>

/* Add include for DTS generated information */
#include <devicetree.h>

#endif /* !_ASMLANGUAGE */
#define IOCON_PIO_DIGITAL_EN	0x80u
#define IOCON_PIO_FUNC0		0x00u
#define IOCON_PIO_FUNC1		0x01u
#define IOCON_PIO_FUNC2		0x02u
#define IOCON_PIO_FUNC3		0x03u
#define IOCON_PIO_FUNC4		0x04u
#define IOCON_PIO_FUNC5		0x05u
#define IOCON_PIO_FUNC6		0x06u
#define IOCON_PIO_FUNC7		0x07u
#define IOCON_PIO_I2CDRIVE_LOW	0x00u
#define IOCON_PIO_I2CFILTER_EN	0x00u
#define IOCON_PIO_I2CSLEW_I2C	0x00u
#define IOCON_PIO_INPFILT_OFF	0x0100u
#define IOCON_PIO_INV_DI	0x00u
#define IOCON_PIO_MODE_INACT	0x00u
#define IOCON_PIO_OPENDRAIN_EN	0x400u
#define IOCON_PIO_SLEW_STANDARD	0x00u
#define IOCON_PIO_SLEW_FAST     0x20u
#define IOCON_PIO_SLEW_SUPERFAST 0x220u
#define IOCON_PIO_MODE_PULLUP	0x00u
#define IOCON_PIO_MODE_PLAIN	0x10u
#define IOCON_PIO_MODE_REPEAT	0x08u
#define IOCON_PIO_MODE_PULLDOWN	0x18u


#define IOCON_PIO_I2C_PIN_MODE_PULLUP	0x18u
#define IOCON_PIO_I2C_PIN_MODE_NORMAL	0x08u

#endif /* _SOC__H_ */
