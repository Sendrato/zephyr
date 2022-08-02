/*
 * Copyright (c) 2017, NXP
 * Copyright (c) 2021, Vincent van der Locht
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for nxp_k32w platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the nxp_k32w platform.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include <aarch32/cortex_m/exc.h>
#include <fsl_power.h>
#include <fsl_clock.h>
#include <fsl_common.h>
#include <fsl_device_registers.h>
#include <fsl_pint.h>
#include <fsl_flash.h>

#if defined(CONFIG_BT)
#include <radio.h>
#endif

#define BOARD_MAINCLK_FRO12M            0U
#define BOARD_MAINCLK_XTAL32M           2U
#define BOARD_MAINCLK_FRO32M            3U
#define BOARD_MAINCLK_FRO48M            4U

/**
 *
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */
#define CPU_FREQ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

void z_platform_init(void)
{
 
    // Disable interrupts
    __asm volatile ("cpsid i");


     // Enable SRAM clock used by Stack
    __asm volatile ("LDR R0, =0x40000220\n\t"
                    "MOV R1, #56\n\t"
                    "STR R1, [R0]");

    __asm volatile(
        ".set   cpu_ctrl,       0x40000800\t\n"
        ".set   coproc_boot,    0x40000804\t\n"
        ".set   coproc_stack,   0x40000808\t\n"
        "LDR    R0,=coproc_boot\t\n"          // load co-processor boot address (from CPBOOT)
        "LDR    R0,[R0]\t\n"                  // get address to branch to
        "MOVS   R0,R0\t\n"                    // Check if 0
        "BEQ.N  masterboot\t\n"               // if zero in boot reg, we just branch to  real reset
        "LDR    R1,=coproc_stack\t\n"         // load co-processor stack pointer (from CPSTACK)
        "LDR    R1,[R1]\t\n"
        "MOV    SP,R1\t\n"
        "BX     R0\t\n"                       // branch to boot address
        "masterboot:\t\n");

    // Reenable interrupts
    __asm volatile ("cpsie i");

}

static ALWAYS_INLINE void clock_init(void)
{

#if defined(CONFIG_SOC_K32W061) || defined(CONFIG_SOC_QN9090)

    /* Force clock to switch to FRO32M to speed up initialization */
    SYSCON -> MAINCLKSEL = BOARD_MAINCLK_FRO32M;

    /* MODEM master priority = 3 - highest */
    SYSCON -> AHBMATPRIO = 0x00000300;

    /* Security code to allow debug access */
    SYSCON->CODESECURITYPROT = 0x87654320;

    /* In the absence of a proper setting in SHCSR, all faults go to a HardFault
     * Since we want a finer discrimination for BusFaults in order to catch Faults when
     * accessing the Flash on pages in error, lets set  BUSFAULTENA.
     * The other may turnout usefull later
     */
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk ;
    // SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk ;
    // SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk ;

    SYSCON->ASYNCAPBCTRL |= ((1 << SYSCON_ASYNCAPBCTRL_ENABLE_SHIFT) & SYSCON_ASYNCAPBCTRL_ENABLE_MASK);

    CLOCK_EnableClock(kCLOCK_Xtal32k);

    /* BOARD_BootClockRUN(..) */

    /*!< Set up the clock sources */
    CLOCK_EnableClock(kCLOCK_Fro12M);  /*!< Ensure FRO 12MHz is on */
    CLOCK_EnableClock(kCLOCK_Fro32M);  /*!< Ensure FRO 32MHz is on */
    CLOCK_EnableClock(kCLOCK_Fro48M);  /*!< Ensure FRO 48MHz is on */
    CLOCK_EnableAPBBridge();           /*!< The Async_APB clock is enabled. */
    CLOCK_EnableClock(kCLOCK_Xtal32M); /*!< Enable XTAL 32 MHz output */
    /*!< Configure RTC OSC */
    CLOCK_EnableClock(kCLOCK_Fro32k);       /*!< Enable RTC FRO 32 KHz output */
    CLOCK_AttachClk(kFRO32K_to_OSC32K_CLK); /*!< Switch OSC32K to FRO32K */
    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivRtcClk, 1U, false);     /*!< Set RTCCLKDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);     /*!< Set AHBCLKDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivSystickClk, 1U, false); /*!< Set SYSTICKCLKDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivTraceClk, 1U, false);   /*!< Set TRACECLKDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivSpifiClk, 2U, false);   /*!< Set SPIFICLKDIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivDmicClk, 1U, false);    /*!< Set DMICCLKDIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivWdtClk, 1U, true);      /*!< Set WDTCLKDIV divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO48M_to_MAIN_CLK);    /*!< Switch MAIN_CLK to FRO48M */
    CLOCK_AttachClk(kMAIN_CLK_to_ASYNC_APB); /*!< Switch ASYNC_APB to MAIN_CLK */
    CLOCK_AttachClk(kFRO32M_to_OSC32M_CLK);  /*!< Switch OSC32M_CLK to FRO32M */
    CLOCK_AttachClk(kFRO32K_to_OSC32K_CLK);  /*!< Switch OSC32K_CLK to FRO32K */
    CLOCK_AttachClk(kOSC32M_to_USART_CLK);   /*!< Switch USART_CLK to OSC32M */
    CLOCK_AttachClk(kMAIN_CLK_to_SPIFI);     /*!< Switch SPIFI to MAIN_CLK */
    CLOCK_AttachClk(kMAIN_CLK_to_DMI_CLK);   /*!< Switch DMI_CLK to MAIN_CLK */
    CLOCK_AttachClk(kOSC32K_to_WDT_CLK);     /*!< Switch WDT_CLK to OSC32K */
    CLOCK_AttachClk(kOSC32M_to_SPI_CLK);     /*!< Switch SPI_CLK to OSC32M */
    CLOCK_AttachClk(kOSC32M_to_I2C_CLK);     /*!< Switch I2C_CLK to OSC32M */

    /* Enables the clock for the I/O controller: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);
    FLASH_SetReadMode(FLASH, true);

    SYSCON -> MAINCLKSEL = BOARD_MAINCLK_FRO48M;

    //SystemCoreClock = 48000000U;

#endif /* CONFIG_SOC_K32W061 || CONFIG_SOC_QN9090 */
}

static ALWAYS_INLINE void enable_internal_ntag(void)
{
    ASYNC_SYSCON->NFCTAGPADSCTRL = (
            ASYNC_SYSCON_NFCTAGPADSCTRL_VDD_EHS0(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_VDD_EHS1(1) |

            ASYNC_SYSCON_NFCTAGPADSCTRL_INT_INVERT(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_INT_ENZI(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_INT_FILTEROFF(1) |

            ASYNC_SYSCON_NFCTAGPADSCTRL_I2C_SDA_OD(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_I2C_SDA_ENZI(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_I2C_SDA_OD(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_I2C_SCL_ENZI(1) |
            ASYNC_SYSCON_NFCTAGPADSCTRL_I2C_SCL_OD(1)
    );
}


/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */

static int nxp_k32_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	/* old interrupt lock level */
	unsigned int oldLevel;

	/* disable interrupts */
	oldLevel = irq_lock();

	/* Initialize FRO/system clock to 48 MHz */
	clock_init();

    FLASH_SetReadMode(FLASH, true);

    /* check if we come from power down */
    reset_cause_t reset_cause = POWER_GetResetCause();
    //INIT_DBG_LOG("Reset Cause=%x PMC reset cause:%x", reset_cause, PMC->RESETCAUSE);

    POWER_Init();

    /* Need to be done only at power On
     * Configure the trimmed default active voltages */
    if(reset_cause  != RESET_WAKE_PD)
    {
        POWER_SetTrimDefaultActiveVoltage();
    }

    CLOCK_XtalBasicTrim();


#ifdef CONFIG_GPIO_MCUX_LPC
	/* Turn on PINT device*/
	PINT_Init(PINT);
#endif

	/*
	 * install default handler that simply resets the CPU if configured in
	 * the kernel, NOP otherwise
	 */
	NMI_INIT();



#if defined(CONFIG_BT)
    vRadio_SkipXTALInit();
    vRadio_DisableZBRadio();
    vRadio_ActivateXtal32MRadioBiasing();
#endif

    /* It is mandatory to reset FLEXCOM peripherals because FLEXCOM is not reset on wakeup from power down
     * Except if FLEXCOM is retained during power down mode
     * No need to reset other blocks, these are already reset by Hardware */
    if ( (reset_cause == RESET_WAKE_PD) )
    {
        if (!(PMC->PDSLEEPCFG & PMC_PDSLEEPCFG_PDEN_PD_COMM0_MASK))
        {
            RESET_PeripheralReset(kUSART0_RST_SHIFT_RSTn);
            RESET_PeripheralReset(kUSART1_RST_SHIFT_RSTn);
            RESET_PeripheralReset(kI2C0_RST_SHIFT_RSTn);
            RESET_PeripheralReset(kSPI0_RST_SHIFT_RSTn);
        }

        RESET_PeripheralReset(kGPIO0_RST_SHIFT_RSTn);
        RESET_PeripheralReset(kADC0_RST_SHIFT_RSTn);

    }

    enable_internal_ntag();

	/* restore interrupt state */
	irq_unlock(oldLevel);

	return 0;
}

SYS_INIT(nxp_k32_init, PRE_KERNEL_1, 0);

