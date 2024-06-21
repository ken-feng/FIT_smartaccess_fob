/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v15.0
processor: MKW38A512xxx4
package_id: MKW38A512VFT4
mcu_data: ksdk2_0
processor_version: 15.0.1
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '42', peripheral: LPUART0, signal: RX, pin_signal: PTC6/LLWU_P14/RF_RFOSC_EN/I2C1_SCL/LPUART0_RX/TPM2_CH0}
  - {pin_num: '43', peripheral: LPUART0, signal: TX, pin_signal: PTC7/LLWU_P15/SPI0_PCS2/I2C1_SDA/LPUART0_TX/TPM2_CH1, open_drain: disable}
  - {pin_num: '18', peripheral: GPIOB, signal: 'GPIO, 2', pin_signal: ADC0_SE3/CMP0_IN3/PTB2/RF_NOT_ALLOWED/LLWU_P9/DTM_TX/TPM0_CH0/TPM1_CH0/TPM2_CH0, direction: OUTPUT}
  - {pin_num: '23', peripheral: GPIOB, signal: 'GPIO, 18', pin_signal: ADC0_SE4/CMP0_IN2/PTB18/LPUART1_CTS_b/I2C1_SCL/TPM_CLKIN0/TPM0_CH0/NMI_b, direction: INPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    gpio_pin_config_t gpiob_pin18_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB2 (pin 18)  */
    GPIO_PinInit(GPIOB, 2U, &gpiob_pin18_config);

    gpio_pin_config_t gpiob_pin23_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTB18 (pin 23)  */
    GPIO_PinInit(GPIOB, 18U, &gpiob_pin23_config);

    /* PORTB18 (pin 23) is configured as PTB18 */
    PORT_SetPinMux(PORTB, 18U, kPORT_MuxAsGpio);

    /* PORTB2 (pin 18) is configured as PTB2 */
    PORT_SetPinMux(PORTB, 2U, kPORT_MuxAsGpio);

    /* PORTC17 (pin 46) is configured as LPUART1_RX */
	#if defined __HW_VENUS_EVT1_H
    PORT_SetPinMux(PORTC, (17U), kPORT_MuxAlt8);		// UART1

	#else
	#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
    PORT_SetPinMux(PORTA, (17U), kPORT_MuxAlt3);		// UART1
	#else
    PORT_SetPinMux(PORTC, (6U), kPORT_MuxAlt4);			// UART0
	#endif
	#endif

    /* PORTC7 (pin 43) is configured as LPUART0_TX */
	#if defined __HW_VENUS_EVT1_H
    PORT_SetPinMux(PORTC, (18U), kPORT_MuxAlt8);		// UART1
	#else
	#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
    PORT_SetPinMux(PORTA, (18U), kPORT_MuxAlt3);		// UART1
	#else
    PORT_SetPinMux(PORTC, (7U), kPORT_MuxAlt4);			// UART0
	#endif
	#endif

	#if (defined(UWB_FEATURE_SUPPORT) && (UWB_FEATURE_SUPPORT == 1U))
    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_LPUART1TXSRC_MASK | SIM_SOPT5_LPUART1RXSRC_MASK | SIM_SOPT5_LPUART1ODE_MASK)))

                  /* LPUART0 Transmit Data Source Select: LPUART1_TX pin. */
                  | SIM_SOPT5_LPUART1TXSRC(SOPT5_LPUART1TXSRC_0b00)

                  /* LPUART0 Receive Data Source Select: LPUART1_RX pin. */
                  | SIM_SOPT5_LPUART1RXSRC(SOPT5_LPUART1RXSRC_0b0)

                  /* LPUART0 Open Drain Enable: Open drain is disabled on LPUART1. */
                  | SIM_SOPT5_LPUART1ODE(SOPT5_LPUART1ODE_0b0));
	#else
    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK | SIM_SOPT5_LPUART0ODE_MASK)))

                  /* LPUART0 Transmit Data Source Select: LPUART0_TX pin. */
                  | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_0b00)

                  /* LPUART0 Receive Data Source Select: LPUART_RX pin. */
                  | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_0b0)

                  /* LPUART0 Open Drain Enable: Open drain is disabled on LPUART0. */
                  | SIM_SOPT5_LPUART0ODE(SOPT5_LPUART0ODE_0b0));
	#endif
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI0Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI .
 *
 * END ****************************************************************************************************************/
void BOARD_InitSPI0Pins(void)
{
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_VENUS_EVT1_H

#else
    /* Enable Clocks: Port C Gate Control */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* SPI0_SCK: PORTC16 (pin 45) */
    PORT_SetPinMux(BOARD_SPI0_CLK_PORT, BOARD_SPI0_CLK_PIN, kPORT_MuxAlt2);
    /* SPI0_SOUT: PORTC17 (pin 46) */
    PORT_SetPinMux(BOARD_SPI0_MISO_PORT, BOARD_SPI0_MISO_PIN, kPORT_MuxAlt2);
    /* SPI0_SIN: PORTC18 (pin 47) */
    PORT_SetPinMux(BOARD_SPI0_MOSI_PORT, BOARD_SPI0_MOSI_PIN, kPORT_MuxAlt2);
    /* SPI0_CS: PORTC19 (pin 48) */
    PORT_SetPinMux(BOARD_SPI0_CS_PORT, BOARD_SPI0_CS_PIN, kPORT_MuxAlt2);
#endif
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI1Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI.
 *
 * END ****************************************************************************************************************/
void BOARD_InitSPI1Pins(void)
{
//Modify (Ken):NXP-V0001 NO.1 -20240304
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_VENUS_EVT1_H
    /* Enable Clocks: Port A Gate Control */
    CLOCK_EnableClock(kCLOCK_PortA);

    /* SPI1_SOUT: PORTA16 (pin 4) */
    PORT_SetPinMux(BOARD_SPI1_MISO_PORT, BOARD_SPI1_MISO_PIN, kPORT_MuxAlt2);
    /* SPI1_SIN: PORTA17 (pin 5) */
    PORT_SetPinMux(BOARD_SPI1_MOSI_PORT, BOARD_SPI1_MOSI_PIN, kPORT_MuxAlt2);
    /* SPI1_SCK: PORTA18 (pin 6) */
    PORT_SetPinMux(BOARD_SPI1_CLK_PORT, BOARD_SPI1_CLK_PIN, kPORT_MuxAlt2);
    /* SPI0_CS: PORTA19 (pin 7) */
    //PORT_SetPinMux(PORTC, PIN19_IDX, kPORT_MuxAlt2);
#endif
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitRanger4Pins
 * Description   : Init UWB module connected Pins
 *
 * END ****************************************************************************************************************/
void BOARD_InitRanger4Pins(void)
{
#if UWB_FEATURE_SUPPORT
    //==========================================================================
    // SPI Init
    //==========================================================================
    #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
    BOARD_InitSPI1Pins();
    #else
    BOARD_InitSPI0Pins();
    #endif

    //==========================================================================
    // Enable Clocks
    //==========================================================================
    /* Enable Clocks: Port A Gate Control */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Enable Clocks: Port C Gate Control */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Enable Clocks: Port C Gate Control */
    CLOCK_EnableClock(kCLOCK_PortC);
    //==========================================================================
    /*** Manual chip select (CS): PORTA19 output***/
    //==========================================================================
    PORT_SetPinMux(BOARD_UWB_CS_PORT, BOARD_UWB_CS_PIN, kPORT_MuxAsGpio);
    port_pin_config_t csPinconfig =
    {
        kPORT_PullDisable,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_CS_PORT, BOARD_UWB_CS_PIN, &csPinconfig);
    gpio_pin_config_t csPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalOutput
    };
    GPIO_PinInit(BOARD_UWB_CS_GPIO, BOARD_UWB_CS_PIN, &csPinGPIOConfig);

    //==========================================================================
    /*** Ready pin(RDY): PORTC5 input***/
    //==========================================================================
    PORT_SetPinMux(BOARD_UWB_RDY_PORT, BOARD_UWB_RDY_PIN, kPORT_MuxAsGpio);
    port_pin_config_t readyPinconfig =
    {
        kPORT_PullUp,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterEnable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_RDY_PORT, BOARD_UWB_RDY_PIN, &readyPinconfig);

    gpio_pin_config_t readyPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalInput
    };
    GPIO_PinInit(BOARD_UWB_RDY_GPIO, BOARD_UWB_RDY_PIN, &readyPinGPIOConfig);

    //==========================================================================
    /*** Reset pin(RST): PORTC19 output***/
    //==========================================================================
    PORT_SetPinMux(BOARD_UWB_RST_PORT, BOARD_UWB_RST_PIN, kPORT_MuxAsGpio);
    port_pin_config_t rstPinconfig =
    {
        #if defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_G2_H || defined __HW_VENUS_EVT1_H
        kPORT_PullUp,
        #else
        kPORT_PullDisable,
        #endif
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_RST_PORT, BOARD_UWB_RST_PIN, &rstPinconfig);
    gpio_pin_config_t rstPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalOutput
    };
    GPIO_PinInit(BOARD_UWB_RST_GPIO, BOARD_UWB_RST_PIN, &rstPinGPIOConfig);

    //==========================================================================
    /*** Interrupt pin: PORTC6 input***/
    //==========================================================================
    // This port not use now. Because this port also be used by LpUart0, so
    // it should be initlized after LpUart0 was initlized.
    PORT_SetPinMux(BOARD_UWB_INT_PORT, BOARD_UWB_INT_PIN, kPORT_MuxAsGpio);
    port_pin_config_t intPinconfig =
    {
        kPORT_PullUp,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterEnable,
        kPORT_LowDriveStrength,
        kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(BOARD_UWB_INT_PORT, BOARD_UWB_INT_PIN, &intPinconfig);

    gpio_pin_config_t intPinGPIOConfig =
    {
        .outputLogic = 1u,
        .pinDirection = kGPIO_DigitalInput
    };
    GPIO_PinInit(BOARD_UWB_INT_GPIO, BOARD_UWB_INT_PIN, &intPinGPIOConfig);
#endif
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
