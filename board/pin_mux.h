/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define SOPT5_LPUART0ODE_0b0 	0x00u    /*!<@brief LPUART0 Open Drain Enable: Open drain is disabled on LPUART0. */
#define SOPT5_LPUART0RXSRC_0b0 	0x00u  /*!<@brief LPUART0 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART0TXSRC_0b00 0x00u /*!<@brief LPUART0 Transmit Data Source Select: LPUART0_TX pin */

#define SOPT5_LPUART1ODE_0b0 	0x00u    /*!<@brief LPUART0 Open Drain Enable: Open drain is disabled on LPUART0. */
#define SOPT5_LPUART1RXSRC_0b0 	0x00u  /*!<@brief LPUART0 Receive Data Source Select: LPUART_RX pin */
#define SOPT5_LPUART1TXSRC_0b00 0x00u /*!<@brief LPUART0 Transmit Data Source Select: LPUART0_TX pin */
/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

//=====================================================================================================================
// [ Configuration SPI0 ] -
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.3 -20240506
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_EVT1_H || defined __HW_VENUS_EVT1_H

#else
    /*! @name PORTC16 (number 45), SPI0_CLK */
    #define BOARD_SPI0_CLK_PORT PORTC /*!<@brief PORT device name: PORTC */
    #define BOARD_SPI0_CLK_PIN  16U    /*!<@brief PORTC pin index: 16 */

    /*! @name PORTC17 (number 46), SPI0_MISO */
    #define BOARD_SPI0_MISO_PORT PORTC /*!<@brief PORT device name: PORTC */
    #define BOARD_SPI0_MISO_PIN 17U     /*!<@brief PORTC pin index: 17 */

    /*! @name PORTC18 (number 47), SPI0_MOSI */
    #define BOARD_SPI0_MOSI_PORT PORTC /*!<@brief PORT device name: PORTC */
    #define BOARD_SPI0_MOSI_PIN 18U     /*!<@brief PORTC pin index: 18 */

    /*! @name PORTC18 (number 47), SPI0_MOSI */
    #define BOARD_SPI0_CS_PORT PORTC /*!<@brief PORT device name: PORTC */
    #define BOARD_SPI0_CS_PIN  19U     /*!<@brief PORTC pin index: 18 */
#endif
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI0Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI split mode
 *
 * END ****************************************************************************************************************/
void BOARD_InitSPI0Pins(void);

//=====================================================================================================================
// [ Configuration SPI1 ] -
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.1 -20240304
/*! @name PORTA16 (number 4), SPI1_SOUT */
#define BOARD_SPI1_MISO_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SPI1_MISO_PIN  16U    /*!<@brief PORTA pin index: 16 */

/*! @name PORTA17 (number 5), SPI1_SIN */
#define BOARD_SPI1_MOSI_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SPI1_MOSI_PIN 17U     /*!<@brief PORTA pin index: 17 */

/*! @name PORTA18 (number 6), SPI1_CLK */
#define BOARD_SPI1_CLK_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_SPI1_CLK_PIN 18U     /*!<@brief PORTA pin index: 18 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitSPI1Pin
 * Description   : Init SPI0 pins, CS pin will be manually controlled for SPI split mode
 *
 * END ****************************************************************************************************************/
void BOARD_InitSPI1Pins(void);

//=====================================================================================================================
// [ Configuration UWB ] -
//=====================================================================================================================
//Modify (Ken):NXP-V0001 NO.3 -20240319
#if defined __HW_BLE_UWB_G2_H || defined __HW_BLE_UWB_FOB_H || defined __HW_BLE_UWB_EVT1_H || defined __HW_VENUS_EVT1_H
      /*! @name PORTA19 (number 7), UWB_CS output*/
      #define BOARD_UWB_CS_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
      #define BOARD_UWB_CS_PORT PORTA /*!<@brief PORT device name: PORTA */
      #define BOARD_UWB_CS_PIN  19U     /*!<@brief PORTA pin index: 19 */

      #if defined __HW_BLE_UWB_FOB_H || defined __HW_VENUS_EVT1_H
      /*! @name PORTC4 (number xx), UWB_RDY input*/
      #define BOARD_UWB_RDY_GPIO GPIOC /*!<@brief GPIO device name: GPIOB */
      #define BOARD_UWB_RDY_PORT PORTC /*!<@brief PORT device name: PORTB */
      #define BOARD_UWB_RDY_PIN  4U     /*!<@brief PORTC pin index: 0 */
      #else
      /*! @name PORTB0 (number xx), UWB_RDY input*/
      #define BOARD_UWB_RDY_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_UWB_RDY_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_UWB_RDY_PIN  0U     /*!<@brief PORTC pin index: 0 */
      #endif

      /*! @name PORTC1 (number xx), UWB_RST output */
      #define BOARD_UWB_RST_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_RST_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_RST_PIN  1U     /*!<@brief PORTC pin index: 1 */

      /*! @name PORTB18 (number xx), UWB_INT input */
      #define BOARD_UWB_INT_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
      #define BOARD_UWB_INT_PORT PORTB /*!<@brief PORT device name: PORTB */
      #define BOARD_UWB_INT_PIN  18U     /*!<@brief PORTC pin index: 18 */
#else
      /*! @name PORTA19 (number 7), UWB_CS output*/
      #define BOARD_UWB_CS_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
      #define BOARD_UWB_CS_PORT PORTA /*!<@brief PORT device name: PORTA */
      #define BOARD_UWB_CS_PIN 19U     /*!<@brief PORTA pin index: 19 */

      /*! @name PORTC5 (number 41), UWB_RDY input*/
      #define BOARD_UWB_RDY_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_RDY_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_RDY_PIN 5U     /*!<@brief PORTC pin index: 5 */

      /*! @name PORTC19 (number 48), UWB_RST output */
      #define BOARD_UWB_RST_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_RST_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_RST_PIN 19U     /*!<@brief PORTC pin index: 19 */

      /*! @name PORTC6 (number 42), UWB_INT input */
      #define BOARD_UWB_INT_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
      #define BOARD_UWB_INT_PORT PORTC /*!<@brief PORT device name: PORTC */
      #define BOARD_UWB_INT_PIN 6U     /*!<@brief PORTC pin index: 6 */
#endif

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitRanger4Pins
 * Description   : Init UWB connected Pins
 *
 * END ****************************************************************************************************************/
void BOARD_InitRanger4Pins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
