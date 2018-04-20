/*!
 * \file      board-config.h
 *
 * \brief     Board configuration (danh cho dong STM32L1 - Board LoRaMote)
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * Define indicating if an external IO expander is to be used
 */

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PA_2

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5
#define RADIO_NSS                                   PA_4

/*
#define RADIO_DIO_0                                 PB_1
#define RADIO_DIO_1                                 PB_10
#define RADIO_DIO_2                                 PB_11
#define RADIO_DIO_3                                 PB_7
#define RADIO_DIO_4                                 PB_5
#define RADIO_DIO_5                                 PB_4
*/
#define RADIO_ANT_SWITCH_RX                         PC_13
#define RADIO_ANT_SWITCH_TX                         PA_4

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PD_0
#define OSC_HSE_OUT                                 PD_1

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define JTAG_TMS                                    PA_13
#define JTAG_TCK                                    PA_14
#define JTAG_TDI                                    PA_15
#define JTAG_TDO                                    PB_3
#define JTAG_NRST                                   PB_4

#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9

#define CON_EXT_1                                   PB_13
#define CON_EXT_3                                   PB_15
#define CON_EXT_7                                   PB_12
#define CON_EXT_8                                   PB_14
#define CON_EXT_9                                   PA_1
#define BAT_LEVEL_PIN                               PA_3
#define BAT_LEVEL_CHANNEL                           ADC_CHANNEL_3

#define BOOT_1                                      PB_2

#define GPS_PPS                                     PA_8
#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define PIN_PB6                                     PB_6
#define WKUP1                                       PA_0

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            CON_EXT_1
#define RADIO_DBG_PIN_RX                            CON_EXT_3

#endif // __BOARD_CONFIG_H__
