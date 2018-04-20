/*!
 * \file      board.h
 *
 * \brief	 cau hinh phan cung ket noi voi kit chip stm32f103rctx
 * 				+ copy code gen mac dinh tu CUBEMX
 *
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
 * \endcode
 *
 * \author    Kien Hoang ( Sanslab )
 *
 *
 */
#ifndef BOARD_DRIVER_H
#define BOARD_DRIVER_H

#include "stm32f1xx_hal.h"


// khai bao to hop phan chuan giao tiep

#ifdef __cplusplus
extern "C" {
#endif


static RTC_HandleTypeDef hrtc;

static SPI_HandleTypeDef hspi1;

static UART_HandleTypeDef huart4;

// private function prototype (cac ham khoi tao he thong)

void SystemClock_Config(void);

void MX_GPIO_Init(void);

void MX_RTC_Init(void);

void MX_SPI1_Init(void);

void MX_UART4_Init(void);



// ham khoi tao toan bo he thong
void BoardInit(void);

#ifdef __cplusplus
}
#endif

#endif













