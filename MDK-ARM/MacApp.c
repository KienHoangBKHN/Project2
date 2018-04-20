// File Name: MacApp.c 
// Author: Kien Hoang
// Date: 13.04.2018

#include <..\Utilities\utilities.h>
#include "..\Board\board.h"
#include "..\System\gpio.h"
#include "..\Mac\LoRaMac.h"
#include "..\MacApp\Commissioning.h" 

#define ACTIVE_REGION LORAMAC_REGION_EU868 // default, active region
#define APP_TX_DUTYCYCLE 5000 // 5s  
#define APP_TX_DUTYCYCLE_RND  1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
 #if defined( REGION_EU868 )
#include "..\Mac\LoRaMacTest.h"
/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true
#endif

// lora application port
#define LORAWAN_APP_PORT 2

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if(OVER_THE_AIR_ACTIVATION == 0)
static uint8_t NwkSkey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

// device address
static uint8_t DevAddr = LORAWAN_DEVICE_ADDRESS;
#endif

// application port 
static uint8_t AppPort = LORAWAN_APP_PORT;

// user application data size
static uint8_t AppDataSize = 16;
static uint8_t AppDataSizeBackup = 16;

// user application data buffer size
#define LORAWAN_APP_DATA_MAX_SIZE                           242

// user application data
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

// Indicates if the node is sending confirmed or unconfirmed messages
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

// defines the application data transmission duty cycle
static uint32_t TxDutyCycleTime;

// Timer to handle the application data transmission duty cycle
static TimerEvent_t TxNextPacketTimer;

// Specifies the state of the application LED
static bool AppLedStateOn = false;

// Timer to handle the state of LED1
static TimerEvent_t Led1Timer;
// Timer to handle the state of LED2
static TimerEvent_t Led2Timer;

// Indicates if a new packet can be sent
static bool NextTx = true;

// device state
static enum eDeviceState{
	DEVICE_STATE_INIT,
	DEVICE_STATE_JOIN,
	DEVICE_STATE_SEND,
	DEVICE_STATE_CYCLE,
	DEVICE_STATE_SLEEP
}DeviceState;


// LoRaWAN compliance tests support data (structure)
struct ComplianceTest_s
{
	bool Running;
	uint8_t State;
	bool IsTxComfirmed;
	uint8_t AppPort;
	uint8_t AppDataSize;
	uint8_t *AppDataBuffer;
	uint16_t	DownLinkCounter;
	bool LinkCheck;
	uint8_t DemoMargin;
	uint8_t NbGateways;
}ComplianceTest;

// tao 3 doi tuong LED => indicator
extern Gpio_t Led1;
extern Gpio_t Led2;
extern Gpio_t Led3;

// ham chuan bi payload cua khung du lieu
/*
// thuat toan
* function: PrepareTxFrame(uin8_t port)
* input: app port
*	output:  


*/



















