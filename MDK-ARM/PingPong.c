/*
* /file PingPong.c
* /bref: chuong trinh Ping-Pong test code driver xay dung phia duoi
* /Data: 11/04/2018
* believe yourself
*
*
*/

#include <string.h>
#include "..\Board\board.h"
#include "..\System\gpio.h"
#include "..\Utilities\delay.h"
#include "..\Board\timer.h"
#include "..\Radio\radio.h"

/*
#if defined( REGION_AS923 )
#define RF_FREQUENCY                                923000000 // Hz
#elif defined( REGION_AU915 )
#define RF_FREQUENCY                                915000000 // Hz
#elif defined( REGION_CN779 )
#define RF_FREQUENCY                                779000000 // Hz
#elif defined( REGION_EU868 )
#define RF_FREQUENCY                                868000000 // Hz
#elif defined( REGION_KR920 )
#define RF_FREQUENCY                                920000000 // Hz
#elif defined( REGION_IN865 )
#define RF_FREQUENCY                                865000000 // Hz
#elif defined( REGION_US915 )
#define RF_FREQUENCY                                915000000 // Hz
#elif defined( REGION_US915_HYBRID )
#define RF_FREQUENCY                                915000000 // Hz
#endif
*/
// dinh nghia cac tham so cau hinh
#define RF_FREQUENCY                                915000000 // Hz- REGION_AU915

#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

// dinh nghia trang thai Module
typedef enum{
	LOWPOWER,
	RX,
	RX_TIMEOUT,
	RX_ERROR,
	TX,
	TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PongMsg[] = "PONG";
const uint8_t PingMsg[] = "PING";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

uint8_t RssiValue = 0;
uint8_t SnrValue = 0;

// radio events fucntion pointers

static RadioEvents_t RadioEvents;

// LED GPIO pins objects
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);

int Test(void)
{
	bool isMaster = true;
	uint8_t i;
	
	// khoi tao phan cung
	BoardInit();
	
	// khoi tao radio 
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	
	Radio.Init(&RadioEvents);
	
	Radio.SetChannel(RF_FREQUENCY);
	// cau hinh module LORA
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
			LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000
	);
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
			LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
	
			Radio.Rx(RX_TIMEOUT_VALUE);
			while(1)
			{
				switch( State )
				{
					case RX:
						if(isMaster == true)
						{
							if(BufferSize > 0)
							{
								if(strncmp ((const char *)Buffer, (const char *)PongMsg, 4) == 0)  // neu bo dem khac rong check PONG
								{
									// HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // blink LED1
									// indicates on a led
									// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) ^ 1);
									// send frame PING 
									Buffer[0] = 'P';
									Buffer[1] = 'I';
									Buffer[2] = 'N';
									Buffer[3] = 'G';
									// fill the buffer with numbers for the payload
									for(i = 4; i < BufferSize; i++)
									{
										Buffer[i] = i - 4;
									}
									DelayMs(1);
									Radio.Send(Buffer, BufferSize);
								}
								else if(strncmp((const char *) Buffer, (const char *)PingMsg, 4) == 0)  // khong phai PONG thi check PING
								{
									// a master already exist then become a slave
									isMaster = false;
									// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // set led off
									Radio.Rx(RX_TIMEOUT_VALUE);
								}
							}
						} // neu khong phai master
						else
						{
							if(BufferSize > 0)
							{
								if(strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
								{
									// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) ^ 1);
									
									// send the reply to the PONG msg
									Buffer[0] = 'P';
									Buffer[1] = 'O';
									Buffer[2] = 'N';
									Buffer[3] = 'G';
									// fill the buffer with numbers for the payload
									for(i = 4; i < BufferSize; i++)
									{
										Buffer[i] = i - 4;
									}
									DelayMs(1);
									Radio.Send(Buffer, BufferSize);
								}
								else // valid reception but not a PING as expected
								{    // set device is master and start again
									isMaster = true;
									Radio.Rx(RX_TIMEOUT_VALUE);
								}
							}
						}
						State = LOWPOWER;
						break;
					case TX:
					{
						// indicates on a led that we have sent a PING {master}
						// indicates on a led that we have sent a PING {master} LED2
						
						// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) ^ 1);
						Radio.Rx(RX_TIMEOUT_VALUE);
						State = LOWPOWER;
						break;
					}
					case RX_TIMEOUT:
					case RX_ERROR:
					{
						if(isMaster == true)
						{
							// send the next PING frame
						Buffer[0] = 'P';
						Buffer[1] = 'I';
						Buffer[2] = 'N';
						Buffer[3] = 'G';
						// fill the buffer with numbers for the payload
						for(i = 4; i < BufferSize; i++)
						{
								Buffer[i] = i - 4;
						}
						DelayMs(1);
						Radio.Send(Buffer, BufferSize);
						}
						else
						{
							Radio.Rx(RX_TIMEOUT_VALUE);
						}
						State = LOWPOWER;
						break;
					}
					case LOWPOWER:
					default:
						// set lowpower
						break;
				}
				// TimerLowPowerHandler();
			}
}

void OnTxDone( void )

{
    Radio.Sleep( );
    State = TX;
}



void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}



void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}


