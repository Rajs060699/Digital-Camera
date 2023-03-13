/****************************
 * Author Rajesh Srirangam
 * Title spi.h
 * Description Used for SPI Initialisation
 * Reference links:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/spi.h
 */


/************HEADERS*************/
#include "main.h"
#include "stdbool.h"
#include "usart.h"
/**********************************/

/***********MACROS******************/
#define BUFFER_MAX_SIZE 4096                               //Buffer Max Size
#define FLAG_Mask                  ((uint32_t)0x00FFFFFF)  // CEC FLAG mask
/*********************************/

/***********GLOBAL HEADERS**********/
extern uint32_t sendlen ; 
extern uint32_t haveRev ;
extern uint32_t noRev;
extern uint8_t *picbuf	;
extern bool receive_OK;	
extern bool send_OK ; 
/**********************************/

/**************FUNCTION PROTOTYPES*********/
uint8_t DMA1_Init(void);
void SPI1_Init(void);			 
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
void DMA1_RX(uint8_t *p , uint32_t len);
void DMA1_SendtoUsart(uint8_t *p , uint32_t len);
void SendbyUSART1( void);
void SingleCapTransfer(void);
void Start_JPEG_Capture(void);
/******************************************/
