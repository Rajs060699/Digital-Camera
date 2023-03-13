/****************************
 * Author Rajesh Srirangam
 * Title I2C.h
 * Description Used for I2C Initialisation
 * Reference link:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/sccb_bus.h
 */

/***********HEADERS***********/
#include "stm32f1xx.h"
/****************************/

/***********MACROS*************/
#define I2C_TIM 1
/*******************************/

/***********FUNCTION PROTOTYPES************/
 void SCCB_SID_STATE();
void I2C_Clock_High();
void I2C_Clock_Low();
void I2C_Data_High();
void I2C_Data_Low();
void I2C_Start(void);
void I2C_Stop(void);
void I2C_NoACK(void);
void I2C_ACK(void);
uint8_t I2C_Write_Byte(uint8_t);
uint8_t I2C_Read_Byte(void);
/******************************************/

