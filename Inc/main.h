/****************************
 * Author Rajesh Srirangam
 * Title main.h
 */

/***********HEADERS****/
#include "stm32f1xx_hal.h"
/***********************/

/**********MACROS*************/
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_10
#define I2C_SDA_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_11
#define I2C_SCL_GPIO_Port GPIOB
/******************************/

/*********FUNCTION PROTOTYPES******/
void Error_Handler(void);
void delay_us(uint16_t);
/**********************************/
