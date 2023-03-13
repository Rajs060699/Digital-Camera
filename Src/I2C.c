/******************************
 * Author Rajesh Sriangam
 * Title I2C.c
 * Description Used for I2C Initialisation
 * Reference link:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/sccb_bus.c
 */

/************HEADERS*************/
#include "I2C.h"
#include "main.h"
/*********************************/

 /*******************************
  * Function  void I2C_Clock_High()
  * Return NULL
  * Param NULL
  * Description Used for making clock high
  */
 void I2C_Clock_High()
 {
	 HAL_GPIO_WritePin(GPIOB,11,SET);
 }
 /*******************************
  * Function  void I2C_Clock_Low()
  * Return NULL
  * Param NULL
  * Description Used for making clock low
  */
 void I2C_Clock_Low()
  {
 	 HAL_GPIO_WritePin(GPIOB,11,RESET);
  }
 /*******************************
   * Function  void I2C_Data_High()
   * Return NULL
   * Param NULL
   * Description Used for making Data High
   */
 void I2C_Data_High()
 {
	 HAL_GPIO_WritePin(GPIOB,10,SET);
 }
 /*******************************
    * Function  void I2C_Data_Low()
    * Return NULL
    * Param NULL
    * Description Used for making Data High
    */
 void I2C_Data_Low()
  {
 	 HAL_GPIO_WritePin(GPIOB,10,RESET);
  }
 /*******************************
  * Function void I2C_Start(void)
  * Return NULL
  * Param NULL
  * Description Used for I2C Start
  */
void I2C_Start(void)
{
    I2C_Data_High();                   //I2C data made high
    delay_us(I2C_TIM);                 //1 microsecond delay
    I2C_Clock_High();                  //I2C data made low
    delay_us(I2C_TIM);                  //1 microsecond delay
    I2C_Data_Low();                    //I2C data made low
    delay_us(I2C_TIM);                 //1 microsecond delay
    I2C_Clock_Low();                   //I2C clock made low
    delay_us(I2C_TIM);                 //1 microsecond delay
}
/*******************************
 * Function void I2C_Stop(void)
 * Return NULL
 * Param NULL
 * Description Used for I2C Stop
 */
void I2C_Stop(void)
{
    I2C_Data_Low();                     //I2C Data Bus Made Low
    delay_us(I2C_TIM);                  //1 microsecond delay
    I2C_Clock_High();                   //I2C Clock Bus Made High
    delay_us(I2C_TIM);                  //1 microsecond delay
    I2C_Data_High();                    //I2C Data Bus Made High
    delay_us(I2C_TIM);                  //1 microsecond delay
}
/*******************************
 * Function void I2C_NoACK(void)
 * Return NULL
 * Param NULL
 * Description Used for Intialising No Acknowledgment
 */

void I2C_NoACK(void)
{	
	I2C_Data_High();                      //I2C Data Bus Made High
	delay_us(I2C_TIM);                    //1 microsecond delay
	I2C_Clock_High();                     //I2C Clock Bus Made High
	delay_us(I2C_TIM);                    //1 microsecond delay
	I2C_Clock_Low();                      //I2C Clock Bus Made High
	delay_us(I2C_TIM);                    //1 microsecond delay
	I2C_Data_Low();                       //I2C Data Bus Made Low
	delay_us(I2C_TIM);                    //1 microsecond delay
}
/*******************************
 * Function void I2C_ACK(void)
 * Return NULL
 * Param NULL
 * Description Used for Intialising Acknowledgment
 */
void I2C_ACK(void)
{	
	I2C_Data_Low();                   //I2C data Bus made low
	delay_us(I2C_TIM);                //1 microsecond delay
	I2C_Clock_Low();                  //I2C Clock Bus made low
	delay_us(I2C_TIM);                //1 microsecond delay
	I2C_Clock_High();                 //I2C Clock Bus made High
	delay_us(I2C_TIM);                //1 microsecond delay
	I2C_Clock_Low();                  //I2C Clock Bus made low
	delay_us(I2C_TIM);                //1 microsecond delay
	I2C_Data_Low();                   //I2C data Bus made low
	delay_us(I2C_TIM);                //1 microsecond delay
}
/*******************************
 * Function uint8_t I2C_Write_Byte(void)
 * Return tem
 * Param data
 * Description Used for writing I2C data
 */
uint8_t I2C_Write_Byte(uint8_t data)
{
	uint32_t i;
	uint8_t tem;
	for(i = 0; i < 8; i++) 
	{
		if((data<<i) & 0x80)                   //Input data value in SDA Pin
		{
			I2C_Data_High();                  //I2C Data Bus made High
		}
		else 
		{
			I2C_Data_Low();                   //I2C Data Bus made Low
		}
		delay_us(I2C_TIM);
		I2C_Clock_High();
		delay_us(I2C_TIM);
		I2C_Clock_Low();

	}
	delay_us(I2C_TIM);                      //1 microsecond delay
	I2C_Clock_High();
	delay_us(I2C_TIM);                      //1 microsecond delay
	if(I2C_SDA_Pin==1)
	{
		tem = 0;               
	}
	else 
	{
		tem = 1;                
	}
	I2C_Clock_Low();                       //I2C Clock Bus Made Low
	delay_us(I2C_TIM);                     //1 microsecond delay
	return tem;  
}
/*******************************
 * Function uint8_t I2C_Read_Byte(void)
 * Return read
 * Param NULL
 * Description Used for Reading I2C Data
 */
uint8_t I2C_Read_Byte(void)
{	
	uint32_t i;
	uint8_t read = 0;
	delay_us(I2C_TIM);                     //1 mircosecond delay
	for(i = 8; i > 0; i--) 
	{		     
		delay_us(I2C_TIM);                 //1 mircosecond delay
		I2C_Clock_High();                  //Clock bus made High
		delay_us(I2C_TIM);                 //1 mircosecond delay
		read = read << 1;                  //Capture receive bit
		if(I2C_SDA_Pin==1)
		{
			read += 1; 
		}
		I2C_Clock_Low();                    //Clock bus made High
		delay_us(I2C_TIM);                  //1 mircosecond delay
	}	
	return read;
}

