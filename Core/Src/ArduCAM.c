/******************************
 * Author Rajesh Sriangam
 * Title ArduCAM.c
 * Description Used for ArduCAM API's Initialisation
 * Reference link:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/ArduCAM.c
 */



/****************HEADERS**************/
#include <I2C.h>
#include "ArduCAM.h"
#include "spi.h"
#include "usart.h"
#include "ov2640_regs.h"
/*************************************/

/**************GLOBAL HEADERS***************/
byte sensor_model = 0;
byte sensor_addr = 0;
byte m_fmt = JPEG;
uint32_t length = 0;
/********************************************/

/*****************************************
 * Function void ArduCAM_Init(byte model)
 * Param NULL
 * Return NULL
 * Description Used for ArduCAM Initialisation
 */
void ArduCAM_Init(byte model) 
{
	  wrSensorReg8_8(0xff, 0x01);
      wrSensorReg8_8(0x12, 0x80);
      if(m_fmt == JPEG)
      {
		wrSensorRegs8_8(OV2640_JPEG_INIT);        //OV2640 JPEG Initialisation is done
		wrSensorRegs8_8(OV2640_YUV422);          //Camera Data is converted to YUV 422
		wrSensorRegs8_8(OV2640_JPEG);            //Camera Data converted from RGB 565 to OV2640 JPEG
		wrSensorReg8_8(0xff, 0x01);             //Device Control Intialisation is done
		wrSensorReg8_8(0x15, 0x00);             //COM10 Register is Enabled
		wrSensorRegs8_8(OV2640_320x240_JPEG);   //320*240 JPEG is enabled for OV2640
      }
      else
      {
        wrSensorRegs8_8(OV2640_QVGA);          //OV2640 QVGA is enabled
      }
}
/****************************
 * Function void CS_HIGH
 * Param NULL
 * Return NULL
 * Description Used for making CS Pin High
 */
void CS_HIGH(void)
{
 	HAL_GPIO_WritePin(GPIOA, 4,GPIO_PIN_SET);
}
/****************************
 * Function void CS_LOW
 * Param NULL
 * Return NULL
 * Description Used for making CS Pin Low
 */
void CS_LOW(void)
{
 	HAL_GPIO_WritePin(GPIOA, 4,GPIO_PIN_RESET);
}
/****************************
 * Function void set_format
 * Param byte
 * Return NULL
 * Description Used for setting format
 */
void set_format(byte fmt)
{
  if (fmt == JPEG)
    m_fmt = JPEG;                    //JPEG Format is enabled
}
/****************************
 * Function void bus_read
 * Param int
 * Return value
 * Description Used for reading camera commands from SPI
 */
uint8_t bus_read(int address)
{
	 uint8_t value;
     CS_LOW();                                 //Chip Select Made Low
	 SPI1_ReadWriteByte(address);              //Address is passed to SPI Bus
	 value = SPI1_ReadWriteByte(0x00);
	 CS_HIGH();                                //Chip Select Made High
	 return value;
}
/****************************
 * Function void bus_write
 * Param int,int
 * Return value
 * Description Used for writing camera commands to SPI
 */
uint8_t bus_write(int address,int value)
{
	CS_LOW();                                    //Chip Select Made Low
	delay_us(10);                                //10 microseconds delay
	SPI1_ReadWriteByte(address);                 //Send address to SPI Bus
	SPI1_ReadWriteByte(value);                   //Send Data to SPI Bus
	delay_us(10);                                //10 microseconds delay
	CS_HIGH();                                   //Chip Select Made High
	return 1;
}
/****************************
 * Function uint8_t read_reg(uint8_t addr)
 * Param uint8_t
 * Return data
 * Description Used for reading data from Arduchip Internal Register
 */
uint8_t read_reg(uint8_t addr)
{
	uint8_t data;
	data = bus_read(addr & 0x7F);               //Read camera commands from SPI
	return data;
}
/****************************
 * Function void write_reg(uint8_t addr, uint8_t data)
 * Param address,data
 * Return value
 * Description Used for writing data to Arduchip Internal Register
 */
void write_reg(uint8_t addr, uint8_t data)
{
	 bus_write(addr | 0x80, data);             //Write camera commands to SPI
}
/****************************
 * Function uint8_t read_fifo(void)
 * Param NULL
 * Return data
 * Description Used for reading single fifo data
 */
uint8_t read_fifo(void)
{
	uint8_t data;
	data = bus_read(SINGLE_FIFO_READ);            //Read single fifo data from SPI
	return data;
}
/****************************
 * Function void set_fifo_burst()
 * Param NULL
 * Return NULL
 * Description Used for setting the read memory into burst read mode
 */
void set_fifo_burst()
{
	SPI1_ReadWriteByte(BURST_FIFO_READ);           //Transmit and Receive Burst data from SPI
}
/****************************
 * Function void flush_fifo(void)
 * Param NULL
 * Return NULL
 * Description Used for reset the pointer to zero
 */
void flush_fifo(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);     //Clear capture flag
}
/****************************
 * Function void start_capture(void)
 * Param NULL
 * Return NULL
 * Description Used for issuing capture command
 */
void start_capture(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);      //Enable capture command for camera
}
/****************************
 * Function void clear_fifo_flag(void )
 * Param NULL
 * Return NULL
 * Description Used for clearing capture flag so that next command can be processed
 */
void clear_fifo_flag(void )
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);      //reset fifo pointer too zero
}
/****************************
 * Function uint32_t read_fifo_length(void)
 * Param NULL
 * Return len
 * Description Used for determining the length of current captured image
 */
uint32_t read_fifo_length(void)
{
	uint32_t len1,len2,len3,len=0;
	len1 = read_reg(FIFO_SIZE1);
	len2 = read_reg(FIFO_SIZE2);
	len3 = read_reg(FIFO_SIZE3) & 0x7f;
	len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return len;	
}
/****************************
 * Function uint8_t get_bit(uint8_t addr, uint8_t bit)
 * Param address,bit
 * Return temp
 * Description Used for return trigger bit needed for ArduCAM OV2640
 */
uint8_t get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}
/****************************
 * Function void OV2640_set_JPEG_size(uint8_t size)
 * Param size
 * Return NULL
 * Description Used for setting resolution of 320*240
 */
void OV2640_set_JPEG_size()
{
		wrSensorRegs8_8(OV2640_320x240_JPEG);

}

/****************************
 * Function byte wrSensorReg8_8(int regID, int regDat)
 * Param regID~register address,regDat~register Data
 * Return int
 * Description Used for write a single sensors internal register over I2C
 */
byte wrSensorReg8_8(int regID, int regDat)
{
	delay_us(5);                                  //5 microseconds delay
	I2C_Start();                             //I2C Start
	if(I2C_Write_Byte(sensor_addr) == 0)     //Used for checking sensor address
	{
		I2C_Stop();                          //I2c Stop
		return 1;
	}
	delay_us(5);                                  //5 microseconds delay
	if(I2C_Write_Byte(regID) == 0)           //Used for checking sensor address
	{
		I2C_Stop();                          //I2C Stop
		return 2;                                       
	}
	delay_us(5);                                  //5 microseconds delay
	if(I2C_Write_Byte(regDat)==0)            //Used for checking register data
	{
		I2C_Stop();                          //I2C Stop
		return 3;
	}
	I2C_Stop();                              //I2C Stop
	return 0;
}
/****************************
 * Function byte rdSensorReg8_8(uint8_t, uint8_t*)
 * Param regID~register address,regDat~Register Data
 * Return regDat
 * Description Used for reading single sensor internal register value
 */

byte rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{
	delay_us(10);                                       //10 microseconds delay
	I2C_Start();
	if(I2C_Write_Byte(sensor_addr) == 0)           //Checks for sensor address
	{
		I2C_Stop();                                //I2C Stop
		return 1;                                        
	}
	delay_us(10);                                       //10 microseconds delay
	if (I2C_Write_Byte(regID)==0)                  //Checks for Register address
	{
		I2C_Stop();                                //I2C Stop
		return 2;                                       
	}
	I2C_Stop();                                    //I2C Stop
	delay_us(10);                                       //10 microseconds delay
	I2C_Start();                                   //I2C Start
	if(I2C_Write_Byte(sensor_addr|0x01)==0)
	{
		I2C_Stop();                                //I2C stop
		return 3;                                          
	}
	delay_us(10);                                       //10 microseconds delay
	*regDat = I2C_Read_Byte();                     //register data is received via I2C
	I2C_NoACK();                              //No acknowledgement received
	I2C_Stop();                                    //I2C stop
	return 0;                
}
/****************************
 * Function int wrSensorRegs8_8(const struct reglist[] )
 * Param reglist[]
 * Return error status
 * Description Used for write array of settings into sensor's internal register over I2C
 */
int wrSensorRegs8_8(const struct sensor_reg reglist[])
{
  int err = 0;
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;
  while ((reg_addr != 0xff) | (reg_val != 0xff))         //Checks whether register address or register data is equal to 0xFF or not
  {
    reg_addr = next->reg;
    reg_val = next->val;
    err = wrSensorReg8_8(reg_addr, reg_val);            //used for checking error status
    next++;
  }
  return err;
}

