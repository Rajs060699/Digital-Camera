/****************************
 * Author Rajesh Srirangam
 * Title ArduCAM.h
 * Description Used for Arducam Drivers Initialisation
 * Reference link :https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/ArduCAM.h
 */

/*************HEADERS***********/
#include "stdbool.h"
/********************************/

/***************************MACROS*******************************/
#define byte uint8_t
#define OV2640_CHIPID_HIGH 	0x0A
#define OV2640_CHIPID_LOW 	0x0B
#define JPEG	            1
#define OV2640  	        1
#define OV2640_320x240 		1	//320x240

//Define maximum frame buffer size
#if (defined OV2640_MINI_2MP)
#define MAX_FIFO_SIZE		0x5FFFF			//384KByte
#elif (defined OV5642_MINI_5MP || defined OV5642_MINI_5MP_BIT_ROTATION_FIXED || defined ARDUCAM_SHIELD_REVC)
#define MAX_FIFO_SIZE		0x7FFFF			//512KByte
#else
#define MAX_FIFO_SIZE		0x7FFFFF		//8MByte
#endif 

/****************************************************/
/* ArduChip registers definition 											*/
/****************************************************/
#define RWBIT									0x80  //READ AND WRITE BIT IS BIT[7]

#define ARDUCHIP_TEST1       	0x00  //TEST register

#if !(defined OV2640_MINI_2MP)
	#define ARDUCHIP_FRAMES			  0x01  //FRAME control register, Bit[2:0] = Number of frames to be captured																		//On 5MP_Plus platforms bit[2:0] = 7 means continuous capture until frame buffer is full
#endif

#define ARDUCHIP_TIM       		0x03  //Timming control
#if !(defined OV2640_MINI_2MP)
	#define HREF_LEVEL_MASK    		0x01  //0 = High active , 		1 = Low active
	#define VSYNC_LEVEL_MASK   		0x02  //0 = High active , 		1 = Low active
	#define LCD_BKEN_MASK      		0x04  //0 = Enable, 					1 = Disable
	#if (defined ARDUCAM_SHIELD_V2)
		#define PCLK_REVERSE_MASK  	0x08  //0 = Normal PCLK, 		1 = REVERSED PCLK
	#else
		#define PCLK_DELAY_MASK  		0x08  //0 = data no delay,		1 = data delayed one PCLK
	#endif
	//#define MODE_MASK          		0x10  //0 = LCD mode, 				1 = FIFO mode
#endif
//#define FIFO_PWRDN_MASK	   		0x20  	//0 = Normal operation, 1 = FIFO power down
//#define LOW_POWER_MODE			  0x40  	//0 = Normal mode, 			1 = Low power mode

#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define FIFO_RDPTR_RST_MASK     0x10
#define FIFO_WRPTR_RST_MASK     0x20

#define ARDUCHIP_GPIO			  0x06  //GPIO Write Register
#if !(defined (ARDUCAM_SHIELD_V2) || defined (ARDUCAM_SHIELD_REVC))
#define GPIO_RESET_MASK			0x01  //0 = Sensor reset,							1 =  Sensor normal operation
#define GPIO_PWDN_MASK			0x02  //0 = Sensor normal operation, 	1 = Sensor standby
#define GPIO_PWREN_MASK			0x04	//0 = Sensor LDO disable, 			1 = sensor LDO enable
#endif

#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation
#define SINGLE_FIFO_READ		0x3D  //Single FIFO read operation

#define ARDUCHIP_REV       		0x40  //ArduCHIP revision
#define VER_LOW_MASK       		0x3F
#define VER_HIGH_MASK      		0xC0

#define ARDUCHIP_TRIG      		0x41  //Trigger source
#define VSYNC_MASK         		0x01
#define SHUTTER_MASK       		0x02
#define CAP_DONE_MASK      		0x08

#define FIFO_SIZE1				0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2				0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3				0x44  //Camera write FIFO size[18:16]

#ifndef _SENSOR_
#define _SENSOR_
/*************************************************************************/
/***********STRUCTURES*********/
struct sensor_reg {
	uint16_t reg;
	uint16_t val;
};
/******************************/
#endif


/*********GLOBAL HEADERS*********/
extern byte sensor_model;
extern byte sensor_addr;
extern uint32_t length;
extern uint8_t is_header;
/**********************************/

/**********FUNCTION PROTOTYPES***********/
/*************ARDUCAM API's***************/
void ArduCAM_Init(byte model );
void ArduCAM_CS_init(void);
void CS_HIGH(void);
void CS_LOW(void);
void OV2640_set_JPEG_size();
void set_format(byte fmt);
void flush_fifo(void);
void start_capture(void);
void clear_fifo_flag(void);
uint8_t read_fifo(void);
uint32_t read_fifo_length(void);
void set_fifo_burst(void);
uint8_t read_fifo_burst(void);
uint8_t read_reg(uint8_t addr);
void write_reg(uint8_t addr, uint8_t data);	
uint8_t get_bit(uint8_t addr, uint8_t bit);
uint8_t bus_write(int address, int value);
uint8_t bus_read(int address);	
byte wrSensorReg8_8(int regID, int regDat);
int  wrSensorRegs8_8(const struct sensor_reg*);
byte rdSensorReg8_8(uint8_t regID, uint8_t* regDat);
/*****************************************************/
