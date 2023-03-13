/****************************
 * Author Rajesh Srirangam
 * Title ov2640_regs.h
 * Description Used for Arducam 2640 Initialisation
 * Reference links: https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/ov2640_regs.h
 */

/**************HEADERS*************/
#include "ArduCAM.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/***********************************/

/**************MACROS**************/
#define OV2640_CHIPID_HIGH 	0x0A
#define OV2640_CHIPID_LOW 	0x0B
/*********************************/

/*******************************
 * Function sensor_reg OV2640_QVGA
 * Description Used for 320*240 pixel range for video and image
 */

const struct sensor_reg OV2640_QVGA[]  =
{
	{0xff, 0x0},                              //Device Control Initialisation
	{0x2c, 0xff},                             //Reserved
	{0x2e, 0xdf},                             //Vsync Pulse Width 8 bits
	{0xff, 0x1},                              //Device Control Initialisation
	{0x3c, 0x32},                             //Register 32 used for PCLK
	{0x11, 0x0},                              //Internal Frequency Off
	{0x9, 0x2},                               //Common Control 2x capability
	{0x4, 0xa8},                              //Horizontal Mirror and HREF Enable
	{0x13, 0xe5},                             //Banding Filter Enable,AGC~Auto Mode Enable,Exposure Control~Auto
	{0x14, 0x48},                             //AGC Gain Ceiling~8x
	{0x2c, 0xc},                              //Reserved
	{0x33, 0x78},                             //Reserved
	{0x3a, 0x33},
	{0x3b, 0xfb},
	{0x3e, 0x0}, 
	{0x43, 0x11},                              //Reserved
	{0x16, 0x10},                              //Reserved
	{0x39, 0x2},                               //Reserved
	{0x35, 0x88},                              //Reserved

	{0x22, 0xa},                               //Reserved
	{0x37, 0x40},                              //Reserved
	{0x23, 0x0},                               //Reserved
	{0x34, 0xa0},                              //Zoom Window Disable
	{0x6, 0x2},                                //Reserved
	{0x6, 0x88},                               //Reserved
	{0x7, 0xc0},                               //Reserved
	{0xd, 0xb7},                               //Reserved
	{0xe, 0x1},                                //Reserved
	{0x4c, 0x0},                               //Reserved
	{0x4a, 0x81},                              //Reserved
	{0x21, 0x99},                              //Reserved
	{0x24, 0x40},                              //Luminance Signal High range for AGC
	{0x25, 0x38},                              //Luminance Signal Low range for AGC
	{0x26, 0x82},                              //Fast Mode Step Threshold Range Enabled
	{0x5c, 0x0},                               //Reserved
	{0x63, 0x0},                               //Reserved
	{0x46, 0x22},                              //Zoom Mode Vertical Enable
	{0xc, 0x3a},                               //Enable Live video output,Auto Set Enable,60Hz set band frequency
	{0x5d, 0x55},                              //16 Zone average weight option
	{0x5e, 0x7d},                              //16 Zone average weight option
	{0x5f, 0x7d},                              //16 Zone average weight option
	{0x60, 0x55},                              //16 Zone average weight option
	{0x61, 0x70},                              //Histogram Algorithm Low Level
	{0x62, 0x80},                              //Histogram Algorithm Low Level
	{0x7c, 0x5},                               //Reserved
	{0x20, 0x80},                              //Reserved
	{0x28, 0x30},                              //Reserved
	{0x6c, 0x0},                               //Reserved
	{0x6d, 0x80},                              //Reserved
	{0x6e, 0x0},                               //Reserved
	{0x70, 0x2},                               //Reserved
	{0x71, 0x94},                              //Reserved
	{0x73, 0xc1},                              //Reserved
	{0x3d, 0x34},                              //Reserved
	{0x12, 0x4},                               //Zoom Mode Enabled
	{0x5a, 0x57},                              //Reserved
	{0x4f, 0xbb},                              //50hz Frequency EC Banding
	{0x50, 0x9c},                              //60hz Frequency EC banding
	{0xff, 0x0},                               //Device Control Initialisation
	{0xe5, 0x7f},                              //Reserved
	{0xf9, 0xc0},                              //Microntroller Reset Enable,Boot ROM Enable
	{0x41, 0x24},                              //Reserved
	{0xe0, 0x14},                              //JPEG and DVP Enabled
	{0x76, 0xff},                              //Reserved
	{0x33, 0xa0},                              //Reserved
	{0x42, 0x20},                              //Reserved
	{0x43, 0x18},                              //Reserved
	{0x4c, 0x0},                               //Reserved
	{0x87, 0xd0},                              //BPC and WPC Enable
	{0xd7, 0x3},                               //Reserved
	{0xd9, 0x10},                              //Reserved
	{0xd3, 0x82},                              //Auto Mode Enabled,Sysclk(24) given to YUV0 and Sysclk(12) given to RAW
	{0xc8, 0x8},                               //Reserved
	{0xc9, 0x80},                              //Reserved
	{0x7c, 0x0},                               //SDE Indirect Register~Address
	{0x7d, 0x0},                               //SDE Indirect Register~Data
	{0x7c, 0x3},                               //SDE Indirect Register~Address
	{0x7d, 0x48},                               //SDE Indirect Register~Data
	{0x7d, 0x48},                               //SDE Indirect Register~Data
	{0x7c, 0x8},                                //SDE Indirect Register~Address
	{0x7d, 0x20},                                //SDE Indirect Register~Data
	{0x7d, 0x10},                                //SDE Indirect Register~Data
	{0x7d, 0xe},                                 //SDE Indirect Register~Data
	{0x90, 0x0},                                 //Reserved
	{0x91, 0xe},                                 //Reserved
	{0x91, 0x1a},                                 //Reserved
	{0x91, 0x31}, 								  //Reserved
	{0x91, 0x5a},   							  //Reserved
	{0x91, 0x69}, 								   //Reserved
	{0x91, 0x75},                                  //Reserved
	{0x91, 0x7e}, 								  //Reserved
	{0x91, 0x88},                                 //Reserved
	{0x91, 0x8f},                                 //Reserved
	{0x91, 0x96},								  //Reserved
	{0x91, 0xa3},                                 //Reserved
	{0x91, 0xaf},                                  //Reserved
	{0x91, 0xc4},                                 //Reserved
	{0x91, 0xd7},                                  //Reserved
	{0x91, 0xe8},                                  //Reserved
	{0x91, 0x20},                                  //Reserved
	{0x92, 0x0},                                   //Reserved
	{0x93, 0x6},                                   //Reserved
	{0x93, 0xe3},                                  //Reserved
	{0x93, 0x3},                                   //Reserved
	{0x93, 0x3},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x2},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x93, 0x0},                                   //Reserved
	{0x96, 0x0},                                   //Reserved
	{0x97, 0x8},                                   //Reserved
	{0x97, 0x19},                                  //Reserved
	{0x97, 0x2},                                   //Reserved
	{0x97, 0xc},                                   //Reserved
	{0x97, 0x24},                                  //Reserved
	{0x97, 0x30},                                  //Reserved
	{0x97, 0x28},                                  //Reserved
	{0x97, 0x26},                                   //Reserved
	{0x97, 0x2},                                   //Reserved
	{0x97, 0x98},                                  //Reserved
	{0x97, 0x80},                                  //Reserved
	{0x97, 0x0},                                   //Reserved
	{0x97, 0x0},                                   //Reserved
	{0xa4, 0x0},                                   //Reserved
	{0xa8, 0x0},                                   //Reserved
	{0xc5, 0x11},                                  //Reserved
	{0xc6, 0x51},                                  //Reserved
	{0xbf, 0x80},                                  //Reserved
	{0xc7, 0x10},                                  //Reserved
	{0xb6, 0x66},                                  //Reserved
	{0xb8, 0xa5},                                  //Reserved
	{0xb7, 0x64},                                  //Reserved
	{0xb9, 0x7c},                                  //Reserved
	{0xb3, 0xaf},                                  //Reserved
	{0xb4, 0x97},                                  //Reserved
	{0xb5, 0xff},                                  //Reserved
	{0xb0, 0xc5},                                  //Reserved
	{0xb1, 0x94},                                  //Reserved
	{0xb2, 0xf},                                   //Reserved
	{0xc4, 0x5c},                                  //Reserved
	{0xa6, 0x0},                                   //Reserved
	{0xa7, 0x20},                                  //Reserved
	{0xa7, 0xd8},                                  //Reserved
	{0xa7, 0x1b},                                  //Reserved
	{0xa7, 0x31},                                  //Reserved
	{0xa7, 0x0},                                   //Reserved
	{0xa7, 0x18},                                  //Reserved
	{0xa7, 0x20},                                  //Reserved
	{0xa7, 0xd8},                                  //Reserved
	{0xa7, 0x19},                                  //Reserved
	{0xa7, 0x31},                                  //Reserved
	{0xa7, 0x0},                                   //Reserved
	{0xa7, 0x18},                                  //Reserved
	{0xa7, 0x20},                                  //Reserved
	{0xa7, 0xd8},                                  //Reserved
	{0xa7, 0x19},                                  //Reserved
	{0xa7, 0x31},                                  //Reserved
	{0xa7, 0x0},                                   //Reserved
	{0xa7, 0x18},                                  //Reserved
	{0x7f, 0x0},                                   //Reserved
	{0xe5, 0x1f},                                  //Reserved
	{0xe1, 0x77},                                  //Reserved
	{0xdd, 0x7f},                                  //Reserved
	{0xc2, 0xe},                                   //CIP,DMY and RAW_GMA Enabled
	{0xff, 0x0},                                   //DSP address
	{0xe0, 0x4},                                   //DVP Enabled
	{0xc0, 0xc8},                                  //Image Horizontal Size Enabled
	{0xc1, 0x96},                                  //Image Vertical Size ENabled
	{0x86, 0x3d},                                  //Reserved
	{0x51, 0x90},                                  //H_SIZE Enabled(Horizontal Size)
	{0x52, 0x2c},                                  //V_SiZE Enabled(Vertical size)
	{0x53, 0x0},                                   //Offset X Enabled
	{0x54, 0x0},                                   //Offset Y Enabled
	{0x55, 0x88},                                  //VHYX Enabled
	{0x57, 0x0},                                   //Reserved
	{0x50, 0x92},                                  //LP,Vertical Scale Divider and Horizontal Scale divider enabled
	{0x5a, 0x50},                                  //ZMOW Enabled
	{0x5b, 0x3c},                                  //ZMOH Enabled
	{0x5c, 0x0},                                   //OUTW Enabled
	{0xd3, 0x4},                                   //DVP clock Enabled
	{0xe0, 0x0},                                   //CIF Enabled
	{0xff, 0x0},                                   //Device Control Initialisation
	{0x5, 0x0},                                    //DSP Enabled
	{0xda, 0x8},                                   //RGB 565 Enabled
	{0xd7, 0x3},                                   //Reserved
	{0xe0, 0x0},                                   //CIF Enabled
	{0x5, 0x0},                                    //DSP Enabled
	{0xff,0xff},                                   //Sensor Address Enabled

};        


/*******************************
 * Function sensor_reg OV2640_JPEG_INIT
 * Description Used for JPEG Initialisation
 */
const struct sensor_reg OV2640_JPEG_INIT[]  =
{                                                 //{register address,value}
  { 0xff, 0x00 },                                 //Device Control Initialisation
  { 0x2c, 0xff },                                 //Reserved
  { 0x2e, 0xdf },                                 //VSYNC Pulse Width MSB 8 bits Enabled
  { 0xff, 0x01 },                                 //Device Control Initialisation
  { 0x3c, 0x32 },                                 //Reserved
  { 0x11, 0x00 },	                              //Internal Frequency Disbaled
  { 0x09, 0x02 },                                 //Output drive is doubled
  { 0x04, 0x28 },                                 //HREF and Vertical Flip is Enabled
  { 0x13, 0xe5 },                                 //Band Filter Enabled ,AGC set to Auto,Exposure Control set to Auto
  { 0x14, 0x48 },                                 //AGC Gain is increased to 8 times
  { 0x2c, 0x0c },                                 //Reserved
  { 0x33, 0x78 },                                 //Reserved
  { 0x3a, 0x33 },                                 //Reserved
  { 0x3b, 0xfB },                                 //Reserved
  { 0x3e, 0x00 },                                 //Reserved
  { 0x43, 0x11 },                                 //Reserved
  { 0x16, 0x10 },                                 //Reserved
  { 0x39, 0x92 },                                 //Reserved
  { 0x35, 0xda },                                 //Reserved
  { 0x22, 0x1a },                                 //Reserved
  { 0x37, 0xc3 },                                 //Reserved
  { 0x23, 0x00 },                                 //Reserved
  { 0x34, 0xc0 },                                 //AGC High Gain control
  { 0x36, 0x1a },                                 //Reserved
  { 0x06, 0x88 },                                 //Reserved
  { 0x07, 0xc0 },                                 //Reserved
  { 0x0d, 0x87 },                                 //Reserved
  { 0x0e, 0x41 },                                 //Reserved
  { 0x4c, 0x00 },                                 //Reserved
  { 0x48, 0x00 },                                 //Zoom Mode Vertical Window Enabled
  { 0x5B, 0x00 },                                 //Reserved
  { 0x42, 0x03 },                                 //Reserved
  { 0x4a, 0x81 },                                 //Reserved
  { 0x21, 0x99 },                                 //Reserved
  { 0x24, 0x40 },                                 //Luminance Signal High Range Enabled
  { 0x25, 0x38 },                                 //Luminance Signal Low Range Enabled
  { 0x26, 0x82 },                                 //fast Mode Large Step threshold enabled
  { 0x5c, 0x00 },                                 //Reserved
  { 0x63, 0x00 },                                 //Reserved
  { 0x61, 0x70 },                                 //Histogram Algorith Low Level enabled
  { 0x62, 0x80 },                                 //Histogram Algorith High Level enabled
  { 0x7c, 0x05 },                                 //Reserved
  { 0x20, 0x80 },                                 //Reserved
  { 0x28, 0x30 },                                 //Reserved
  { 0x6c, 0x00 },                                 //Reserved
  { 0x6d, 0x80 },                                 //Reserved
  { 0x6e, 0x00 },                                 //Reserved
  { 0x70, 0x02 },                                 //Reserved
  { 0x71, 0x94 },                                 //Reserved
  { 0x73, 0xc1 },                                 //Reserved
  { 0x12, 0x40 },                                 //SVGA Mode is enabled
  { 0x17, 0x11 },                                 //Horizontal Window Start MSB 8 bits Enabled
  { 0x18, 0x43 },                                 //Horizontal Window End   MSB 8 bits Enabled
  { 0x19, 0x00 },                                 //Vertical Window Start MSB 8 bits Enabled
  { 0x1a, 0x4b },                                 //Vertical Window End MSB 8 bits Enabled
  { 0x32, 0x09 },                                 //Horizontal Window Enabled
  { 0x37, 0xc0 },                                 //Reserved
  { 0x4f, 0x60 },                                 //Reserved
  { 0x50, 0xa8 },                                 //60 Hz Banding AEC 8 LSB Enabled
  { 0x6d, 0x00 },                                 //Reserved
  { 0x3d, 0x38 },                                 //Reserved
  { 0x46, 0x3f },                                 //Frame length bit is enabled
  { 0x4f, 0x60 },                                 //50  Hz Banding AEC 8 LSB Enabled
  { 0x0c, 0x3c },                                 //Set banding manually done for 50hz,Auto Set Banding,Snapshot option enabled for video mode
  { 0xff, 0x00 },                                 //Device Control Initialisation
  { 0xe5, 0x7f },                                 //Reserved
  { 0xf9, 0xc0 },                                 //Microcontroller reset and boot rom enabled
  { 0x41, 0x24 },                                 //Reserved
  { 0xe0, 0x14 },                                 //JPEG and DVP Enabled
  { 0x76, 0xff },                                 //Reserved
  { 0x33, 0xa0 },                                 //Reserved
  { 0x42, 0x20 },                                 //Reserved
  { 0x43, 0x18 },                                 //Reserved
  { 0x4c, 0x00 },                                 //Reserved
  { 0x87, 0xd5 },                                 //SDE,UV_AVG and CMX enabled
  { 0x88, 0x3f },                                 //Reserved
  { 0xd7, 0x03 },                                 //Reserved
  { 0xd9, 0x10 },                                 //Reserved
  { 0xd3, 0x82 },                                 //Auto Mode Enabled,Sysclk(24) given to YUV0 and Sysclk(12) given to RAW
  { 0xc8, 0x08 },                                 //Reserved
  { 0xc9, 0x80 },                                 //Reserved
  { 0x7c, 0x00 },                                 //Reserved
  { 0x7d, 0x00 },                                 //Reserved
  { 0x7c, 0x03 },                                 //Reserved
  { 0x7d, 0x48 },                                 //Reserved
  { 0x7d, 0x48 },                                 //Reserved
  { 0x7c, 0x08 },                                 //Reserved
  { 0x7d, 0x20 },                                 //Reserved
  { 0x7d, 0x10 },                                 //Reserved
  { 0x7d, 0x0e },                                 //Reserved
  { 0x90, 0x00 },                                 //Reserved
  { 0x91, 0x0e },                                 //Reserved
  { 0x91, 0x1a },                                //Reserved
  { 0x91, 0x31 },                                 //Reserved
  { 0x91, 0x5a },                                 //Reserved
  { 0x91, 0x69 },                                 //Reserved
  { 0x91, 0x75 },                                //Reserved
  { 0x91, 0x7e },                               //Reserved
  { 0x91, 0x88 },                               //Reserved
  { 0x91, 0x8f },                              //Reserved
  { 0x91, 0x96 },                               //Reserved
  { 0x91, 0xa3 },                               //Reserved
  { 0x91, 0xaf },                              //Reserved
  { 0x91, 0xc4 },                                //Reserved
  { 0x91, 0xd7 },                                //Reserved
  { 0x91, 0xe8 },                                //Reserved
  { 0x91, 0x20 },                                //Reserved
  { 0x92, 0x00 },                                //Reserved
  { 0x93, 0x06 },                                //Reserved
  { 0x93, 0xe3 },                                //Reserved
  { 0x93, 0x05 },                                //Reserved
  { 0x93, 0x05 },                               //Reserved
  { 0x93, 0x00 },                                //Reserved
  { 0x93, 0x04 },                               //Reserved
  { 0x93, 0x00 },                                //Reserved
  { 0x93, 0x00 },                               //Reserved
  { 0x93, 0x00 },                               //Reserved
  { 0x93, 0x00 },                               //Reserved
  { 0x93, 0x00 },                               //Reserved
  { 0x93, 0x00 },                               //Reserved
  { 0x93, 0x00 },                               //Reserved
  { 0x96, 0x00 },                              //Reserved
  { 0x97, 0x08 },                               //Reserved
  { 0x97, 0x19 },                               //Reserved
  { 0x97, 0x02 },                               //Reserved
  { 0x97, 0x0c },                               //Reserved
  { 0x97, 0x24 },                               //Reserved
  { 0x97, 0x30 },                               //Reserved
  { 0x97, 0x28 },                               //Reserved
  { 0x97, 0x26 },                               //Reserved
  { 0x97, 0x02 },                               //Reserved
  { 0x97, 0x98 },                               //Reserved
  { 0x97, 0x80 },                              //Reserved
  { 0x97, 0x00 },                              //Reserved
  { 0x97, 0x00 },                              //Reserved
  { 0xc3, 0xed },                              //CIP,DMY,RAW_GMAAWb,AWB_GAIN,PRE bits are enabled
  { 0xa4, 0x00 },                              //Reserved
  { 0xa8, 0x00 },                              //Reserved
  { 0xc5, 0x11 },                              //Reserved
  { 0xc6, 0x51 },                              //Reserved
  { 0xbf, 0x80 },                              //Reserved
  { 0xc7, 0x10 },                             //Reserved
  { 0xb6, 0x66 },                              //Reserved
  { 0xb8, 0xA5 },                              //Reserved
  { 0xb7, 0x64 },                              //Reserved
  { 0xb9, 0x7C },                              //Reserved
  { 0xb3, 0xaf },                              //Reserved
  { 0xb4, 0x97 },                              //Reserved
  { 0xb5, 0xFF },                              //Reserved
  { 0xb0, 0xC5 },                              //Reserved
  { 0xb1, 0x94 },                              //Reserved
  { 0xb2, 0x0f },                              //Reserved
  { 0xc4, 0x5c },                              //Reserved
  { 0xc0, 0x64 },                              //Image Horizontal Size Enabled
  { 0xc1, 0x4B },                              //Image Vertical Size Enabled
  { 0x8c, 0x00 },                             //Horizontal Size enabled
  { 0x86, 0x3D },                               //Reserved
  { 0x50, 0x00 },                               //CTRL is disabled
  {0x51, 0x90},                                  //H_SIZE Enabled(Horizontal Size)
  {0x52, 0x2c},                                  //V_SiZE Enabled(Vertical size)
  {0x53, 0x0},                                   //Offset X Enabled
  {0x54, 0x0},                                   //Offset Y Enabled
  {0x55, 0x88},                                  //VHYX Enabled
  {0x57, 0x0},                                   //Reserved
  {0x50, 0x92},                                  //LP,Vertical Scale Divider and Horizontal Scale divider enabled
  {0x5a, 0x50},                                  //ZMOW Enabled
  {0x5b, 0x3c},                                  //ZMOH Enabled
  {0x5c, 0x0},                                   //OUTW Enabled
  { 0xd3, 0x00 },	                             //DVP clock is disabled
  { 0xc3, 0xed },                                //CIP,DMY,RAW_GMAAWb,AWB_GAIN,PRE bits are enabled
  { 0x7f, 0x00 },                                //Reserved
  { 0xda, 0x00 },                                //Image Mode is disabled
  { 0xe5, 0x1f },                                //Reserved
  { 0xe1, 0x67 },                                //Reserved
  { 0xe0, 0x00 },                                //Reset is disabled
  { 0xdd, 0x7f },                                //Reserved
  { 0x05, 0x00 },                               //DSP is enabled
  { 0x12, 0x40 },                                //Reserved
  { 0xd3, 0x04 },	                             //DVP clock is enabled
  { 0xc0, 0x16 },                                //Image Horizontal Size Enabled
  { 0xC1, 0x12 },                                //Image Verical Size Enabled
  { 0x8c, 0x00 },                                //SIZE L is enabled
  { 0x86, 0x3d },                                //Reserved
  { 0x50, 0x00 },                               //CTRL is disabled
  {0x51, 0x90},                                  //H_SIZE Enabled(Horizontal Size)
  {0x52, 0x2c},                                  //V_SiZE Enabled(Vertical size)
  {0x53, 0x0},                                   //Offset X Enabled
  {0x54, 0x0},                                   //Offset Y Enabled
  {0x55, 0x88},                                  //VHYX Enabled
  {0x57, 0x0},                                   //Reserved
  {0x50, 0x92},                                  //LP,Vertical Scale Divider and Horizontal Scale divider enabled
  {0x5a, 0x50},                                  //ZMOW Enabled
  {0x5b, 0x3c},                                  //ZMOH Enabled
  {0x5c, 0x0},                                   //OUTW Enabled
  { 0xff, 0xff },                                //Sensor address is enabled
};             

/*******************************
 * Function sensor_reg OV2640_YUV422
 * Description Used for extracting YUV422 Camera data
 */
const struct sensor_reg OV2640_YUV422[]  =
{
  { 0xFF, 0x00 },                                    //Device control Initialisation
  { 0x05, 0x00 },                                    //DSP Enabled
  { 0xDA, 0x10 },                                    //JPEG Output Enabled
  { 0xD7, 0x03 },                                    //Reserved
  { 0xDF, 0x00 },                                    //Reserved
  { 0x33, 0x80 },                                    //Reserved
  { 0x3C, 0x40 },                                    //Reserved
  { 0xe1, 0x77 },                                    //Reserved
  { 0x00, 0x00 },                                    //Reserved
  { 0xff, 0xff },                                    //Sensor address is initialised
};

/*******************************
 * Function sensor_reg OV2640_JPEG
 * Description Used for initialisation of JPEG Mode
 */
const struct sensor_reg OV2640_JPEG[]  =  
{
  { 0xe0, 0x14 },                                    //JPEG Enabled
  { 0xe1, 0x77 },                                    //Reserved
  { 0xe5, 0x1f },                                    //Reserved
  { 0xd7, 0x03 },                                    //Reserved
  { 0xda, 0x10 },                                    //Y8 Enabled
  { 0xe0, 0x00 },                                    //CIF Enabled
  { 0xFF, 0x01 },                                    //Device Control Initialisation
  { 0x04, 0x08 },                                    //Reserved
  { 0xff, 0xff },                                    //Sensor address is enabled
}; 


/*******************************
 * Function sensor_reg OV2640_320*240_JPEG
 * Description Used for displaying data of 320*240 in JPEG Mode
 */

const struct sensor_reg OV2640_320x240_JPEG[]  =  
{
  { 0xff, 0x01 },                                    //Device Control Initialisation
  { 0x12, 0x40 },                                    //SVGA Mode Enabled
  { 0x17, 0x11 },                                    //Horizontal Window Start MSB Enabled
  { 0x18, 0x43 },                                   //Horizontal Window End MSB Enabled
  { 0x19, 0x00 },                                   //Vertical Window Line Start MSB Enabled
  { 0x1a, 0x4b },                                   //Vertical Window Line End MSB Enabled
  { 0x32, 0x09 },                                   //Horizontal Window start and end position is enabled
  { 0x4f, 0xca },                                   //50 Hz Banding AEC Enabled
  { 0x50, 0xa8 },                                   //60Hz Banding AEC Enabled
  { 0x5a, 0x23 },                                  //Reserved
  { 0x6d, 0x00 },                                  //Reserved
  { 0x39, 0x12 },                                  //Reserved
  { 0x35, 0xda },                                  //Reserved
  { 0x22, 0x1a },                                  //Reserved
  { 0x37, 0xc3 },                                  //Reserved
  { 0x23, 0x00 },                                  //Reserved
  { 0x34, 0xc0 },                                  //Reserved
  { 0x36, 0x1a },                                  //Reserved
  { 0x06, 0x88 },                                  //Reserved
  { 0x07, 0xc0 },                                  //Reserved
  { 0x0d, 0x87 },                                  //Reserved
  { 0x0e, 0x41 },                                  //Reserved
  { 0x4c, 0x00 },                                 //Reserved
  { 0xff, 0x00 },                                  //Device Control Initialisation
  { 0xe0, 0x04 },                                  //DVP Enabled
  { 0xc0, 0x64 },                                  //Image Horizontal Scale Enabled
  { 0xc1, 0x4b },                                  //Image Vertical Scale Enabled
  { 0x86, 0x35 },                                  //DCW,SDE,UV_AVG,CMX Enabled
  { 0x50, 0x89 },                                  //LP_Divider,V_Divider and H_Divider is Enabled
  { 0x51, 0xc8 },                                  //H_SIZE Enabled
  { 0x52, 0x96 },                                  //V_SIZE Enabled
  { 0x53, 0x00 },                                  //Offset_X Enabled
  { 0x54, 0x00 },                                  //Offset_Y Enabled
  { 0x55, 0x00 },                                  //Disabled
  { 0x57, 0x00 },                                  //Disabled
  { 0x5a, 0x50 },                                   //OUTW Enabled
  { 0x5b, 0x3c },                                   //OUTH Enabled
  { 0x5c, 0x00 },                                   //Disabled
  { 0xe0, 0x00 },                                   //Reset Disabled
  { 0xff, 0xff },                                 //Sensor address enabled
};


       
