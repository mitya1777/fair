#ifndef INC_I2C_KSV_H_
#define INC_I2C_KSV_H_

#include "stm32f0xx.h"


#define PCA9555_WRITE_ADDR  	0x40
#define PCA9555_READ_ADDR 	 	0x41

#define PCA_I2C_TIMEOUT         ((uint32_t)0x3FFFF) //I2C Time out
#define PCA_I2C_SPEED      		400000 							//I2C Speed

#define PCA_FLAG_TIMEOUT        ((uint32_t)0x1000)
#define PCA_LONG_TIMEOUT        ((uint32_t)(10 * PCA_FLAG_TIMEOUT))

#define PCA_I2C                 I2C2
#define PCA_I2C_CLK             RCC_APB1Periph_I2C2
#define PCA_I2C_SCL_PIN         GPIO_Pin_10                 //PB10
#define PCA_I2C_GPIO_PORT		GPIOB                       //GPIOB
#define PCA_I2C_GPIO_CLK		RCC_AHBPeriph_GPIOB
#define PCA_I2C_SDA_PIN         GPIO_Pin_11                 //PB11

#define I2C_ERROR				((uint8_t)0x01)
#define I2C_BUS_BUSY			((uint8_t)0x02)
#define I2C_OK					((uint8_t)0x00)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define KS0108_SCREEN_WIDTH		128
#define KS0108_SCREEN_HEIGHT	64

#define KS0108_CS1				(1<<7)
#define KS0108_CS2				(1<<6)
#define KS0108_RES				(1<<5)
#define KS0108_RW				(1<<4)
#define KS0108_DI				(1<<3)
#define KS0108_EN				(1<<2)
#define PCADUMMY				(1<<1)
#define SW244_OE				(1<<0)

#define DISPLAY_STATUS_BUSY		0x80

#define DISPLAY_SET_Y       	0x40
#define DISPLAY_SET_X       	0xB8
#define DISPLAY_START_LINE  	0xC0
#define DISPLAY_ON_CMD			0x3E
#define ON						0x01
#define OFF						0x00
//-----------------------------------------------------------------------------
void LL_I2C2_Initialization(void);
void LL_TIM1_Initialization(void);
void SetPeriod(uint16_t percents);
uint8_t PCA9555_Write2Byte (uint8_t address, uint8_t Lbyte, uint8_t Hbyte);
uint8_t PCA9555Init(uint8_t Lconf, uint8_t Hconf);
uint8_t PCA9555_ReadReg (uint8_t address, uint8_t* Lbyte, uint8_t* Hbyte);
uint8_t PCASwRead(void);
void PCALCD_Reset(void);
void PCALCD_FillData(uint8_t dataToFill);
uint8_t PCALCD_ReadStatus(uint8_t controller);
void PCALCD_WriteCommand(uint8_t commandToWrite, uint8_t controller);
void PCALCD_GoTo(unsigned char x, unsigned char y);
uint8_t PCALCD_ReadData(void);
void PCALCD_WriteData(uint8_t dataToWrite);
void PCALCD_WriteDataChar(uint8_t * simv);
void PCALCD_FlushRow(uint8_t row);

//-----------------------------------------------------------------------------
void PCALCD_Initalize(void);
void BUF_GoTo(unsigned char x, unsigned char y);
void BUF_WhiteTheScreen(void);
void BUF_BlackTheScreen(void);
void BUF_ChessTheScreen1(void);
void BUF_ChessTheScreen2(void);
void BUF_WriteChar(char charToWrite);
void BUF_FillRow(uint8_t row, uint8_t DataToFill);
void BUF_WriteString(char * stringToWrite);
void BUF_SetPixel(unsigned char x, unsigned char y);
void BUF_logo(uint8_t obj);
void BUF_InvertColumn(char x, char y);
void BUF_Rectangle(unsigned char x, unsigned char y, unsigned char b, unsigned char a);
void BUF_Circle(unsigned char cx, unsigned char cy ,unsigned char radius);
void BUF_Line(unsigned int X1,unsigned int Y1,unsigned int X2,unsigned int Y2);
void BUF_Batt(float * Vcc);
void BUF_FillRowNum(uint8_t row, uint8_t DataToFill, uint8_t NumCol);
void RCC_HSI14Cmd(FunctionalState NewState);


#endif
