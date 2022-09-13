#include "wg12864.h"
#include "stm32f0xx.h"
#include "main.h"
#include <stdio.h>
#include "stm32f0xx_ll_i2c.h"
#include "wg12864_font58.h"

volatile uint32_t PCA_Timeout = PCA_LONG_TIMEOUT;
volatile uint8_t CtrlWd;										// byte reserving for the WB12864 lcd line driving

uint8_t screen_x = 0x00;
uint8_t screen_y = 0x00;

uint8_t buf_x = 0x00;
uint8_t buf_y = 0x00;
volatile uint8_t LcdBuf[8][128];

uint16_t TimerPeriod = 0;
uint16_t Channel2Pulse = 0;

//-----------------------------------------------------------------------------
void LL_I2C2_Initialization(void)
{
	LL_I2C_InitTypeDef I2C_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/**I2C2 GPIO Configuration
	PB10   ------> I2C2_SCL
	PB11   ------> I2C2_SDA
	*/

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
	LL_I2C_DisableOwnAddress2(I2C2);
	LL_I2C_DisableGeneralCall(I2C2);
	LL_I2C_EnableClockStretching(I2C2);
	I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
	//I2C_InitStruct.Timing = 0xB0420F13;					//	100 kHz
	I2C_InitStruct.Timing = 0x2010091A;					//	400 kHz
	I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
	I2C_InitStruct.DigitalFilter = 0;
	I2C_InitStruct.OwnAddress1 = 0;
	I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
	LL_I2C_Init(I2C2, &I2C_InitStruct);
	LL_I2C_EnableAutoEndMode(I2C2);
	LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);
}

//-----------------------------------------------------------------------------
void LL_TIM1_Initialization(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM1 GPIO Configuration
  PB0   ------> TIM1_CH2N
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Compute the value to be set in ARR register to generate signal frequency at 10.0 Khz
  //TimerPeriod = (SystemCoreClock / 10000) - 1;
  TimerPeriod = (uint16_t)TIMER;
  //Compute CCR2 value to generate a duty cycle at 50% for channel 2
  //Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
  //Compute CCR2 value to generate a duty cycle at 10% for channel 2
  Channel2Pulse = (uint16_t) (((uint32_t) 1 * (TimerPeriod - 1)) / 10);

  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = TimerPeriod;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;

  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.CompareValue = Channel2Pulse;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;

  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);

  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;

  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

  TIM1 -> CR1 |= TIM_CR1_ARPE;
  TIM1 -> CCMR1 &= ~TIM_CCMR1_CC2S;
  TIM1 -> CCMR1 |= TIM_CCMR1_OC1PE;
  TIM1 -> CCMR1 |= TIM_CCMR1_OC1FE;
  TIM1 -> CCER |= TIM_CCER_CC2E;
  TIM1 -> BDTR |= TIM_BDTR_MOE;

  TIM1 -> CR1 |= TIM_CR1_CEN;
}


//-----------------------------------------------------------------------------
void SetPeriod(uint16_t percents)
{
	TIM1 -> CR1 &= ~TIM_CR1_CEN;

	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
	LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
	Channel2Pulse = (uint16_t) (((uint32_t) (100 - per) * (TimerPeriod - 1)) / 100);

	//Channel 2 Configuration in PWM mode
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_ENABLE;
	TIM_OC_InitStruct.CompareValue = Channel2Pulse;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
	TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
	TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
	TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

	TIM1 -> CR1 |= TIM_CR1_ARPE;
	TIM1 -> CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC1PE;
	TIM1 -> CCMR1 |= TIM_CCMR1_OC1FE;
	TIM1 -> CCER |= TIM_CCER_CC2E;
	TIM1 -> BDTR |= TIM_BDTR_MOE;

	TIM1 -> CR1 |= TIM_CR1_CEN;
}



//-----------------------------------------------------------------------------
uint8_t PCA9555_Write2Byte (uint8_t address, uint8_t Lbyte, uint8_t Hbyte)
{
	//Test on BUSY Flag
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_BUSY(PCA_I2C) != RESET)	{
		if((PCA_Timeout--) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Configure slave address, nbytes, reload and generate start
	LL_I2C_HandleTransfer(PCA_I2C,
						  PCA9555_WRITE_ADDR,
						  LL_I2C_ADDRSLAVE_7BIT,
						  1,
						  LL_I2C_MODE_RELOAD,
						  LL_I2C_GENERATE_START_WRITE);

	//Wait until TXIS flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_TXIS(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Send PAC9555 address
	LL_I2C_TransmitData8(PCA_I2C, address);
	//Wait until TCR flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while((LL_I2C_IsActiveFlag_TCR(PCA_I2C)) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Update CR2 : set Slave Address , set write request, generate Start and set end mode
	LL_I2C_HandleTransfer(PCA_I2C,
						  PCA9555_WRITE_ADDR,
						  LL_I2C_ADDRSLAVE_7BIT,
						  2,
						  LL_I2C_MODE_AUTOEND,
						  LL_I2C_GENERATE_NOSTARTSTOP);

	//Wait until TXIS flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_TXIS(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Lbyte
	LL_I2C_TransmitData8(PCA_I2C, Lbyte);
	//Wait until TXIS flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_TXIS(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Hbyte
	LL_I2C_TransmitData8(PCA_I2C, Hbyte);
	//Wait until STOPF flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_STOP(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Clear STOPF flag
	LL_I2C_ClearFlag_STOP(PCA_I2C);
	return I2C_OK;
}


//-----------------------------------------------------------------------------
uint8_t PCA9555Init(uint8_t Lconf, uint8_t Hconf)
{
	return PCA9555_Write2Byte (0x06, Lconf, Hconf);
}


//-----------------------------------------------------------------------------
uint8_t PCA9555_ReadReg (uint8_t address, uint8_t* Lbyte, uint8_t* Hbyte)
{
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_BUSY(PCA_I2C))	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Again, start another transfer using the "transfer handling" function, the end bit being set in software this time
	//round, generate a start condition and indicate you will be writing data to the device.
	LL_I2C_HandleTransfer(PCA_I2C,
						  PCA9555_WRITE_ADDR,
						  LL_I2C_ADDRSLAVE_7BIT,
						  1,
						  LL_I2C_MODE_SOFTEND,
						  LL_I2C_GENERATE_START_WRITE);

	//Wait until TXIS flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_TXIS(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//Send the address of the register you wish to read
	LL_I2C_TransmitData8(PCA_I2C, address);
	//Wait until TCR flag is set
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_TC(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	//As per, start another transfer, we want to read 2 bytes. Generate a start condition and indicate that we want to read.
	LL_I2C_HandleTransfer(PCA_I2C,
						  PCA9555_WRITE_ADDR,
						  LL_I2C_ADDRSLAVE_7BIT,
						  2,
						  LL_I2C_MODE_AUTOEND,
						  LL_I2C_GENERATE_START_READ);

	//Wait until the RX register is full of data!
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_RXNE(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}

	*Lbyte = LL_I2C_ReceiveData8(PCA_I2C);
	//Wait until the RX register is full of data!
	PCA_Timeout = PCA_LONG_TIMEOUT;
	while(LL_I2C_IsActiveFlag_RXNE(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}
	*Hbyte = LL_I2C_ReceiveData8(PCA_I2C);
	//Wait for the stop condition to be sent
	while(LL_I2C_IsActiveFlag_STOP(PCA_I2C) == RESET)	{
		if((PCA_Timeout --) == 0x00)	{
			return I2C_BUS_BUSY;
		}
	}
	//Clear the stop flag for next transfers
	LL_I2C_ClearFlag_STOP(PCA_I2C);

	return I2C_OK;
}


//-----------------------------------------------------------------------------
uint8_t PCASwRead(void)
{
	uint8_t tmp, dummy;

	PCA9555Init(0xff,0x00);	//младший байт на ввод, старший на вывод
	CtrlWd |= KS0108_RW | KS0108_CS1 | KS0108_CS2 | KS0108_RES | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	CtrlWd &=~SW244_OE;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);
	PCA9555_ReadReg (0x00, &tmp, &dummy);
	CtrlWd |= KS0108_RW | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);
	return tmp;
}


//-----------------------------------------------------------------------------
void PCALCD_Reset(void)
{
	PCA9555Init(0xFF,0x00);	//младший байт на ввод, старший на вывод
	CtrlWd |= KS0108_RW | KS0108_CS1 | KS0108_CS2 | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	CtrlWd &=~KS0108_RES;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//OE=1 for switch, RESET=0 for WG12864, R/W=1
	CtrlWd |= KS0108_RES | SW244_OE | PCADUMMY;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//OE=1 for switch, RESET=1 for WG12864, R/W=1
}


//-----------------------------------------------------------------------------
void PCALCD_FillData(uint8_t dataToFill)		//fill full row
{
	uint8_t i;
	PCA9555Init(0x00,0x00);	//оба байта на вывод
	CtrlWd |= KS0108_DI | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~(KS0108_EN | KS0108_RW);

	for(i = 0x00; i < 128; i ++)	{
		if(!(screen_x / 64))	{
			CtrlWd &= ~KS0108_CS1;
			CtrlWd |=KS0108_CS2;		//выбрали 1-й кристалл
		}
		else	{
			CtrlWd &=~KS0108_CS2; CtrlWd |=KS0108_CS1;
		}

		CtrlWd |= KS0108_EN;
		PCA9555_Write2Byte(0x02, dataToFill, CtrlWd);
		CtrlWd &= ~KS0108_EN;
		PCA9555_Write2Byte(0x02, dataToFill, CtrlWd);
		screen_x ++;
	}
}


//-----------------------------------------------------------------------------
uint8_t PCALCD_ReadStatus(uint8_t controller)
{
	uint8_t status, dummy;

	PCA9555Init(0xff,0x00);	//младший байт на ввод, старший на вывод
	CtrlWd |= KS0108_RW | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	if(!controller){CtrlWd &=~KS0108_CS1;}		//выбрали 1-й кристалл
	else{CtrlWd &=~KS0108_CS2;}					//выбрали 2-й кристалл
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//поставили D/I в ноль и соответствующий CS в ноль
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//строб E в 1
	PCA9555_ReadReg (0x00, &status, &dummy);
	CtrlWd |= KS0108_RW | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);
	return status;
}


//-----------------------------------------------------------------------------
void PCALCD_WriteCommand(uint8_t commandToWrite, uint8_t controller)
{
	while(PCALCD_ReadStatus(controller) & DISPLAY_STATUS_BUSY);
	PCA9555Init(0x00,0x00);														//	configuration the two I/O PCA9555 set as output
	CtrlWd &=~(KS0108_EN | KS0108_DI | KS0108_RW);
	CtrlWd |= KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	if(!controller){CtrlWd &=~KS0108_CS1;}										//	the 1st controller is chosen
	else{CtrlWd &=~KS0108_CS2;}													//	the 2nd controller is chosen
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte(0x02, commandToWrite, CtrlWd);
	CtrlWd |= KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	PCA9555_Write2Byte(0x02, commandToWrite, CtrlWd);
}


//-----------------------------------------------------------------------------
void PCALCD_GoTo(unsigned char x, unsigned char y)
{
	screen_x = x;
	screen_y = y;

	PCA9555Init(0x00,0x00);	//оба байта на вывод
	CtrlWd |= KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~(KS0108_EN | KS0108_DI | KS0108_RW);

	CtrlWd &=~KS0108_CS1;		//выбрали 1-й кристалл
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_Y | 0, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_Y | 0, CtrlWd);
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_X | y, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_X | y, CtrlWd);
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_START_LINE | 0, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_START_LINE | 0, CtrlWd);
	CtrlWd |=KS0108_CS1;
	CtrlWd &=~KS0108_CS2;		//выбрали 2-й кристалл
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_Y | 0, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_Y | 0, CtrlWd);
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_X | y, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_X | y, CtrlWd);
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_START_LINE | 0, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_START_LINE | 0, CtrlWd);
	//----------------
	if(!(x / 64)){CtrlWd &=~KS0108_CS1;}		//выбрали 1-й кристалл
	else{CtrlWd &=~KS0108_CS2;}
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_Y | (x % 64), CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_Y | (x % 64), CtrlWd);
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_X | y, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, DISPLAY_SET_X | y, CtrlWd);
}


//-----------------------------------------------------------------------------
uint8_t PCALCD_ReadData(void)
{
	uint8_t data, dummy;
	PCA9555Init(0xff,0x00);	//младший байт на ввод, старший на вывод
	CtrlWd |= KS0108_DI | KS0108_RW | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	if(!(screen_x / 64)){CtrlWd &=~KS0108_CS1;}		//выбрали 1-й кристалл
	else{CtrlWd &=~KS0108_CS2;}					//выбрали 2-й кристалл
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//строб E в 1
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//строб E в 0
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);	//строб E в 1

	PCA9555_ReadReg (0x00, &data, &dummy);

	CtrlWd |= KS0108_RW | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~KS0108_EN;
	CtrlWd &=~KS0108_DI;
	PCA9555_Write2Byte (0x02, 0x00, CtrlWd);

	screen_x++;
	return data;
}


//-----------------------------------------------------------------------------
void PCALCD_WriteData(uint8_t dataToWrite)
{
	PCA9555Init(0x00,0x00);	//оба байта на вывод
	CtrlWd |= KS0108_DI | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~(KS0108_EN | KS0108_RW);

	if(!(screen_x / 64)){CtrlWd &=~KS0108_CS1;}		//выбрали 1-й кристалл
	else{CtrlWd &=~KS0108_CS2;}					//выбрали 2-й кристалл
	PCA9555_Write2Byte (0x02, dataToWrite, CtrlWd);
	CtrlWd |=KS0108_EN;
	PCA9555_Write2Byte (0x02, dataToWrite, CtrlWd);
	CtrlWd &=~KS0108_EN;
	PCA9555_Write2Byte (0x02, dataToWrite, CtrlWd);
	screen_x++;
}


//-----------------------------------------------------------------------------
void PCALCD_WriteDataChar(uint8_t * simv)
{
	uint8_t i;
	PCA9555Init(0x00,0x00);	//оба байта на вывод
	CtrlWd |= KS0108_DI | KS0108_CS1 | KS0108_CS2 | KS0108_RES | SW244_OE | PCADUMMY;
	CtrlWd &=~(KS0108_EN | KS0108_RW);

	simv[5]=0x00;
	for(i=0;i<6;i++)	{
		if(!(screen_x / 64))	{
			CtrlWd &=~KS0108_CS1;
			CtrlWd |=KS0108_CS2;
		}		//выбрали 1-й кристалл
		else	{
			CtrlWd &=~KS0108_CS2;
			CtrlWd |=KS0108_CS1;
		}
		CtrlWd |=KS0108_EN;
		PCA9555_Write2Byte (0x02, simv[i], CtrlWd);
		CtrlWd &=~KS0108_EN;
		PCA9555_Write2Byte (0x02, simv[i], CtrlWd);
		screen_x++;
	}
}


//-----------------------------------------------------------------------------
void PCALCD_FlushRow(uint8_t row)
{
	uint8_t j;

	PCALCD_GoTo(0,row);

	CtrlWd |=  (KS0108_DI | SW244_OE | KS0108_RES);
	CtrlWd &= ~(KS0108_EN | KS0108_RW);

	screen_x = 0x00;
	for(j = 0x00; j < 0x80; j ++)	{
		if(!(screen_x / 64))	{
			CtrlWd &= ~KS0108_CS1;
			CtrlWd |=  KS0108_CS2;										//	the 1st controller is chosen
		}
		else	{
			CtrlWd &= ~KS0108_CS2;
			CtrlWd |=  KS0108_CS1;										//	the 2st controller is chosen
		}

		PCA9555_Write2Byte (0x02, LcdBuf[row][j], CtrlWd);
		CtrlWd |= KS0108_EN;
		PCA9555_Write2Byte (0x02, LcdBuf[row][j], CtrlWd);
		CtrlWd &= ~KS0108_EN;
		PCA9555_Write2Byte (0x02, LcdBuf[row][j], CtrlWd);
		screen_x++;
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// далее идут графические примитивы for KS108
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PCALCD_Initalize(void)
{
	PCALCD_WriteCommand((DISPLAY_ON_CMD | ON), 0);
	PCALCD_WriteCommand((DISPLAY_ON_CMD | ON), 1);
}


//-----------------------------------------------------------------------------
void BUF_GoTo(unsigned char x, unsigned char y)
//здесь y - номер строки
{
	buf_x = x;
	buf_y = y;
}


//-----------------------------------------------------------------------------
void BUF_WhiteTheScreen(void)
{
	unsigned char i, j;
	for(i = 0x00; i < 0x08; i ++)	{
		for(j = 0x00;j < 0x80; j ++)	{
			LcdBuf[i][j] = 0x00;
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_BlackTheScreen(void)
{
	unsigned char i, j;
	for(i = 0x00; i < 0x08; i ++)	{
		for(j = 0x00;j < 0x80; j ++)	{
			LcdBuf[i][j] = 0xFF;
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_WriteChar(char charToWrite)
{
	uint8_t i;
	for(i=0;i<5;i++)	{
		LcdBuf[buf_y][buf_x]=*(font5x8 + (5 * charToWrite) + i);
		buf_x++;
	}
	LcdBuf[buf_y][buf_x]=0x00;
	buf_x++;
}


//-----------------------------------------------------------------------------
void BUF_FillRow(uint8_t row, uint8_t DataToFill)
{
	uint8_t i;

	BUF_GoTo(0,row);
	for(i=0;i<128;i++){LcdBuf[row][i]=DataToFill;}
}


//-----------------------------------------------------------------------------
void BUF_WriteString(char * stringToWrite)
//конец строки \0, обрабатывается символ \n и граница экрана
{
	char temp;
	while(*stringToWrite)	{
		temp=*stringToWrite;
		if((temp==0x0A)||(buf_x>123))	{
			BUF_GoTo(0,(buf_y+1) );
			*stringToWrite++;
		}
		else	{
			BUF_WriteChar(temp);
			*stringToWrite++;
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_logo(uint8_t obj)
{
	uint8_t i,j;

	for(i = 0x00; i < 0x08; i ++)	{
		for(j = 0x00; j < 0x80; j ++)	{
			switch(obj)	{
				case 0x01:
					LcdBuf[7 - i] [j] = uku_title[j][i];
					break;
				case 0x02:
					LcdBuf[7 - i] [j] = uku_title_inv[j][i];
					break;
			}
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_SetPixel(unsigned char x, unsigned char y)
{
	LcdBuf[y/8][x] |= (1 << (y % 8));
}


//-----------------------------------------------------------------------------
void BUF_InvertColumn(char x, char y)
//pixel column inversion by the current position
{
	BUF_GoTo(x, y);
	LcdBuf[y][x]^=0xff;
}


//-----------------------------------------------------------------------------
void BUF_Rectangle(unsigned char x, unsigned char y, unsigned char b, unsigned char a)
{
	unsigned char j;
	for(j = 0; j < a; j++)	{
		BUF_SetPixel(x, y + j);
		BUF_SetPixel(x + b - 1, y + j);
	}
	
	for(j = 0; j < b; j++)	{
		BUF_SetPixel(x + j, y);
		BUF_SetPixel(x + j, y + a - 1);
	}
}


//-----------------------------------------------------------------------------
void BUF_Circle(unsigned char cx, unsigned char cy ,unsigned char radius)
{
	int x, y, xchange, ychange, radiusError;
	x = radius;
	y = 0;
	xchange = 1 - 2 * radius;
	ychange = 1;
	radiusError = 0;
	while(x >= y)	{
		BUF_SetPixel(cx+x, cy+y);
		BUF_SetPixel(cx-x, cy+y);
		BUF_SetPixel(cx-x, cy-y);
		BUF_SetPixel(cx+x, cy-y);
		BUF_SetPixel(cx+y, cy+x);
		BUF_SetPixel(cx-y, cy+x);
		BUF_SetPixel(cx-y, cy-x);
		BUF_SetPixel(cx+y, cy-x);
		y++;
		radiusError += ychange;
		ychange += 2;

		if ( 2*radiusError + xchange > 0 )	{
			x--;
			radiusError += xchange;
			xchange += 2;
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_Line(unsigned int X1,unsigned int Y1,unsigned int X2,unsigned int Y2)
{
	int CurrentX, CurrentY, Xinc, Yinc,
    	Dx, Dy, TwoDx, TwoDy,
		TwoDxAccumulatedError, TwoDyAccumulatedError;

	Dx = (X2-X1); Dy = (Y2-Y1);
	TwoDx = Dx + Dx; TwoDy = Dy + Dy;
	CurrentX = X1; CurrentY = Y1;
	Xinc = 1; Yinc = 1;

	if(Dx < 0)	{
		Xinc = -1;
		Dx = -Dx;
		TwoDx = -TwoDx;
	}
	
	if(Dy < 0)	{
		Yinc = -1;
		Dy = -Dy;
		TwoDy = -TwoDy;
	}
	
	BUF_SetPixel(X1,Y1);
	
	if ((Dx != 0) || (Dy != 0))	{
		if (Dy <= Dx)	{
			TwoDxAccumulatedError = 0;
			do	{
				CurrentX += Xinc;
				TwoDxAccumulatedError += TwoDy;
				if(TwoDxAccumulatedError > Dx)	{
					CurrentY += Yinc;
					TwoDxAccumulatedError -= TwoDx;
				}
				
				BUF_SetPixel(CurrentX,CurrentY);
			}	while (CurrentX != X2);
		}
		else	{
			TwoDyAccumulatedError = 0;
			do	{
				CurrentY += Yinc;
				TwoDyAccumulatedError += TwoDx;
				if(TwoDyAccumulatedError > Dy)	{
					CurrentX += Xinc;
					TwoDyAccumulatedError -= TwoDy;
				}
				
				BUF_SetPixel(CurrentX,CurrentY);
			}	while (CurrentY != Y2);
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_Batt(float * Vcc)
{
	uint8_t i;

	LcdBuf[0][127] = 0x7e;
	LcdBuf[0][126] = 0x42;
	LcdBuf[0][125] = 0x42;
	LcdBuf[0][124] = 0x42;
	LcdBuf[0][123] = 0x42;
	LcdBuf[0][122] = 0x42;
	LcdBuf[0][121] = 0x42;
	LcdBuf[0][120] = 0x42;
	LcdBuf[0][119] = 0x42;
	LcdBuf[0][118] = 0x24;
	LcdBuf[0][117] = 0x3c;
	
	if(*Vcc >= 3.8f)	{
		LcdBuf[0][118]=0x3c;
		for(i = 0; i <= 8; i++)	{
			LcdBuf[0][119+i] = 0x7e;
		}
	}
	else	{
		if(*Vcc >= 3.7f)	{
			for(i = 0; i <= 8; i++)	{
				LcdBuf[0][119+i] = 0x7e;
			}
		}
	  	else	{
			if(*Vcc >= 3.6f)	{
				for(i = 1; i <= 8; i++)	{
					LcdBuf[0][119+i] = 0x7e;
				}
			}
			else	{
				if(*Vcc >= 3.5f)	{
					for(i = 2; i <=8; i++)	{
						LcdBuf[0][119+i] = 0x7e;
					}
				}
			  	else	{
					if(*Vcc >= 3.4f)	{
						for(i = 3; i <=8; i++)	{
							LcdBuf[0][119+i] = 0x7e;
						}
					}
					else	{
						if(*Vcc >= 3.3f)	{
							for(i = 4; i <= 8; i++)	{
								LcdBuf[0][119+i] = 0x7e;
							}
						}
				  		else	{
							if(*Vcc >= 3.2f)	{
								for(i = 5; i<=8; i++)	{
									LcdBuf[0][119+i] = 0x7e;
								}
							}
							else	{
								if(*Vcc >= 3.1f)	{
									for(i = 6; i<= 8; i++)	{
										LcdBuf[0][119+i] = 0x7e;
									}
								}
					  			else	{
									if(*Vcc >= 3.0f)	{
										for(i = 7; i <=8; i++)	{
											LcdBuf[0][119+i] = 0x7e;
										}
									}
								}
					  		}
						}
				  	}
				}
			}
		}
	}
}


//-----------------------------------------------------------------------------
void BUF_FillRowNum(uint8_t row, uint8_t DataToFill, uint8_t NumCol)
//row - string number; DataToFill - byte-filler; NumCol - column quantity for filling
{
	uint8_t i;

	BUF_GoTo(0, row);
	for(i = 0; i < NumCol; i++)	{
		LcdBuf[row][i] = DataToFill;
	}
}


//-----------------------------------------------------------------------------
void LcdConfig(void)
{
	uint32_t i;

	LL_I2C2_Initialization();
	PCALCD_Initalize();
	BUF_ClearScreen();
	for(i = 0; i < 8; i++)	{
		PCALCD_FlushRow(i);
	}
}
