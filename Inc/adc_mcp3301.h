#ifndef ADC_MCP3301_H_
#define ADC_MCP3301_H_

#define PORTADC				GPIOB
#define ADC_CS				LL_GPIO_PIN_12
#define ADC_SCK				LL_GPIO_PIN_13
#define ADC_MISO			LL_GPIO_PIN_14
#define ADC_MOSI			LL_GPIO_PIN_15

#define SPIADC				SPI2

#define ADC_MCP3301_ON		(GPIOB -> BRR =  GPIO_BRR_BR_12)
#define ADC_MCP3301_OFF		(GPIOB -> BSRR = GPIO_BSRR_BS_12)

//=================================================================================================

void LL_ADC_MCP3301_Initialization(void);
void ExtAdcConf(void);
uint16_t adc_mcp3301_get_data(void);
void EADC(float *Res);


#endif /* ADC_MCP3301_H_ */
