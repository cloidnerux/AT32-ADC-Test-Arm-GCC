#include <stdio.h>
#include "at32f4xx.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

#define BUZZER_PORT GPIOA
#define BUZZER_PIN GPIO_Pins_4

#define BUTTON_PORT GPIOA
#define BUTTON_PIN GPIO_Pins_1

ADC_InitType ADC_InitStructure;
DMA_InitType DMA_InitStructure;
TMR_TimerBaseInitType  TMR_TimeBaseStructure;
TMR_OCInitType  TMR_OCInitStructure;


__IO uint16_t ADC1ConvertedValue = 0, ADC3ConvertedValue = 0;

uint32_t buzzerFreq;


void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

int main(void)
{
    RCC_ClockType RccClkSource;
  
    /* System clocks configuration */
    RCC_Configuration();
  
    RCC_GetClocksFreq(&RccClkSource);
    if (SysTick_Config(RccClkSource.AHBCLK_Freq / 1000))//1ms
    { 
        /* Capture error */ 
        while (1);
    }
    
    /* NVIC configuration ------------------------------------------------------*/
    NVIC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();
  
//    /* DMA1 channel1 configuration ----------------------------------------------*/
//    DMA_Reset(DMA1_Channel1);
//    DMA_InitStructure.DMA_PeripheralBaseAddr    = ADC1_DR_Address;
//    DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)&ADC1ConvertedValue;
//    DMA_InitStructure.DMA_Direction             = DMA_DIR_PERIPHERALSRC;
//    DMA_InitStructure.DMA_BufferSize            = 1;
//    DMA_InitStructure.DMA_PeripheralInc         = DMA_PERIPHERALINC_DISABLE;
//    DMA_InitStructure.DMA_MemoryInc             = DMA_MEMORYINC_DISABLE;
//    DMA_InitStructure.DMA_PeripheralDataWidth   = DMA_PERIPHERALDATAWIDTH_HALFWORD;
//    DMA_InitStructure.DMA_MemoryDataWidth       = DMA_MEMORYDATAWIDTH_HALFWORD;
//    DMA_InitStructure.DMA_Mode                  = DMA_MODE_CIRCULAR;
//    DMA_InitStructure.DMA_Priority              = DMA_PRIORITY_HIGH;
//    DMA_InitStructure.DMA_MTOM                  = DMA_MEMTOMEM_DISABLE;
//    DMA_Init(DMA1_Channel1, &DMA_InitStructure);  
//    /* Enable DMA1 channel1 */
//    DMA_ChannelEnable(DMA1_Channel1, ENABLE);

//    /* DMA2 channel5 configuration ----------------------------------------------*/
//    DMA_Reset(DMA2_Channel5);
//    DMA_InitStructure.DMA_PeripheralBaseAddr    = ADC3_DR_Address;
//    DMA_InitStructure.DMA_MemoryBaseAddr        = (uint32_t)&ADC3ConvertedValue;
//    DMA_InitStructure.DMA_Direction             = DMA_DIR_PERIPHERALSRC;
//    DMA_InitStructure.DMA_BufferSize            = 1;
//    DMA_InitStructure.DMA_PeripheralInc         = DMA_PERIPHERALINC_DISABLE;
//    DMA_InitStructure.DMA_MemoryInc             = DMA_MEMORYINC_DISABLE;
//    DMA_InitStructure.DMA_PeripheralDataWidth   = DMA_PERIPHERALDATAWIDTH_HALFWORD;
//    DMA_InitStructure.DMA_MemoryDataWidth       = DMA_MEMORYDATAWIDTH_HALFWORD;
//    DMA_InitStructure.DMA_Mode                  = DMA_MODE_CIRCULAR;
//    DMA_InitStructure.DMA_Priority              = DMA_PRIORITY_HIGH;
//    DMA_InitStructure.DMA_MTOM                  = DMA_MEMTOMEM_DISABLE;
//    DMA_Init(DMA2_Channel5, &DMA_InitStructure);  
//    /* Enable DMA2 channel5 */
//    DMA_ChannelEnable(DMA2_Channel5, ENABLE);
	
	//Get the system core clock down to 24 MHz
	uint16_t PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	
	TMR_TimeBaseStructure.TMR_DIV = PrescalerValue;
	TMR_TimeBaseStructure.TMR_CounterMode = TMR_CounterDIR_Up;
	TMR_TimeBaseStructure.TMR_Period = 4095;		//Gives 5.8kHz
	TMR_TimeBaseStructure.TMR_ClockDivision = 0;
	TMR_TimeBaseStructure.TMR_RepetitionCounter = 0;
	
	TMR_TimeBaseInit(TMR2, &TMR_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	/*TMR_OCInitStructure.TMR_OCMode = TMR_OCMode_Timing;
	TMR_OCInitStructure.TMR_OutputState = TMR_OutputState_Enable;
	TMR_OCInitStructure.TMR_OutputNState = TMR_OutputNState_Enable;
	TMR_OCInitStructure.TMR_Pulse = 2047;
	TMR_OCInitStructure.TMR_OCPolarity = TMR_OCPolarity_High;
	TMR_OCInitStructure.TMR_OCNPolarity = TMR_OCNPolarity_High;
	TMR_OCInitStructure.TMR_OCIdleState = TMR_OCIdleState_Set;
	TMR_OCInitStructure.TMR_OCNIdleState = TMR_OCNIdleState_Set;*/
	TMR_Cmd(TMR2, ENABLE);

	//TMR_OC1Init(TMR1, &TMR_OCInitStructure);
	TMR_INTConfig(TMR2, TMR_INT_Overflow, ENABLE);

	
//    /* ADC1 configuration ------------------------------------------------------*/
//    ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;
//    ADC_InitStructure.ADC_ScanMode          = DISABLE;
//    ADC_InitStructure.ADC_ContinuousMode    = ENABLE;
//    ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;
//    ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;
//    ADC_InitStructure.ADC_NumOfChannel      = 1;
//    ADC_Init(ADC1, &ADC_InitStructure);
//    /* ADC1 regular channels configuration */ 
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_28_5);    
//    /* Enable ADC1 DMA */
//    ADC_DMACtrl(ADC1, ENABLE);

//    /* ADC2 configuration ------------------------------------------------------*/
//    ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;
//    ADC_InitStructure.ADC_ScanMode          = DISABLE;
//    ADC_InitStructure.ADC_ContinuousMode    = ENABLE;
//    ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;
//    ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;
//    ADC_InitStructure.ADC_NumOfChannel      = 1;
//    ADC_Init(ADC2, &ADC_InitStructure);
//    /* ADC2 regular channels configuration */ 
//    ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 1, ADC_SampleTime_28_5);
//    /* Enable ADC2 EOC interrupt */
//    ADC_INTConfig(ADC2, ADC_INT_EC, ENABLE);

//    /* ADC3 configuration ------------------------------------------------------*/
//    ADC_InitStructure.ADC_Mode              = ADC_Mode_Independent;
//    ADC_InitStructure.ADC_ScanMode          = DISABLE;
//    ADC_InitStructure.ADC_ContinuousMode    = ENABLE;
//    ADC_InitStructure.ADC_ExternalTrig      = ADC_ExternalTrig_None;
//    ADC_InitStructure.ADC_DataAlign         = ADC_DataAlign_Right;
//    ADC_InitStructure.ADC_NumOfChannel      = 1;
//    ADC_Init(ADC3, &ADC_InitStructure);
//    /* ADC3 regular channel14 configuration */ 
//    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_28_5);
//    /* Enable ADC3 DMA */
//    ADC_DMACtrl(ADC3, ENABLE);

//    /* Enable ADC1 */
//    ADC_Ctrl(ADC1, ENABLE);

//    /* Enable ADC1 reset calibration register */   
//    ADC_RstCalibration(ADC1);
//    /* Check the end of ADC1 reset calibration register */
//    while(ADC_GetResetCalibrationStatus(ADC1));
//    /* Start ADC1 calibration */
//    ADC_StartCalibration(ADC1);
//    /* Check the end of ADC1 calibration */
//    while(ADC_GetCalibrationStatus(ADC1));

//    /* Enable ADC2 */
//    ADC_Ctrl(ADC2, ENABLE);

//    /* Enable ADC2 reset calibration register */   
//    ADC_RstCalibration(ADC2);
//    /* Check the end of ADC2 reset calibration register */
//    while(ADC_GetResetCalibrationStatus(ADC2));
//    /* Start ADC2 calibration */
//    ADC_StartCalibration(ADC2);
//    /* Check the end of ADC2 calibration */
//    while(ADC_GetCalibrationStatus(ADC2));

//    /* Enable ADC3 */
//    ADC_Ctrl(ADC3, ENABLE);

//    /* Enable ADC3 reset calibration register */   
//    ADC_RstCalibration(ADC3);
//    /* Check the end of ADC3 reset calibration register */
//    while(ADC_GetResetCalibrationStatus(ADC3));
//    /* Start ADC3 calibration */
//    ADC_StartCalibration(ADC3);
//    /* Check the end of ADC3 calibration */
//    while(ADC_GetCalibrationStatus(ADC3));

//    /* Start ADC1 Software Conversion */ 
//    ADC_SoftwareStartConvCtrl(ADC1, ENABLE);
//    /* Start ADC2 Software Conversion */ 
//    ADC_SoftwareStartConvCtrl(ADC2, ENABLE);
//    /* Start ADC3 Software Conversion */ 
//    ADC_SoftwareStartConvCtrl(ADC3, ENABLE);

    while (1)
    {
		if(GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN) == Bit_SET){
			buzzerFreq = 0;
		} else {
			buzzerFreq = 0;
		}
    }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_APB2CLK_Div4); //96/4=24MHz
    
    /* Enable peripheral clocks ------------------------------------------------*/
    /* Enable DMA1 and DMA2 clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1 | RCC_AHBPERIPH_DMA2, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR2, ENABLE);
    /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC1 | RCC_APB2PERIPH_ADC2 |
                         RCC_APB2PERIPH_ADC3 | RCC_APB2PERIPH_GPIOC |
													RCC_APB2PERIPH_GPIOA, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
	
    /* Configure PC.02, PC.03 and PC.04 (ADC Channel12, ADC Channel13 and 
     ADC Channel14) as analog inputs */
    GPIO_InitStructure.GPIO_Pins = GPIO_Pins_2 | GPIO_Pins_3 | GPIO_Pins_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_ANALOG;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//PA5 is the supply hold
	GPIO_InitStructure.GPIO_Pins = BUZZER_PIN | GPIO_Pins_5;
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pins_5);
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
	
	GPIO_InitStructure.GPIO_Pins = BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;
	GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Configures Vector Table base location.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Configure and enable ADC interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel                      = ADC1_2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd                   = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TMR2_GLOBAL_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
