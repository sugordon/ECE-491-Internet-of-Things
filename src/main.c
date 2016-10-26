// ------------------------------ //
// ------ Headers & Macros ------ //
// ------------------------------ //

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"

#define BUFFER_SIZE 16

// -------------------------------- //
// --- Prototypes & Definitions --- //
// -------------------------------- //

// Private variables
volatile uint32_t time_var1, time_var2, debounce;
volatile char received_buffer[BUFFER_SIZE+1];

// Used to store clock frequencies, for debugging
RCC_ClocksTypeDef clkfreqs;

void InitPLL(void);
void InitUSART(void);
void InitGPIOD(void);
void InitDAC(void);

void SendData(USART_TypeDef* USARTx, volatile char *s);

void IncrementCounter();
void Delay(volatile uint32_t nCount);

void INTPD0_Config();
void INTTIM2_Config();
void TIM2_IRQHandler(void);
void EXTI0_IRQHandler(void);
void SendDebug(void);

// --------------------------- //
// ---------- Main ----------- //
// --------------------------- //

int main(void) {

    INTTIM2_Config();
    INTPD0_Config();
    
    InitPLL();
    InitUSART();
    InitGPIOD();
    //InitDAC();

    SendDebug();

    for(;;) {
        //SendData(USART2, "hello world");
    }

    return 0;
}

// -------------------------- //
// --------- Debug ---------- //
// -------------------------- //

// This should be done after USART is configured
void SendDebug() {

    char buffer[256];
    for (int i = 0; i < 256; i++) {
        buffer[i] = 0;
    }

    sprintf(buffer, "SYSCLK source: %d\n", RCC_GetSYSCLKSource());
    SendData(USART2, buffer);
     /*Check SYSCLK, AHB, APB1 frequencies*/
    sprintf(buffer, "SYS/PLL: %d\n", (int)clkfreqs.SYSCLK_Frequency);
    SendData(USART2, buffer);
    sprintf(buffer, "AHB: %d\n", (int)clkfreqs.HCLK_Frequency);
    SendData(USART2, buffer);
    sprintf(buffer, "APB1: %d\n", (int)clkfreqs.PCLK1_Frequency);
    SendData(USART2, buffer);
}

// -------------------------- //
// ---------- PLL ----------- //
// -------------------------- //

void InitPLL(void) {

    /* Configure PLL output */
    uint32_t PLLM = 8, PLLN = 96, PLLP = 2, PLLQ = 2;
    RCC_PLLConfig(RCC_PLLSource_HSI, PLLM, PLLN, PLLP, PLLQ); // fPLL = fHSI*N/(M*P) = 96MHz

    /* Wait until PLL is ready */
    while(!RCC_GetFlagStatus(RCC_FLAG_PLLRDY));

    /* Use PLL as system clock */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // change SYSCLK to PLL
    /*fprintf(stderr, "SYSCLK source: %d\n", RCC_GetSYSCLKSource());*/
}

// ---------------------------- //
// ---------- USART ----------- //
// ---------------------------- //

/* Configure USART parameters, esp. baud rate */
void InitUSART(void) {

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // Clock

    /* Configure AHB and APB1 for USART clock */
    RCC_HCLKConfig(RCC_SYSCLK_Div2); // fAHB = fSYS/2 = 48MHz
    RCC_PCLK1Config(RCC_HCLK_Div2); // fAPB1 = fAHB/2 = 24MHz
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // turn on AHB
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // turn on APB1

    /* Software debugging; check clock frequencies */
    RCC_GetClocksFreq(&clkfreqs);
    //fprintf(stderr, "SYS/PLL: %d\n", clkfreqs.SYSCLK_Frequency);
    //fprintf(stderr, "AHB: %d\n", clkfreqs.HCLK_Frequency);
    //fprintf(stderr, "APB1: %d\n", clkfreqs.PCLK1_Frequency);

    /* Physical debugging; check MCO1/MCO2 with O-scope */
    RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_1); // check SYSCLK w/ no division
    // RCC_MCO2Source_SYSCLK: System clock (SYSCLK) selected as MCO2 source
    // RCC_MCO2Source_HSE: HSE clock selected as MCO2 source
    // RCC_MCO2Source_PLLCLK: main PLL clock selected as MCO2 source
    RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1); // check another clock with no division
    // RCC_MCO1Source_HSI: HSI clock selected as MCO1 source
    // RCC_MCO1Source_LSE: LSE clock selected as MCO1 source
    // RCC_MCO1Source_HSE: HSE clock selected as MCO1 source
    // RCC_MCO1Source_PLLCLK: main PLL clock selected as MCO1 source

    // I/O

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Use PA2 and PA3 for Tx and Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    // Conf

    USART_OverSampling8Cmd(USART2, ENABLE); // oversample by 8 to increase baud rate cap

    USART_InitStructure.USART_BaudRate = 2e6;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure); // initialize with parameteters

    // Enable the interrupt
    /*USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);*/

    // Enable
    USART_Cmd(USART2, ENABLE);
}

/* Writes out a string to the passed in USART */
/* The string is passed as a pointer */
void SendData(USART_TypeDef* USARTx, volatile char *s){

    while(*s) {
        // wait until data register is empty
        while( !(USARTx->SR & 0x00000040) );
        USART_SendData(USARTx, *s);
        *s++;
    }
}

// --------------------------- //
// ---------- GPIO ----------- //
// --------------------------- //

/* Configure GPIO pins */
void InitGPIOD(void) {

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure GPIOD output pins for LEDs */

    // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Configure PD12, PD13, PD14 and PD15 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Configure output pins for MCO1, MCO2 */
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Configure PA8 in output mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Configure PC9 in output mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* Use LEDs (GPIOD outputs) as a 4-bit binary counter */
void IncrementCounter(void) {

    int LED_MASK = 0b1111 << 12;
    GPIOD->ODR += (1 << 12);
    if (GPIOD->ODR & (LED_MASK == 0b1111))
        GPIOD->ODR ^= 0b1111 << 12;
}

// -------------------------- //
// ---------- DAC ----------- //
// -------------------------- //

void InitDAC(void) {

    /*DAC_InitTypeDef DAC_InitStructure;*/
    GPIO_InitTypeDef GPIO_InitStructure;

    // Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configuration
    //DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
    //DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    //DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
    //DAC_Init(DAC_Channel_1, &DAC_InitStructure);

    // IO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable DAC Channel1
    //DAC_Cmd(DAC_Channel_1, ENABLE);

    // Set DAC Channel1 DHR12L register
    //DAC_SetChannel1Data(DAC_Align_12b_R, 0);
}

// ---------------------------- //
// ---------- Delay ----------- //
// ---------------------------- //

/* Delay a number of systick cycles (1ms) */
void Delay(volatile uint32_t nCount) {
    time_var1 = nCount;
    while(time_var1){};
}

// --------------------------------- //
// ---------- Interrupts ----------- //
// --------------------------------- //

/* Configure external input interrupt (pin PD0) */
void INTPD0_Config(void) {

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clocks for GPIOD and SYSCGF */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

/* Configure timer interrupt (TIM2) */
void INTTIM2_Config(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 1 MHz down to 1 KHz (1 ms)
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* TIM IT enable */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
}

/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {

    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
    	if (debounce == 0) {
    	    IncrementCounter();
    	    SendData(USART2, "Hello world\n\r");
    	    debounce = 500;
    	}

    	/* Clear interrupt flag */
    	EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/* Handle timer interrupt */
void TIM2_IRQHandler(void) {

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    	if (time_var1) {
    	    time_var1--;
    	}
    	time_var2++;
    	if (debounce) { // debounce pin 0
    	    debounce--;
    	}
    	if (time_var2 % 100 == 0) {
            //Do stuff
            //IncrementCounter();
    	}
    }
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {
}
