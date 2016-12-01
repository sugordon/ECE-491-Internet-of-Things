/*
 * Course:          ECE491 Internet of Things
 * Instructors:     Alin Cosmanescu, Jared Harwayne-Gidansky
 * Project:         BT speaker with MP3 decoding
 * Group members:   Arber Duka, Jeffrey Shih, Gordon Su, Jerry Qiu
 *
 */

// ------------------------------ //
// ------ Headers & Macros ------ //
// ------------------------------ //

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "mp3dec.h"

#define BUFFER_SIZE 2048 // number of audio samples is 1152?
//FrameSize = 144 * BitRate / (SampleRate + Padding)

//#define MP3DECODE

// -------------------------------- //
// --- Prototypes & Definitions --- //
// -------------------------------- //

/* Private variables */

volatile uint32_t time_var1, time_var2, debounce;

volatile char sent_buffer[257];
volatile char received_buffer[BUFFER_SIZE+1];
static int16_t audio_buffer[BUFFER_SIZE];

MP3FrameInfo mp3FrameInfo;
HMP3Decoder hMP3Decoder;

/* Function prototypes */

void InitSYSCLK(void);
void InitUSART(void);
void InitGPIO(void);
void InitDAC(void);

void SendData(USART_TypeDef* USARTx, volatile char *s);
void ReceiveData(UsART_TypeDef *USARTx);

void SendDebug(void);

void DecodeMP3Data(void);
void SendMP3FrameInfo(void);
void IncrementCounter(void);

void Delay(volatile uint32_t nCount);

void INTPD0_Config();
void INTTIM2_Config();
void TIM2_IRQHandler(void);
void EXTI0_IRQHandler(void);

// --------------------------- //
// ---------- Main ----------- //
// --------------------------- //

int main(void) {

    /* Configure microcontroller */
    InitSYSCLK();
    InitGPIO();
    InitUSART();
    //InitDAC();

    /* Configure interrupts */
    INTTIM2_Config();
    INTPD0_Config();

    /* Debug stats */
    SendDebug();
    
    /* Set up MP3 decoder */
    hMP3Decoder = MP3InitDecoder();

    for(;;) {
        #ifdef MP3DECODE
        ReceiveData(USART2);
        DecodeMP3Data();
        #endif
    }

    return 0;
}

// -------------------------- //
// --------- Debug ---------- //
// -------------------------- //

// This should be done after USART is configured
void SendDebug(void) {

    /*
    for (int i = 0; i < sizeof(sent_buffer); i++) {
        buffer[i] = 0;
    }
    */

    /* Check SYSCLK source, should be 0x08 if PLL */
    sprintf(sent_buffer, "SYSCLK source: %d\n", RCC_GetSYSCLKSource());
    SendData(USART2, sent_buffer);

    /* Check prescalers, HSI, VCO, SYSCLK, AHB, APB1 frequencies */
    sprintf(sent_buffer, "fHSI: %d\n", HSI_VALUE); // 16MHz
    SendData(USART2, sent_buffer);

    int pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM
    int plln = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6
    int pllvco = HSI_VALUE * plln / pllm;
    sprintf(sent_buffer, "PLLN: %d\n", plln);    // 8
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "PLLM: %d\n", pllm);    // 96
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "fVCO: %d\n", pllvco);  // 192MHz
    SendData(USART2, sent_buffer);

    RCC_ClocksTypeDef clkfreqs;
    RCC_GetClocksFreq(&clkfreqs);
    int pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1 ) * 2;
    sprintf(sent_buffer, "PLLP: %d\n", pllp);    // 2
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "fSYS/fPLL: %d\n", (int)clkfreqs.SYSCLK_Frequency); // 96MHz
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "fAHB: %d\n", (int)clkfreqs.HCLK_Frequency);         // 48MHz
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "fAPB1: %d\n", (int)clkfreqs.PCLK1_Frequency);       // 24MHz
    SendData(USART2, sent_buffer);
}

// -------------------------- //
// ---------- MP3 ----------- //
// -------------------------- //

void DecodeMP3Data(void) {

    int offset, err;
    int bytes_left = BUFFER_SIZE;
    char *mp3_ptr = received_buffer;
    int16_t *samples = audio_buffer;

    /* Get start of next frame */
    offset = MP3FindSyncWord((unsigned char*)mp3_ptr, bytes_left);
    if (offset == -1) {
        SendData(USART2, "Sync not found.\n");
        return;
    }

    /* Decode frame */
    bytes_left -= offset;
    mp3_ptr += offset;
    err = MP3Decode(hMP3Decoder, (unsigned char**)&mp3_ptr, &bytes_left, samples, 0);
    if (err) {
        SendData("Error decoding MP3 frame: ");
        switch (err) {
            case ERR_MP3_INDATA_UNDERFLOW:
                SendData("ERR_MP3_INDATA_UNDERFLOW.\n");
                //outOfData = 1;
                break;
            case ERR_MP3_MAINDATA_UNDERFLOW:
                // do nothing - next call to decode will provide more mainData
                SendData("ERR_MP3_MAINDATA_UNDERFLOW.\n");
                break;
            case ERR_MP3_FREE_BITRATE_SYNC:
                SendData("ERR_MP3_FREE_BITRATE_SYNC.\n");
            default:
                //outOfData = 1;
                SendData("DEFAULT.\n");
                break;
        }
    } else {
        SendData(USART2, "Successfully decoded frame.\n");
        SendData(USART2, "Decoded samples:");
        SendData(USART2, (char *)samples);
        SendData("\n");
        MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
        SendMP3FrameInfo();
    }
}

void SendMP3FrameInfo(void) {
    SendData("Decoded frame information:\n");
    sprintf(sent_buffer, "Bitrate: %d\n", mp3DecInfo.bitrate);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Bitrate: %d\n", mp3DecInfo.bitrate);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Number of channels: %d\n", mp3DecInfo.nChans);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Sample rate: %d\n", mp3DecInfo.samprate);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Bits per sample: %d\n", mp3DecInfo.bitsPerSample);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Output samples: %d\n", mp3DecInfo.outputSamps);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Layer: %d\n", mp3DecInfo.layer);
    SendData(USART2, sent_buffer);
    sprintf(sent_buffer, "Version: %d\n", mp3DecInfo.version);
    SendData(USART2, sent_buffer);
}

// -------------------------- //
// -------- Counter --------- //
// -------------------------- //

/* Use LEDs (GPIOD outputs) as a 4-bit binary counter */
void IncrementCounter(void) {

    int LED_MASK = 0b1111 << 12;
    GPIOD->ODR += (1 << 12);
    if (GPIOD->ODR & (LED_MASK == 0b1111)) {
        GPIOD->ODR ^= 0b1111 << 12;
    }
}

// -------------------------- //
// --------- SYSCLK --------- //
// -------------------------- //

void InitSYSCLK(void) {

    /* Restore default settings so that PLL can be configured */
    RCC_DeInit();

    /* Disable PLL before configuring */
    RCC_PLLCmd(DISABLE);

    /* Configure PLL output */
    uint32_t PLLM = 8, PLLN = 96, PLLP = 2, PLLQ = 2;
    RCC_PLLConfig(RCC_PLLSource_HSI, PLLM, PLLN, PLLP, PLLQ); // fPLL = fHSI*N/(M*P) = 96MHz

    /* Enable PLL after configuring */
    RCC_PLLCmd(ENABLE);

    /* Wait until PLL is ready */
    while(!RCC_GetFlagStatus(RCC_FLAG_PLLRDY));

    /* Use PLL as system clock */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); // change SYSCLK to PLL
    /*fprintf(stderr, "SYSCLK source: %d\n", RCC_GetSYSCLKSource());*/
}

// --------------------------- //
// ---------- GPIO ----------- //
// --------------------------- //

/* Configure GPIO pins */
void InitGPIO(void) {

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO peripheral clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Configure GPIOD output pins for LEDs (PD12, PD13, PD14 and PD15) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Configure GPIOA and GPIOC output pins for MCO1 (PA8), MCO2 (PC9) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PA8 and PC9 for alternate function */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);

     /* Redirect clocks to MCO1 and MCO2, check with O-scope */
    RCC_MCO2Config(RCC_MCO2Source_SYSCLK, RCC_MCO2Div_1); // check SYSCLK w/ no division
    // RCC_MCO2Source_SYSCLK: System clock (SYSCLK) selected as MCO2 source
    // RCC_MCO2Source_HSE: HSE clock selected as MCO2 source
    // RCC_MCO2Source_PLLCLK: main PLL clock selected as MCO2 source
    RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1); // check another clock with no division
    // RCC_MCO1Source_HSI: HSI clock selected as MCO1 source
    // RCC_MCO1Source_LSE: LSE clock selected as MCO1 source
    // RCC_MCO1Source_HSE: HSE clock selected as MCO1 source
    // RCC_MCO1Source_PLLCLK: main PLL clock selected as MCO1 source
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
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // turn on AHB for Tx/Rx
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // turn on APB1

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
void SendData(USART_TypeDef* USARTx, volatile char *s) {

    while(*s) {
        // wait until previous transmission is complete
        while(!USART_GetFlagStatus(USARTx, USART_FLAG_TC)); // TC in USART_SR reg
        //while( !(USARTx->SR & 0x00000040) );
        USART_SendData(USARTx, *s);
        s++;
    }
}


/* Receive data until the buffer is full */
void ReceiveData(USART_TypeDef* USARTx){ 

    int i;
    for (i = 0; i < BUFFER_SIZE; i++) {
        // wait until received data is ready to be read
        while(!USART_GetFlagStatus(USARTx, USART_FLAG_RXNE)); // RXNE in USART_SR reg
        received_buffer[i] = USART_ReceiveData(USARTx);
    }
    received_buffer[BUFFER_SIZE] = '\0'; // put in for now
}

// -------------------------- //
// ---------- DAC ----------- //
// -------------------------- //

void InitDAC(void) {

    /*DAC_InitTypeDef DAC_InitStructure;*/
    GPIO_InitTypeDef GPIO_InitStructure;

    // Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

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
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

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
            //SendData(USART2, "Hello world\n\r");
            //MP3FreeDecoder(hMP3Decoder);
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
            //DecodeMP3Data();
        }
    }
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {
}
