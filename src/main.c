#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "tm_stm32f4_usart.h"
#include "stm32f4xx_conf.h"

#define BUFFER_SIZE 16

void INTTIM_Config();
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

// Private variables
volatile uint32_t time_var1, time_var2, debounce;
volatile char received_buffer[BUFFER_SIZE+1];

void init();
void Delay(volatile uint32_t nCount);
void incrementCounter();
void EXTI0_IRQHandler(void);
void Configure_PA0(void);
void SendData(USART_TypeDef* USARTx, volatile char *s);
void USARTCommandReceived(char * command);
void ClearCommand();

//writes out a string to the passed in usart. The string is passed as a pointer
void SendData(USART_TypeDef* USARTx, volatile char *s){

    while(*s){
	// wait until data register is empty
	while( !(USARTx->SR & 0x00000040) );
	USART_SendData(USARTx, *s);
	*s++;
    }
}

int main(void) {
    init();
    SendData(USART2, "hello world");

    /*int LED_MASK = 0b1111 << 12;*/
    /*while(1) {*/
    /*GPIOD->ODR += (1 << 12);*/
    /*if(GPIOD->ODR & LED_MASK == 0b1111) */
    /*GPIOD->ODR ^= 0b1111 << 12;*/
    /*Delay(500);*/
    /*}*/


    for(;;) {
    }

    return 0;
}

//This method is executed when data is received on the RX line (this is the interrupt), this method can process the
//data thats been received and decide what to do. It is executed for each character received, it reads each character
//and checks to see if it received the enter key (ascii code 13) or if the total number of characters received is greater
//that the buffer size.
//Note that there is no reference to this method in our setup code, this is because the name of this method is defined in the
//startup_stm32f4xx.S (you can find this in the startup_src folder). When listening for interrupts from USART 2 or 3 you would
//define methods named USART2_IRQHandler or USART3_IRQHandler
/*void USART1_IRQHandler(void){*/
/*//check the type of interrupt to make sure we have received some data.*/
/*if( USART_GetITStatus(USART1, USART_IT_RXNE) ){*/
/*char t = USART1->DR; //Read the character that we have received*/

/*if( (DataReceivedCounter < BUFFER_SIZE) && t != 13 ){*/
/*received_buffer[DataReceivedCounter] = t;*/
/*DataReceivedCounter++;*/
/*}*/
/*else{ // otherwise reset the character counter and print the received string*/
/*DataReceivedCounter = 0;*/
/*//only raise a command event if the enter key was pressed otherwise just clear*/
/*if(t == 13){*/
/*USARTCommandReceived(received_buffer);*/
/*}*/

/*ClearCommand();*/

/*}*/
/*}*/
/*}*/

//this method is called when a command is received from the USART, a command is only received when enter
//is pressed, if the buffer length is exceeded the buffer is cleared without raising a command
/*void USARTCommandReceived(char * command){*/
/*SendData(USART1, received_buffer);*/

/*if        (compare(command, "L5ON") == 0){*/
/*STM_EVAL_LEDOn(LED5);*/
/*}else 	if(compare(command, "L5OFF") == 0){*/
/*STM_EVAL_LEDOff(LED5);*/

/*}else 	if(compare(command, "L6ON") == 0){*/
/*STM_EVAL_LEDOn(LED6);*/
/*}else 	if(compare(command, "L6OFF") == 0){*/
/*STM_EVAL_LEDOff(LED6);*/

/*}else 	if(compare(command, "L3ON") == 0){*/
/*STM_EVAL_LEDOn(LED3);*/
/*}else 	if(compare(command, "L3OFF") == 0){*/
/*STM_EVAL_LEDOff(LED3);*/

/*}else 	if(compare(command, "L4ON") == 0){*/
/*STM_EVAL_LEDOn(LED4);*/
/*}else 	if(compare(command, "L4OFF") == 0){*/
/*STM_EVAL_LEDOff(LED4);*/
/*}*/
/*}*/
/*void ClearCommand(){*/
/*int i =0;*/
/*for(i=0;i < BUFFER_SIZE; i++){*/
/*received_buffer[i] = 0;*/
/*}*/

/*}*/

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
    time_var1 = nCount;
    while(time_var1){};
}

/*
   void calculation_test() {
   float a = 1.001;
   int iteration = 0;

   for(;;) {
   int value = iteration % 4;
   switch (value) {
   case 0:
   GPIO_ResetBits(GPIOD, GPIO_Pin_14);
   GPIO_ResetBits(GPIOD, GPIO_Pin_12);
   break;
   case 1:
   GPIO_SetBits(GPIOD, GPIO_Pin_12);
   GPIO_ResetBits(GPIOD, GPIO_Pin_14);
   break;
   case 2:
   GPIO_SetBits(GPIOD, GPIO_Pin_14);
   GPIO_ResetBits(GPIOD, GPIO_Pin_12); break;
   case 3:
   GPIO_SetBits(GPIOD, GPIO_Pin_14);
   GPIO_SetBits(GPIOD, GPIO_Pin_12);
   break;
   }

   Delay(1000);

   time_var2 = 0;
   for (int i = 0;i < 1000000;i++) {
   a += 0.01 * sqrtf(a);
   }

   printf("Time:      %lu\n", time_var2);
   printf("Iteration: %i\n", iteration);
   printf("Value:     %lu\n", (unsigned long)a);
//		printf("Value F:   %.5f\n", -a);
printf("Value F2:  %s\n\n", ftostr(-a, 5));

iteration++;
}
}
*/

void init() {
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    //	DAC_InitTypeDef  DAC_InitStructure;

    // ---------- SysTick timer -------- //
    //if (SysTick_Config(SystemCoreClock / 1000)) {
    // Capture error
    //	while (1){};
    //}

    INTTIM_Config();
    Configure_PA0();

    // GPIOD Periph clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Configure PD12, PD13, PD14 and PD15 in output pushpull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    // ------ UART ------ //

    // Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // IO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    // Conf
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &USART_InitStructure);

    //Enable the interupt
    /*USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);*/

    // Enable
    USART_Cmd(USART2, ENABLE);


    // ---------- DAC ---------- //

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

/*
 * Called from systick handler
 *
 void timing_handler() {
 int value = (time_var2/5000) % 4;
 switch (value) {
 case 0:
 GPIO_ResetBits(GPIOD, GPIO_Pin_14);
 GPIO_ResetBits(GPIOD, GPIO_Pin_12);
 break;
 case 1:
 GPIO_SetBits(GPIOD, GPIO_Pin_12);
 GPIO_ResetBits(GPIOD, GPIO_Pin_14);
 break;
 case 2:
 GPIO_SetBits(GPIOD, GPIO_Pin_14);
 GPIO_ResetBits(GPIOD, GPIO_Pin_12);
 break;
 case 3:
 GPIO_SetBits(GPIOD, GPIO_Pin_14);
 GPIO_SetBits(GPIOD, GPIO_Pin_12);
 break;
 }
 if (time_var1) {
 time_var1--;
 }

 time_var2++;
 }
 */

/* Configure pins to be interrupts */
void Configure_PA0(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
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

/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
	if (debounce == 0) {
	    incrementCounter();
	      /*SendData(USART2, "tecsploits USART connection initialised, visit us @tecsploit.com");*/
	    TM_USART_Puts(USART2, "Hello world\n\r");
	    debounce = 500;
	}

	/* Clear interrupt flag */
	EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void incrementCounter(void)
{
    int LED_MASK = 0b1111 << 12;
    GPIOD->ODR += (1 << 12);
    if(GPIOD->ODR & (LED_MASK == 0b1111))
	GPIOD->ODR ^= 0b1111 << 12;
}


void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	if (time_var1) {
	    time_var1--;
	}
	time_var2++;
	if (debounce) {
	    debounce--;
	}
	if (time_var2 % 100 == 0) {
	    //Do stuff
	}
    }
}

void INTTIM_Config(void) {

    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM2 gloabal Interrupt */
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


/*
 * Dummy function to avoid compiler error
 */
void _init() {
}

