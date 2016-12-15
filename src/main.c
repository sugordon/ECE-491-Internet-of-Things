#include <stdio.h>
#include "stm32f4xx_conf.h"

#include "main.h"

unsigned char isProcessing = 0; // flag indicating that tasks are being processed

int main(void) {
    unsigned char i = 0;
    for (i = 0; i < N_TASKS; i++) {
	tasks[i].period = 10;
	tasks[i].elapsed_time = tasks[i].period;
	tasks[i].task_function = (task_functions[i]);
    }

    //TimerSet(TIMER_TICK);
    //TimerOn();
    INTTIM_Config();

    while(1) {
	/*Sleep(); // until interrupted by ISR*/
    }

    return 0;
}

void Task1Function(void) {
    static unsigned char init = 1;
    if (init) { // Initialization behavior
	init = 0;
    } else { // Normal behavior
	// DO WHATEVER THE FUCK YOU WANT
    }
}

void Task2Function(void) {
    static unsigned char init = 1;
    if (init) { // Initialization behavior
	init = 0;
    } else { // Normal behavior
    }
}

void Task3Function(void) {
    static unsigned char init = 1;
    if (init) { // Initialization behavior
	init = 0;
    } else { // Normal behavior
    }
}


/* Handle timer interrupt */
void TIM_IRQHandler(void) {

    unsigned char i;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	if (!isProcessing) {
	    for (i = 0; i < N_TASKS; i++) {
		// Execute task function when period is reached
		if (tasks[i].elapsed_time >= tasks[i].period) {
		    tasks[i].task_function();
		    tasks[i].elapsed_time = 0; // reset elapsed time 
		}
		tasks[i].elapsed_time += TIMER_TICK;
	    }
	    isProcessing = 0;
	} else {
	    printf("Tick occurred prematurely\n");
	}
    }
}

/* Configure timer interrupts */
void INTTIM_Config(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* Select priority group 0, no preemption, only subpriorities */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Configure (3) timer interrupts */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*
       NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);

       NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);
       */

    /* Enable the clocks used for the timer */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /*
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
       */

    /* Time base configuration */
    // NOT SURE IF I DID THIS CORRECTLY
    // Assuming internal clock is used, refer to datasheet section about TIM2-5 (general purpose)
    //TIMx->CR1: TIM_CounterMode and TIM_ClockDivision, also determines the clock source
    //TIMx->ARR: TIM_Period
    //TIMx->PSC: TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 1 MHz down to 1 KHz (1 ms) i.e. each tick is 1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1; // 16 MHz Clock down to 1 MHz (adjust per your clock)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /*
       TIM_TimeBaseStructure.TIM_Period = 500 - 1; // 1 MHz down to 0.5 KHz (2 ms)
       TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
       TIM_TimeBaseStructure.TIM_ClockDivision = 0;
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
       TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

       TIM_TimeBaseStructure.TIM_Period = 250 - 1; // 1 MHz down to 0.25 KHz (4 ms)
       TIM_TimeBaseStructure.TIM_Prescaler = 16 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
       TIM_TimeBaseStructure.TIM_ClockDivision = 0;
       TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
       TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
       */

    /* Enable the timer interrupts */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    /*
       TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
       TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
       */

    /* Enable actual timers */
    TIM_Cmd(TIM2, ENABLE);
    /*
       TIM_Cmd(TIM3, ENABLE);
       TIM_Cmd(TIM4, ENABLE);
       */
}
/*
 * Dummy function to avoid compiler error
 */
void _init() {
}
