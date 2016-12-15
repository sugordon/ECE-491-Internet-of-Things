/*
 * main.h
 *
 *  Created on: 10 jul 2012
 *      Author: BenjaminVe
 */

#ifndef MAIN_H_
#define MAIN_H_

// Function prototypes

#define N_TASKS 3
#define TIMER_TICK 10

typedef struct task {
	unsigned long period; // task's period 
	unsigned long elapsed_time; // since task's last tick
	void (*task_function)(void); // task's function, called during task's tick
} task;

void INTTIM_Config(void);

task tasks[N_TASKS];

void Task1Function(void);
void Task2Function(void);
void Task3Function(void);

//void (*task_functions[N_TASKS])(void);

//const unsigned long task_periods[N_TASKS] = {100, 200, 400};

#endif /* MAIN_H_ */
