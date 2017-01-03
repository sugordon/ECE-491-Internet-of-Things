/*
 * main.h
 *
 */

#ifndef MAIN_H_
#define MAIN_H_

/* For Ethernet */

// Enable DHCP, if disabled static address is used
#define USE_DHCP 

// Uncomment SERIAL_DEBUG to enables retarget of printf
// to serial port (COM1 on STM32 evalboard) 
// for debug purpose
#define SERIAL_DEBUG 

//MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5
#define MAC_ADDR0   2
#define MAC_ADDR1   0
#define MAC_ADDR2   0
#define MAC_ADDR3   0
#define MAC_ADDR4   0
#define MAC_ADDR5   0

// Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   0
#define IP_ADDR3   10

// NETMASK
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

// Gateway Address
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   0
#define GW_ADDR3   1

/* For the RIOS Task Scheduler */
#define N_TASKS 2
#define TIMER_TICK 10

typedef struct task {
	unsigned long period; // task's period 
	unsigned long elapsed_time; // since task's last tick
	void (*task_function)(void); // task's function, called during task's tick
} task;

task tasks[N_TASKS];

void Task1Function(void);
void Task2Function(void);
void Task3Function(void);

void INTTIM_Config(void);

//void (*task_functions[N_TASKS])(void);

//const unsigned long task_periods[N_TASKS] = {100, 200, 400};

#endif /* MAIN_H_ */
