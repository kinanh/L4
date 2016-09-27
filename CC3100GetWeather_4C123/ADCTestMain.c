// ADCTestMain.c
// Runs on TM4C123
// This program periodically samples ADC channel 0 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Daniel Valvano
// September 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// center of X-ohm potentiometer connected to PE3/AIN0
// bottom of X-ohm potentiometer connected to ground
// top of X-ohm potentiometer connected to +3.3V 
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "ADCSWTrigger.h"
#include "../inc/tm4c123gh6pm.h"
#include "PLL.h"
#include "fixed.h"
#include "ST7735.h"
#include "string.h"




//#include <cstdio>


#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

uint32_t timer[1000];
uint32_t ADC[1000];
int16_t xaxis[4096];
int16_t plot[4096]; //global arrays auto initialize to 0
int32_t XMax;
int32_t XMin;
int32_t YMax;
int32_t newindex;
uint32_t Jitter;
//uint32_t testadc;

volatile uint32_t ADCvalue;
// This debug function initializes Timer0A to request interrupts
// at a 100 Hz frequency.  It is similar to FreqMeasure.c.
void Timer0A_Init100HzInt(void){
  volatile uint32_t delay;
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  delay = SYSCTL_RCGCTIMER_R;      // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = 0;                // configure for 32-bit timer mode
  // **** timer0A initialization ****
                                   // configure for periodic mode
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAILR_R = 799999;         // start value for 100 Hz interrupts
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 32-b, periodic, interrupts
  // **** interrupt initialization ****
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = 1<<19;              // enable interrupt 19 in NVIC
}

// **************SysTick_Init*********************
// Initialize SysTick periodic interrupts
// Input: interrupt period
//        Units of period are 12.5ns (assuming 50 MHz clock)
//        Maximum is 2^24-1
//        Minimum is determined by length of ISR
// Output: none
void SysTick_Init(uint32_t period){long sr;
  sr = StartCritical();
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  //EndCritical(sr);
}



// ***************** TIMER1_Init ****************
// Activate TIMER1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer1_Init(void){
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
	// PeriodicTask = task;          // user function
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = 0xFFFFFFFF;    // 4) reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
// TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00008000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 37, interrupt number 21
  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 21 in NVIC
  TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
  PF2 ^= 0x04;                   // profile
  PF2 ^= 0x04;                   // profile
  ADCvalue = ADC0_InSeq3();
	if (newindex<1000){
	ADC[newindex] = ADCvalue;
	timer[newindex] = TIMER1_TAR_R;
	newindex = newindex+1;
	}
  PF2 ^= 0x04;                   // profile
}

void SysTick_Handler(void){
  PF2 ^= 0x02;                // toggle PF2                // toggle PF2
}
// returns jitter in 12.5ns units
void Time_Jitter(void){
	uint32_t temp;
	uint32_t l = 0;
	uint32_t lastT = timer[l]; 
	uint32_t newT = timer[l+1];
	uint32_t max = lastT-newT;
	uint32_t min = max; 
	for(l=1; l<999;l++){
		lastT= timer[l];	
		newT = timer[l+1];
		if(lastT>newT){										// no rollover, calculate time difference
			temp = lastT-newT;
		} else if(lastT<newT) {						//rollover occured, calculate time difference
			temp = 0xFFFFFFFF - newT +lastT; 
		} else{
		temp = 0;
		}									//time difference of 0, error 
		// update min and max values
		if (temp>max){
			max = temp;
		}
		if (temp<min){
			min = temp;
		}
	}
	Jitter = max - min;  
}

// Counts frequency of ADC outputs and stores data in array plot[]
void ADC_Noise(void){
	uint32_t max = ADC[0];
	uint32_t min = ADC[0];
	uint32_t in;
	int32_t maxy=0;
	//uint32_t ymax = 0; // may be useful to impliment later for graphing bounds
  for(int32_t l=0; l<1000;l++){
		in = ADC[l];
		//test = plot[in];
		plot[in] = plot[in] + 1;
		//test = plot[in];
		if (ADC[l]>max){
			max = ADC[l];
		}
		if (ADC[l]<min){
			min = ADC[l];
		}
	}
	for (int32_t k=0; k<4096;k++){
		if (plot[k]>maxy){
		maxy = plot[k];
		}
	}
	
	XMax = max;
	XMin = min;
	YMax = maxy;
}

int main(void){	
  PLL_Init(Bus80MHz);         // 80 mhz
	ST7735_InitR(INITR_REDTAB);	// screen
  SYSCTL_RCGCGPIO_R |= 0x20;            // activate port F
  ADC0_InitSWTriggerSeq3_Ch9();         // allow time to finish activating
 Timer0A_Init100HzInt();               // set up Timer0A for 100 Hz interrupts
 Timer1_Init();
	//SysTick_Init(8005);										// slightly off 8000 to intertupt at 10khz
  GPIO_PORTF_DIR_R |= 0x06;             // make PF2, PF1 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x06;          // disable alt funct on PF2, PF1
  GPIO_PORTF_DEN_R |= 0x06;             // enable digital I/O on PF2, PF1
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF00F)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
	
  PF2 = 0;                     				 	// turn off LED
  EnableInterrupts();
	//PF1 = 0x02; 
  while(newindex<1000){
    PF1 ^= 0x02;  											// toggles when running in main
  }
	Time_Jitter();
	ADC_Noise();
	for(uint16_t l=0; l<4096; l++){
		xaxis[l] = l;
	}
	ST7735_FillScreen(0);  // set screen to black
  ST7735_SetCursor(0,0);
	char title[20];
	 int32_t jit;
	 jit = Jitter*12.5;
	sprintf(title, "PMF; Jitter=%d ns ",jit); 
	ST7735_XYplotInit(title,XMin-2, XMax+2, -10, YMax);
  ST7735_XYplot(4096, xaxis, plot);
	while(1){
		PF1 = 0x00;
	}
}

