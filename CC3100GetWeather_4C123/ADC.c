// ADC.c
// Runs on TM4C123
// Provide functions that initialize ADC0 SS3 to be triggered by
// software and trigger a conversion, wait for it to finish,
// and return the result.
// Kinan Hernandez / Alice Lam
// August 6, 2015

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
//#include <stdlib.h>
//#include <stdio.h>
//#include "ADCSWTrigger.h"
//#include "PLL.h"
//#include "ST7735.h"
//#include "string.h"
//#include "Timer1.h"
//#include  "tm4c123gh6pm_startup_ccs.c"
#include "ST7735.h"


#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))
/*	
void DisableInterrupts(void); // Disable interrupts
*/
void DisableInterrupts(void){
	__asm ("    CPSID  I\n");
}

void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// There are many choices to make when using the ADC, and many
// different combinations of settings will all do basically the
// same thing.  For simplicity, this function makes some choices
// for you.  When calling this function, be sure that it does
// not conflict with any other software that may be running on
// the microcontroller.  Particularly, ADC0 sample sequencer 3
// is used here because it only takes one sample, and only one
// sample is absolutely needed.  Sample sequencer 3 generates a
// raw interrupt when the conversion is complete, but it is not
// promoted to a controller interrupt.  Software triggers the
// ADC0 conversion and waits for the conversion to finish.  If
// somewhat precise periodic measurements are required, the
// software trigger can occur in a periodic interrupt.  This
// approach has the advantage of being simple.  However, it does
// not guarantee real-time.
//
// A better approach would be to use a hardware timer to trigger
// the ADC0 conversion independently from software and generate
// an interrupt when the conversion is finished.  Then, the
// software can transfer the conversion result to memory and
// process it after all measurements are complete.

// This initialization function sets up the ADC according to the
// following parameters.  Any parameters not explicitly listed
// below are not modified:
// Max sample rate: <=125,000 samples/second
// Sequencer 0 priority: 1st (highest)
// Sequencer 1 priority: 2nd
// Sequencer 2 priority: 3rd
// Sequencer 3 priority: 4th (lowest)
// SS3 triggering event: software trigger
// SS3 1st sample source: Ain9 (PE4)
// SS3 interrupts: enabled but not promoted to controller
void ADC0_InitSWTriggerSeq3_Ch9(void){ 
   int32_t k=0;                               // 1) activate clock for Port E
 // SYSCTL_RCGCGPIO_R |= 0x10;
	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
  while(k<10000){
	k++;
	}
  
	//	GPIO_PORTE_DIR_R &= ~0x10;      // 2) make PE4 input
  //GPIO_PORTE_AFSEL_R |= 0x10;     // 3) enable alternate function on PE4
  //GPIO_PORTE_DEN_R &= ~0x10;      // 4) disable digital I/O on PE4
  //GPIO_PORTE_AMSEL_R |= 0x10;     // 5) enable analog functionality on PE4
  //
		 GPIO_PORTD_DIR_R &= ~0x02;  // 3.6) make PD1 input
      GPIO_PORTD_AFSEL_R |= 0x02; // 4.6) enable alternate function on PD1
      GPIO_PORTD_DEN_R &= ~0x02;  // 5.6) disable digital I/O on PD1
      GPIO_PORTD_AMSEL_R |= 0x02; // 6.6) enable analog functionality on PD1
//  while((SYSCTL_PRADC_R&0x0001) != 0x0001){};    // good code, but not yet implemented in simulator

	SYSCTL_RCGCADC_R |= 0x0001;   // 7) activate ADC0 
 // ADC0_PC_R &= ~0xF;              // 7) clear max sample rate field
  //ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
  //ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
  //ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
  //ADC0_EMUX_R &= ~0xF000;         // 10) seq3 is software trigger
  //ADC0_SSMUX3_R &= ~0x000F;       // 11) clear SS3 field
  //ADC0_SSMUX3_R += 9;             //    set channel
  //ADC0_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
  //ADC0_IM_R &= ~0x0008;           // 13) disable SS3 interrupts
  //ADC0_ACTSS_R |= 0x0008;         // 14) enable sample sequencer 3
  //ADC0_SAC_R = ADC_SAC_AVG_64X;		// 15) enable hardware averaging 64 bit

}


//------------ADC0_InSeq3------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
uint32_t ADC0_InSeq3(void){  uint32_t result;
  ADC0_PSSI_R = 0x0008;            // 1) initiate SS3
  while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done
    // if you have an A0-A3 revision number, you need to add an 8 usec wait here
  result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result
  ADC0_ISC_R = 0x0008;             // 4) acknowledge completion
  return result;
}

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

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
  PF2 ^= 0x04;                   // profile
  PF2 ^= 0x04;                   // profile
  ADCvalue = ADC0_InSeq3();
  PF2 ^= 0x04;                   // profile
}

uint32_t getADC(void){
	return ADCvalue; 
}
