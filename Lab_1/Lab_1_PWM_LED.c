//Michael Marin
//Lab 1 - PWM LED

#include "tm4c123gh6pm.h"

#define LED               (*((volatile unsigned long *)0x400253B8))
#define SWITCHES          (*((volatile unsigned long *)0x400253C4))

// prototypes
void SysTick_Init(void);
void PortF_Init(void);				// port F settings
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

// global variables
unsigned long H,L;
unsigned char sw1 				= 0x01;
unsigned char sw2 				= 0x10;
unsigned char on 				= 0x04;  // blue led
unsigned char off				= 0x00;  // no signal
unsigned long count = 0;

int main(void){
  DisableInterrupts();  // disable interrupts while initializing
	SysTick_Init();				
	PortF_Init();
  EnableInterrupts();   // enable after all initialization are done
  
	while(1){

    WaitForInterrupt(); // low power mode
  }
}

void SysTick_Init(void){
  H = L = 40000;					// 50%
  NVIC_ST_CTRL_R = 0;         		// disable SysTick during setup
  NVIC_ST_RELOAD_R = L-1;			// reload value
  NVIC_ST_CURRENT_R = 0;      		// any write to current clears it
 
	NVIC_SYS_PRI3_R =				// priority 2
		(NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; 
									// enable SysTick with core clock and interrupts	
  NVIC_ST_CTRL_R = 0x07;
	
}

void SysTick_Handler(void){
  if(LED){  										
    LED = off;
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
  } else{
    LED = on;
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
  }
	count = count +1;
}
void PortF_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock  
  delay = SYSCTL_RCGC2_R;           // Delay
 
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock PortF PF0  

  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R &= ~0x1F;      // disable analog functionality on PF
  GPIO_PORTF_PCTL_R &= ~0xFFFFF;    // clears PortF GPIO
  GPIO_PORTF_DIR_R |= 0x0E;         // leds outputs
  GPIO_PORTF_DIR_R &= ~0x11;        // make built-in buttons inputs
	
  GPIO_PORTF_AFSEL_R &= ~0x1F;      // disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;         // enable digital I/O   
  GPIO_PORTF_PUR_R |= 0x11;         // enable weak pull-up on both switches

  GPIO_PORTF_PCTL_R &= ~0x1F; 			// configure PF4,0 as GPIO
	
  GPIO_PORTF_IS_R &= ~0x11;         // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;        //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;        //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;          // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;          // (f) arm interrupt on PF4,PF0
	
  LED = off;						// leds initially off
	
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) port f priority 2
  NVIC_EN0_R = 0x40000000;      		// (h) enable interrupt 30 in NVIC port f
	
}


// L range: 8000, 16000, 24000, 32000, 40000, 48000, 56000, 64000, 72000
// power:   10%    20%   30%    40%    50%    60%    70%    80%    90%
void GPIOPortF_Handler(void){ // called on touch of either SW1 or SW2
  if(SWITCHES == sw2){  						// SW2 touch (right side)
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag
    if(L > 8000) L = L-8000;  // slow down
  }
  if(SWITCHES == sw1){  									// SW1 touch (left side)
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag
    if(L < 72000) L = L + 8000;   // speed up
  }
	
  H = 80000-L; // constant period of 1ms, variable duty cycle
}


