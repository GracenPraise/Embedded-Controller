/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : EunChan Kim 21801017
Modified         : 10-13-2023
Language/ver     : C++ in Keil uVision

Description      : [LAB] EXTI 
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"

int cnt = 0;

void EXTI15_10_IRQHandler(void);
void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// Initialiization 
void setup(void)
{
	RCC_HSI_init();
	sevensegment_display_init();																// LED configuration
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);				// EXTI initialization , button pin, priority : 0
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);			// BUTTON_PIN : INPUT
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);			// BUTTON_PIN : PULL_UP

}

void EXTI15_10_IRQHandler(void) { 
	
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		cnt ++;
		sevensegment_display(cnt%10);
		for(volatile int i = 0; i < 500000; i++){}
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
	
}
