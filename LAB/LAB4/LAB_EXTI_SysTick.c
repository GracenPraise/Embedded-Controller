/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : EunChan Kim 21801017
Modified         : 10-13-2023
Language/ver     : C++ in Keil uVision

Description      : [LAB] SysTick 
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

int cnt = 0;

void setup(void);
void EXTI15_10_IRQHandler(void);


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(cnt%10);
		delay_ms(1000);
		cnt ++;
		
		if (cnt >9){
			cnt =0;
		}
		SysTick_reset();
	}
}


void setup(void){

	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);				// EXTI initialization , button pin, priority : 0
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);			// BUTTON_PIN : INPUT
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);			// BUTTON_PIN : PULL_UP

}


void EXTI15_10_IRQHandler(void) { 
	
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		cnt = 0;
		sevensegment_display(cnt%10);
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
	
}

