/**
******************************************************************************
* @author  EunChan Kim 21801017
* @Mod		 2023-11-03 by YKKIM  	
* @brief   Embedded Controller:  LAB: Stepper Motor
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecSysTIck.h"
#include "ecStepper.h"


#define BUTTON_PIN 13


void setup(void);
void EXTI15_10_IRQHandler(void);

uint32_t flag = 0;

int main(void){
	
	setup();
	
	Stepper_step(2048, 0, FULL); //steps/rev = 64x32
	
	while(1){
		if(flag == 1){
			Stepper_stop();
		}		
		else{
			Stepper_step(2048, 0, FULL);		
		} 		 						
	}
}


void setup(){

	RCC_PLL_init(); 
	SysTick_init();
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	
	Stepper_init(GPIOB, 10, GPIOB, 4, GPIOB, 5, GPIOB, 3);
	Stepper_setSpeed(1);
}
void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) {
			flag ^= 1;
			Stepper_stop();
			clear_pending_EXTI(BUTTON_PIN);
   }
} 