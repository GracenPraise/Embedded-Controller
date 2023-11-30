#include "stm32f411xe.h"
#include "math.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h" 
#include "ecEXTI.h"

// Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define PWM_PIN PA_0
#define DIR_PIN 0

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);
int cnt = 0;
int pause = 1;
int flag = 0;

int main(void) {
	setup();	
	
	while(1){
		}		
}

void setup(void) {   
   // System clock setting
   RCC_PLL_init();
	
   
   // Pin setting
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);
   GPIO_init(GPIOC, DIR_PIN, OUTPUT);
	 GPIO_otype(GPIOC, DIR_PIN, EC_PUSH_PULL); // Set LED PA5 as an Push-Pull
	 GPIO_write(GPIOC, DIR_PIN, HIGH);
   
   // Timer setting 

	 TIM_UI_init(TIM3, 500);			// TIM3 Update-Event Interrupt every 500 msec 
	 TIM_UI_enable(TIM3);
   NVIC_EnableIRQ(TIM3_IRQn);   // TIM3 interrupt request enabled   
   NVIC_SetPriority(TIM3_IRQn, 2); // TIM3 interrupt priority
   
   // EXTI setting
   EXTI_init(GPIOC, 13, FALL, 0); // Initialize C port 13 pin
   NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable request interrupt
   NVIC_SetPriority(EXTI15_10_IRQn, 3); // Set priority
    
   // PWM setting
   PWM_init(PWM_PIN); // set PA 1 as PWM output pin
   PWM_period(PWM_PIN, 1); // 1msec PWM period
}

void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) {
				if(pause==0){ // toggle the pause
					pause = 1;
				}
				else{
					pause = 0;
				}
        clear_pending_EXTI(BUTTON_PIN);
   }
} 

void TIM3_IRQHandler(void){
  if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){ 
    GPIO_write(GPIOC, DIR_PIN,LOW); // Set PC0 to LOW
    
    switch(flag){
      case 0 : 
        PWM_duty(PWM_PIN, 0.25*pause); // Set duty ratio to 25%
        break;
      case 1 :
        PWM_duty(PWM_PIN, 0.75*pause); // Set duty ratio to 75%
        break;  
    }
    
    cnt ++;
  
    if(cnt%4==0){
      flag ^= 1; // flag switching
    }

    TIM3->SR &= ~ TIM_SR_UIF;                    
  }
}
