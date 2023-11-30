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
#define PWM_PIN PA_1

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);
int cnt = 0;

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
   PWM_period(PWM_PIN, 20); // 20msec PWM period
}

void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) {
      PWM_duty(PWM_PIN, 0.025);
        cnt = 0;
        clear_pending_EXTI(BUTTON_PIN);
   }
} 

void TIM3_IRQHandler(void){
   if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF){ 
      PWM_duty(PWM_PIN, 0.025 + (cnt / 17.0) * 0.1); // Increase duty cycle 18 times from 0.5 msec to 2.5 msec
      cnt ++;
      if(cnt > 17) cnt = 0; // for reset

      TIM3->SR &= ~ TIM_SR_UIF;                    
   }
}
