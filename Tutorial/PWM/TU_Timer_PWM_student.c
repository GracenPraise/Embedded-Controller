/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  Tutorial _PWM
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"

#define LED_PIN 	5

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	RCC_PLL_init();                         // System Clock = 84MHz
	SysTick_init();                         // for delay_ms()
	GPIO_init(GPIOA, LED_PIN, 2);     // GPIOA 5 ALTERNATE function
	GPIO_ospeed(GPIOA, LED_PIN, EC_HIGH);   // GPIOA 5 HIGH SPEED
	GPIO_pupd(GPIOA, 5, EC_NONE);
	GPIO_otype(GPIOA, 5,EC_PUSH_PULL);
	
	// TEMP: TIMER Register Initialiization --------------------------------------------------------		
	TIM_TypeDef *TIMx;
	TIMx = TIM2;
	
	// GPIO: ALTERNATIVE function setting
	GPIOA->AFR[0]	 =  1<<(4*LED_PIN);          						// AF1 at PA5 = TIM2_CH1 (p.150)
	
	// TIMER: PWM setting
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;              				// Enable TIMER clock
	
	TIMx->CR1 &= 	~TIM_CR1_DIR;           		// Direction Up-count
	
	uint32_t prescaler = 839;
	TIMx->PSC = prescaler; 						              // Set Timer CLK = 100kHz : (PSC + 1) = 84MHz/100kHz --> PSC = ?
	
	TIMx->ARR = 99;									        // Auto-reload: Upcounting (0...ARR). 
																				// Set Counter CLK = 1kHz : (ARR + 1) = 100kHz/1kHz --> ARR = ?
	
	TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;  			// Clear ouput compare mode bits for channel 1
	TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;                  			// OC1M = 110 for PWM Mode 1 output on ch1
	TIMx->CCMR1	|= TIM_CCMR1_OC1PE;    		// Output 1 preload enable (make CCR1 value changable)
	
	TIMx->CCR1 = 99/2;      										// Output Compare Register for channel 1 	
	
	TIMx->CCER &= ~TIM_CCER_CC1P;    			// select output polarity: active high	
	TIMx->CCER |= TIM_CCER_CC1E;												// Enable output for ch1
	
	TIMx->CR1  |= TIM_CR1_CEN;      			// Enable counter
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
			//Create the code to change the brightness of LED as 10kHZ (use "delay(1000)")
				for (int i=0;i<3;i++){
					TIM2->CCR1 = 99*i/2;
					delay_ms(1000);
				}
		}
	}
// Initialiization 
void setup(void)
{	
	RCC_PLL_init();       // System Clock = 84MHz
	SysTick_init();       // for delay_ms()
}



