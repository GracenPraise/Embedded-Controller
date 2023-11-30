#include "ecSysTick.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"

#define LED_PIN	5
#define BUTTON_PIN 13

// Initialiization 
void setup(void)
{
	RCC_PLL_init();
	SysTick_init();
	GPIO_init(GPIOA, LED_PIN, OUTPUT);
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PD);
	// Priority Highest(0) External Interrupt 
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
}

int main(void) {
	setup();
	while (1) {}
}

//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		bit_toggling(GPIOA, 5);
		clear_pending_EXTI(BUTTON_PIN); 
	}
}