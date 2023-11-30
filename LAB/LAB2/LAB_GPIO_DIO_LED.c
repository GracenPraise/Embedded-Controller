/**
******************************************************************************
* @author	EunChan Kim 21801017
* @Mod		23.09.25
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);


int main(void) { 
	// Initialiization --------------------------------------------------------
	RCC_GPIOA_enable();
	RCC_GPIOC_enable();
	
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
			
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
			for(volatile int i = 0; i < 300000; i++){} // for debouncing
			bit_toggling(GPIOA,LED_PIN);			
			
		}			
	}
}


// Initialiization 
void setup(void){
	
	RCC_HSI_init();	
	
	// LED pin configuration
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);		// pull-up resistor
	GPIO_otype(GPIOA, LED_PIN, 1);	// LED_PIN as open-drain output
	GPIO_ospeed(GPIOA, LED_PIN, 01); // Set medium speed
	GPIO_write(GPIOA, LED_PIN, LOW);	//Set LED_PIN to a LOW
	
	// button pin configuration
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU); // pull-up resistor
	
}


