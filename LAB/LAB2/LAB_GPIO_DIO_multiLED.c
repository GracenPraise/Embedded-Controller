/**
******************************************************************************
* @author	EunChan Kim 21801017
* @Mod		23.09.25
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - As Button B1 is Pressed, light one LED at a time, in sequence.
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PA5 5 
#define LED_PA6 6 
#define LED_PA7 7 
#define LED_PB6 6 
#define BUTTON_PIN 13

void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	RCC_GPIOA_enable();
	RCC_GPIOB_enable();
	RCC_GPIOC_enable();
	
	setup();
	int cnt = 0;
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
			
			
			if(cnt == 1){ 
				
				if(GPIO_read(GPIOB,LED_PB6) == 1){ // When the count is 1, if the LED on the PB6 is on, turn it off and turn on PA5.
					bit_toggling(GPIOB,LED_PB6);
				} 
				bit_toggling(GPIOA,LED_PA5); // Turn on PA5	
			}
			else if(cnt == 2){ // Turn off PA5 and turn on PA6 when the count is 2.
				bit_toggling(GPIOA,LED_PA5);
				bit_toggling(GPIOA,LED_PA6);
			}
			else if(cnt == 3){ // Turn off PA6 and turn on PA7 when the count is 3.
				bit_toggling(GPIOA,LED_PA6);
				bit_toggling(GPIOA,LED_PA7);
			}
			else if(cnt == 4){ // Turn off PA7 and turn on PB6 when the count is 4.
				bit_toggling(GPIOA,LED_PA7);
				bit_toggling(GPIOB,LED_PB6);
				cnt = 0; // Initialize the Count 
			}
			for(volatile int i = 0; i < 300000; i++){} // for debouncing		
			cnt++;  // Increase the count by 1 with each button pressed	
		}			
	}
}


// Initialiization 
void setup(void){
	
	RCC_HSI_init();	
	
	// LED pin configuration
	GPIO_init(GPIOA, LED_PA5, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA5, EC_PU);	// pull-up resistor
	GPIO_otype(GPIOA, LED_PA5, 0); // Set LED PA5 as an Push-Pull
	GPIO_ospeed(GPIOA, LED_PA5, 01); // Set medium speed
	GPIO_write(GPIOA, LED_PA5, LOW); // Set state as Low
	
	GPIO_init(GPIOA, LED_PA6, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA6, EC_PU);	// pull-up resistor
	GPIO_otype(GPIOA, LED_PA6, 0); // Set LED PA6 as an Push-Pull
	GPIO_ospeed(GPIOA, LED_PA6, 01); // Set medium speed
	GPIO_write(GPIOA, LED_PA6, LOW); // Set state as Low
	
	GPIO_init(GPIOA, LED_PA7, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PA7, EC_PU);	// pull-up resistor
	GPIO_otype(GPIOA, LED_PA7, 0); // Set LED PA7 as an Push-Pull
	GPIO_ospeed(GPIOA, LED_PA7, 01); // Set medium speed
	GPIO_write(GPIOA, LED_PA7, LOW); // Set state as Low
	
	GPIO_init(GPIOB, LED_PB6, OUTPUT);    // calls RCC_GPIOB_enable()
	GPIO_pupd(GPIOB, LED_PB6, EC_PU);	// pull-up resistor
	GPIO_otype(GPIOB, LED_PB6, 0); // Set LED PB6 as an Push-Pull
	GPIO_ospeed(GPIOB, LED_PB6, 01); // Set medium speed
	GPIO_write(GPIOB, LED_PB6, LOW); // Set state as Low
	
	// Button pin configuration
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU); // pull-up resistor
}