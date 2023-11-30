/**
******************************************************************************
* @author	EunChan Kim 21801017
* @Mod		23.10.06
* @brief	Embedded Controller:  LAB: GPIO Digital InOut 7-segment
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
#define LED_PC7 7
#define LED_PA9 9
#define LED_PA8 8
#define LED_PB10 10
#define BUTTON_PIN 13

void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
			cnt++;
			//sevensegment_decoder(cnt % 10);
			sevensegment_display(cnt % 10);				
		}			
		
		for(volatile int i = 0; i < 500000; i++){}
	
	}

}


// Initialiization 
void setup(void){
	RCC_HSI_init();	
	//sevensegment_init();
	sevensegment_display_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU); // pull-up resistor
}