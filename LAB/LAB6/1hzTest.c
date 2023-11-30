/**
******************************************************************************
* @author  EunChan Kim 21801017
* @Mod		 2023-10-31 by YKKIM  	
* @brief   Embedded Controller:  LAB - Timer Input Capture - with Ultrasonic Distance Sensor
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecUART_simple.h"
#include "ecSysTIck.h"

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

#define TRIG PA_6
#define ECHO PB_6

void setup(void);

int main(void){
	
	setup();
	
	while(1){

	}
}




void setup(){

	RCC_PLL_init(); 
	SysTick_init();
	UART2_init();
  
// PWM configuration ---------------------------------------------------------------------	
	PWM_init(TRIG);			// PA_6: Ultrasonic trig pulse
	PWM_period_ms(TRIG, 1000);    // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_ms(TRIG, 500);   // PWM pulse width of 10us
	
	
}
