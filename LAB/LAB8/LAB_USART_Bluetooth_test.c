/**
******************************************************************************
* @author  EunChan Kim 21801017
* @Mod		 2023-11-10 by YKKIM  	
* @brief   Embedded Controller:  LAB_USART_Bluetooth
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"
#include "ecPinNames.h"
#include "ecPWM.h"

#define MAX_BUF 	10
#define END_CHAR 	13

#define UP       'W'
#define RIGHT    'D'
#define LEFT     'A'
#define STOP     'S'


static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

float R_duty=0;
float L_duty=0;

void setup(void);


void main(){	
	setup();
  while(1){
		GPIO_write(GPIOC,2,0);
		GPIO_write(GPIOC,3,0);
		PWM_duty(PA_0, R_duty);
		PWM_duty(PA_1, L_duty);
		}
}

void setup(void){
  RCC_PLL_init();
	SysTick_init();
	
  // BT serial init 
  UART1_init();
  UART1_baud(BAUD_9600);
	
	USART_setting(USART1, GPIOA, 9, GPIOA, 10, BAUD_9600);
	
	GPIO_init(GPIOA,5,OUTPUT);
	GPIO_init(GPIOC,2,OUTPUT);
	GPIO_init(GPIOC,3,OUTPUT);

	PWM_init(PA_0);	
	PWM_period_us(PA_0, 200);
	PWM_init(PA_1);	
	PWM_period_us(PA_1, 200);

}

void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
    BT_Data = USART1_read();		// RX from UART1 (BT)
		USART1_write(&BT_Data,1);		
		if(BT_Data==UP){
			R_duty=0.8;
			L_duty=0.8;
	  }
		else if(BT_Data==LEFT){
			R_duty=0.8;
			L_duty=0.5;
		}
		else if(BT_Data==RIGHT){
			R_duty=0.5;
			L_duty=0.8;
		}
		else if(BT_Data==STOP){
			R_duty=0;
			L_duty=0;
		}
		else if(BT_Data == 'L'){
       BT_Data = USART_read(USART1);
       if(BT_Data == '0'){
          GPIO_write(GPIOA,5, 0);
          USART_write(USART1, (uint8_t*) "L0", 5);
       }
			 else if(BT_Data == '1'){
          GPIO_write(GPIOA,5, 1);
          USART_write(USART1, (uint8_t*) "L1", 5);
       }
		 }

	}
	USART_write(USART1, "\r\n", 1);

}