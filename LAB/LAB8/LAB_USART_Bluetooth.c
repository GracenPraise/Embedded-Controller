#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"
#include "ecPinNames.h"
#include "ecPWM.h"

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

float i=0;
float j=0;

void setup(void){
  RCC_PLL_init();

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


void main(){	
	setup();
  while(1){
		GPIO_write(GPIOC,2,LOW);
		GPIO_write(GPIOC,3,LOW);
		PWM_duty(PA_0, i);
		PWM_duty(PA_1, j);
		}
}

void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
    BT_Data = USART1_read();		// RX from UART1 (BT)
		USART1_write(&BT_Data,1);		
		if(BT_Data=='W'){
			i=0.8;
			j=0.8;
	  }
		else if(BT_Data=='A'){
			i=0.8;
			j=0.5;
		}
		else if(BT_Data=='D'){
			i=0.5;
			j=0.8;
		}
		else if(BT_Data=='S'){
			i=0;
			j=0;
		}
		else if (BT_Data == 'L') {
			BT_Data = USART_read(USART1);
			if (BT_Data == '0') {
				GPIO_write(GPIOA, 5, 0);
				USART_write(USART1, (uint8_t*)"L0", 5);
			}
			else if (BT_Data == '1') {
				GPIO_write(GPIOA, 5, 1);
				USART_write(USART1, (uint8_t*)"L1", 5);
			}
		}

	}
	USART_write(USART1, "\r\n", 1);
}