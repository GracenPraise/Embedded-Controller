/**
******************************************************************************
* @author  EunChan Kim 21801017
* @Mod		 2023-11-07 by YKKIM  	
* @brief   Embedded Controller:  LAB: LAB_USART_LED
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"


static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;


void setup(void){
  RCC_PLL_init();
	SysTick_init();
  
  // USB serial init
  UART2_init();
  UART2_baud(BAUD_38400);

	
	USART_setting(USART1,GPIOA,10,GPIOA,9,BAUD_38400);
	GPIO_init(GPIOA,5,OUTPUT);
	
}


void main(){	
	setup();
    
    	while(1){

    }
}


void USART2_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART2_RXNE()){
		PC_Data = USART2_read();		// RX from UART2 (PC)
		USART2_write(&PC_Data,1);		// TX to USART2	 (PC)
			
		USART1_write(&PC_Data,1);		// TX to USART1	 (BT)	
		printf("MCU_1 sent : %c \r\n",PC_Data); // TX to USART2(PC)
	}
}

void USART1_IRQHandler(){          		// USART2 RX Interrupt : Recommended
	if(is_USART1_RXNE()){
        	BT_Data = USART1_read();		// RX from UART1 (BT)

		printf("MCU_1 received : %c \r\n",BT_Data); // TX to USART2(PC)
		if(BT_Data == 'L'){
				GPIO_write(GPIOA,5,LOW);
		}
		else if(BT_Data == 'H'){
				GPIO_write(GPIOA,5,HIGH);
		}
	}
}