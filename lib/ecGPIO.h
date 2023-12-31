/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : EunChan Kim 21801017
Created          : 05-03-2021
Modified         : 09-20-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#define INPUT  0x00
#define OUTPUT 0x01
#define EC_AF     0x02
#define ANALOG 0x03

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13

#define EC_NONE 0
#define EC_PU 1
#define EC_PD 2

#define EC_PUSH_PULL 0
#define EC_OPEN_DRAIN 1

#define EC_LOW 0
#define EC_MEDIUM 1
#define EC_FAST 2
#define EC_HIGH 3

// 7 segement display
#define LED_PA0   0
#define LED_PA1 	1
#define LED_PA2		2
#define LED_PA5 	5
#define LED_PA6 	6
#define LED_PA7 	7
#define LED_PA8 	8
#define LED_PA9 	9
#define LED_PA10 	10

#define LED_PB0 	0
#define LED_PB3 	3
#define LED_PB4 	4
#define LED_PB5 	5
#define LED_PB6 	6
#define LED_PB9   9
#define LED_PB10 	10

#define LED_PC1 	1
#define LED_PC7 	7

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);

void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);

int  GPIO_read(GPIO_TypeDef *Port, int pin);

void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);

void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);

void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);

void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);

void bit_toggling(GPIO_TypeDef *Port, int pin);

void sevensegment_init(void); 

void sevensegment_decoder(uint8_t num);

void sevensegment_display_init(void); 

void sevensegment_display(uint8_t  num);

void LED_toggle(uint8_t cnt);

void sevensegment_display_LH(uint8_t num);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
