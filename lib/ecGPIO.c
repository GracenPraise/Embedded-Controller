/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : EunChan Kim 21801017
Created          : 05-03-2021
Modified         : 09-25-2023
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	if (Port == GPIOD)
		RCC_GPIOD_enable();
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	if (Port == GPIOH)
		RCC_GPIOH_enable();


	GPIO_mode(Port, pin, mode);
	
}

// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	Port->OSPEEDR &= ~(3UL<<(pin*2));  
	Port->OSPEEDR |= speed<<(pin*2);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
	Port->OTYPER &= ~(1UL<<pin);
	Port->OTYPER |= (type<<pin);
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
	Port->PUPDR &= ~(3UL<<(pin*2));
	Port->PUPDR |= pupd<<(pin*2);
}

int GPIO_read(GPIO_TypeDef *Port, int pin){
	int Val = (Port->IDR)>>pin & 1;
	return Val;    	
}

void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	Port->ODR &= ~(1UL<<pin);
	Port->ODR |= Output<<pin;
}

void bit_toggling(GPIO_TypeDef *Port, int pin){
	Port->ODR ^= (1<<pin);
	for(volatile int i = 0; i < 500000; i++){}
}




typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO;

void sevensegment_init(void) {
    GPIO pins[8] = {
        {GPIOA, 5},
        {GPIOA, 6},
        {GPIOA, 7},
        {GPIOB, 6},
        {GPIOC, 7},
        {GPIOA, 9},
        {GPIOA, 8},
        {GPIOB, 10}
    };

    // Initialize GPIO pins
    for (int i = 0; i < 8; ++i) {
			
        GPIO_init(pins[i].port, pins[i].pin, OUTPUT);
        GPIO_pupd(pins[i].port, pins[i].pin, EC_NONE);     // No Pull-up-Pull-down
        GPIO_otype(pins[i].port, pins[i].pin, EC_PUSH_PULL); // Set LED as Push-Pull
        GPIO_ospeed(pins[i].port, pins[i].pin, EC_MEDIUM);   // Set medium speed
        GPIO_write(pins[i].port, pins[i].pin, HIGH);        // Set state as HIGH
    }

    // initial state = 0
    for (int i = 0; i < 8; ++i) {
			
        if (pins[i].port == GPIOC && pins[i].pin == 7){ 
            GPIO_write(pins[i].port, pins[i].pin, HIGH);	
        } 
				else{
            GPIO_write(pins[i].port, pins[i].pin, LOW); // other led is LOW
        }
    }
}


unsigned int state[10][8]={
		
    {0,0,0,0,0,0,1,0}, //zero
    {1,0,0,1,1,1,1,0}, //one
    {0,0,1,0,0,1,0,0}, //two
    {0,0,0,0,1,1,0,0}, //three
    {1,0,0,1,1,0,0,0}, //four
    {0,1,0,0,1,0,0,0}, //five
    {0,1,0,0,0,0,0,0}, //six
    {0,0,0,1,1,0,1,0}, //seven
    {0,0,0,0,0,0,0,0}, //eight
    {0,0,0,0,1,0,0,0}, //nine
										
};


void sevensegment_decoder(uint8_t  num){
	
	GPIO_write(GPIOA, 8, state[num][0]);		// led a
	GPIO_write(GPIOB, 10,state[num][1]);		// led b
	GPIO_write(GPIOA, 7, state[num][2]);		// led c
	GPIO_write(GPIOA, 6, state[num][3]);		// led d
	GPIO_write(GPIOA, 5, state[num][4]);		// led e
	GPIO_write(GPIOA, 9, state[num][5]);		// led f
	GPIO_write(GPIOC, 7, state[num][6]);		// led g
	GPIO_write(GPIOB, 6, state[num][7]);		// led dp
}

void sevensegment_display_init(void){
	    GPIO pins[4] = {
        {GPIOA, 7},
        {GPIOB, 6},
        {GPIOC, 7},
        {GPIOA, 9},
    };
		// Initialize GPIO pins
    for (int i = 0; i < 4; ++i) {
			
        GPIO_init(pins[i].port, pins[i].pin, OUTPUT);
        GPIO_pupd(pins[i].port, pins[i].pin, EC_NONE);     // No Pull-up-Pull-down
        GPIO_otype(pins[i].port, pins[i].pin, EC_PUSH_PULL); // Set LED as Push-Pull
        GPIO_ospeed(pins[i].port, pins[i].pin, EC_MEDIUM);   // Set medium speed
        GPIO_write(pins[i].port, pins[i].pin, LOW);          // Set to LOW 
    }

} 

unsigned int binary[10] = {
	0b0000, // zero
  0b0001, // one
  0b0010, // two
	0b0011,	// three
	0b0100,	// four
	0b0101,	// five
	0b0110,	// six
	0b0111,	// seven
	0b1000,	// eight
  0b1001, // nine
};



void sevensegment_display(uint8_t num){

	GPIO pins[4] = {
		{GPIOA, 7},
		{GPIOB, 6},
		{GPIOC, 7},
		{GPIOA, 9},
   };

	// Send the binary code to the GPIO pins
	for (int i = 0; i < 4; i++) {
		 GPIO_write(pins[i].port, pins[i].pin, (binary[num] >> i) & 1); 
	}

}

