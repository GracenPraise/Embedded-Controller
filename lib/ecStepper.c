#include "stm32f4xx.h"
#include "ecStepper.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
	{0b1100,{S1,S3}}, 
 	{0b0110,{S2,S0}},
 	{0b0011,{S3,S1}},
 	{0b1001,{S0,S2}}
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  	uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 	{0b1000,{S1,S7}},
	{0b1100,{S2,S0}},
	{0b0100,{S3,S1}},
	{0b0110,{S4,S2}},
	{0b0010,{S5,S3}},
	{0b0011,{S6,S4}},
	{0b0001,{S7,S5}},
	{0b1001,{S0,S6}}

};



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
	//  GPIO Digital Out Initiation
	myStepper.port1 = port1;
	myStepper.pin1  = pin1;
	// Repeat for port2,pin3,pin4 
	myStepper.port2 = port2;
	myStepper.pin2  = pin2;
	myStepper.port3 = port3;
	myStepper.pin3  = pin4;
	myStepper.port4 = port4;
	myStepper.pin4  = pin4;
	
	GPIO_init(port1, pin1, OUTPUT);
	GPIO_init(port2, pin2, OUTPUT);
	GPIO_init(port3, pin3, OUTPUT);
	GPIO_init(port4, pin4, OUTPUT);
	
	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	
	// Pin1 ~ Port4
	GPIO_pupd(port1, pin1, EC_NONE);
	GPIO_otype(port1, pin1,EC_PUSH_PULL);
	GPIO_ospeed(port1, pin1, EC_FAST);
	
	GPIO_pupd(port2, pin2, EC_NONE);
	GPIO_otype(port2, pin2,EC_PUSH_PULL);
	GPIO_ospeed(port2, pin2, EC_FAST);
	
	GPIO_pupd(port3, pin3, EC_NONE);
	GPIO_otype(port3, pin3,EC_PUSH_PULL);
	GPIO_ospeed(port3, pin3, EC_FAST);
	
	GPIO_pupd(port4, pin4, EC_NONE);
	GPIO_otype(port4, pin4,EC_PUSH_PULL);
	GPIO_ospeed(port4, pin4, EC_FAST);
}


void Stepper_pinOut (uint32_t state, uint32_t mode){	
   	if (mode == FULL){         // FULL mode
			GPIO_write(myStepper.port1, myStepper.pin1, FSM_full[state].out >> 3 & 1);
			GPIO_write(myStepper.port2, myStepper.pin2, FSM_full[state].out >> 2 & 1);
			GPIO_write(myStepper.port3, myStepper.pin3, FSM_full[state].out >> 1 & 1);
			GPIO_write(myStepper.port4, myStepper.pin4, FSM_full[state].out >> 0 & 1);

	}	 
 	else if (mode == HALF){    // HALF mode

			GPIO_write(myStepper.port1, myStepper.pin1, FSM_half[state].out >> 3 & 1);
			GPIO_write(myStepper.port2, myStepper.pin2, FSM_half[state].out >> 2 & 1);
			GPIO_write(myStepper.port3, myStepper.pin3, FSM_half[state].out >> 1 & 1);
			GPIO_write(myStepper.port4, myStepper.pin4, FSM_half[state].out >> 0 & 1);
	}
}


void Stepper_setSpeed (long whatSpeed){      // rpm [rev/min]
		step_delay = 60000 / (step_per_rev*whatSpeed); //YOUR CODE   // Convert rpm to  [msec] delay 
}


void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode){
    static uint32_t state = 0;
    static uint32_t last_step_time = 0;
    uint32_t current_time = SysTick_val(); 

    if ((current_time - last_step_time) >= step_delay){
        last_step_time = current_time; 
        myStepper._step_num = steps;

        if(myStepper._step_num > 0){ // run for step size
            if (mode == FULL) 		 												
                state = FSM_full[state].next[direction]; // state = next state
            else if (mode == HALF) 
                state = FSM_half[state].next[direction]; // state = next state		

            Stepper_pinOut(state, mode);
						delay_ms(step_delay);
            myStepper._step_num--; 
        }
    }
}



void Stepper_stop (void){ 
    	myStepper._step_num = 0;    
	// All pins(A,AN,B,BN) set as DigitalOut '0'
			GPIO_write(myStepper.port1, myStepper.pin1, myStepper._step_num);
			GPIO_write(myStepper.port2, myStepper.pin2, myStepper._step_num);
			GPIO_write(myStepper.port3, myStepper.pin3, myStepper._step_num);
			GPIO_write(myStepper.port4, myStepper.pin4, myStepper._step_num);
}

