# Library Documentation

### Embedded Controller - STM32F411 Driver Library

Written by: EunChan Kim

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10/11

MCU: STM32F411RE, Nucleo-64



**Table of Contents**

- [ADC](#ADC)
  - [ADC_init()](#ADC_init())
  - [ADC_trigger()](#ADC_trigger())
  - [ADC_conversion()](#ADC_conversion())
  - [ADC_sequence()](#ADC_sequence())
  - [ADC_start()](#ADC_start())
  - [ADC_read()](#ADC_read())
  - [is_ADC_EOC()](#is_ADC_EOC())
  - [is_ADC_OVR()](#is_ADC_OVR())
  - [clear_ADC_OVR()](#clear_ADC_OVR())
  - [ADC_pinmap()](#ADC_pinmap())
- EXTI
  - [EXTI_init()](#EXTI_init())
  - [EXTI_enable()](#EXTI_enable())
  - [EXTI_disable()](#EXTI_disable())
  - [is_pending_EXTI()](#is_pending_EXTI())
  - [clear_pending_EXTI()](#clear_pending_EXTI())

- GPIO
  - [GPIO_init()](#GPIO_init())
  - [GPIO_mode()](#GPIO_mode())
  - [GPIO_ospeed()](#GPIO_ospeed())
  - [GPIO_otype()](#GPIO_otype())
  - [GPIO_pupd()](#GPIO_pupd())
  - [GPIO_read()](#GPIO_read())
  - [GPIO_write()](#GPIO_write())
  - [bit_toggling()](#bit_toggling())
  - [sevensegment_init()](#sevensegment_init())
  - [sevensegment_decoder()](#sevensegment_decoder())
  - [sevensegment_display_init()](#sevensegment_display_init())
  - [sevensegment_display()](#sevensegment_display())

- Input Capture
  - [ICAP_init()](#ICAP_init())
  - [ICAP_setup()](#ICAP_setup())
  - [ICAP_counter_us()](#ICAP_counter_us())
  - [is_CCIF()](#is_CCIF())
  - [clear_CCIF()](#clear_CCIF())
  - [ICAP_capture()](#ICAP_capture())
  - [ICAP_pinmap()](#ICAP_pinmap())

- Pin Names

  - [ecPinmap()](#ecPinmap())

- PWM

  - [PWM_init()](#PWM_init())
  - [PWM_period_ms()](#PWM_period_ms())
  - [PWM_period()](#PWM_period())
  - [PWM_period_us()](#PWM_period_us())
  - [PWM_pulsewidth()](#PWM_pulsewidth())
  - [PWM_pulsewidth_ms()](#PWM_pulsewidth_ms())
  - [PWM_pulsewidth_us()](#PWM_pulsewidth_us())
  - [PWM_duty()](#PWM_duty())
  - [PWM_pinmap()](#PWM_pinmap())

- RCC

  - [RCC_HSI_init()](#RCC_HSI_init())
  - [RCC_PLL_init()](#RCC_PLL_init())
  - [RCC_GPIOA_enable()](#RCC_GPIOA_enable())
  - [RCC_GPIOB_enable()](#RCC_GPIOB_enable())
  - [RCC_GPIOC_enable()](#RCC_GPIOC_enable())
  - [RCC_GPIOD_enable()](#RCC_GPIOD_enable())
  - [RCC_GPIOE_enable()](#RCC_GPIOE_enable())
  - [RCC_GPIOH_enable()](#RCC_GPIOH_enable())

- Stepper Motor

  - [Stepper_init()](#Stepper_init())
  - [Stepper_pinOut()](#Stepper_pinOut())
  - [Stepper_setSpeed()](#Stepper_setSpeed())
  - [Stepper_step()](#Stepper_step())
  - [Stepper_stop()](#Stepper_stop())

- SysTick

  - [SysTick_init()](#SysTick_init())
  - [SysTick_Handler()](#SysTick_Handler())
  - [SysTick_counter()](#SysTick_counter())
  - [delay_ms()](#delay_ms())
  - [SysTick_reset()](#SysTick_reset())
  - [SysTick_val()](#SysTick_val())
  - [SysTick_enable()](#SysTick_enable())
  - [SysTick_disable()](#SysTick_disable())

- Timer

  - [TIM_init()](#TIM_init())
  - [TIM_period_us()](#TIM_period_us())
  - [TIM_period_ms()](#TIM_period_ms())
  - [TIM_UI_init()](#TIM_UI_init())
  - [TIM_UI_enable()](#TIM_UI_enable())
  - [TIM_UI_disable()](#TIM_UI_disable())
  - [is_UIF()](#is_UIF())
  - [clear_UIF()](#clear_UIF())

- USART

  - [fputc()](#fputc)

  - [fgetc()](#fgetc)

  - [USART_write()](#USART_write)

  - [is_USART_RXNE()](#is_USART_RXNE)

  - [USART_read()](#USART_read)

  - [USART_setting()](#USART_setting)

  - [UART_baud()](#UART_baud)

  - [USART_delay()](#USART_delay)

  - [UART1_init()](#UART1_init)

  - [UART2_init()](#UART2_init)

    

### ADC

**Header File**

`#include "ecADC.h"`

```c
#ifndef __MY_ADC_H
#define __MY_ADC_H
#include "stm32f411xe.h"
#include "ecPinNames.h"


// ADC trigmode
#define SW 0
#define TRGO 1

// ADC contmode
#define CONT 0
#define SINGLE 1

// Edge Type
#define RISE 1
#define FALL 2
#define BOTH 3

#define _DEFAULT 0

/////////////////////////////////////////////////////
// ADC default setting
/////////////////////////////////////////////////////

// ADC init
// Default:  one-channel mode, continuous conversion
// Default: HW trigger - TIM3 counter, 1msec
void ADC_init(PinName_t pinName);
void JADC_init(PinName_t pinName);


// Multi-Channel Scan Sequence 
void ADC_sequence(PinName_t *seqCHn, int seqCHnums); 
void JADC_sequence(PinName_t *seqCHn, int seqCHnums); 

// ADC start
void ADC_start(void);

// flag for ADC interrupt
uint32_t is_ADC_EOC(void);
uint32_t is_ADC_OVR(void);
void clear_ADC_OVR(void);

// read ADC value
uint32_t ADC_read(void);


/////////////////////////////////////////////////////
// Advanced Setting
/////////////////////////////////////////////////////
// Conversion mode change: CONT, SINGLE / Operate both ADC,JADC
void ADC_conversion(int convMode); 					
void ADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);

// JADC setting
void JADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);

// Private Function
void ADC_pinmap(PinName_t pinName, uint32_t *chN);

#endif
```



#### ADC_init()

ADC initialization function for one channel

```c
void ADC_init(PinName_t pinName);
```

**Parameters**

- **pinName:** Input the pin name you want to use

**Example code**

```c
ADC_init(PB_1);
```



#### ADC_trigger()

This function is used to set the ADC trigger.

```c
void ADC_trigger(TIM_TypeDef* TIMx, int msec, int edge);
```

**Parameters**

- **TIMx:** Select the timer to be used
- **msec:** Set the trigger period (in milliseconds)
- **edge:** Select the trigger edge (RISE, FALL, BOTH)

**Example Code**

```c
ADC_trigger(TIM3, 100, RISE);
```



#### ADC_conversion()

This function sets the ADC conversion mode.

```c
void ADC_conversion(int convMode);
```

**Parameters**

- **convMode:** Select the conversion mode (CONT for continuous conversion, SINGLE for single conversion)

**Example Code**

```c
ADC_conversion(CONT);
```



#### ADC_sequence()

This function sets the ADC conversion sequence for multiple channels.



```c
void ADC_sequence(PinName_t *seqCHn, int seqCHnums);
```

**Parameters**

- **seqCHn:** An array of channel names for conversion sequence
- **seqCHnums:** The number of channels in the sequence

**Example Code**

```c
PinName_t sequence[] = {PA_0, PA_1, PA_4};
ADC_sequence(sequence, 3);
```



#### ADC_start()

This function starts the ADC conversion.

```c
void ADC_start(void);
```

**Example Code**

```c
ADC_start();
```



#### ADC_read()

This function reads the ADC value.

```c
uint32_t ADC_read(void);
```

**Returns**

- The ADC conversion value.

**Example Code**

```c
uint32_t value = ADC_read();
```



#### is_ADC_EOC()

This function checks if the ADC conversion has ended.

```c
uint32_t is_ADC_EOC(void);
```

**Returns**

- Returns 1 if the conversion has ended, 0 otherwise.

**Example Code**

```c
if(is_ADC_EOC()){
	// Conversion ended
}
```



#### is_ADC_OVR()

This function checks if the ADC has overflowed.

```c
uint32_t is_ADC_OVR(void);
```

**Returns**

- Returns 1 if the ADC has overflowed, 0 otherwise.

**Example Code**

```c
if(is_ADC_OVR()){
	// ADC overflowed
}
```



#### clear_ADC_OVR()

This function clears the ADC overflow flag.

```c
void clear_ADC_OVR(void);
```

**Example Code**

```c
clear_ADC_OVR();
```



#### ADC_pinmap()

This function maps the GPIO pin to the ADC channel.

```c
void ADC_pinmap(PinName_t pinName, uint32_t *chN);
```

**Parameters**

- **pinName:** The name of the pin to map
- **chN:** The pointer to store the channel number

**Example Code**

```c
uint32_t channel;
ADC_pinmap(PB_1, channel);
```





### EXTI

**Header File**

`#include "ecEXTI.h"`

```c
#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#include "stm32f411xe.h"

#define FALL 0
#define RISE 1
#define BOTH 2

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void EXTI_init(GPIO_TypeDef *Port, int pin, int trig, int priority);
void EXTI_enable(uint32_t pin);
void EXTI_disable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */
	 
#endif
```



#### EXTI_init()

This function initializes the external interrupts (EXTI) on a specific GPIO pin.

```c
void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **Pin:** Specifies the GPIO pin
- **trig_type:** Specifies the trigger type (FALL, RISE, BOTH)
- **priority:** Specifies the priority for the interrupt

**Example Code**

```c
EXTI_init(GPIOA, 1, RISE, 2);
```



#### EXTI_enable()

This function enables the external interrupt on a specific GPIO pin.

```c
void EXTI_enable(uint32_t pin);
```

**Parameters**

- **pin:** Specifies the GPIO pin

**Example Code**

```c
EXTI_enable(1);
```



#### EXTI_disable()

This function disables the external interrupt on a specific GPIO pin.

```c
void EXTI_disable(uint32_t pin);
```

**Parameters**

- **pin:** Specifies the GPIO pin

**Example Code**

```c
EXTI_disable(1);
```



#### is_pending_EXTI()

This function checks if there is a pending interrupt on a specific GPIO pin.

```c
uint32_t is_pending_EXTI(uint32_t pin);
```

**Parameters**

- **pin:** Specifies the GPIO pin

**Returns**

- Returns 1 if there is a pending interrupt, 0 otherwise.

**Example Code**

```c
if(is_pending_EXTI(1)){
	// Pending interrupt
}
```



#### clear_pending_EXTI()

This function clears the pending interrupt on a specific GPIO pin.

```c
void clear_pending_EXTI(uint32_t pin);
```

**Parameters**

- **pin:** Specifies the GPIO pin

**Example Code**

```c
clear_pending_EXTI(1);
```



### GPIO

**Header File**

`#include "ecGPIO.h"`

```c
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
```



#### GPIO_init()

This function initializes a GPIO pin.

```c
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin
- **mode:** Specifies the GPIO mode (Input, Output, Alternate Function, Analog)

**Example Code**

```c
GPIO_init(GPIOA, 5, OUTPUT);
```



#### GPIO_mode()

This function sets the mode of a GPIO pin.

```c
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin
- **mode:** Specifies the GPIO mode (Input, Output, Alternate Function, Analog)

**Example Code**

```c
GPIO_mode(GPIOA, 5, OUTPUT);
```



#### GPIO_ospeed()

This function sets the speed of a GPIO pin.

```c
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin
- **speed:** Specifies the GPIO speed (Low, Medium, Fast, High)

**Example Code**

```c
GPIO_ospeed(GPIOA, 5, MEDIUM);
```



#### GPIO_otype()

This function sets the output type of a GPIO pin.

```c
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin
- **type:** Specifies the GPIO output type (Push-Pull, Open-Drain)

**Example Code**

```c
GPIO_otype(GPIOA, 5, PUSH_PULL);
```



#### GPIO_pupd()

This function sets the pull-up/pull-down configuration of a GPIO pin.

```c
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin
- **pupd:** Specifies the GPIO pull-up/pull-down configuration (No pull-up/pull-down, Pull-up, Pull-down)

**Example Code**

```c
GPIO_pupd(GPIOA, 5, PULL_UP);
```



#### GPIO_read()

This function reads the value of a GPIO pin.

```c
int GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin

**Returns**

- The value of the GPIO pin.

**Example Code**

```c
int value = GPIO_read(GPIOA, 5);
```



#### GPIO_write()

This function writes a value to a GPIO pin.

```c
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin
- **Output:** Specifies the value to write

**Example Code**

```c
GPIO_write(GPIOA, 5, HIGH);
```



#### bit_toggling()

This function toggles the state of a GPIO pin.

```c
void bit_toggling(GPIO_TypeDef *Port, int pin);
```

**Parameters**

- **Port:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin

**Example Code**

```c
bit_toggling(GPIOA, 5);
```



#### sevensegment_init()

This function initializes a 7-segment display.

```c
void sevensegment_init(void);
```



#### sevensegment_decoder()

This function decodes a number for a 7-segment display.

```c
void sevensegment_decoder(uint8_t  num);
```

**Parameters**

- **num:** Specifies the number to decode



#### sevensegment_display_init()

This function initializes a 7-segment display for display a number.

```c
void sevensegment_display_init(void);
```



#### sevensegment_display()

This function displays a number on a 7-segment display.

```c
void sevensegment_display(uint8_t num);
```

**Parameters**

- **num:** Specifies the number to display

**Example Code**

```c
sevensegment_display(3);
```



### Input Capture

**Header File**

`#include "ecICAP.h"`

```c
#ifndef __EC_ICAP_H
#define __EC_ICAP_H
#include "ecPinNames.h"
#include "stm32f411xe.h"
#include "ecTIM.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* Input Capture*/

// ICn selection according to CHn
#define FIRST 1
#define SECOND 2

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

// Input Capture Number
#define IC_1    1
#define IC_2    2
#define IC_3    3
#define IC_4    4

//Input Capture

void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);

void ICAP_init(PinName_t pinName);
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
void ICAP_counter_us(PinName_t pinName, int usec);
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);  // CCnum= 1~4
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t CCnum);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



#### ICAP_init()

This function initializes the Timer Input Capture.

```c
void ICAP_init(PinName_t pinName);
```

**Parameters**

- **pinName:** Specifies the GPIO pin

**Example Code**

```c
ICAP_init(PA0);
```



#### ICAP_setup()

This function configures the selection of TIx-ICy and Edge Type.

```c
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
```

**Parameters**

- **pinName:** Specifies the GPIO pin
- **ICn:** Specifies the Input Capture channel number
- **edge_type:** Specifies the Activation Edge direction (Rising, Falling, or Both)

**Example Code**

```c
ICAP_setup(PA0, 1, IC_RISE);
```



#### ICAP_counter_us()

This function sets the time span for one counter step.

```c
void ICAP_counter_us(PinName_t pinName, int usec);
```

**Parameters**

- **pinName:** Specifies the GPIO pin
- **usec:** Specifies the time in microseconds

**Example Code**

```c
ICAP_counter_us(PA0, 1);
```



#### is_CCIF()

This function checks if the Capture/Compare Interrupt Flag (CCIF) for a specific channel is set.

```c
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters**

- **TIMx:** Specifies the Timer
- **ccNum:** Specifies the channel number

**Example Code**

```c
if (is_CCIF(TIM2, 1))
{
    // Do something
}
```



#### clear_CCIF()

This function clears the Capture/Compare Interrupt Flag (CCIF) for a specific channel.

```c
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters**

- **TIMx:** Specifies the Timer
- **ccNum:** Specifies the channel number

**Example Code**

```c
clear_CCIF(TIM2, 1);
```



#### ICAP_capture()

This function captures the value of the Capture/Compare Register (CCR) for a specific Input Capture channel.

```c
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);
```

**Parameters**

- **TIMx:** Specifies the Timer
- **ICn:** Specifies the Input Capture channel number

**Example Code**

```c
uint32_t capture_Value = ICAP_capture(TIM2, 1);
```



#### ICAP_pinmap()

This function maps a GPIO pin to a Timer and a channel. This function should not be modified.

```c
void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameters**

- **pinName:** Specifies the GPIO pin
- **TIMx:** Specifies the Timer
- **chN:** Specifies the channel number

**Example Code**

```c
TIM_TypeDef *TIMx;
int chN;
ICAP_pinmap(PA0, &TIMx, &chN);
```



### PinNames

**Header File**

`#include "ecPinNames.h"`

```c
#ifndef EC_PINNAMES_H
#define EC_PINNAMES_H

#include "stm32f411xe.h"

#ifdef __cplusplus
extern "C" {
#endif


// Bitwise Macro Definition
#define BIT_SET(REG, BIT)      	((REG) |= 1<< (BIT))
#define BIT_CLEAR(REG, BIT)     ((REG) &= ~1<<(BIT))
#define BIT_READ(REG, BIT)      ((REG)>>BIT & (1))
#define BITS_SET(REG, BIT,NUM)     ((REG) |= NUM<< (BIT))
#define BITS_CLEAR(REG, BIT,NUM)   ((REG) &= ~(NUM<< (BIT))
//#define BITS_CLEAR(REG, BIT,NUM)   ((REG) &= ~((0x1<< NUM)-1)<<(BIT))


// Pinname Config
typedef enum {
    PortA = 0,
    PortB = 1,
    PortC = 2,
    PortD = 3,
    PortE = 4,
    PortF = 5,
    PortG = 6,
    PortH = 7,
    PortI = 8,
    PortJ = 9,
    PortK = 10
} PortName_t;



typedef enum {
    PA_0  = 0x00,
    PA_1  = 0x01,    
    PA_2  = 0x02,
    PA_3  = 0x03,
    PA_4  = 0x04,    
    PA_5  = 0x05,
    PA_6  = 0x06,
    PA_7  = 0x07,
    PA_8  = 0x08,
    PA_9  = 0x09,
    PA_10 = 0x0A,
    PA_11 = 0x0B,
    PA_12 = 0x0C,
    PA_13 = 0x0D,
    PA_14 = 0x0E,
    PA_15 = 0x0F,
    
    PB_0  = 0x10,
    PB_1  = 0x11,
    PB_2  = 0x12,
    PB_3  = 0x13,
    PB_4  = 0x14,
    PB_5  = 0x15,
    PB_6  = 0x16,
    PB_7  = 0x17,
    PB_8  = 0x18,
    PB_9  = 0x19,
    PB_10 = 0x1A,
    PB_12 = 0x1C,
    PB_13 = 0x1D,
    PB_14 = 0x1E,
    PB_15 = 0x1F,

    PC_0  = 0x20,
    PC_1  = 0x21,
    PC_2  = 0x22,
    PC_3  = 0x23,
    PC_4  = 0x24,
    PC_5  = 0x25,
    PC_6  = 0x26,
    PC_7  = 0x27,
    PC_8  = 0x28,
    PC_9  = 0x29,
    PC_10 = 0x2A,
    PC_11 = 0x2B,
    PC_12 = 0x2C,
    PC_13 = 0x2D,
    PC_14 = 0x2E,
    PC_15 = 0x2F,

    PD_2  = 0x32,

    PH_0  = 0x70,
    PH_1  = 0x71,


    // Arduino connector namings
    A0          = PA_0,
    A1          = PA_1,
    A2          = PA_4,
    A3          = PB_0,
    A4          = PC_1,
    A5          = PC_0,
    D0          = PA_3,
    D1          = PA_2,
    D2          = PA_10,
    D3          = PB_3,
    D4          = PB_5,
    D5          = PB_4,
    D6          = PB_10,
    D7          = PA_8,
    D8          = PA_9,
    D9          = PC_7,
    D10         = PB_6,
    D11         = PA_7,
    D12         = PA_6,
    D13         = PA_5,
    D14         = PB_9,
    D15         = PB_8,


    // Not connected
    NC = (int)0xFFFFFFFF
} PinName_t;


void ecPinmap(PinName_t pinName, GPIO_TypeDef **GPIOx, unsigned int *pin);




#ifdef __cplusplus
}
#endif

#endif

```



#### ecPinmap()

This function maps a given pin name to a GPIO port and pin number.

```c
void ecPinmap(PinName_t pinName, GPIO_TypeDef **GPIOx, unsigned int *pin);
```

**Parameters**

- **pinName:** Specifies the pin name
- **GPIOx:** Specifies the GPIO port
- **pin:** Specifies the GPIO pin number

**Example Code**

```c
GPIO_TypeDef *GPIOx;
unsigned int pin;
ecPinmap(PA0, &GPIOx, &pin);
```





### PWM

**Header File**

`#include "ecPWM.h"`

```c
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecTIM.h"  			

#include "ecPinNames.h"


#ifndef __EC_PWM_H
#define __EC_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* PWM Configuration using PinName_t Structure */

/* PWM initialization */
// Default: 84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period
void PWM_init(PinName_t pinName);
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);


/* PWM PERIOD SETUP */
// allowable range for msec:  1~2,000
void PWM_period(PinName_t pinName,  uint32_t msec);	
void PWM_period_ms(PinName_t pinName,  uint32_t msec);	// same as PWM_period()
// allowable range for usec:  1~1,000
void PWM_period_us(PinName_t pinName, uint32_t usec);


/* DUTY RATIO SETUP */
// High Pulse width in msec
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);  // same as void PWM_pulsewidth

void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
// Duty ratio 0~1.0
void PWM_duty(PinName_t pinName, float duty);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



#### PWM_init()

This function initializes the PWM with default settings (84MHz PLL, 1MHz CK_CNT, 50% duty ratio, 1msec period).

```c
void PWM_init(PinName_t pinName);
```

**Parameters**

- **pinName:** Specifies the pin name

**Example Code**

```c
PWM_init(PA0);
```



#### PWM_period_ms()

This function sets the counter period in milliseconds.

```c
void PWM_period_ms(PinName_t pinName, uint32_t msec);
```

**Parameters**

- **pinName:** Specifies the pin name
- **msec:** Specifies the counter period in milliseconds

**Example Code**

```c
PWM_period_ms(PA0, 1000);
```



#### PWM_period()

This function sets the counter period in milliseconds.

```c
void PWM_period(PinName_t pinName, uint32_t msec);
```

**Parameters**

- **pinName:** Specifies the pin name
- **msec:** Specifies the counter period in milliseconds

**Example Code**

```c
PWM_period(PA0, 1000);
```



#### PWM_period_us()

This function sets the counter period in microseconds.

```c
void PWM_period_us(PinName_t pinName, uint32_t usec);
```

**Parameters**

- **pinName:** Specifies the pin name
- **usec:** Specifies the counter period in microseconds

**Example Code**

```c
PWM_period_us(PA0, 1000);
```



#### PWM_pulsewidth()

This function sets the high pulse width in milliseconds.

```c
void PWM_pulsewidth(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameters**

- **pinName:** Specifies the pin name
- **pulse_width_ms:** Specifies the high pulse width in milliseconds

**Example Code**

```c
PWM_pulsewidth(PA0, 500);
```



#### PWM_pulsewidth_ms()

This function sets the high pulse width in milliseconds.

```c
void PWM_pulsewidth_ms(PinName_t pinName, uint32_t pulse_width_ms);
```

**Parameters**

- **pinName:** Specifies the pin name
- **pulse_width_ms:** Specifies the high pulse width in milliseconds

**Example Code**

```c
PWM_pulsewidth_ms(PA0, 500);
```



#### PWM_pulsewidth_us()

This function sets the high pulse width in microseconds.

```c
void PWM_pulsewidth_us(PinName_t pinName, uint32_t pulse_width_us);
```

**Parameters**

- **pinName:** Specifies the pin name
- **pulse_width_us:** Specifies the high pulse width in microseconds

**Example Code**

```c
PWM_pulsewidth_us(PA0, 500);
```



#### PWM_duty()

This function sets the duty cycle from 0 to 1.

```c
void PWM_duty(PinName_t pinName, float duty);
```

**Parameters**

- **pinName:** Specifies the pin name
- **duty:** Specifies the duty cycle from 0 to 1

**Example Code**

```c
PWM_duty(PA0, 0.5);
```



#### PWM_pinmap()

This function maps a given pin name to a timer and channel number.

```c
void PWM_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
```

**Parameters**

- **pinName:** Specifies the pin name
- **TIMx:** Specifies the timer
- **chN:** Specifies the channel number

**Example Code**

```c
TIM_TypeDef *TIMx;
int chN;
PWM_pinmap(PA0, &TIMx, &chN);
```



### RCC

**Header File**

`#include "ecRCC.h"`

```c
#ifndef __EC_RCC_H
#define __EC_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//#include "stm32f411xe.h"

void RCC_HSI_init(void);
void RCC_PLL_init(void);
void RCC_GPIOA_enable(void);
void RCC_GPIOB_enable(void);
void RCC_GPIOC_enable(void);
void RCC_GPIOD_enable(void);
void RCC_GPIOE_enable(void);
void RCC_GPIOH_enable(void);
// void RCC_GPIO_enable(GPIO_TypeDef * GPIOx);

extern int EC_SYSCL;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



#### RCC_HSI_init()

This function initializes the High Speed Internal Clock (HSI = 16 MHz).

```c
void RCC_HSI_init();
```

**Example Code**

```c
RCC_HSI_init();
```



#### RCC_PLL_init()

This function initializes the Phase Locked Loop (PLL) to 84 MHz.

```c
void RCC_PLL_init();
```

**Example Code**

```c
RCC_PLL_init();
```



#### RCC_GPIOA_enable()

This function enables the clock for the GPIOA peripheral.

```c
void RCC_GPIOA_enable();
```

**Example Code**

```c
RCC_GPIOA_enable();
```



#### RCC_GPIOB_enable()

This function enables the clock for the GPIOB peripheral.

```c
void RCC_GPIOB_enable();
```

**Example Code**

```c
RCC_GPIOB_enable();
```



#### RCC_GPIOC_enable()

This function enables the clock for the GPIOC peripheral.

```c
void RCC_GPIOC_enable();
```

**Example Code**

```c
RCC_GPIOC_enable();
```



#### RCC_GPIOD_enable()

This function enables the clock for the GPIOD peripheral.

```c
void RCC_GPIOD_enable();
```

**Example Code**

```c
RCC_GPIOD_enable();
```



#### RCC_GPIOE_enable()

This function enables the clock for the GPIOE peripheral.

```c
void RCC_GPIOE_enable();
```

**Example Code**

```c
RCC_GPIOE_enable();
```



#### RCC_GPIOH_enable()

This function enables the clock for the GPIOH peripheral.

```c
void RCC_GPIOH_enable();
```

**Example Code**

```c
RCC_GPIOH_enable();
```



### Stepper Motor

**Header File**

`#include "ecStepper.h"`

```c
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
	GPIO_TypeDef *port1;
	int pin1;
	GPIO_TypeDef *port2;
	int pin2;
	GPIO_TypeDef *port3;
	int pin3;
	GPIO_TypeDef *port4;
	int pin4;
	uint32_t _step_num;
} Stepper_t;

	 
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_setSpeed(long whatSpeed);
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode); 
void Stepper_stop(void);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



#### Stepper_init()

This function initializes the Stepper motor with the GPIO pins provided.

```c
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Example Code**

```c
Stepper_init(GPIOA, 1, GPIOA, 2, GPIOA, 3, GPIOA, 4);
```



#### Stepper_pinOut()

This function sets the output for the GPIO pins according to the step state and mode (FULL/HALF).

```c
void Stepper_pinOut (uint32_t state, uint32_t mode);
```

**Example Code**

```c
Stepper_pinOut(1, FULL);
Stepper_pinOut(2, HALF);
```



#### Stepper_setSpeed()

This function sets the speed of the stepper motor in rpm (revolutions per minute).

```c
void Stepper_setSpeed (long whatSpeed);
```

**Example Code**

```c
Stepper_setSpeed(1000); // Set speed to 1000rpm
```



#### Stepper_step()

This function moves the stepper motor a certain number of steps in the given direction (clockwise/counter-clockwise) and mode (FULL/HALF).

```c
void Stepper_step(uint32_t steps, uint32_t direction, uint32_t mode);
```

**Example Code**

```c
Stepper_step(200, 1, FULL); // Move 200 steps in clockwise direction in FULL mode
Stepper_step(200, 0, HALF); // Move 200 steps in counter-clockwise direction in HALF mode
```



#### Stepper_stop()

This function stops the stepper motor.

```c
void Stepper_stop (void);
```

**Example Code**

```c
Stepper_stop(); // Stop the stepper motor
```



### SysTick

**Header File**

`#include "ecSysTick.h"`

```c
#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

extern volatile uint32_t msTicks;
void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```



#### SysTick_init()

This function initializes the system tick timer (SysTick).

```c
void SysTick_init(void);
```

**Example Code**

```c
SysTick_init();
```



#### SysTick_Handler()

This function handles the SysTick interrupt.

```c
void SysTick_Handler(void);
```

**Example Code**

```c
SysTick_Handler();
```



#### SysTick_counter()

This function increments the msTicks counter.

```c
void SysTick_counter();
```

**Example Code**

```c
SysTick_counter();
```



#### delay_ms()

This function delays the execution for a specified number of milliseconds.

```c
void delay_ms(uint32_t mesc);
```

**Example Code**

```c
delay_ms(1000); // Delay for 1000ms
```



#### SysTick_reset()

This function resets the SysTick current value register.

```c
void SysTick_reset(void);
```

**Example Code**

```c
SysTick_reset();
```



#### SysTick_val()

This function returns the current value of the SysTick timer.

```c
uint32_t SysTick_val(void);
```

**Example Code**

```c
uint32_t currentTick = SysTick_val();
```



#### SysTick_enable()

This function enables the SysTick timer.

```c
void SysTick_enable(void);
```

**Example Code**

```c
SysTick_enable();
```



#### SysTick_disable()

This function disables the SysTick timer.

```c
void SysTick_disable(void);
```

**Example Code**

```c
SysTick_disable();
```



### Timer

**Header File**

`#include "ecTIM.h"`

```c
#ifndef __EC_TIM_H 
#define __EC_TIM_H
#include "stm32f411xe.h"
#include "ecPinNames.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Input Capture*/
// ICn selection according to CHn
#define FIRST 1
#define SECOND 2

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

// IC Number
#define IC_1 1
#define IC_2 2
#define IC_3 3
#define IC_4 4

/* Timer Configuration */
void TIM_init(TIM_TypeDef *TIMx, uint32_t msec);  
void TIM_period(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);  
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);

void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec); 
void TIM_UI_enable(TIM_TypeDef* TIMx);
void TIM_UI_disable(TIM_TypeDef* TIMx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);

void ICAP_pinmap(PinName_t pinName, TIM_TypeDef **TIMx, int *chN);
void ICAP_init(PinName_t pinName);
void ICAP_setup(PinName_t pinName, int ICn, int edge_type);
void ICAP_counter_us(PinName_t pinName, int usec);
uint32_t ICAP_capture(TIM_TypeDef* TIMx, uint32_t ICn);
uint32_t ICAP_read(TIM_TypeDef *TIMx);
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```



#### TIM_init()

This function initializes a timer with a specified period in milliseconds.

```c
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** Specifies the timer.
- **msec:** Specifies the period in milliseconds.

**Example Code**

```c
TIM_init(TIM2, 1000); // Initialize TIM2 with a period of 1000ms.
```



#### TIM_period_us()

This function sets the period of a timer in microseconds.

```c
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec);
```

**Parameters**

- **TIMx:** Specifies the timer.
- **usec:** Specifies the period in microseconds.

**Example Code**

```c
TIM_period_us(TIM2, 1000); // Set the period of TIM2 to 1000us.
```



#### TIM_period_ms()

This function sets the period of a timer in milliseconds.

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** Specifies the timer.
- **msec:** Specifies the period in milliseconds.

**Example Code**

```c
TIM_period_ms(TIM2, 1000); // Set the period of TIM2 to 1000ms.
```



#### TIM_UI_init()

This function initializes a timer with a specified period in milliseconds and enables the Update Interrupt.

```c
void TIM_UI_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

- **TIMx:** Specifies the timer.
- **msec:** Specifies the period in milliseconds.

**Example Code**

```c
TIM_UI_init(TIM2, 1000); // Initialize TIM2 with a period of 1000ms and enable the Update Interrupt.
```



#### TIM_UI_enable()

This function enables the Update Interrupt for a timer.

```c
void TIM_UI_enable(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx:** Specifies the timer.

**Example Code**

```c
TIM_UI_enable(TIM2); // Enable the Update Interrupt for TIM2.
```



#### TIM_UI_disable()

This function disables the Update Interrupt for a timer.

```c
void TIM_UI_disable(TIM_TypeDef* TIMx);
```

**Parameters**

- **TIMx:** Specifies the timer.

**Example Code**

```c
TIM_UI_disable(TIM2); // Disable the Update Interrupt for TIM2.
```



#### is_UIF()

This function checks if the Update Interrupt Flag for a timer is set.

```c
uint32_t is_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

- **TIMx:** Specifies the timer.

**Example Code**

```c
if (is_UIF(TIM2)) {
  // Do something if the Update Interrupt Flag for TIM2 is set.
}
```



#### clear_UIF()

This function clears the Update Interrupt Flag for a timer.

```c
void clear_UIF(TIM_TypeDef *TIMx);
```

**Parameters**

- **TIMx:** Specifies the timer.

**Example Code**

```c
clear_UIF(TIM2); // Clear the Update Interrupt Flag for TIM2.
```





### USART

**Header File**

`#include "ecUART.h"`

```c
#ifndef __EC_USART_H
#define __EC_USART_H

#include <stdio.h>
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"

#define POL 0
#define INT 1

// You can modify this
#define BAUD_9600	9600
#define BAUD_19200	19200
#define BAUD_38400  	38400
#define BAUD_57600	57600
#define BAUD_115200 	115200
#define BAUD_921600 	921600


// ********************** USART 2 (USB) ***************************
// PA_2 = USART2_TX
// PA_3 = USART2_RX
// Alternate function(AF7), High Speed, Push pull, Pull up
// APB1
// **********************************************************

// ********************** USART 1 ***************************
// PA_9 = USART1_TX (default)	// PB_6  (option)
// PA_10 = USART1_RX (default)	// PB_3 (option)
// APB2
// **********************************************************

// ********************** USART 6 ***************************
// PA_11 = USART6_TX (default)	// PC_6  (option)
// PA_12 = USART6_RX (default)	// PC_7 (option)
// APB2
// **********************************************************

// Configuration UART 1, 2 using default pins 
void UART1_init(void);
void UART2_init(void);	
void UART1_baud(uint32_t baud);
void UART2_baud(uint32_t baud);

// USART write & read
void USART1_write(uint8_t* buffer, uint32_t nBytes);
void USART2_write(uint8_t* buffer, uint32_t nBytes);
uint8_t USART1_read(void);										
uint8_t USART2_read(void);	

// RX Inturrupt Flag USART1,2
uint32_t is_USART1_RXNE(void);
uint32_t is_USART2_RXNE(void);

// private functions
void USART_write(USART_TypeDef* USARTx, uint8_t* buffer, uint32_t nBytes);
void USART_init(USART_TypeDef* USARTx, uint32_t baud);  		
void UART_baud(USART_TypeDef* USARTx, uint32_t baud);											
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
uint8_t USART_read(USART_TypeDef * USARTx);										
void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud); 
void USART_delay(uint32_t us);  

#endif

```



#### fputc()

This function redirects printf() to USART2.

```c
int fputc(int ch, FILE *f);
```

**Parameters**

- **ch:** Specifies the character to be written.
- **f:** Specifies the output stream.

**Example Code**

```c
printf("Hello, World!"); // This will be printed via USART2.
```



#### fgetc()

This function redirects getchar()/scanf() to USART2.

```c
int fgetc(FILE *f);
```

**Parameters**

- **f:** Specifies the input stream.

**Example Code**

```c
char c = getchar(); // This will read a character via USART2.
```



#### USART_write()

This function writes a number of bytes to USART.

```c
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
```

**Parameters**

- **USARTx:** Specifies the USART.
- **buffer:** Specifies the buffer to be written.
- **nBytes:** Specifies the number of bytes to be written.

**Example Code**

```c
uint8_t buffer[] = "Hello, World!";
USART_write(USART2, buffer, sizeof(buffer)); // This will write the buffer to USART2.
```



#### is_USART_RXNE()

This function checks if the RX not empty flag for a USART is set.

```c
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
```

**Parameters**

- **USARTx:** Specifies the USART.

**Example Code**

```c
if (is_USART_RXNE(USART2)) {
  // Do something if the RX not empty flag for USART2 is set.
}
```



#### USART_read()

This function reads a byte from USART.

```c
uint8_t USART_read(USART_TypeDef * USARTx);
```

**Parameters**

- **USARTx:** Specifies the USART.

**Example Code**

```c
uint8_t rxByte = USART_read(USART2); // This will read a byte from USART2.
```



#### USART_setting()

This function sets up a USART with a specified baud rate.

```c
void USART_setting(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, uint32_t baud);
```

**Parameters**

- **USARTx:** Specifies the USART.
- **GPIO_TX:** Specifies the GPIO for TX.
- **pinTX:** Specifies the pin for TX.
- **GPIO_RX:** Specifies the GPIO for RX.
- **pinRX:** Specifies the pin for RX.
- **baud:** Specifies the baud rate.

**Example Code**

```c
USART_setting(USART2, GPIOA, 2, GPIOA, 3, 9600); // This will set up USART2 with a baud rate of 9600.
```



#### UART_baud()

This function sets the baud rate for a USART.

```c
void UART_baud(USART_TypeDef* USARTx, uint32_t baud);
```

**Parameters**

- **USARTx:** Specifies the USART.
- **baud:** Specifies the baud rate.

**Example Code**

```c
UART_baud(USART2, 9600); // This will set the baud rate of USART2 to 9600.
```



#### USART_delay()

This function creates a delay in microseconds.

```c
void USART_delay(uint32_t us);
```

**Parameters**

- **us:** Specifies the delay in microseconds.

**Example Code**

```c
USART_delay(1000); // This will create a delay of 1000us.
```



#### UART1_init(), UART2_init()

These functions initialize UART1 and UART2 respectively.

```c
void UART1_init(void);
void UART2_init(void);
```

**Example Code**

```c
UART1_init(); // This will initialize UART1.
UART2_init(); // This will initialize UART2.
```



#### UART1_baud(), UART2_baud()

These functions set the baud rate for UART1 and UART2 respectively.

```c
void UART1_baud(uint32_t baud);
void UART2_baud(uint32_t baud);
```

**Parameters**

- **baud:** Specifies the baud rate.

**Example Code**

```c
UART1_baud(9600); // This will set the baud rate of UART1 to 9600.
UART2_baud(9600); // This will set the baud rate of UART2 to 9600.
```



#### USART1_write(), USART2_write()

These functions write a number of bytes to USART1 and USART2 respectively.

```c
void USART1_write(uint8_t* buffer, uint32_t nBytes);
void USART2_write(uint8_t* buffer, uint32_t nBytes);
```

**Parameters**

- **buffer:** Specifies the buffer to be written.
- **nBytes:** Specifies the number of bytes to be written.

**Example Code**

```c
uint8_t buffer[] = "Hello, World!";
USART1_write(buffer, sizeof(buffer)); // This will write the buffer to USART1.
USART2_write(buffer, sizeof(buffer)); // This will write the buffer to USART2.
```



#### USART1_read(), USART2_read()

These functions read a byte from USART1 and USART2 respectively.

```c
uint8_t USART1_read(void);
uint8_t USART2_read(void);
```

**Example Code**

```c
uint8_t rxByte1 = USART1_read(); // This will read a byte from USART1.
uint8_t rxByte2 = USART2_read(); // This will read a byte from USART2.
```



#### is_USART1_RXNE(), is_USART2_RXNE()

These functions check if the RX not empty flag for USART1 and USART2 respectively is set.

```c
uint32_t is_USART1_RXNE(void);
uint32_t is_USART2_RXNE(void);
```

**Example Code**

```c
if (is_USART1_RXNE()) {
  // Do something if the RX not empty flag for USART1 is set.
}
if (is_USART2_RXNE()) {
  // Do something if the RX not empty flag for USART2 is set.
}
```

