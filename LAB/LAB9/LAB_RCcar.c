/**
******************************************************************************
* @author  EunChan Kim & Heejun Lee
* @Mod       2023-11-24 by YKKIM     
* @brief   Embedded Controller:  LAB - Line Tracing RC Car
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"
#include "ecPinNames.h"
#include "ecPWM.h"
#include "math.h"
#include "stdio.h"

static volatile uint8_t BT_Data = 0;

static volatile float i=0;
static volatile float j=0;
static volatile int dir=0;
static volatile int speed=0;
static volatile int angle=0;
static volatile char mode;
static volatile char direction;
static volatile float iduty=0;
static volatile float jduty=0;
static volatile int state=0;
char s1[30];

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

// Pin Definitions
#define TRIG PA_6
#define ECHO PB_6

//IR parameter//
uint32_t value1, value2;
int flag = 0;
PinName_t seqCHn[2] = { PB_0, PB_1 };
int IR_diff = 0;
int Stop_flag = 0;
int cnt = 0;
int LED_flag = 0;


static volatile float duty[28][2]={
   {0,0},
   {0.2,0.5},
   {0.3,0.7},   
   {0.4,0.9},
   {0,0},
   {0.3,0.5},
   {0.4,0.7},
   {0.5,0.9},
   {0,0},
   {0.4,0.5},
   {0.5,0.7},
   {0.6,0.9},
   {0,0},
   {0.5,0.5},
   {0.7,0.7},
   {0.9,0.9},
   {0,0},
   {0.5,0.4},
   {0.7,0.5},
   {0.9,0.6},
   {0,0},
   {0.5,0.3},
   {0.7,0.4},
   {0.9,0.5},
   {0,0},
   {0.5,0.2},
   {0.7,0.3},
   {0.9,0.4}
};

struct MODE
{
   char MOD;
   float VEL;
   char STR;
   int DIR;
}Mode;

void setup(void);


void main(){   
   setup();
  while(1){
      if(mode=='M'){
         iduty=fabs(dir-i);
         jduty=fabs(dir-j);
         GPIO_write(GPIOC,2,dir);
         GPIO_write(GPIOC,3,dir);
         //TIM_UI_init(TIM3,10);
         sprintf(s1,"\r\nMOD:%c DIR:%c STR:%.2d VEL:%.2d\r\n",mode,direction,angle,speed);
         USART1_write(s1,30);
         delay_ms(1000);
      
      }

      else if (mode == 'N') {
         dir=0;
         IR_diff = value1 - value2;
         if (Stop_flag == 0) {
            if (IR_diff > 400) {
               angle = 1;
               PWM_duty(PA_0, 0);
               PWM_duty(PA_1, 1);
            }
            else if (IR_diff < -400) {
               angle = -1;
               PWM_duty(PA_0, 1);
               PWM_duty(PA_1, 0);
            }
            else {
               angle = 0;
               PWM_duty(PA_0, 1);
               PWM_duty(PA_1, 1);
            }


         }
         distance = (float)timeInterval * 0.034 / 2;    // [mm] -> [cm]
         if (distance < 15 && distance>0) {
            Stop_flag = 1;
            PWM_duty(PA_0, 0);
            PWM_duty(PA_1, 0);

         }
         else {
               Stop_flag = 0;
         }

      }
   }
}

void setup(void){
  RCC_PLL_init();
   SysTick_init();

  // BT serial init 
  UART1_init();
  UART1_baud(BAUD_9600);
   
   UART2_init();
  UART2_baud(BAUD_9600);
   
   USART_setting(USART1, GPIOA, 9, GPIOA, 10, BAUD_9600);
   
   SysTick_init();
   
   // ADC Init
   ADC_init(PB_0);
   ADC_init(PB_1);
   
   // ADC channel sequence setting
   ADC_sequence(seqCHn, 2);
   
   GPIO_init(GPIOA,5,OUTPUT);
   GPIO_init(GPIOC,2,OUTPUT);
   GPIO_init(GPIOC,3,OUTPUT);
   GPIO_write(GPIOC, 2, LOW);
   GPIO_write(GPIOC, 3, LOW);
   
   PWM_init(PA_0);   
   PWM_period_ms(PA_0, 1);
   PWM_init(PA_1);   
   PWM_period_ms(PA_1, 1);
   
   // Timer setting    
   TIM_UI_init(TIM3, 500);         // TIM3 Update-Event Interrupt every 500 msec 
   TIM_UI_enable(TIM3);
   NVIC_EnableIRQ(TIM3_IRQn);   // TIM3 interrupt request enabled   
   NVIC_SetPriority(TIM3_IRQn, 3); // TIM3 interrupt priority

   TIM_UI_init(TIM4, 500);         // TIM3 Update-Event Interrupt every 500 msec 
   TIM_UI_enable(TIM4);
   NVIC_EnableIRQ(TIM4_IRQn);   // TIM3 interrupt request enabled   
   NVIC_SetPriority(TIM4_IRQn, 4); // TIM3 interrupt priority


   // PWM configuration ---------------------------------------------------------------------   
   PWM_init(TRIG);         // PA_6: Ultrasonic trig pulse
   PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
   PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us


   // Input Capture configuration -----------------------------------------------------------------------   
   ICAP_init(ECHO);       // PB_6 as inpmut caputre
   ICAP_counter_us(ECHO, 10);      // ICAP counter step time as 10us
   ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
   ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect
   
}

void TIM3_IRQHandler(void) {
      if ((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF) {
         cnt++;
         if(mode=='M'){
            PWM_duty(PA_0, iduty);
            PWM_duty(PA_1, jduty);
         }
         if (cnt % 80 == 0) {
            LED_flag ^= 1; // flag switching
         }
         if (mode == 'N') {
            GPIO_write(GPIOA, 5, LED_flag);
            if(Stop_flag == 0){
               if (cnt % 80 == 0){
                  sprintf(s1,"\r\nMOD:Auto DIR:FWD STR:%.2d VEL:3\r\n",angle);
                  USART1_write(s1,30);
                  sprintf(s1,"\r\n%.2d %.2d\r\n",value1,value2);
                  USART1_write(s1,30);
               }

            }
         }

         TIM3->SR &= ~TIM_SR_UIF;
   }
      
}

void USART1_IRQHandler(){                // USART2 RX Interrupt : Recommended
   if(is_USART1_RXNE()){
    BT_Data = USART1_read();      // RX from UART1 (BT)
      USART1_write(&BT_Data,1);
      if(BT_Data=='m'||BT_Data=='M'){
         GPIO_write(GPIOA,5, 1);
         mode='M';
      }
      else if(BT_Data=='F'||BT_Data=='f'){
         dir=0;
         i=0;
         j=0;
         direction='F';
         state=12;
         angle=0;
         speed=0;
      }
      else if(BT_Data=='B'||BT_Data=='b'){
         dir=1;
         i=0;
         j=0;
         direction='B';
         state=12;
         angle=0;
         speed=0;
      }
      else if(BT_Data=='W'||BT_Data=='w'){
         if(speed==3){
            return;
         }
         else{
            state++;
            speed++;
            j=duty[state][0];
            i=duty[state][1];
         }
      }
      else if(BT_Data=='S'||BT_Data=='s'){
         if(speed==0){
            return;
         }
         else{
            state--;
            speed--;
            j=duty[state][0];
            i=duty[state][1];
         }
      }
      else if(BT_Data=='A'||BT_Data=='a'){
         if(angle==-3){
            return;
         }
         else{
            angle--;
            if(dir==0){state-=4;}
            else if(dir==1){state+=4;}
            j=duty[state][0];
            i=duty[state][1];
         }
      }
      else if(BT_Data=='D'||BT_Data=='d'){
         if(angle==3){
            return;
         }
         else{
            angle++;
            if(dir==0){state+=4;}
            else if(dir==1){state-=4;}
            j=duty[state][0];
            i=duty[state][1];
         }
      }
      else if(BT_Data=='P'||BT_Data=='p'){
         i=0;
         j=0;
      }
      else if (BT_Data == 'N' || BT_Data == 'n') {
         mode = 'N';
      }
   }
}


void ADC_IRQHandler(void) {
   if (is_ADC_OVR())
      clear_ADC_OVR();

   if (is_ADC_EOC()) {      // after finishing sequence
      if (flag == 0)
         value1 = ADC_read();
      else if (flag == 1)
         value2 = ADC_read();
      
      flag = !flag;      // flag toggle
   }
}

void TIM4_IRQHandler(void) {
   if (is_UIF(TIM4)) {                     // Update interrupt
      ovf_cnt++;                                       // overflow count
      clear_UIF(TIM4);                           // clear update interrupt flag
   }
   if (is_CCIF(TIM4, 1)) {                         // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      time1 = ICAP_capture(TIM4, IC_1);                           // Capture TimeStart
      clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag 
   }
   else if (is_CCIF(TIM4, 2)) {                            // TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
      time2 = ICAP_capture(TIM4, IC_2);                           // Capture TimeEnd
      timeInterval = 10 * ((ovf_cnt * (TIM4->ARR + 1)) + (time2 - time1));    // (10us * counter pulse -> [msec] unit) Total time of echo pulse
      ovf_cnt = 0;                        // overflow reset
      clear_CCIF(TIM4, 2);                          // clear capture/compare interrupt flag 
   }
}