#include "STM32TimerInterrupt.h"
#include "STM32_ISR_Timer.h"

#define HW_TIMER_INTERVAL_MS  100
#define TIMER_INTERVAL        1000L

STM32Timer ITimer(TIM1);    // Init STM32 timer TIM1
STM32_ISR_Timer ISR_Timer;  // Init STM32_ISR_Timer

const int ledPin =  13;

void setup() {
  pinMode(ledPin, OUTPUT);

  ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler);
  ISR_Timer.setInterval(TIMER_INTERVAL, timerInterrupt);
}

void loop(){
}

void TimerHandler(){
  ISR_Timer.run();
}

void timerInterrupt(){
  digitalWrite(ledPin, !digitalRead(ledPin));
}