/*-------------------------------------------------------------------------------\
@ [LAB] Arduino-STM32 mini-fan
Author           : EunChan Kim
Created          : 08-09-2023
Modified         : 14-09-2023
Language/ver     : Arduino IDE 2.2.0
-------------------------------------------------------------------------------*/

// State definition
#define S0  0   // Fan OFF
#define S1  1   // Fan vel = 50%
#define S2  2   // Fan vel = 100%

// Address number of output in array
#define PWM 0
#define LED 1

// State table definition
typedef struct {
  uint32_t out[2][2];     // output = FSM[state].out[input X][PWM or LED]
  uint32_t next[2][2];    // nextstate = FSM[state].next[input X][input Y]
} State_t;

State_t FSM[3] = {
  { {{0 , 0},    {0 , 255/2}}, {{S0, S0}, {S1, S1}} },
  { {{0 , 255/2},{0 , 255}},   {{S1, S1}, {S2, S2}} },
  { {{0 , 255},  {0 , 0}},     {{S2, S2}, {S0, S0}} }
};

// Pin setting
const int ledPin = 13;
const int pwmPin = 11;
const int btnPin = 3;
const int trigPin = 10;
const int echoPin = 7;

// initialization
uint32_t state = S0;
uint32_t input[2] = {0, 0};
uint32_t pwmOut = 0;
uint32_t ledOut = LOW;

unsigned long duration;
float distance;
int thresh = 5;
unsigned long time = 0;

void setup() {  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);

  // Initialize pwm pin as an output:
  pinMode(pwmPin, OUTPUT);
  
  // initialize the pushbutton pin as an interrupt input:
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPin), pressed, FALLING);

  // Initialize the trigger pin as an output
  pinMode(trigPin, OUTPUT);

  // Initialize the echo pin as an input
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Generate pwm signal on the trigger pin.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  // Distance is calculated using how much time it takes.
  duration = pulseIn(echoPin, HIGH);
  distance = (float)duration / 58.0;

  // Calculate next state, then update State
  nextState();

  // Output of states
  stateOutput();

  analogWrite(pwmPin, pwmOut);
  digitalWrite(ledPin, ledOut);
  
  // Print the message
  Serial.print("Input : ");
  Serial.print(input[1]);
  Serial.print(", ");
  Serial.print("State : ");
  Serial.print(state);
  Serial.print(", ");
  Serial.print("distance = ");
  Serial.print(distance);
  Serial.println(" [cm]");
  
  delay(1000);
}

// When button pressed
void pressed(){
  input[0] = 1;
  stateOutput();
  nextState();
  input[0] = 0;
}

void nextState(){
  if (distance < thresh)
    input[1] = 1;
  else
    input[1] = 0;
    
  // get next state 
  state = FSM[state].next[input[0]][input[1]];
}

void stateOutput(){
  // Blink LED 
  if (state == S0){
    ledOut = 0;  
  }
  else{
    
    if (millis() - time >= 1000){
      if(ledOut == HIGH)
        ledOut = LOW;
      else 
        ledOut = HIGH;

      time = millis();
    }
  }


  pwmOut = FSM[state].out[input[0]][input[1]];  
}
