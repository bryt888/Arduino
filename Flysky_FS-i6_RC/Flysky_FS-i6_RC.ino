#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3
int motor_right_1 = 2;//7;
int motor_right_2 = 3;//8;
int motor_right_EN = 11;
int motor_left_1 = 5;
int motor_left_2 = 6;
int motor_left_EN = 10;
float duty = 0;
float dutyLeft = 0;
float dutyRight = 0;

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(motor_right_1, OUTPUT);
  pinMode(motor_right_2, OUTPUT);
  pinMode(motor_right_EN, OUTPUT);
  pinMode(motor_left_1, OUTPUT);
  pinMode(motor_left_2, OUTPUT);
  pinMode(motor_left_EN, OUTPUT);


  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
}

void loop() {
  rc_read_values();
  //stop motor
  
  if (rc_values[RC_CH2]>1600) {
    //move forward
    digitalWrite(motor_right_1,LOW);
    digitalWrite(motor_right_2,HIGH);
    digitalWrite(motor_left_1,LOW);
    digitalWrite(motor_left_2,HIGH);
    duty = map(rc_values[RC_CH2],1500,2200,0,255);
    //left turn
    if (rc_values[RC_CH4]<1400) {
      dutyRight = map(rc_values[RC_CH4],1500,1000,0,duty);
      dutyLeft = duty - dutyRight;
      Serial.println(dutyLeft);
    } else if (rc_values[RC_CH4]>1600) {//right turn
      dutyLeft = map(rc_values[RC_CH4],1500,2000,0,duty);
      dutyRight = duty - dutyLeft;
    }else {
      dutyLeft = duty;
      dutyRight = duty;
    }
    Serial.println("Forward");
  } else if (rc_values[RC_CH2]<1400) {
    //move backward
    digitalWrite(motor_right_1,HIGH);
    digitalWrite(motor_right_2,LOW);
    digitalWrite(motor_left_1,HIGH);
    digitalWrite(motor_left_2,LOW);
    dutyLeft = map(rc_values[RC_CH2],1500,800,0,255);
    dutyRight = dutyLeft;
    Serial.println("Backward");
  } else {
    digitalWrite(motor_right_1,LOW);
    digitalWrite(motor_right_2,LOW);
    digitalWrite(motor_left_1,LOW);
    digitalWrite(motor_left_2,LOW);
    dutyLeft = 0;
    dutyRight = 0;
    Serial.println("Stop");
  }
  analogWrite(motor_left_EN, dutyLeft);
  analogWrite(motor_right_EN, dutyRight);
  Serial.print("Left:"); Serial.print(dutyLeft); Serial.print("\t"); Serial.print("Right:"); Serial.println(dutyRight); 
  //Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  //Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  //Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);

  delay(200);
}
