#include <SoftwareSerial.h>

const uint8_t MC_EN_L = 8; // PB0
const uint8_t MC_EN_R = 11; // PB3
const uint8_t MC_1L = 9; // PB1
const uint8_t MC_2L = 10; // PB2
const uint8_t MC_1R = 5;// PD5
const uint8_t MC_2R = 6;// PD6

const uint8_t BLT_ST = 4; // PD4
const uint8_t BLT_RX = 12; // PB4
const uint8_t BLT_TX = 13; // PB5
const uint8_t BLT_CON = 7; // PD7

const uint8_t CMD_SIZE = 16;
const char CMD_TERMINATOR = 'K';

const float PWM_FREQ = 490; // Hz
const float MICROSECOND = 1000000; // 1000000us = 1s
const float PWM_WAIT_TIME = MICROSECOND / PWM_FREQ;

char cmd[CMD_SIZE];
int left_pwm = 0, right_pwm = 0;
// software PWM for MC_EN_L
bool is_high = false;
unsigned long on_time = 0, off_time = 0; // in us

void setup() {
  pinMode(MC_EN_L, OUTPUT);
  pinMode(MC_EN_R, OUTPUT);
  pinMode(MC_1L, OUTPUT);
  pinMode(MC_2L, OUTPUT);
  pinMode(MC_1R, OUTPUT);
  pinMode(MC_2R, OUTPUT);

  pinMode(BLT_CON, OUTPUT);

  Serial.begin(9600);

  digitalWrite(MC_EN_L, LOW);
  digitalWrite(MC_EN_R, LOW);
  
  digitalWrite(BLT_CON, HIGH);
  delay(500);
  digitalWrite(BLT_CON, LOW);
  delay(500);
  digitalWrite(BLT_CON, HIGH);
  delay(500);
  digitalWrite(BLT_CON, LOW);
  delay(500);
  digitalWrite(BLT_CON, HIGH);
  delay(500);
}

inline bool blt_connected() {
  return digitalRead(BLT_ST);
}

void loop() {
  if (Serial.available()) {
    Serial.readBytesUntil(CMD_TERMINATOR, cmd, CMD_SIZE);
    int k = sscanf(cmd, "C%d,%d;", &left_pwm, &right_pwm);
    if(k == 2) {
      left_pwm = min(max(left_pwm, -0xFF), 0xFF);
      right_pwm = min(max(right_pwm, -0xFF), 0xFF);
    }

    if(left_pwm > 0) {
      digitalWrite(MC_1L, HIGH);
      digitalWrite(MC_2L, LOW);
    } else if(left_pwm < 0) {
      left_pwm *= -1;
      digitalWrite(MC_1L, LOW);
      digitalWrite(MC_2L, HIGH);
    } else {
      digitalWrite(MC_1L, LOW);
      digitalWrite(MC_2L, LOW);
    }

    if(right_pwm > 0) {
      analogWrite(MC_EN_R, right_pwm);
      digitalWrite(MC_1R, HIGH);
      digitalWrite(MC_2R, LOW);
    } else if(right_pwm < 0) {
      analogWrite(MC_EN_R, right_pwm * (-1));
      digitalWrite(MC_1R, LOW);
      digitalWrite(MC_2R, HIGH);
    } else {
      analogWrite(MC_EN_R, 0);
      digitalWrite(MC_1R, LOW);
      digitalWrite(MC_2R, LOW);
    }
  }

  unsigned long current_time = micros();
  float norm_pwm = (float)left_pwm / 255.0;
  // if is high and time passed since last on time is greater or equal than proper on time
  if(is_high && ((current_time - on_time) >= (PWM_WAIT_TIME * norm_pwm))) {
    digitalWrite(MC_EN_L, LOW);
    is_high = false;
    off_time = current_time;
  // if is low and time passed since last off time is greater or equal than proper off time
  } else if(!is_high && ((current_time - off_time) >= (PWM_WAIT_TIME - PWM_WAIT_TIME * norm_pwm))) {
    digitalWrite(MC_EN_L, HIGH);
    is_high = true;
    on_time = current_time;
  }

  if(!blt_connected()) {
    digitalWrite(MC_1L, LOW);
    digitalWrite(MC_2L, LOW);
    digitalWrite(MC_1R, LOW);
    digitalWrite(MC_2R, LOW);
    digitalWrite(BLT_CON, LOW);
  } else {
    digitalWrite(BLT_CON, HIGH);
  }
}
