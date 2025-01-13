#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "BluetoothSerial.h"

#define ACC_SCL 16
#define ACC_SDA 4

#define FRONT_LEFT_LED_PIN 2
#define FRONT_RIGHT_LED_PIN 13
#define BACK_LEFT_LED_PIN 5
#define BACK_RIGHT_LED_PIN 15

#define STBY1_PIN 17

#define PWMA_PIN 12
#define AIN1_PIN 27
#define AIN2_PIN 14

#define PWMB_PIN 33
#define BIN1_PIN 26
#define BIN2_PIN 25

#define PWMC_PIN 23
#define CIN1_PIN 21
#define CIN2_PIN 22

#define PWMD_PIN 18
#define DIN1_PIN 19
#define DIN2_PIN 32

#define STBY2_PIN 0

#define LED_PWM_RESOLUTION 10
#define LED_PWM_FREQ 100
#define MAX_LED_DC 1 << LED_PWM_RESOLUTION
#define HALF_MAX_LED_DC MAX_LED_DC >> 1
#define MAX_MOTOR_DC 255

#define CMD_SIZE 14
#define PROGRAM_CMDS_COUNT 100
#define CMD_TERMINATOR ';'

// modes
#define FREE_CONTROL 0
#define PROGRAM_CONTROL 1
#define PROGRAM_DEFINITION 2

// program modes
#define IDLE_INSTRUCTION 0
#define ROTATE_INSTRUCTION 1
#define MOVE_INSTRUCTION 2

#define MOTOR_RPM 9800
#define WHEEL_DIM 2.5
#define WHEEL_RAD WHEEL_DIM / 2
#define WHEEL_CIRCUMFERENCE WHEEL_RAD * 2 * PI
#define MOTOR_RPS MOTOR_RPM / 60
#define MOTOR_MAX_SPEED WHEEL_CIRCUMFERENCE * MOTOR_RPS // cm/s

#define MAX_SPEED 1000.0

#define WHEEL_SPAN 76 / 10 // cm

#define ROTATION_FACTOR 2

#define ACC_PULLUP_INTERVAL 50 // ms

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// // Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

String device_name = "CONAN II";

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
BluetoothSerial SerialBT;

char cmd[CMD_SIZE], program[PROGRAM_CMDS_COUNT][CMD_SIZE], next_cmd[CMD_SIZE];
int mode = FREE_CONTROL, instruction = IDLE_INSTRUCTION;

int left_speed = 0, right_speed = 0;
int desired_rotation = 0, desired_distance = 0, desired_speed = 100;
float forward_displacement, rotation_displacement, forward_measured_velocity = 0.0, back_leds_brightness = 0.0;
int instruction_iter = 0, set_instructions = 0;
unsigned long last_time, acc_timer;

bool is_going_forward = false, is_going_backward = false;
bool is_rotating_left = false, is_rotating_right = false;
bool was_bt_connected = false, handle_reverse_leds = false;

void set_left_motors() {
  int duty_cycle = map(abs(left_speed), 0, MAX_SPEED, 0, MAX_MOTOR_DC);

  if(left_speed > 0) { // forward
    digitalWrite(CIN1_PIN, LOW);
    digitalWrite(DIN1_PIN, HIGH);
    digitalWrite(CIN2_PIN, HIGH);
    digitalWrite(DIN2_PIN, LOW);
  } else if(left_speed < 0) { // backward
    digitalWrite(CIN1_PIN, HIGH);
    digitalWrite(DIN1_PIN, LOW);
    digitalWrite(CIN2_PIN, LOW);
    digitalWrite(DIN2_PIN, HIGH);
  } else { // stop
    digitalWrite(CIN1_PIN, LOW);
    digitalWrite(DIN1_PIN, LOW);
    digitalWrite(CIN2_PIN, LOW);
    digitalWrite(DIN2_PIN, LOW);
  }

  analogWrite(PWMC_PIN, duty_cycle);
  analogWrite(PWMD_PIN, duty_cycle);
}

void set_right_motors() {
  int duty_cycle = map(abs(right_speed), 0, MAX_SPEED, 0, MAX_MOTOR_DC);

  if(right_speed > 0) { // forward
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  } else if(right_speed < 0) { // backward
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else { // stop
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
  }

  analogWrite(PWMA_PIN, duty_cycle);
  analogWrite(PWMB_PIN, duty_cycle);
}

void clear_cmd(char* c) {
  c[0] = CMD_TERMINATOR;
  for(int i = 1; i < CMD_SIZE; i++) {
    c[i] = 0;
  }
}

void clear_program() {
  for(int i = 0; i < PROGRAM_CMDS_COUNT; i++) {
    clear_cmd(program[i]);
  }
}

void next_instruction() {
  if(mode != PROGRAM_CONTROL) return;
  if(instruction_iter + 1 < PROGRAM_CMDS_COUNT && program[instruction_iter + 1][0] != CMD_TERMINATOR) {
    clear_cmd(next_cmd);
    strcpy(next_cmd, program[++instruction_iter]);
  } else {
    mode = FREE_CONTROL;
  }
  instruction = IDLE_INSTRUCTION;
}

// in cm per sec
inline float motor_speed(int speed) {
  return MOTOR_MAX_SPEED * (float)speed / MAX_SPEED;
}

inline float rad2deg(float rad) {
  return rad * 180.0 / PI;
}

void setup() {
  ledcAttach(FRONT_LEFT_LED_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcAttach(BACK_LEFT_LED_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcAttach(BACK_RIGHT_LED_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcAttach(FRONT_RIGHT_LED_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);

  // inform setup started
  ledcWrite(FRONT_LEFT_LED_PIN, HALF_MAX_LED_DC);
  ledcWrite(BACK_LEFT_LED_PIN, HALF_MAX_LED_DC);
  ledcWrite(BACK_RIGHT_LED_PIN, HALF_MAX_LED_DC);
  ledcWrite(FRONT_RIGHT_LED_PIN, HALF_MAX_LED_DC);
  delay(200);
  ledcWrite(FRONT_LEFT_LED_PIN, 0);
  ledcWrite(BACK_LEFT_LED_PIN, 0);
  ledcWrite(BACK_RIGHT_LED_PIN, 0);
  ledcWrite(FRONT_RIGHT_LED_PIN, 0);

  pinMode(STBY1_PIN, OUTPUT);
  pinMode(STBY2_PIN, OUTPUT);

  digitalWrite(STBY1_PIN, LOW);
  digitalWrite(STBY2_PIN, LOW);
  
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  pinMode(PWMC_PIN, OUTPUT);
  pinMode(CIN1_PIN, OUTPUT);
  pinMode(CIN2_PIN, OUTPUT);

  pinMode(PWMD_PIN, OUTPUT);
  pinMode(DIN1_PIN, OUTPUT);
  pinMode(DIN2_PIN, OUTPUT);

  Serial.begin(115200);
  SerialBT.begin(device_name);  //Bluetooth device name

  if(!accel.begin(ADXL345_DEFAULT_ADDRESS, ACC_SDA, ACC_SCL))
  {
    Serial.println("No valid sensor found");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_2_G); 

  clear_program();
  last_time = millis();

  // inform setup is done  
  delay(200);
  ledcWrite(FRONT_LEFT_LED_PIN, HALF_MAX_LED_DC);
  ledcWrite(BACK_LEFT_LED_PIN, HALF_MAX_LED_DC);
  ledcWrite(BACK_RIGHT_LED_PIN, HALF_MAX_LED_DC);
  ledcWrite(FRONT_RIGHT_LED_PIN, HALF_MAX_LED_DC);
  delay(1000);
  ledcWrite(FRONT_LEFT_LED_PIN, 0);
  ledcWrite(BACK_LEFT_LED_PIN, 0);
  ledcWrite(BACK_RIGHT_LED_PIN, 0);
  ledcWrite(FRONT_RIGHT_LED_PIN, 0);

  clear_cmd(cmd);

  Serial.println("Configuration complete! :)");
}

void loop() {
  unsigned long now = micros();
  float dt = (float)(now - last_time) / 1000000.0;
  if(mode == PROGRAM_CONTROL) {
    switch(instruction) {
      case ROTATE_INSTRUCTION: {
        if(desired_rotation == 0) {
          next_instruction();
          break;
        }

        if(is_rotating_left || is_rotating_right) {
          rotation_displacement += rad2deg(ROTATION_FACTOR * 2 * motor_speed(desired_speed) * dt / WHEEL_SPAN);

          if(rotation_displacement >= (float)abs(desired_rotation)) {
            left_speed = 0;
            right_speed = 0;
            set_left_motors();
            set_right_motors();
            next_instruction();
            break;
          }
        } else {
          if(desired_rotation > 0) {
            is_rotating_left = false;
            is_rotating_right = true;
            left_speed = desired_speed;
            right_speed = -desired_speed;
          } else {
            is_rotating_left = true;
            is_rotating_right = false;
            left_speed = -desired_speed;
            right_speed = desired_speed;
          }
          set_left_motors();
          set_right_motors();
        }
      } break;
      case MOVE_INSTRUCTION: {
        if(desired_distance == 0) {
          next_instruction();
          break;
        }
        if(is_going_forward || is_going_backward) {
          forward_displacement += motor_speed(desired_speed) * dt;

          if(forward_displacement >= (float)abs(desired_distance)) {
            left_speed = 0;
            right_speed = 0;
            set_left_motors();
            set_right_motors();
            next_instruction();
            break;
          }
        } else {
          if(desired_distance > 0) {
            is_going_forward = true;
            is_going_backward = false;
            left_speed = right_speed = desired_speed;
          } else {
            is_going_forward = true;
            is_going_backward = false;
            handle_reverse_leds = true;
            left_speed = right_speed = -desired_speed;
          }
          set_left_motors();
          set_right_motors();
        }
      } break;
      default: break;
    }
  }

  if(handle_reverse_leds) {
    if(left_speed < 0 && right_speed < 0 && back_leds_brightness != 1) {
      back_leds_brightness = min(back_leds_brightness + dt * 0.5, 1.0);
      ledcWrite(BACK_LEFT_LED_PIN, back_leds_brightness * (float)(MAX_LED_DC));
      ledcWrite(BACK_RIGHT_LED_PIN, back_leds_brightness * (float)(MAX_LED_DC));
    } else if(left_speed != 0 && right_speed != 0) {
      ledcWrite(BACK_LEFT_LED_PIN, 0);
      ledcWrite(BACK_RIGHT_LED_PIN, 0);
    }
  }

  if(!was_bt_connected && SerialBT.hasClient()) {
    handle_reverse_leds = false;
    for(int i = 0; i < HALF_MAX_LED_DC; i++) {
      ledcWrite(FRONT_LEFT_LED_PIN, i);
      ledcWrite(FRONT_RIGHT_LED_PIN, i);
      ledcWrite(BACK_LEFT_LED_PIN, i);
      ledcWrite(BACK_RIGHT_LED_PIN, i);
      delay(1);
    }

    for(int i = HALF_MAX_LED_DC; i >= 0; i--) {
      ledcWrite(FRONT_LEFT_LED_PIN, i);
      ledcWrite(FRONT_RIGHT_LED_PIN, i);
      ledcWrite(BACK_LEFT_LED_PIN, i);
      ledcWrite(BACK_RIGHT_LED_PIN, i);
      delay(1);
    }
    ledcWrite(FRONT_LEFT_LED_PIN, 0);
    ledcWrite(FRONT_RIGHT_LED_PIN, 0);
    ledcWrite(BACK_LEFT_LED_PIN, 0);
    ledcWrite(BACK_RIGHT_LED_PIN, 0);
    SerialBT.println("ready");
    was_bt_connected = true;
  } else if(was_bt_connected && !SerialBT.hasClient()) {
    handle_reverse_leds = false;
    ledcWrite(FRONT_LEFT_LED_PIN, 0);
    ledcWrite(FRONT_RIGHT_LED_PIN, 0);
    ledcWrite(BACK_LEFT_LED_PIN, 0);
    ledcWrite(BACK_RIGHT_LED_PIN, 0);
    delay(100);
    ledcWrite(FRONT_LEFT_LED_PIN, HALF_MAX_LED_DC);
    delay(100);
    ledcWrite(FRONT_LEFT_LED_PIN, 0);
    ledcWrite(FRONT_RIGHT_LED_PIN, HALF_MAX_LED_DC);
    delay(100);
    ledcWrite(FRONT_RIGHT_LED_PIN, 0);
    ledcWrite(BACK_LEFT_LED_PIN, HALF_MAX_LED_DC);
    delay(100);
    ledcWrite(BACK_LEFT_LED_PIN, 0);
    ledcWrite(BACK_RIGHT_LED_PIN, HALF_MAX_LED_DC);
    delay(100);
    ledcWrite(BACK_RIGHT_LED_PIN, 0);
    was_bt_connected = false;
  }

  if(mode == PROGRAM_CONTROL && instruction == IDLE_INSTRUCTION) {
    strcpy(cmd, next_cmd);
  }

  if (SerialBT.available()) {
    clear_cmd(cmd);
    SerialBT.readBytesUntil(CMD_TERMINATOR, cmd, CMD_SIZE);
  }

  if(mode == PROGRAM_DEFINITION && cmd[0] > 'C' ) {
    if(instruction_iter + 1 < 100) {
      strcpy(program[instruction_iter++], cmd);
    }
  } else {
    int k;
    switch(cmd[0]) {
      case 'A': {
        k = sscanf(cmd, "A%d,%d", &left_speed, &right_speed);
        if(k == 2) {
          left_speed = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
          right_speed = constrain(right_speed, -MAX_SPEED, MAX_SPEED);
          handle_reverse_leds = left_speed < 0 && right_speed < 0;
          set_left_motors();
          set_right_motors();
          mode = FREE_CONTROL;
        }
      } break;
      case 'B': {
        clear_program();
        mode = PROGRAM_DEFINITION;
        instruction = IDLE_INSTRUCTION;
        instruction_iter = 0;
        Serial.println("Przejscie do trybu programowania");
      } break;
      case 'C': {
        mode = PROGRAM_CONTROL;
        instruction_iter = 0;
        Serial.println("Przejscie do trybu wykonywania programu");
        strcpy(next_cmd, program[0]);
        Serial.print("Ustawienie nastepnej komendy ");
        Serial.println(next_cmd);
      } break;
      case 'D': { // rotate
        k = sscanf(cmd, "D%d", &desired_rotation);
        if(k == 1) {
          rotation_displacement = 0;
          is_rotating_left = false;
          is_rotating_right = false;
          instruction = ROTATE_INSTRUCTION;
        }
      } break;
      case 'E': {
        k = sscanf(cmd, "E%d", &desired_distance);
        if(k == 1) {
          forward_displacement = 0;
          is_going_forward = false;
          is_going_backward = false;
          instruction = MOVE_INSTRUCTION;
        }
      } break;
      case 'F': {
        k = sscanf(cmd, "F%d", &desired_speed);
        if(k == 1) {
          desired_speed = abs(constrain(desired_speed, -MAX_SPEED, MAX_SPEED));
          next_instruction();
        }
      } break;
      case 'G': {
        instruction_iter = -1;
        next_instruction();
        break;
      }
      case 'H': {
        ledcWrite(FRONT_LEFT_LED_PIN, MAX_LED_DC);
        ledcWrite(FRONT_RIGHT_LED_PIN, MAX_LED_DC);
        Serial.println("Front lights ON");
        next_instruction();
      } break;
      case 'I': {
        ledcWrite(FRONT_LEFT_LED_PIN, 0);
        ledcWrite(FRONT_RIGHT_LED_PIN, 0);
        Serial.println("Front lights OFF");
        next_instruction();
      } break;
      case 'J': {
        digitalWrite(STBY1_PIN, HIGH);
        digitalWrite(STBY2_PIN, HIGH);
        next_instruction();
      } break;
      case 'K': {
        digitalWrite(STBY1_PIN, LOW);
        digitalWrite(STBY2_PIN, LOW);
        next_instruction();
      } break;
      case 'L': {
        for(int i = 0; i < PROGRAM_CMDS_COUNT; i++) {
          if(program[i][0] != ';') {
            SerialBT.println(program[i]);
          }
        }
        next_instruction();
      } break;
      case 'M': {
        SerialBT.print(forward_measured_velocity);
      } break;
      default: break;
    }
  }
  clear_cmd(cmd);
  
  acc_timer += now - last_time;
  if(acc_timer >= ACC_PULLUP_INTERVAL) {
    sensors_event_t event;
    accel.getEvent(&event);
    forward_measured_velocity += -event.acceleration.x * ACC_PULLUP_INTERVAL;
    acc_timer = 0;
  }
  
  last_time = now;
}
