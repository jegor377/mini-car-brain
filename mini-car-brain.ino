#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "BluetoothSerial.h"

#define ACC_SCL 16
#define ACC_SDA 4

#define LED1_PIN 2
#define LED2_PIN 5
#define LED3_PIN 15
#define LED4_PIN 13

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
#define MAX_MOTOR_DC

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
#define MOTOR_MAX_SPEED WHEEL_CIRCUMFERENCE * MOTOR_RPS

#define MAX_SPEED 1000.0


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

char cmd[CMD_SIZE], program[PROGRAM_CMDS_COUNT][CMD_SIZE];
int mode = FREE_CONTROL, instruction = IDLE_INSTRUCTION;

int left_speed, right_speed;
int desired_rotation, desired_distance;
float forward_displacement, rotation_displacement;
int instruction_iter = 0, set_instructions = 0;
unsigned long last_time;

bool is_going_forward, is_going_backward;

void clear_program() {
  for(int i = 0; i < PROGRAM_CMDS_COUNT; i++) {
    strcpy(program[i], ";");
  }
}

void set_left_motors() {
  // float duty_cycle = abs(left_speed) * MAX_MOTOR_DC;
  
  // analogWrite(PWMC_PIN, duty_cycle);
  // analogWrite(PWMD_PIN, duty_cycle);
}

void set_right_motors() {
  // float duty_cycle = abs(right_speed) * MAX_MOTOR_DC;

  // analogWrite(PWMA_PIN, duty_cycle);
  // analogWrite(PWMB_PIN, duty_cycle);
}

void clear_cmd() {
  strcpy(cmd, ";");
}

void next_instruction() {
  if(instruction_iter < PROGRAM_CMDS_COUNT) {
    strcpy(cmd, program[instruction_iter++]);
  } else {
    mode = FREE_CONTROL;
    instruction = IDLE_INSTRUCTION;
  }
}

inline float motor_speed(int speed) {
  return MOTOR_MAX_SPEED * (float)speed / MAX_SPEED;
}

void setup() {
  ledcAttach(LED1_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcAttach(LED2_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcAttach(LED3_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcAttach(LED4_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
  ledcOutputInvert(LED2_PIN, true);
  ledcOutputInvert(LED3_PIN, true);

  // inform setup started
  ledcWrite(LED1_PIN, MAX_LED_DC);
  ledcWrite(LED2_PIN, MAX_LED_DC);
  ledcWrite(LED3_PIN, MAX_LED_DC);
  ledcWrite(LED4_PIN, MAX_LED_DC);
  delay(200);
  ledcWrite(LED1_PIN, 0);
  ledcWrite(LED2_PIN, 0);
  ledcWrite(LED3_PIN, 0);
  ledcWrite(LED4_PIN, 0);

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
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  // Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  if(!accel.begin(ADXL345_DEFAULT_ADDRESS, ACC_SDA, ACC_SCL))
  {
    Serial.println("No valid sensor found");
    while(1);
  }

  clear_program();
  last_time = millis();

  // inform setup is done  
  delay(200);
  ledcWrite(LED1_PIN, MAX_LED_DC);
  ledcWrite(LED2_PIN, MAX_LED_DC);
  ledcWrite(LED3_PIN, MAX_LED_DC);
  ledcWrite(LED4_PIN, MAX_LED_DC);
  delay(1000);
  ledcWrite(LED1_PIN, 0);
  ledcWrite(LED2_PIN, 0);
  ledcWrite(LED3_PIN, 0);
  ledcWrite(LED4_PIN, 0);
}

void loop() {
  if(mode == PROGRAM_CONTROL) {
    float dt = (float)(millis() - last_time) / 1000.0;
    switch(instruction) {
      case ROTATE_INSTRUCTION: {
        ;
      } break;
      case MOVE_INSTRUCTION: {
        if(desired_distance == 0) {
          next_instruction();
          break;
        }
        if(is_going_forward || is_going_backward) {
          forward_displacement += motor_speed(abs(left_speed)) * dt;

          if(forward_displacement >= desired_distance) {
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
          } else if(desired_distance < 0) {
            is_going_forward = true;
            is_going_backward = false;
            left_speed = right_speed = -left_speed;
          }
          set_left_motors();
          set_right_motors();
        }
      } break;
      default: break;
    }
  }

  if (SerialBT.available()) {
    SerialBT.readBytesUntil(CMD_TERMINATOR, cmd, CMD_SIZE);
    if(mode == PROGRAM_CONTROL) {
      instruction_iter--;
    }
  }

  if(mode == PROGRAM_DEFINITION && cmd[0] > 'C' ) {
    if(instruction_iter + 1 < 100) {
      strcpy(program[instruction_iter++], cmd);
    }
  }

  int k;
  switch(cmd[0]) {
    case 'A': {
      k = sscanf(cmd, "A%d,%d;", &left_speed, &right_speed);
      if(k == 2) {
        left_speed = constrain(left_speed, -MAX_SPEED, MAX_SPEED);
        right_speed = constrain(right_speed, -MAX_SPEED, MAX_SPEED);
        set_left_motors();
        set_right_motors();
        mode = FREE_CONTROL;
        clear_cmd();
      }
    } break;
    case 'B': {
      clear_program();
      mode = PROGRAM_DEFINITION;
      instruction = IDLE_INSTRUCTION;
      instruction_iter = 0;
      clear_cmd();
    } break;
    case 'C': {
      mode = PROGRAM_CONTROL;
      clear_cmd();
    } break;
    case 'D': { // rotate
      k = sscanf(cmd, "D%d;", &desired_rotation);
      if(k == 1) {
        rotation_displacement = 0;
        instruction = ROTATE_INSTRUCTION;
      }
    } break;
    case 'E': {
      k = sscanf(cmd, "E%d;", &desired_distance);
      if(k == 1) {
        forward_displacement = 0;
        is_going_forward = false;
        is_going_backward = false;
        instruction = MOVE_INSTRUCTION;
      }
    } break;
    case 'F': {
      k = sscanf(cmd, "F%d;", &left_speed);
      if(k == 1) {
        left_speed = abs(constrain(left_speed, -MAX_SPEED, MAX_SPEED));
        right_speed = left_speed;
        next_instruction();
      }
    } break;
    case 'G': {
      instruction_iter = 0;
      break;
    }
    default: break;
  }

  last_time = millis();
}
