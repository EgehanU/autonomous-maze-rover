#include <Wire.h>
#include <Servo.h>
#include <SharpIR.h>



#define MOVE_STOP 0
#define MOVE_FORWARD 1
#define MOVE_BACKWARD 2
#define MOVE_COUNTER_CLOCKWISE 3
#define MOVE_CLOCKWISE 4

#define SLAVE_ADDRESS 0x04

//Motor pins
#define MOTOR_A_PIN_1 7
#define MOTOR_A_PIN_2 6 //left
#define MOTOR_B_PIN_1 4
#define MOTOR_B_PIN_2 5 //right
#define MOTOR_ENABLE_A 9
#define MOTOR_ENABLE_B 10

#define IR_SENSOR_PIN A9

#define MOTOR_A_DIR_FORWARD LOW
#define MOTOR_A_DIR_REVERSE HIGH
#define MOTOR_B_DIR_FORWARD LOW
#define MOTOR_B_DIR_REVERSE HIGH

// New command for direct servo control
#define SET_SERVO_ANGLE 8

#define SERVO_1_PIN 22
#define SERVO_2_PIN 23

// Encoder pins, B is for the direction
#define ENCODER_RIGHT_A 18
#define ENCODER_RIGHT_B 19
#define ENCODER_LEFT_A 16
#define ENCODER_LEFT_B 17

#define PULSES_PER_REV 14 // to get te actual turns use 200 to get decimal value 20 is used


SharpIR sensor(SharpIR::GP2Y0A21YK0F, IR_SENSOR_PIN);

Servo servo1;
Servo servo2;

uint16_t ir_distance = 0; 
uint8_t command = 0;
uint8_t device_speed = 0;
uint8_t device_turning_speed = 0;
uint8_t servo_angle = 0; 

volatile long pulse_count_right = 0;
volatile long pulse_count_left = 0;
float rotations_right = 0;
float rotations_left = 0;

byte last_command = 0;
bool reset_encoders = false; 

void setup(){
  
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(received_data);
  Wire.onRequest(send_data);

  // Initialize motor pins 
  pinMode(MOTOR_A_PIN_1, OUTPUT);
  pinMode(MOTOR_B_PIN_1, OUTPUT);
  pinMode(MOTOR_A_PIN_2, OUTPUT);
  pinMode(MOTOR_B_PIN_2, OUTPUT);
  pinMode(MOTOR_ENABLE_A, OUTPUT);
  pinMode(MOTOR_ENABLE_B, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);

  digitalWrite(MOTOR_ENABLE_A, HIGH);
  digitalWrite(MOTOR_ENABLE_B, HIGH);

  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);
  
  // Encoder setup
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), count_right, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), count_left, RISING);
}

void count_left(){
    if (digitalRead(ENCODER_LEFT_B) == HIGH)
      pulse_count_left++;
    else
      pulse_count_left--;  
}

void count_right(){
  if (digitalRead(ENCODER_RIGHT_B) == HIGH)
    pulse_count_right++;
  else
    pulse_count_right--;  
}

void update_rotations(){
    rotations_right = (pulse_count_right / PULSES_PER_REV) * -1;
    rotations_left = (pulse_count_left / PULSES_PER_REV);
}

void clear_rotation_count(){
    pulse_count_right = 0;
    pulse_count_left = 0;
    rotations_right = 0;
    rotations_left = 0;
}

void received_data(int byte_count) {
  if (byte_count >= 3) {  
    command = Wire.read(); 
    device_speed = Wire.read();
    device_turning_speed = Wire.read();
    
    // Check if this is a servo angle command or reset encoders command
    if (command == SET_SERVO_ANGLE && byte_count >= 4) {
      servo_angle = Wire.read(); // Read the fourth byte as servo angle
      execute_command(command);
      return;
    }
    
    // Check if this is a reset encoders command (99)
    if (command == 99) {
      clear_rotation_count();
      return;
    }
    
    execute_command(command);
  }
}

void send_data(){
    uint8_t data[10];
  
    // Get distance from Sharp ir sensor in cm
    ir_distance = sensor.getDistance();
  
    data[0] = (ir_distance >> 8); // High byte
    data[1] = (ir_distance & 0xFF); // Lower byte
  
    // Always update rotations before sending
    update_rotations();
  
    memcpy(&data[2], &rotations_right, 4);
    memcpy(&data[6], &rotations_left, 4);

    Wire.write(data, 10);
}

uint8_t rotation_comparison(){
    if(abs(rotations_right) == abs(rotations_left))
      return 0;
    else if(abs(rotations_right) > abs(rotations_left))
      return 1;
    else
      return 2; 
}
// probably not going to be used
void controller(byte last_command, uint8_t vel){
  uint8_t comparison = rotation_comparison();
  if(comparison == 1){
    float difference = abs(rotations_right) - abs(rotations_left);
    while(abs(difference) > 1){
      switch(last_command){
      case MOVE_FORWARD:
        digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_FORWARD);
        analogWrite(MOTOR_B_PIN_2, vel);
        break;
      case MOVE_BACKWARD:
        digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_REVERSE);
        analogWrite(MOTOR_B_PIN_2, vel);
        break;
      case MOVE_COUNTER_CLOCKWISE:
        digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_REVERSE);
        analogWrite(MOTOR_B_PIN_2, vel);
        break;
      case MOVE_CLOCKWISE:
        digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_FORWARD);
        analogWrite(MOTOR_B_PIN_2, vel);
        break;  
      }
      update_rotations();
      difference = abs(rotations_right) - abs(rotations_left);
    }
      digitalWrite(MOTOR_A_PIN_1, LOW);
      digitalWrite(MOTOR_B_PIN_1, LOW);
      analogWrite(MOTOR_A_PIN_2, 0);
      analogWrite(MOTOR_B_PIN_2, 0);
  }
  else if(comparison == 2){
    float difference = abs(rotations_left) - abs(rotations_right);
      while(abs(difference) > 1){
        switch(last_command){
          case MOVE_FORWARD:
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_FORWARD);
            analogWrite(MOTOR_A_PIN_2, vel);
            break;
          case MOVE_BACKWARD:
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_REVERSE);
            analogWrite(MOTOR_A_PIN_2, vel);
            break;
          case MOVE_COUNTER_CLOCKWISE:
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_FORWARD);
            analogWrite(MOTOR_A_PIN_2, vel);
            break;
          case MOVE_CLOCKWISE:
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_REVERSE);
            analogWrite(MOTOR_A_PIN_2, vel);
            break;
        }
        update_rotations();
        difference = abs(rotations_left) - abs(rotations_right);
      }
      digitalWrite(MOTOR_A_PIN_1, LOW);
      digitalWrite(MOTOR_B_PIN_1, LOW);
      analogWrite(MOTOR_A_PIN_2, 0);
      analogWrite(MOTOR_B_PIN_2, 0);
  }
}

void control_motor_direction_and_speed(byte command) {
    switch (command) {
        case MOVE_STOP:
            update_rotations();
            digitalWrite(MOTOR_A_PIN_1, LOW);
            digitalWrite(MOTOR_B_PIN_1, LOW);
            analogWrite(MOTOR_A_PIN_2, 0);
            analogWrite(MOTOR_B_PIN_2, 0);
            //controller(last_command, device_speed);
            break;
        case MOVE_FORWARD:
          
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_FORWARD);
            digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_FORWARD);
            analogWrite(MOTOR_A_PIN_2, device_speed); //left
            analogWrite(MOTOR_B_PIN_2, device_speed); // right
            last_command = MOVE_FORWARD;
            break;
        case MOVE_BACKWARD:
            
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_REVERSE);
            digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_REVERSE);
            analogWrite(MOTOR_A_PIN_2, device_speed);
            analogWrite(MOTOR_B_PIN_2, device_speed * 1.1);
            last_command = MOVE_BACKWARD;
            break;
        case MOVE_COUNTER_CLOCKWISE:
          
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_FORWARD);
            digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_REVERSE);
            analogWrite(MOTOR_A_PIN_2, device_turning_speed);
            analogWrite(MOTOR_B_PIN_2, device_turning_speed);
            last_command = MOVE_COUNTER_CLOCKWISE;
            break;
        case MOVE_CLOCKWISE:
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_REVERSE);
            digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_FORWARD);
            analogWrite(MOTOR_A_PIN_2, device_turning_speed * 1.2);
            analogWrite(MOTOR_B_PIN_2, device_turning_speed * 1.1);
            last_command = MOVE_CLOCKWISE;
            break;
         default:
            break;
    }
}

void execute_command(byte command){
  switch(command){
    case MOVE_STOP:
      control_motor_direction_and_speed(MOVE_STOP);
      break;
     case MOVE_FORWARD:
      control_motor_direction_and_speed(MOVE_FORWARD);
      break;
     case MOVE_BACKWARD:
      control_motor_direction_and_speed(MOVE_BACKWARD);
      break;
     case MOVE_COUNTER_CLOCKWISE:
      control_motor_direction_and_speed(MOVE_COUNTER_CLOCKWISE);
      break;
     case MOVE_CLOCKWISE:
      control_motor_direction_and_speed(MOVE_CLOCKWISE);
      break;
     case SET_SERVO_ANGLE:
      // Set both servos to the provided angle
      servo1.write(servo_angle);
      servo2.write(servo_angle);
      break;
     default:
      control_motor_direction_and_speed(MOVE_STOP);
      break;
    }
}

void loop(){
  if(sensor.getDistance() < 9){ // Object very close 
    execute_command(MOVE_STOP);
    while(true){
      delay(1000);  
    } 
  }   
}