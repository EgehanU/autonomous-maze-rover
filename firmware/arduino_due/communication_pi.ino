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
#define MOTOR_A_PIN_2 6 //right
#define MOTOR_B_PIN_1 4
#define MOTOR_B_PIN_2 5 //left
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

#define PULSES_PER_REV 14 // to get te actual turns use 140 to get decimal value 14 is used

// Distance filter settings
#define DISTANCE_SAMPLES 5

SharpIR sensor(SharpIR::GP2Y0A21YK0F, IR_SENSOR_PIN);

Servo servo1;
Servo servo2;


static float kp = 3.5, ki = 0.3, kd = 0.0;  
static float error = 0, integral = 0, derivative = 0, last_error = 0;
static unsigned long last_pid_time = 0;

uint16_t ir_distance = 0; 
uint8_t command = 0;
uint8_t device_speed = 0;
uint8_t device_turning_speed = 0;
uint8_t servo_angle = 0; 

volatile long pulse_count_right = 0;
volatile long pulse_count_left = 0;
float rotations_right = 0;
float rotations_left = 0;

long prev_pulse_count_right = 0;
long prev_pulse_count_left = 0;

byte last_command = 0;
bool reset_encoders = false; 
bool stop_flag = false;
bool slave_flag_stop = false;

// Distance filter variables
uint16_t distance_readings[DISTANCE_SAMPLES];
uint8_t reading_index = 0;
bool readings_initialized = false;

uint8_t counter_ir = 0;
unsigned long start_time = 0;
bool test_started = false;
unsigned long system_start_time = 0;

uint16_t get_filtered_distance() {
  uint16_t current_reading = sensor.getDistance();
  
  distance_readings[reading_index] = current_reading;
  reading_index = (reading_index + 1) % DISTANCE_SAMPLES;
  
  if (!readings_initialized && reading_index == 0) {
    readings_initialized = true;
  }
  
  if (!readings_initialized) {
    return current_reading;
  }
  
  // Calculate median of readings
  uint16_t sorted[DISTANCE_SAMPLES];
  memcpy(sorted, distance_readings, sizeof(distance_readings));
  
  // Simple bubble sort for small array
  for (int i = 0; i < DISTANCE_SAMPLES - 1; i++) {
    for (int j = 0; j < DISTANCE_SAMPLES - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        uint16_t temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  
  return sorted[DISTANCE_SAMPLES / 2];
}

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

  delay(1000);
  system_start_time = millis();
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
    uint8_t data[11]; 

    ir_distance = sensor.getDistance();

    data[0] = (ir_distance >> 8);     // High byte
    data[1] = (ir_distance & 0xFF);   // Low byte

    update_rotations();

    memcpy(&data[2], &rotations_right, 4);
    memcpy(&data[6], &rotations_left, 4);

    // Add slave_flag_stop as the 11th byte
    data[10] = slave_flag_stop ? 1 : 0;

    Wire.write(data, 11);
}


uint8_t rotation_comparison(){
    if(abs(rotations_right) == abs(rotations_left))
      return 0;
    else if(abs(rotations_right) > abs(rotations_left))
      return 1;
    else
      return 2; 
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
            //analogWrite(MOTOR_A_PIN_2, device_speed*1.0242); //right
            //analogWrite(MOTOR_B_PIN_2, device_speed); // left
            last_command = MOVE_FORWARD;
            break;
        case MOVE_BACKWARD:
            
            digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_REVERSE);
            digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_REVERSE);
            analogWrite(MOTOR_A_PIN_2, device_speed);
            analogWrite(MOTOR_B_PIN_2, device_speed * 1.08);
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
  stop_flag = false;
  switch(command){
    case MOVE_STOP:
      control_motor_direction_and_speed(MOVE_STOP);
      last_command = MOVE_STOP;  
      error = 0;
      integral = 0;
      derivative = 0;
      last_error = 0;
      stop_flag = true;
      break;
     case MOVE_FORWARD:
      control_motor_direction_and_speed(MOVE_FORWARD);
      last_command = MOVE_FORWARD;  
      break;
     case MOVE_BACKWARD:
      control_motor_direction_and_speed(MOVE_BACKWARD);
      last_command = MOVE_BACKWARD;  
      error = 0;
      integral = 0;
      derivative = 0;
      last_error = 0;
      break;
     case MOVE_COUNTER_CLOCKWISE:
      control_motor_direction_and_speed(MOVE_COUNTER_CLOCKWISE);
      last_command = MOVE_COUNTER_CLOCKWISE; 
      error = 0;
      integral = 0;
      derivative = 0;
      last_error = 0;
      break;
     case MOVE_CLOCKWISE:
      control_motor_direction_and_speed(MOVE_CLOCKWISE);
      last_command = MOVE_CLOCKWISE;  
      error = 0;
      integral = 0;
      derivative = 0;
      last_error = 0;
      break;
     case SET_SERVO_ANGLE:
      servo1.write(servo_angle);
      servo2.write(servo_angle);
      break;
     default:
      control_motor_direction_and_speed(MOVE_STOP);
      last_command = MOVE_STOP;  
      break;
    }
}

/*

void loop() {
  static unsigned long last_pid_time = 0;
  uint16_t dist = sensor.getDistance();


  if (!stop_flag && dist <= 11 && last_command == MOVE_FORWARD) {
    Serial.println("Object detected. Stopping.");
    execute_command(MOVE_STOP);
    stop_flag = true;
    return;
  }

  if (last_command == MOVE_FORWARD && !stop_flag) {
    unsigned long current_time = millis();

    if (current_time - last_pid_time >= 30) {
      long diff_right = pulse_count_right - prev_pulse_count_right;
      long diff_left = pulse_count_left - prev_pulse_count_left;

      prev_pulse_count_right = pulse_count_right;
      prev_pulse_count_left = pulse_count_left;

      error = diff_right - diff_left;
      integral += error;
      integral = constrain(integral, -100, 100);
      derivative = error - last_error;
      last_error = error;

      if (abs(error) < 1) error = 0;

      float correction = kp * error + ki * integral;
      correction = constrain(correction, -80, 80);

      int base_speed = device_speed;
      int left_speed = constrain(base_speed + correction, 0, 255);
      int right_speed = constrain(base_speed - correction, 0, 255);

      analogWrite(MOTOR_A_PIN_2, left_speed);   // right motor
      analogWrite(MOTOR_B_PIN_2, right_speed);  // left motor

      last_pid_time = current_time;
    }
  }
}

void loop() {
  uint16_t dist = sensor.getDistance();  // this is accurate, as you confirmed

  if (!stop_flag && dist <= 11 && last_command == MOVE_FORWARD) {
    Serial.println("Object detected. Stopping.");
    execute_command(MOVE_STOP);
    stop_flag = true;
  }
}



*/
void loop() {
    static unsigned long last_pid_time = 0;
    static bool obstacle_detected = false;
    static uint8_t obstacle_confirm_count = 0;
    static unsigned long last_obstacle_check = 0;

    unsigned long current_time = millis();
    Serial.print("slave_flag_stop:");
    Serial.println(slave_flag_stop ? "TRUE" : "FALSE");

    if (current_time - last_obstacle_check >= 50) {
        uint16_t dist = get_filtered_distance();

        if (last_command == MOVE_FORWARD && !obstacle_detected) {
            if (dist <= 11){
                obstacle_confirm_count++;
                if (obstacle_confirm_count >= 2) {
                    Serial.print("Object detected ");
                    Serial.print(dist);
                   

                    execute_command(MOVE_STOP);
                    obstacle_detected = true;
                    stop_flag = true;
                    obstacle_confirm_count = 0;

                    // Reset PID variables
                    error = 0;
                    integral = 0;
                    derivative = 0;
                    last_error = 0;
                }
            } else {
                obstacle_confirm_count = 0;
            }
        }

        if (obstacle_detected && dist > 11) {
            obstacle_detected = false;
            stop_flag = false;
            slave_flag_stop = false;
        }
        if(obstacle_detected) slave_flag_stop = true;
     

        last_obstacle_check = current_time;
    }

    if (last_command == MOVE_FORWARD && !obstacle_detected && !stop_flag) {
        if (current_time - last_pid_time >= 30) {
            long diff_right = pulse_count_right - prev_pulse_count_right;
            long diff_left = pulse_count_left - prev_pulse_count_left;

            prev_pulse_count_right = pulse_count_right;
            prev_pulse_count_left = pulse_count_left;

            error = diff_left + diff_right; 

            if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
                integral = 0;
            }

            integral += error;
            integral = constrain(integral, -40, 40);

            derivative = error - last_error;
            last_error = error;

            if (abs(error) < 1) error = 0;

            float correction = kp * error + ki * integral;
            correction = constrain(correction, -30, 30);

            int base_speed = device_speed;
            int right_speed = constrain(base_speed + correction * 1.0005, 80, 255); // small bias to right mootor
            int left_speed = constrain(base_speed - correction, 80, 255);

            if (!obstacle_detected && !stop_flag) {
                digitalWrite(MOTOR_A_PIN_1, MOTOR_A_DIR_FORWARD);
                digitalWrite(MOTOR_B_PIN_1, MOTOR_B_DIR_FORWARD);
                analogWrite(MOTOR_A_PIN_2, right_speed);  
                analogWrite(MOTOR_B_PIN_2, left_speed);  
            }

            last_pid_time = current_time;

        }
    }

    if (obstacle_detected && (last_command == MOVE_FORWARD)){
        slave_flag_stop = true;
        digitalWrite(MOTOR_A_PIN_1, LOW);
        digitalWrite(MOTOR_B_PIN_1, LOW);
        analogWrite(MOTOR_A_PIN_2, 0);
        analogWrite(MOTOR_B_PIN_2, 0);
    }
}