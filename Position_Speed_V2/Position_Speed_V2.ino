#include <BluetoothSerial.h>
#include <Arduino.h>
#include <math.h>

BluetoothSerial SerialBT;

// Pin definitions [Your existing pin definitions]
// === Pin Definitions ===

// Encoder Pins
//left encoder (unchanged)
#define leftEncoderA 4  // Changed to match your new setup
#define leftEncoderB 5  // Changed to match your new setup

//right encoder (unchanged)
#define rightEncoderA 34  // Changed to match your new setup
#define rightEncoderB 35  // Changed to match your new setup

// Motor Driver Pins
// Right Motor
int rightMotorPin1 = 32; 
int rightMotorPin2 = 33; 
int enable1Pin = 14; 

// Left Motor
int leftMotorPin1 = 12; 
int leftMotorPin2 = 13; 
int enable2Pin = 15; 

// Encoder variables
volatile long leftCounter = 0, rightCounter = 0;
volatile bool changeLeft = false, changeRight = false;
unsigned long lastTime, currentTime;
double leftEncoderSpeed, rightEncoderSpeed;  // Speed in meters/second

// PID Constants
const float KP_L = 1.8;
const float KI_L = 5.0;
const float KD_L = 0.1;
const float KP_R = 1.8;
const float KI_R = 5.0;
const float KD_R = 0.1;

// Robot Physical Parameters
const float WHEEL_RADIUS = 0.065;  // meters
const float WHEEL_BASE = 0.26;     // meters
const float PULSES_PER_REV = 700;
const float POSITION_TOLERANCE = 0.05;  // meters
const float ANGLE_TOLERANCE = 0.05;     // radians

// Encoder calculation constants
const double metersToPulses = PULSES_PER_REV / (2 * PI * WHEEL_RADIUS);  // Pulses per meter
const long encoderInterval = 10;        // Update interval in ms

// Target values (will be set via Bluetooth)
float target_x = 0.0;
float target_y = 0.0;
float target_speed = 0.0;

// === LEDC Channels and Configuration ===
const int freq = 30000;
const int resolution = 8;

// PID variables for each motor
struct PIDController {
    float integral = 0;
    float prev_error = 0;
    float prev_time = 0;
    
    float compute(float setpoint, float measurement, float kp, float ki, float kd) {
        float current_time = millis() / 1000.0;
        float dt = current_time - prev_time;
        if (dt == 0) return 0;
        
        float error = setpoint - measurement;
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        prev_error = error;
        prev_time = current_time;
        return output;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
};

PIDController left_pid, right_pid;

// Encoder variables
volatile long left_count = 0;
volatile long right_count = 0;
volatile long prev_left_count = 0;
volatile long prev_right_count = 0;


// Encoder variables for RPM calculation
volatile long leftCount = 0, rightCount = 0;
unsigned long rpmLastTime = 0;
float leftRPM = 0, rightRPM = 0;

// Position tracking variables
float x_pos = 0, y_pos = 0, theta = 0;
volatile long left_pos = 0, right_pos = 0;

void IRAM_ATTR leftEncoderISR() {
    bool A = digitalRead(leftEncoderA);
    bool B = digitalRead(leftEncoderB);
    leftCount += (A == B) ? 1 : -1;
    left_pos += (A == B) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
    bool A = digitalRead(rightEncoderA);
    bool B = digitalRead(rightEncoderB);
    rightCount += (A == B) ? -1 : 1;  // Inverted logic for right motor
    right_pos += (A == B) ? -1 : 1;
}

void setup() {
  // sets the pins as outputs:
  // Right Motor
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  // Left Motor
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // Configure PWM
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);

  // Set up encoder pins
  pinMode(leftEncoderA, INPUT_PULLUP);
  pinMode(leftEncoderB, INPUT_PULLUP);
  pinMode(rightEncoderA, INPUT_PULLUP);
  pinMode(rightEncoderB, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), rightEncoderISR, RISING);
  
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot_Position");
}

void updateRPM() {
    if (millis() - rpmLastTime >= 1000) {  // Update every second
        leftRPM = (leftCount / PULSES_PER_REV) * 60.0;
        rightRPM = (rightCount / PULSES_PER_REV) * 60.0;
        
        leftCount = 0;
        rightCount = 0;
        rpmLastTime = millis();
    }
}

void updateOdometry() {
    static unsigned long last_time = 0;
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    if (dt == 0) return;
    
    // Calculate wheel velocities using total position counts
    float left_delta = ((left_pos - prev_left_count) / PULSES_PER_REV) * (2 * PI * WHEEL_RADIUS);
    float right_delta = ((right_pos - prev_right_count) / PULSES_PER_REV) * (2 * PI * WHEEL_RADIUS);
    
    float distance = (left_delta + right_delta) / 2.0;
    float d_theta = (right_delta - left_delta) / WHEEL_BASE;
    
    x_pos += distance * cos(theta + d_theta/2);
    y_pos += distance * sin(theta + d_theta/2);
    theta = fmod(theta + d_theta + PI, 2*PI) - PI;

    prev_left_count = left_pos;
    prev_right_count = right_pos;
    last_time = current_time;
}

void processCommand() {
    if (SerialBT.available()) {
        String cmd = SerialBT.readStringUntil('\n');
        int first_comma = cmd.indexOf(',');
        int second_comma = cmd.indexOf(',', first_comma + 1);
        
        if (first_comma > 0 && second_comma > first_comma) {
            target_x = cmd.substring(0, first_comma).toFloat();
            target_y = cmd.substring(first_comma + 1, second_comma).toFloat();
            target_speed = cmd.substring(second_comma + 1).toFloat();
            
            // Reset PID controllers when new target received
            left_pid.reset();
            right_pid.reset();
            
            SerialBT.printf("New target: %.2f, %.2f at speed %.2f\n", 
                           target_x, target_y, target_speed);
        }
    }
}

void moveToPosition() {
    float dx = target_x - x_pos;
    float dy = target_y - y_pos;
    float distance = sqrt(dx*dx + dy*dy);
    
    if (distance < POSITION_TOLERANCE) {
        stopMotors();
        SerialBT.println("Target reached!");
        return;
    }
    
    // Calculate target angle and error
    float target_angle = atan2(dy, dx);
    float angle_error = fmod(target_angle - theta + PI, 2*PI) - PI;
    
    // Calculate desired wheel velocities
    float v = target_speed * (distance > 0.1 ? 1.0 : distance/0.1);  // Ramp down speed near target
    float omega = 2.0 * angle_error;  // Proportional angle correction
    
    float v_left = v - omega * WHEEL_BASE / 2;
    float v_right = v + omega * WHEEL_BASE / 2;
    
    // Get current wheel velocities
    float left_velocity = getWheelVelocity(left_count, prev_left_count);
    float right_velocity = getWheelVelocity(right_count, prev_right_count);
    
    // Compute PID outputs
    float left_output = left_pid.compute(v_left, left_velocity, KP_L, KI_L, KD_L);
    float right_output = right_pid.compute(v_right, right_velocity, KP_R, KI_R, KD_R);
    
    // Apply motor commands
    setMotorPWM(left_output, right_output);
}

float getWheelVelocity(long current_count, long prev_count) {
    static unsigned long last_time = 0;
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    if (dt == 0) return 0;
    
    float delta_distance = (current_count - prev_count) / PULSES_PER_REV * 2 * PI * WHEEL_RADIUS;
    last_time = current_time;
    return delta_distance / dt;
}

void stopMotors() {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(enable1Pin, 0);
    analogWrite(enable2Pin, 0);
}

void setMotorPWM(float left_pwm, float right_pwm) {
    // Constrain and apply PWM values
    left_pwm = constrain(left_pwm, -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);
    
    // Set motor directions
    digitalWrite(leftMotorPin1, left_pwm >= 0 ? LOW : HIGH);
    digitalWrite(leftMotorPin2, left_pwm >= 0 ? HIGH : LOW);
    digitalWrite(rightMotorPin1, right_pwm >= 0 ? LOW : HIGH);
    digitalWrite(rightMotorPin2, right_pwm >= 0 ? HIGH : LOW);
    
    // Set PWM values
    analogWrite(enable1Pin, abs(left_pwm));
    analogWrite(enable2Pin, abs(right_pwm));
}

void loop() {
    processCommand();
    updateRPM();
    updateOdometry();
    moveToPosition();
    
    static unsigned long last_status = 0;
    if (millis() - last_status > 100) {
        SerialBT.printf("RPM L:%.1f RPM   R:%.1f RPM\n", leftRPM, rightRPM);
        SerialBT.printf("POS: %.2f m, %.2f m, %.2f degrees\n", x_pos, y_pos, theta * 180.0/PI);
        last_status = millis();
    }
    
    delay(10);
}