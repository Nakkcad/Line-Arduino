#include <Servo.h>

// Motor pins (L298N)
#define ENA 10
#define ENB 11
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9

// Sensor line follower
#define LEFT_IR 2
#define MID_IR 3
#define RIGHT_IR 12

// Ultrasonic pins
#define TRIG 5
#define ECHO 4

// Servo pin
#define SERVO_PIN 13

// Motor speeds
#define BASE_SPEED 10
#define TURN_SPEED 10
#define CORRECTION_FACTOR 30

// Variables
int leftSensor, rightSensor;
long duration;
int distance;
int lastError = 0;
int leftDistance, rightDistance, centerDistance;

const int OBSTACLE_TURN_SPEED = 200; // Faster for obstacle avoidance
const int OBSTACLE_TURN_DELAY = 500; // Longer for obstacle avoidance

// Obstacle detection
const int OBSTACLE_DISTANCE = 15; // cm

Servo myServo;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);
    pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  

  myServo.attach(SERVO_PIN);
  myServo.write(90); // Center position
  delay(500);

  // Start with motors stopped
  stopMotors();
  
  Serial.begin(9600);
}

void loop() {
  // Read sensors with minimal delay
  leftSensor = digitalRead(LEFT_IR);
  rightSensor = digitalRead(RIGHT_IR);
  
  centerDistance = readUltrasonic();
  
  // Simple sensor display (only left and right shown)
  Serial.print(leftSensor);
  Serial.print("|-|"); // Middle sensor shown as disabled
  Serial.print(rightSensor);
Serial.print(" | US: ");
    Serial.println(centerDistance);  
  // Check for obstacle
  if (centerDistance > 0 && centerDistance < OBSTACLE_DISTANCE) {
    Serial.print("OBSTACLE! ");
    avoidObstacle();
    return;
  }

  // Line following logic with immediate response
  followLine();
  
  // Small delay for stability
  delay(5);
}

void followLine() {
  int error = 0;
  
  // Sensor states: 0 = on line (black), 1 = off line (white)
  // Calculate error based on sensor readings
  if (leftSensor == 0 && rightSensor == 1) {
    // Left sensor on line, turn right
    error = -1;
  } 
  else if (leftSensor == 1 && rightSensor == 0) {
    // Right sensor on line, turn left
    error = 1;
  }
  else if (leftSensor == 0 && rightSensor == 0) {
    // Both sensors on line, straight
    error = 0;
  }
  else if (leftSensor == 1 && rightSensor == 1) {
    // Both sensors off line, use last error for recovery
    error = lastError;
  }
  
  // Apply motor control based on error
  switch(error) {
    case -1: // Turn right
      moveRight();
      break;
    case 1: // Turn left
      moveLeft();
      break;
    case 0: // Move forward
      moveForward();
      break;
  }
  
  lastError = error;
}

void avoidObstacle() {
  Serial.print("Scanning for clear path... ");
  
  // Stop motors
  stopMotors();
  delay(500);
  
  // Look right
  myServo.write(0);
  delay(800);
  rightDistance = readUltrasonic();
  
  // Look left
  myServo.write(180);
  delay(800);
  leftDistance = readUltrasonic();
  
  // Return to center
  myServo.write(90);
  delay(500);
  
  Serial.print("L:");
  Serial.print(leftDistance);
  Serial.print("cm R:");
  Serial.print(rightDistance);
  Serial.print("cm -> ");
  
  // Decide which way to turn (obstacle avoidance turn)
  if (leftDistance > rightDistance && leftDistance > OBSTACLE_DISTANCE) {
    Serial.println("OBSTACLE TURN LEFT");
    obstacleTurnLeft();
  }
  else{
    Serial.println("OBSTACLE TURN RIGHT");
    obstacleTurnRight();
  }
}


int readUltrasonic() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2;
  
  return distance;
}

void moveForward() {
  // Both motors forward at base speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
}

void moveLeft() {
  // Right motor forward, left motor slower or reverse for sharp turn
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Left motor reverse for sharper turn
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void moveRight() {
  // Left motor forward, right motor slower or reverse for sharp turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Right motor reverse for sharper turn
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// =============================================
// OBSTACLE AVOIDANCE TURNS (Strong, longer duration)
// =============================================
void obstacleTurnLeft() {
  // Turn LEFT: Right motor forward, Left motor backward
  digitalWrite(IN1, LOW);   // Left motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Right motor forward
  digitalWrite(IN4, LOW);
  analogWrite(ENA, OBSTACLE_TURN_SPEED);
  analogWrite(ENB, OBSTACLE_TURN_SPEED);
  delay(OBSTACLE_TURN_DELAY);
  stopMotors();
  delay(100);
}

void obstacleTurnRight() {
  // Turn RIGHT: Left motor forward, Right motor backward
  digitalWrite(IN1, HIGH);  // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Right motor backward
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, OBSTACLE_TURN_SPEED);
  analogWrite(ENB, OBSTACLE_TURN_SPEED);
  delay(OBSTACLE_TURN_DELAY);
  stopMotors();
  delay(100);
}


// Optional: For more precise control with proportional correction
void followLineProportional() {
  leftSensor = digitalRead(LEFT_IR);
  rightSensor = digitalRead(RIGHT_IR);
  
  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;
  
  // Proportional correction
  if (leftSensor == 0 && rightSensor == 1) {
    // Need to turn right - slow left motor
    leftSpeed = BASE_SPEED - CORRECTION_FACTOR;
    rightSpeed = BASE_SPEED + CORRECTION_FACTOR;
  } 
  else if (leftSensor == 1 && rightSensor == 0) {
    // Need to turn left - slow right motor
    leftSpeed = BASE_SPEED + CORRECTION_FACTOR;
    rightSpeed = BASE_SPEED - CORRECTION_FACTOR;
  }
  
  // Apply motor speeds
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, constrain(leftSpeed, 0, 255));
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}