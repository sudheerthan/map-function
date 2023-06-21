#include <stdio.h>
#include <stdint.h>

#define max_step_value 25000
#define max_degree_value 180

float map_func(float value, float s_min, float s_max, float d_min, float d_max)
{
    float s_out = s_max - s_min;
    float d_out = d_max - d_min;
    float scale = (value - s_min) / s_out;
    float t_out = (d_min + scale * d_out );
    return t_out;
    
}

int main()
{
    float value, s_min, s_max, d_min, d_max;
    printf("enter value, from min value ,from max value , to min value, to max value");
    scanf("%f %f %f %f %f",&value , &s_min, &s_max, &d_min, &d_max );
    printf("value = %f , from min = %f , from max = %f , to min = %f , to max = %f \n", value, s_min, s_max, d_min, d_max);
    float out = map_func(value, s_min, s_max, d_min, d_max);
    printf("the output value is %f \n",out);

    return 0;
}
#include <Servo.h>
#include <NewPing.h>

// Pin assignments
const int servoPin = 9;        // Pin for the servo motor
const int trigPin = 7;         // Pin for the ultrasonic sensor's trigger
const int echoPin = 6;         // Pin for the ultrasonic sensor's echo
const int motorEnableA = 10;   // Enable pin for the L298N motor driver (motor A)
const int motorInput1 = 11;    // Input 1 pin for motor A
const int motorInput2 = 12;    // Input 2 pin for motor A

// Ultrasonic sensor parameters
const int maxDistance = 200;   // Maximum distance to measure (in centimeters)
const int threshold = 20;      // Distance threshold for object detection (in centimeters)

// Servo motor parameters
const int servoScanDelay = 15; // Delay between servo angle increments (in milliseconds)
const int servoStartPos = 90;  // Starting position of the servo motor (in degrees)

Servo servo;                   // Servo object for controlling the SG90 servo motor
NewPing sonar(trigPin, echoPin, maxDistance);  // NewPing object for ultrasonic sensor

void setup() {
  servo.attach(servoPin);        // Attaching the servo to the servo pin
  servo.write(servoStartPos);    // Setting the initial position of the servo motor

  pinMode(motorEnableA, OUTPUT);
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  int distance = sonar.ping_cm();  // Measure the distance to the nearest obstacle (in centimeters)
  
  if (distance > 0 && distance < threshold) {
    // Object detected, search for a possible path
    int maxDistance = 0;        // Holds the maximum distance obtained during scanning
    int maxAngle = 0;           // Holds the angle with the maximum distance

    for (int angle = 0; angle <= 180; angle += 15) {
      servo.write(angle);      // Rotate the servo motor to the specified angle
      delay(servoScanDelay);   // Wait for the servo to reach the desired position
      int currentDistance = sonar.ping_cm();  // Measure the distance at the current angle

      if (currentDistance > maxDistance) {
        maxDistance = currentDistance;
        maxAngle = angle;
      }
    }

    servo.write(servoStartPos);  // Set the servo motor back to the starting position

    // Move the car towards the shortest distance
    if (maxAngle < 90) {
      // Turn left
      digitalWrite(motorInput1, HIGH);
      digitalWrite(motorInput2, LOW);
    } else {
      // Turn right
      digitalWrite(motorInput1, LOW);
      digitalWrite(motorInput2, HIGH);
    }
    
    // Adjust the motor speed here if necessary
    analogWrite(motorEnableA, 255);  // Set motor speed to maximum (255)

    delay(500);  // Adjust this delay according to your needs
    stopCar();   // Stop the car after a certain delay
  }
}

void stopCar() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  analogWrite(motorEnableA, 0);  // Set motor speed to zero
