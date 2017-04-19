#include <AFMotor.h>  // Motor shield lib

int trigPin = A0;
int echoPin = A1;
const float SOS = 0.034; // Speed of sound

AF_DCMotor motor1(1, MOTOR12_64KHZ); // 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_64KHZ); // 64KHz pwm
AF_DCMotor motor4(4, MOTOR34_64KHZ); // 64KHz pwm

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

/*
 * Set the speed of a motor. mSpeed value is between -255 and 255
 */
void setMotor(AF_DCMotor motor, int mSpeed) {
  if(mSpeed > 0) {
    motor.run(FORWARD); // Lets go forward! 
  } else if (mSpeed < 0) {
    motor.run(BACKWARD); // Lets go backward!
  } else {
    motor.run(RELEASE); // Stop motor
  }

  motor.setSpeed(abs(mSpeed));
}

/*
 * Get the distance read by the ultrasonic sensor
 */
float getDistance() {
  long duration = 0;
  float distance = 0;
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*SOS/2;
  
  return distance;
}

void loop() {
  float distance = getDistance();
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  
  if (distance > 20) {
    setMotor(motor1, 255);
    setMotor(motor2, 255);
    setMotor(motor3, 255);
    setMotor(motor4, 255);
  } else {
    setMotor(motor1, -255);
    setMotor(motor2, -255);
    setMotor(motor3, -255);
    setMotor(motor4, -255);
  }
}
