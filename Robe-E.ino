#include <Servo.h>

#include <AFMotor.h>  // Motor shield lib

#define M_R 0.4 //Ratio of motor speeds to motor 1

int trigPin = A0;
int echoPin = A1;
int servoPin = 9;
int maxSpeed = 200;
int distanceThreshold = 20;


const float SOS = 0.034; // Speed of sound
Servo neck;

AF_DCMotor frontLeft(1, MOTOR12_64KHZ); // 64KHz pwm
AF_DCMotor backLeft(2, MOTOR12_64KHZ); // 64KHz pwm
AF_DCMotor backRight(3, MOTOR34_64KHZ); // 64KHz pwm
AF_DCMotor frontRight(4, MOTOR34_64KHZ); // 64KHz pwm

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);

  neck.attach(servoPin);
}

/*
 * Set the speed of a motor. mSpeed value is between -200 and 200
 */
void setMotor(AF_DCMotor motor, float mSpeed) {
  if(mSpeed > 0) {
    motor.run(FORWARD); // Lets go forward! 
  } else if (mSpeed < 0) {
    motor.run(BACKWARD); // Lets go backward!
  } else {
    motor.run(RELEASE); // Stop motor
  }

  motor.setSpeed(abs(mSpeed));
}

void setServo(Servo servo, float angle) {
  servo.write(angle);
  delay(15);
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
  distance = duration*SOS/2;
  
  return distance;
}

void rotateToAngle(int angle, float distance) {
  if (distance < distanceThreshold){
    //Rotate until you find distance bigger than the threshold
    int curDist = 0;
    do{
      turnInPlace(maxSpeed);
      curDist = getDistance();  
    } while (curDistance <= distanceThreshold);    
  }

  //check if angle < or > than 90
  //turn left or right according to angle compared to 90
  //while turning measure current distance
  //stop turning whin the distance is within a certain threshold of the target distance
  //kick out

  //additional edge cases  
}

void turnLeft(int speed) {
  setMotor(frontLeft, speed);
  setMotor(backLeft, speed);
  setMotor(backRight, 0);
  setMotor(frontRight, 0);
}

void turnRight(int speed) {
  setMotor(frontLeft, 0);
  setMotor(backLeft, 0);
  setMotor(backRight, speed);
  setMotor(frontRight, speed);
}

void turnInPlace(bool isLeft, int speed) {
  if (isLeft){
    setMotor(frontLeft, speed);
    setMotor(backLeft, speed);
    setMotor(backRight, -speed);
    setMotor(frontRight, -speed);
  }
  else{
    setMotor(frontLeft, -speed);
    setMotor(backLeft, -speed);
    setMotor(backRight, speed);
    setMotor(frontRight, speed);
  } 
}

void loop() {
  float distance = getDistance();
  const int threshold = 20;

  turnLeft(200);
  delay (5000);
  turnRight(200);
  delay(5000);
  turnInPlace(true, 200);
  delay(5000);
  turnInPlace(false, 200);
  delay(5000);
  
  /*
  // Prints the distance on the Serial Monitor
//  Serial.print("Distance: ");
//  Serial.println(distance);

  setServo(neck, 90);
  while (distance > threshold) {
//    setMotor(frontLeft, 200);
//    setMotor(backLeft, 200);
//    setMotor(backRight, 200);
//    setMotor(frontRight, 200);

    distance = getDistance();
    Serial.println("dancing on the moon");
  }

    // STOP!!!
    setMotor(frontLeft, 0);
    setMotor(backLeft, 0);
    setMotor(backRight, 0);
    setMotor(frontRight, 0);

    // LOOK AROUND!!
    setServo(neck, 0);
    float md = 0; // Maximum distance
    int angleAtMd = 0; // Angle at maximum distance
    int increment = 5; // Angle between 

    for (unsigned int cAngle = 0; cAngle < 180; cAngle += increment) {
      setServo(neck, cAngle);
      float cDist = getDistance();
      if (cDist > md) {
        md = cDist;
        angleAtMd = cAngle;
      }
      Serial.print("Current Distance: ");
      Serial.println(cDist);
      Serial.print("Current Angle: ");
      Serial.println(cAngle);
      
    }

    setServo(neck, 90);

    Serial.print("Max Distance: ");
    Serial.println(md);
    Serial.print("Max Angle: ");
    Serial.println(angleAtMd);
    delay(50000);
    exit(0);

    rotateToAngle(angleAtMd);
    */
}
