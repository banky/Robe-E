#include <Servo.h>
#include <AFMotor.h>  // Motor shield library

#define M_R 0.4       //Ratio of motor speeds copared to motor 1 (compensating for

int trigPin = A0;
int echoPin = A1;
int servoPin = 9;
int maxSpeed = 200;
int minDistance = 20;
int distanceThreshold = 5;
int angleIncrement = 5;


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

/* Set the speed of a motor. mSpeed value is between -200 and 200 */
void setMotor(AF_DCMotor motor, float mSpeed) {
  if(mSpeed > 0) {
    motor.run(FORWARD);     //Forward 
  } else if (mSpeed < 0) {
    motor.run(BACKWARD);    //Backwards
  } else {
    motor.run(RELEASE);     //Stop
  }
  
  motor.setSpeed(abs(mSpeed));
}

/* Turn the servo to a given angle */
void setServo(Servo servo, float angle) {
  servo.write(angle);
  delay(15);
}

/* Get the distance read by the ultrasonic sensor */
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

/* Turns the robot to face the direction in which it should continue moving forward
 * Turns left or right based on the angle
 * Determiines position based on where the maximum distance was found */
void rotateToPosition(int angle, float distance) {

  //When no distance is found above the minimum threshold turn left until a distance above the threshold is found
  if (distance < minDistance){ 
    int curDist = 0;   
    do{
      turnInPlace(true, maxSpeed);
      curDist = getDistance();  
    } while (curDist <= minDistance); 
      
    //// turn a little extra to make sure robot clears the obstacle? ///
  }
  
  //When a maximum distance is found above the distance threshold turn until that distance is found again 
  else{     
    if (angle < 90){
      //Turn right
      turnUntil(false, distance);
    }    
    else if (angle > 90){
      //Turn left
      turnUntil(true, distance);
    } 

    /* If angle == 90 && distance > minDistance
     * Robot should just continue going straight
     * This should never happen since robot would have been going straight when an obstacle was detected at the min distance */    
  }

  /// Additional edge cases ? ///  
}

/* Turn the robot until a distance within the threshold of the target distance is found */
void turnUntil (bool isLeft, int targetDistance){
  int curDist = 0;
  do{
    turnInPlace(isLeft, maxSpeed);
    curDist = getDistance(); 
  }while(curDist <= (targetDistance - distanceThreshold) || curDist >= (targetDistance + distanceThreshold));

  //// turn a little extra to make sure robot clears the obstacle? ///
  
}

/* Move robot forward */
void goForward(int speed) {
  setMotor(frontLeft, speed);
  setMotor(backLeft, speed);
  setMotor(backRight, 0);
  setMotor(frontRight, 0);
}

/* Stop movememnt of robot */
void stopMoving() {
  setMotor(frontLeft, 0);
  setMotor(backLeft, 0);
  setMotor(backRight, 0);
  setMotor(frontRight, 0);
}

/* Turn robot to the left while also moving forward */
void turnLeft(int speed) {
  setMotor(frontLeft, speed);
  setMotor(backLeft, speed);
  setMotor(backRight, 0);
  setMotor(frontRight, 0);
}

/* Turn robot to the right while also moving forward */
void turnRight(int speed) {
  setMotor(frontLeft, 0);
  setMotor(backLeft, 0);
  setMotor(backRight, speed);
  setMotor(frontRight, speed);
}

/* Turn robot to the left or right without moving forward */
void turnInPlace(bool isLeft, int speed) {
  //Turn left
  if (isLeft){
    setMotor(frontLeft, speed);
    setMotor(backLeft, speed);
    setMotor(backRight, -speed);
    setMotor(frontRight, -speed);
  }
  //Turn right
  else{
    setMotor(frontLeft, -speed);
    setMotor(backLeft, -speed);
    setMotor(backRight, speed);
    setMotor(frontRight, speed);
  } 
}

/* Find the angle with the furthest correlating distance infront of the robot */
void findMaxDistance(int &angleMaxDistance, float &maxDistance){
  maxDistance = 0;
  float currentDistance;
  
  for (unsigned int currentAngle = 0; currentAngle < 180; currentAngle += angleIncrement) {
    setServo(neck, currentAngle);
    currentDistance = getDistance();
    
    if (currentDistance > maxDistance) {
      maxDistance = currentDistance;
      angleMaxDistance = currentAngle;
    }

    //Output current distance and angle found
    Serial.print("Current Distance: ");
    Serial.println(currentDistance);
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
    
  }

  //Return sensor to forward direction
  setServo(neck, 90);

  //Output max distance and angle found
  Serial.print("Max Distance: ");
  Serial.println(maxDistance);
  Serial.print("Max Angle: ");
  Serial.println(angleMaxDistance);
  
  delay(1000);  
}

void loop() {
  float distance = getDistance();
  float maxDistance = 0;
  int angleMaxDistance = 0;
  
  //Set the ultrasonic sensor to face straight ahead
  setServo(neck, 90);

  //Go forward until an obstacle near by is detected
  while (distance > minDistance) {
    goForward(maxSpeed);
    distance = getDistance();
    Serial.println("No obstacles detected");
  }

  //Stop and output the distance to the obstacle found
  stopMoving();
  Serial.print("Obstacle found: ");
  Serial.println(getDistance());

  //Determine the the furthest distance in front of robot and at what angle it was found
  findMaxDistance(angleMaxDistance, maxDistance);
  //Rotate the robot to the position where the max distance was found
  rotateToPosition(angleMaxDistance, maxDistance);
}
