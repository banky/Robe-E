#define TRIG_PIN A0
#define ECHO_PIN A1

const float SOS = 0.034; // Speed of sound
long duration = 0;
float distance = 0;
int threshold = 30; // Distance threshold

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
}

// Turn left
void turnLeft() {
  
}

// Turn right
void turnRight() {
  
}

// Look around
void scan() {
  
}

void loop() {
  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance= duration*SOS/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}
