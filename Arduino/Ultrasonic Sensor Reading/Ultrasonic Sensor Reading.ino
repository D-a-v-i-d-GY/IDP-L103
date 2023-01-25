//pin definition
#define trigPin 13
#define echoPin 12

void setup() {
Serial.begin (9600);
pinMode(trigPin, OUTPUT); 
pinMode(echoPin, INPUT);
}

void loop() {
unsigned long duration, distance;
// Activate the sensor
digitalWrite(trigPin, LOW);
delayMicroseconds(2); // Just a delay
digitalWrite(trigPin, HIGH);
delayMicroseconds(10); // Triggers the sensor
digitalWrite(trigPin, LOW);

// Start measuring
duration = pulseIn(echoPin, HIGH); // Measure the duration of the pulse
distance = (duration/2) / 29.1;
if (distance >= 390){distance=-999;}
else{Serial.print(distance);}
delay(250);
}