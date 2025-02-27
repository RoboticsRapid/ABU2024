#define pulPin 2
#define dirPin 5
#define endPin 8
int x;
void setup() {
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(endPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {

  for (x = 0; x <= 200; x++) {
    digitalWrite(dirPin, HIGH);
    Serial.println(x);
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(endPin, LOW);
  delay(1000);
}
