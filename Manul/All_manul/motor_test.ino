#define DIRA 2
#define DIRB 3
#define PWMA 4
#define PWMB 5
float RPM = 900;
void setup() {
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  Serial.begin(9600);
}

void loop() {
int speed = (RPM/1620)*255;
  // motorA
  analogWrite(PWMA, speed);
  analogWrite(DIRA, LOW);
  // motorB
  analogWrite(PWMA, speed);
  analogWrite(DIRA, HIGH);
  Serial.println(speed);
}
