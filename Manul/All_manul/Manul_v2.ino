#include <PS4Controller.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

int fo;
int back;
int left;
int right;
int turn_left;
int turn_right;

unsigned long time_a;
unsigned long time_b;

// DC Motor_Forward_Left
#define Motor4_IN7 33
#define Motor4_IN8 32

// DC Motor_Backward_Left
#define Motor1_IN1 26
#define Motor1_IN2 25

// DC Motor_Forward_Right
#define Motor2_IN3 12
#define Motor2_IN4 13

// DC Motor_Backward_Right
#define Motor3_IN5 27
#define Motor3_IN6 14

//Relay Pin
#define Relay_FeedBall 19
#define Relay_X 18
#define Relay_Y 5

//Servo
#define servoRT 0
#define servoLT 1
#define servoLB 2
#define servoRB 3
#define servo5 4
#define servo6 6
#define servo7 7

//PCA
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//StepperX
boolean buttonStateX;
boolean lastStateX;
boolean stateX = 0;
//StepperY
boolean buttonStateY;
boolean lastStateY;
boolean stateY = 0;
//Servo1,2
boolean buttonState_S12;
boolean lastState_S12;
boolean state_S12 = 0;
//Servo3,4
boolean buttonState_S34;
boolean lastState_S34;
boolean state_S34 = 0;

boolean buttonStateBall_F;
boolean lastStateBall_F;
boolean stateBall_F = 0;

boolean buttonStateBall_CN;
boolean lastStateBall_CN;
boolean stateBall_CN = 0;

boolean buttonStateBall_Arm;
boolean lastStateBall_Arm;
boolean stateBall_Arm = 0;

bool Run = false;

void setup() {
  Serial.begin(115200);
  PS4.begin();
  // Relay
  pinMode(Relay_FeedBall, OUTPUT);
  pinMode(Relay_X, OUTPUT);
  pinMode(Relay_Y, OUTPUT);

  //Motor Drive
  pinMode(Motor1_IN1, OUTPUT);
  pinMode(Motor1_IN2, OUTPUT);
  pinMode(Motor2_IN3, OUTPUT);
  pinMode(Motor2_IN4, OUTPUT);
  pinMode(Motor3_IN5, OUTPUT);
  pinMode(Motor3_IN6, OUTPUT);
  pinMode(Motor4_IN7, OUTPUT);
  pinMode(Motor4_IN8, OUTPUT);

  analogWrite(Motor1_IN1, 0);
  analogWrite(Motor1_IN2, 0);
  analogWrite(Motor2_IN3, 0);
  analogWrite(Motor2_IN4, 0);
  analogWrite(Motor3_IN5, 0);
  analogWrite(Motor3_IN6, 0);
  analogWrite(Motor4_IN7, 0);
  analogWrite(Motor4_IN8, 0);

  time_a = millis();
  time_b = millis();
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(20);
  // HOME();
}

bool St_ArmBall = true;
bool St_ArmTop = true;
bool St_ArmBottom = true;

void loop() {
  Connect();
  if (Run == true) {
    motion();
    //Relay_X
    int buttonX = PS4.Square();
    buttonStateX = buttonX;
    if ((buttonStateX == 0) && (lastStateX == 1))
      stateX = !stateX;
    lastStateX = buttonStateX;

    //Relay_Y
    int buttonY = PS4.Triangle();
    buttonStateY = buttonY;
    if ((buttonStateY == 0) && (lastStateY == 1))
      stateY = !stateY;
    lastStateY = buttonStateY;

    //Servo Bottom RL
    int button_S12 = PS4.Cross();
    buttonState_S12 = button_S12;
    if ((buttonState_S12 == 0) && (lastState_S12 == 1))
      state_S12 = !state_S12;
    lastState_S12 = buttonState_S12;

    //Servo Top RL
    int button_S34 = PS4.Circle();
    buttonState_S34 = button_S34;
    if ((buttonState_S34 == 0) && (lastState_S34 == 1))
      state_S34 = !state_S34;
    lastState_S34 = buttonState_S34;

    //Ball Cannon
    int buttonBall_CN = PS4.Up();
    buttonStateBall_CN = buttonBall_CN;
    if ((buttonStateBall_CN == 0) && (lastStateBall_CN == 1))
      stateBall_CN = !stateBall_CN;
    lastStateBall_CN = buttonStateBall_CN;

    //Ball Feed
    int buttonBall_F = PS4.R1();
    buttonStateBall_F = buttonBall_F;
    if ((buttonStateBall_F == 0) && (lastStateBall_F == 1))
      stateBall_F = !stateBall_F;
    lastStateBall_F = buttonStateBall_F;

    //Armball
    int buttonBall_Arm = PS4.L1();
    buttonStateBall_Arm = buttonBall_Arm;
    if ((buttonStateBall_Arm == 0) && (lastStateBall_Arm == 1))
      stateBall_Arm = !stateBall_Arm;
    lastStateBall_Arm = buttonStateBall_Arm;

    //Relay Feed Ball
    if (PS4.R1() == 1) {
      digitalWrite(Relay_FeedBall, HIGH);
    } else if (PS4.R1() == 0) {
      digitalWrite(Relay_FeedBall, LOW);
    }

    //Relay Y
    if (PS4.Triangle() == 1) {
      if (stateY == 0) {
        digitalWrite(Relay_Y, HIGH);
      } else if (stateY == 1) {
        digitalWrite(Relay_Y, LOW);
      }
    }

    //Relay X
    if (PS4.Square() == 1) {
      if (stateX == 0) {
        digitalWrite(Relay_X, HIGH);
      } else if (stateX == 1) {
        digitalWrite(Relay_X, LOW);
      }
    }

    //Servo Bottom LR
    if (PS4.Cross() == 0) {
      St_ArmBottom = true;
    } else if (PS4.Cross() == 1 && state_S12 == 0 && St_ArmBottom == true) {
      Servo_BLR();
    } else if (PS4.Cross() == 1 && state_S12 == 1 && St_ArmBottom == true) {
      Servo_BLR();
    }

    //Servo Top LR
    if (PS4.Circle() == 0) {
      St_ArmTop = true;
    } else if (PS4.Circle() == 1 && state_S34 == 0 && St_ArmTop == true) {
      Servo_TLR();
    } else if (PS4.Circle() == 1 && state_S34 == 1 && St_ArmTop == true) {
      Servo_TLR();
    }

    //Servo_Armball
    if (PS4.L1() == 0) {
      St_ArmBall = true;
    } else if (PS4.L1() == 1 && stateBall_Arm == 0 && St_ArmBall == true) {
      ServoArm_ball();
    } else if (PS4.L1() == 1 && stateBall_Arm == 1 && St_ArmBall == true) {
      ServoArm_ball();
    }
    //Ball Cannon
    if (PS4.Up() == 1 && stateBall_CN == 0) {
      Ball_Cannon();
    } else if (PS4.Up() == 1 && stateBall_CN == 1) {
      Ball_Cannon();
    }

    //Home
    if (PS4.PSButton() == 1) {
      HOME();
    }
  } else if (Run == false) {
    HOME();
    motion();
  }
}

//Servo Bottom
void Servo_BLR() {
  if (state_S12 == 0) {
    for (int pulse = 280; pulse < 530; pulse += 1) {  //530
      pwm.setPWM(servoLB, 0, pulse);
    }
    for (int pulse = 510; pulse > 260; pulse -= 1) {  //260
      pwm.setPWM(servoRB, 0, pulse);
    }
  }
  if (state_S12 == 1) {
    for (int pulse = 530; pulse > 280; pulse -= 1) {  //530
      pwm.setPWM(servoLB, 0, pulse);
    }
    for (int pulse = 260; pulse < 510; pulse += 1) {  //260
      pwm.setPWM(servoRB, 0, pulse);
    }
  }
  St_ArmBottom = false;
}

//Servo Top
void Servo_TLR() {
  if (state_S34 == 0) {
    for (int pulse = 340; pulse < 465; pulse += 1) {  //465
      pwm.setPWM(servoRT, 0, pulse);
    }
    for (int pulse = 540; pulse > 410; pulse -= 1) {  //410
      pwm.setPWM(servoLT, 0, pulse);
    }
  }
  if (state_S34 == 1) {
    for (int pulse = 465; pulse > 340; pulse -= 1) {  //465
      pwm.setPWM(servoRT, 0, pulse);
    }
    for (int pulse = 410; pulse < 540; pulse += 1) {  //410
      pwm.setPWM(servoLT, 0, pulse);
    }
  }
  St_ArmTop = false;
}

//Servo ArmBall
void ServoArm_ball() {
  if (PS4.L1() == 1 && stateBall_Arm == 0) {
    for (int pulse = 590; pulse > 300; pulse -= 1) {  //300
      pwm.setPWM(servo5, 0, pulse);
    }
  }
  if (PS4.L1() == 1 && stateBall_Arm == 1) {
    for (int pulse = 300; pulse < 590; pulse += 1) {  //300
      pwm.setPWM(servo5, 0, pulse);
      motion();
      delay(2);
    }
  }
  St_ArmBall = false;
}

//Ball Cananon
void Ball_Cannon() {
  if (stateBall_CN == 1) {
    for (int pulse = 272; pulse > 195; pulse -= 1) { //270
      pwm.setPWM(servo6, 0, pulse); 
      pwm.setPWM(servo7, 0, pulse);
    }
  }
  if (stateBall_CN == 0) {
    for (int pulse = 195; pulse < 272; pulse += 1) { //270
      pwm.setPWM(servo6, 0, pulse);
      pwm.setPWM(servo7, 0, pulse);
    }
    // pwm.setPWM(servo6, 0, 250);
    // pwm.setPWM(servo7, 0, 250);
    // if (millis() - time_b >= 100) {
    //   for (int pulse = 275; pulse > 250; pulse -= 1) {
    //     pwm.setPWM(servo6, 0, pulse);
    //     pwm.setPWM(servo7, 0, pulse);
    //   }
    //   time_b = millis();
    // }
  }
}


void Connect() {
  if (PS4.isConnected()) {
    Run = true;
  } else {
    Run = false;
  }
}

void HOME() {
}

void motion() {
  fo = map(PS4.LStickY(), 45, 127, 0, 255);
  back = map(PS4.LStickY(), -127, -45, 255, 0);
  right = map(PS4.LStickX(), 30, 127, 0, 255);
  left = map(PS4.LStickX(), -127, -30, 255, 0);
  turn_right = map(PS4.RStickX(), 20, 127, 0, 120);
  turn_left = map(PS4.RStickX(), -127, -20, 120, 0);


  if (PS4.LStickY() >= 40) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, fo);
      analogWrite(Motor1_IN2, 0);
      analogWrite(Motor2_IN3, fo);
      analogWrite(Motor2_IN4, 0);
      analogWrite(Motor3_IN5, fo);
      analogWrite(Motor3_IN6, 0);
      analogWrite(Motor4_IN7, fo);
      analogWrite(Motor4_IN8, 0);
      time_a = millis();
    }
  } else if (PS4.LStickY() <= -40) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, 0);
      analogWrite(Motor1_IN2, back);
      analogWrite(Motor2_IN3, 0);
      analogWrite(Motor2_IN4, back);
      analogWrite(Motor3_IN5, 0);
      analogWrite(Motor3_IN6, back);
      analogWrite(Motor4_IN7, 0);
      analogWrite(Motor4_IN8, back);
      time_a = millis();
    }
  } else if (PS4.LStickX() >= 30) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, 0);
      analogWrite(Motor1_IN2, right);
      analogWrite(Motor2_IN3, right);
      analogWrite(Motor2_IN4, 0);
      analogWrite(Motor3_IN5, 0);
      analogWrite(Motor3_IN6, right);
      analogWrite(Motor4_IN7, right);
      analogWrite(Motor4_IN8, 0);
      time_a = millis();
    }
  } else if (PS4.LStickX() <= -30) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, left);
      analogWrite(Motor1_IN2, 0);
      analogWrite(Motor2_IN3, 0);
      analogWrite(Motor2_IN4, left);
      analogWrite(Motor3_IN5, left);
      analogWrite(Motor3_IN6, 0);
      analogWrite(Motor4_IN7, 0);
      analogWrite(Motor4_IN8, left);
      time_a = millis();
    }
  } else if (PS4.RStickX() <= -20) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, 0);
      analogWrite(Motor1_IN2, turn_left);
      analogWrite(Motor2_IN3, turn_left);
      analogWrite(Motor2_IN4, 0);
      analogWrite(Motor3_IN5, turn_left);
      analogWrite(Motor3_IN6, 0);
      analogWrite(Motor4_IN7, 0);
      analogWrite(Motor4_IN8, turn_left);
      time_a = millis();
    }
  } else if (PS4.RStickX() >= 20) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, 0);
      analogWrite(Motor1_IN1, turn_right);
      analogWrite(Motor1_IN2, 0);
      analogWrite(Motor2_IN3, 0);
      analogWrite(Motor2_IN4, turn_right);
      analogWrite(Motor3_IN5, 0);
      analogWrite(Motor3_IN6, turn_right);
      analogWrite(Motor4_IN7, turn_right);
      analogWrite(Motor4_IN8, 0);
      time_a = millis();
    }
  } else if (Run == false) {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, 0);
      analogWrite(Motor1_IN2, 0);
      analogWrite(Motor2_IN3, 0);
      analogWrite(Motor2_IN4, 0);
      analogWrite(Motor3_IN5, 0);
      analogWrite(Motor3_IN6, 0);
      analogWrite(Motor4_IN7, 0);
      analogWrite(Motor4_IN8, 0);
      time_a = millis();
    }
  } else {
    if (millis() - time_a >= 10) {
      analogWrite(Motor1_IN1, 0);
      analogWrite(Motor1_IN2, 0);
      analogWrite(Motor2_IN3, 0);
      analogWrite(Motor2_IN4, 0);
      analogWrite(Motor3_IN5, 0);
      analogWrite(Motor3_IN6, 0);
      analogWrite(Motor4_IN7, 0);
      analogWrite(Motor4_IN8, 0);
      time_a = millis();
    }
  }
}