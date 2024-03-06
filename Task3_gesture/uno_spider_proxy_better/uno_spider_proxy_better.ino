// 0 Home
// 1 Forward
// 2 Backward
// 3 Left
// 4 Right
// 5 Turn Left
// 6 Turn Right

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SoftwareSerial UnoSerial(0, 1); // RX | TX

#define maximum_joint_angle 470
#define minimum_joint_angle 110

#define maximum_leg_angle 315
#define minimum_leg_angle 100

#define leg_maximum_offset 60
#define leg_minimum_offset 20

#define joint_maximum_offset 135
#define joint_minimum_offset 45

#define fixing_error 0
#define walking_offset 10

uint16_t joint_initial_angle, leg_initial_angle;

String input;
uint16_t angle;

void setup() {
  Serial.begin(9600);
  UnoSerial.begin(57600);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  delay(10);
}

static int int_data = 0;

void serialReader() {
  while (UnoSerial.available() > 0) {
    int_data = UnoSerial.parseInt();
    if (UnoSerial.read() == '\n') {
      //      Serial.print("Uno receive : ");
      //      Serial.println(int_data);

      break;
    }
  }
}

void clearSerial() {
  int_data = 0;
  static uint32_t premillis = 0;
  premillis = millis();
  while (true) {
    if (millis() - premillis > 100 ) {
      break;
    }
    else {
      serialReader();
    }
  }
}
void loop() {

  Serial.println(int_data);

  serialReader();

  if (int_data == 0) {
    initial_home();
    Serial.println("Home");
  }

  else if (int_data == 1) {
    Serial.println("Forward");
    leg3_backward();
    leg4_backward_parkinson();
    delay(100);
    slickback_1();
    slickfront_3();
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    delay(500);

    leg2_forward();
    leg1_forward_parkinson();
    delay(100);
    slickback_2();
    slickfront_4();
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    clearSerial();
  }

  else if (int_data == 2) {
    Serial.println("Backward");
    leg1_backward();
    leg2_backward_parkinson();
    delay(100);
    slickfront_1();
    slickback_3();
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    delay(500);

    leg4_forward();
    leg3_forward_parkinson();
    delay(100);
    slickfront_2();
    slickback_4();
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    clearSerial();
  }

  else if (int_data == 3) {
    Serial.println("Left");
    leg2_backward();
    leg3_backward_parkinson();
    delay(100);
    slickfront_2();
    slickback_4();
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    delay(500);

    leg1_forward();
    leg4_forward_parkinson();
    delay(100);
    slickback_1();
    slickfront_3();
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    clearSerial();
  }

  else if (int_data == 4) {
    Serial.println("Right");
    leg4_backward();
    leg1_backward_parkinson();
    delay(100);
    slickfront_4();
    slickback_2();
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    delay(500);

    leg3_forward();
    leg2_forward_parkinson();
    delay(100);
    slickfront_1();
    slickback_3();
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset + walking_offset));
    clearSerial();
  }
}

void initial_home(void) {
  joint_initial_angle = (maximum_joint_angle + minimum_joint_angle) / 2;
  leg_initial_angle = maximum_leg_angle - 20;

  //Joint
  pwm.setPWM(0, 0, joint_initial_angle);
  pwm.setPWM(3, 0, joint_initial_angle);
  pwm.setPWM(9, 0, joint_initial_angle);
  pwm.setPWM(11, 0, joint_initial_angle);

  //Leg
  pwm.setPWM(1, 0, leg_initial_angle);
  pwm.setPWM(2, 0, leg_initial_angle);
  pwm.setPWM(8, 0, leg_initial_angle);
  pwm.setPWM(10, 0, leg_initial_angle);
}

//========================================================
//====                 Leg 1                          ====
//========================================================

void leg1_forward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(2, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(0, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset, leg_maximum_offset);
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg1_forward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(2, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(0, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset + walking_offset, leg_maximum_offset);
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg1_backward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(2, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(0, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset);
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg1_backward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(2, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(0, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset + walking_offset);
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void slickfront_1(void) {
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));
}

void slickback_1(void) {
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));

}

//========================================================
//====                 Leg 2                          ====
//========================================================

void leg2_forward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(1, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(3, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset, leg_maximum_offset);
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg2_forward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(1, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(3, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset + walking_offset, leg_maximum_offset);
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg2_backward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(1, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(3, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset);
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg2_backward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(1, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(3, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset + walking_offset);
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void slickfront_2(void) {
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));
}

void slickback_2(void) {
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_maximum_offset + fixing_error));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));

}

//========================================================
//====                 Leg 3                          ====
//========================================================

void leg3_forward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(8, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(9, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset, leg_maximum_offset);
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg3_forward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(8, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(9, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset + walking_offset, leg_maximum_offset);
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg3_backward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(8, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(9, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset);
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg3_backward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(8, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(9, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset + walking_offset);
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void slickfront_3(void) {
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));
}

void slickback_3(void) {
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));
}

//========================================================
//====                 Leg 4                          ====
//========================================================

void leg4_forward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(10, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(11, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset, leg_maximum_offset);
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg4_forward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(10, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_maximum_offset; i >= joint_minimum_offset; i--) {
    pwm.setPWM(11, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_minimum_offset + walking_offset, leg_maximum_offset);
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg4_backward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(10, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(11, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset);
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void leg4_backward_parkinson(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  // leg_maximum_offset: 60  leg_minimum_offset: 20
  for (int i = leg_minimum_offset; i <= leg_maximum_offset; i++) {
    pwm.setPWM(10, 0, degree_to_pulse_leg(i));
    delay(5);
  }
  delay(100);

  // joint_maximum_offset: 135  joint_minimum_offset: 45
  for (int i = joint_minimum_offset; i <= joint_maximum_offset; i++) {
    pwm.setPWM(11, 0, degree_to_pulse_joint(i));
    float leg_angle = map(i, joint_minimum_offset, joint_maximum_offset, leg_maximum_offset, leg_minimum_offset + walking_offset);
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_angle));
    delay(5);
  }
}

void slickfront_4(void) {
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_minimum_offset - fixing_error));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));
}

void slickback_4(void) {
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_maximum_offset));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));
}

int degree_to_pulse_joint(uint16_t degree) {
  uint16_t pulse = map(degree, 0 , 180 , 470 , 110);
  return pulse;
}

int degree_to_pulse_leg(uint16_t degree) {
  uint16_t pulse = map(degree, 0 , 90, 315, 100);
  return pulse;
}
