#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define maximum_joint_angle 470
#define minimum_joint_angle 110

#define maximum_leg_angle 315
#define minimum_leg_angle 100

#define leg_maximum_offset 60
#define leg_minimum_offset 20

#define joint_maximum_offset 135
#define joint_minimum_offset 45

#define fixing_error 0

uint16_t joint_initial_angle, leg_initial_angle;

String input;
uint16_t angle;

void setup() {
  Serial.begin(115200);
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  delay(10);

  Serial.print("Initialize... \n");
  initial_home();
  delay(5000);
  Serial.print("Rest to Home position \n");
  Serial.print("Press Enter to initiate \n");
}

void loop() {


  leg3_backward();
//  leg4_backward();
//  delay(100);
//  slickback_1();
//  slickfront_3();
//  delay(500);
//  leg2_forward();
//  leg1_forward();
//  delay(100);
//  slickback_2();
//  slickfront_4();
//  delay(500);

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
  for (int i = 135; i >= 45; i--) {
    pwm.setPWM(0, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
    delay(5);
  }
}

void leg1_backward(void) {
  //Position1 joint to 135, leg to initial
  pwm.setPWM(0, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(2, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Position2
  for (int i = 45; i <= 135; i++) {
    pwm.setPWM(0, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(2, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
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
  for (int i = 135; i >= 45; i--) {
    pwm.setPWM(3, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
    delay(5);
  }
}

void leg2_backward(void) {
  // Pos1
  pwm.setPWM(3, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(1, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Pos2
  for (int i = 45; i <= 135; i++) {
    pwm.setPWM(3, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(1, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
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
  for (int i = 135; i >= 45; i--) {
    pwm.setPWM(9, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
    delay(5);
  }
}

void leg3_backward(void) {
  // Pos1
  pwm.setPWM(9, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(8, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Pos2
  for (int i = 45; i <= 135; i++) {
    pwm.setPWM(9, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(8, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
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
  for (int i = 135; i >= 45; i--) {
    pwm.setPWM(11, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
    delay(5);
  }
}

void leg4_backward(void) {
  // Pos1
  pwm.setPWM(11, 0, degree_to_pulse_joint(joint_minimum_offset));
  pwm.setPWM(10, 0, degree_to_pulse_leg(leg_minimum_offset));

  //Pos2
  for (int i = 45; i <= 135; i++) {
    pwm.setPWM(11, 0, degree_to_pulse_joint(i));
    float leg_angle = abs(i - 90);
    leg_angle = map(leg_angle, 45, 0 , leg_minimum_offset , leg_maximum_offset);
    pwm.setPWM(10, 0, degree_to_pulse_leg(leg_angle));
    //Serial.println(leg_angle);
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
