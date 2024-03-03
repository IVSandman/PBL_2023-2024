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

uint16_t joint_initial_angle, leg_initial_angle;

String input;
uint16_t angle;

// -------------------------- Ultrasoic ---------------------------------------------

const int pingPin_front = 11;
int inPin_front = 10;

const int pingPin_right = 13;
int inPin_right = 12;

const int pingPin_left = 9;
int inPin_left = 8;

long duration_front, duration_right, duration_left;
long distance_front, distance_right, distance_left;

void setup() {
  Serial.begin(115200);
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  pinMode(pingPin_front, OUTPUT);
  pinMode(inPin_front, INPUT);
  pinMode(pingPin_right, OUTPUT);
  pinMode(inPin_right, INPUT);
  pinMode(pingPin_left, OUTPUT);
  pinMode(inPin_left, INPUT);
  delay(10);

  Serial.print("Ultrasonic on....\n");

  Serial.print("Initialize... \n");
  initial_home();
  Serial.print("Rest to Home position \n");
  Serial.print("Press Enter to initiate \n");
}


void loop() {
  distance_front = detect_front();
  distance_right = detect_right();
  distance_left = detect_left();
  Serial.print(distance_right);
  Serial.print("cm\t");
  Serial.print(distance_front);
  Serial.print("cm\t");
  Serial.print(distance_left);
  Serial.print("cm\n");

  if (distance_front <= 25) {
    ball_front();
    delay(3000);
    initial_home();
  }

  if (distance_right <= 25) {
    ball_right();
    delay(3000);
    initial_home();
  }
  
  if (distance_left <= 25) {
    ball_left();
    delay(3000);
    initial_home();
  }
}

void ball_right() {
  pwm.setPWM(1, 0, degree_to_pulse_leg(80));
  pwm.setPWM(2, 0, degree_to_pulse_leg(80));
}

void ball_left() {
  pwm.setPWM(8, 0, degree_to_pulse_leg(80));
  pwm.setPWM(10, 0, degree_to_pulse_leg(80));
}

void ball_front() {
  pwm.setPWM(2, 0, degree_to_pulse_leg(70));
  pwm.setPWM(10, 0, degree_to_pulse_leg(70));
}

void initial_home(void) {
  joint_initial_angle = (maximum_joint_angle + minimum_joint_angle) / 2;
  leg_initial_angle = maximum_leg_angle - 20;

  //Joint
  pwm.setPWM(0, 0, degree_to_pulse_joint(90 + 30));
  pwm.setPWM(3, 0, degree_to_pulse_joint(90 - 30));
  pwm.setPWM(9, 0, degree_to_pulse_joint(90 + 30));
  pwm.setPWM(11, 0, degree_to_pulse_joint(90 - 30));

  //Leg
  pwm.setPWM(1, 0, leg_initial_angle);
  pwm.setPWM(2, 0, leg_initial_angle);
  pwm.setPWM(8, 0, leg_initial_angle);
  pwm.setPWM(10, 0, leg_initial_angle);
}


int degree_to_pulse_joint(uint16_t degree) {
  uint16_t pulse = map(degree, 0 , 180 , 470 , 110);
  return pulse;
}

int degree_to_pulse_leg(uint16_t degree) {
  uint16_t pulse = map(degree, 0 , 90, 315, 100);
  return pulse;
}

long detect_front()
{
  digitalWrite(pingPin_front, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_front, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_front, LOW);
  duration_front = pulseIn(inPin_front, HIGH);

  return duration_front / 29 / 2;
}

long detect_right()
{
  digitalWrite(pingPin_right, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_right, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_right, LOW);
  duration_right = pulseIn(inPin_right, HIGH);

  return duration_right / 29 / 2;
}

long detect_left()
{
  digitalWrite(pingPin_left, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_left, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin_left, LOW);
  duration_left = pulseIn(inPin_left, HIGH);

  return duration_left / 29 / 2;
}
