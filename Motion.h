// MOTION_H
// by Fathurur Said(ffs221)
// V2: modified the delay
// Function Vehicle Motion, using polling instead of delay
// http://arduiniana.org/libraries/pwmservo/

//   Board                     SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   -----                     -----------   -----------   -----------
//   Arduino Uno, Duemilanove       9            10          (none)
//   Arduino Mega                  11            12            13
//   Sanguino                      13            12          (none)
//   Teensy 1.0                    17            18            15
//   Teensy 2.0                    14            15             4
//   Teensy++ 1.0 or 2.0           25            26            27
//   Teensy LC & 3.x                 (all PWM pins are usable) (20-23,3-6,9,10)
#ifndef MOTION_H_INCLUDED
#define MOTION_H_INCLUDED
#include <PWMServo.h>
PWMServo myservo, myservo2; // create servo object to control a servo left-right
//extern PWMServo myservo,myservo2;
int Clip(int Val) {
  return (Val < 0 ? 0 : ((Val > 180) ? 180 : Val));
}
void PWM_mov(PWMServo myservo_tmp, int vec = 90, int dtime = 150) {
  myservo_tmp.write((90 + vec) % 180);
  if (dtime != 0) {
    delay(dtime);
    myservo_tmp.write(90);
  }
  delay(15);
}
void Motion_stop(int dtime = 15) {
  myservo.write(90);//stop
  myservo2.write(90);//stop
  if (dtime != 0)delay(dtime);
}
void Motion_init() {
  myservo.attach(SERVO_PIN_A);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(SERVO_PIN_B);
  Motion_stop();
}
void Straight(int vec = 85, int dtime = 150) {
  //vec--speed,+forward,-backward;dtime--running time,0 to not stop;
  //thus this control method cannot be precise;
  myservo.write(Clip(90 - vec + 3));
  myservo2.write(Clip(90 + vec + 3));
  if (dtime != 0) {
    delay(dtime);
    Motion_stop(0);
  }
  delay(15);
}
void Rotate(int vec = 38, int dtime = 360) {
  //vec--speed,+left,-right;dtime--running time,0 to not stop;
  //thus this control method cannot be precise;
  myservo.write(Clip(90 + vec));
  myservo2.write(Clip(90 + vec));
  if (dtime != 0) {
    delay(dtime);
    Motion_stop(0);
  }
  delay(15);
}
void Tunning(int vec = 50, int dtime = 150, int ove = 30) {
  //Turning in running, with original speed=ove
  //vec--speed,+left,-right;dtime--running time,0 to not stop;
  //thus this control method cannot be precise;
  myservo.write(Clip(90 + vec - 2 + ove));
  myservo2.write(Clip(90 + vec + 2 + ove));
  if (dtime != 0) {
    delay(dtime);
    myservo.write(Clip(90 + ove));
    myservo2.write(Clip(90 + ove));
  }
  delay(15);
}


#endif // MOTION_H_INCLUDED
