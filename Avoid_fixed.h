//Avoid_fixed.h:
//Avoid.h Algorithm designed by Xinyu Ma
//Changing structure by Fathurur Said 5.2.2018
//Bug fixed by Fathurur Said 5.6.2018
#ifndef AVOID_FIXED_H_INCLUDED
#define AVOID_FIXED_H_INCLUDED
#include "get_distance.h"
#include "Motion.h"
#define FREQUENCY_DIFF 20//the max variation of the frequncy
#define RIGHT_DISTANCE 700
#define STRAIGHT_DISTANCE 1400
#define LEFT_DISTANCE 700
#define DISTANCE_THRESHOLD 15//if the distance is larger than this, it will go
extern double get_distance();
extern double mag_average;
extern void light_led();//light the led when find the last mine
extern double max_frequency, max_frequency_last;
bool Straight_avoid(int vec = 50, int dtime = 700 , uint16_t d_thr = DISTANCE_THRESHOLD ) {
  //Going straight with avoiding
  unsigned long microseconds = micros();    //Overflows after around 70 minutes!
  Straight(vec, 0);
  while (micros() < (microseconds + dtime * 1000)) {
    if (get_distance() <= d_thr) {
      Motion_stop();
      return false;
    }
  }
  if (dtime != 0) Motion_stop();
  return true;
}
bool avoid(double target_freq, uint16_t max_target = 9500) { //it should be a loop
  //if find the last beacon return true, otherwise return false
  //if this is the targeted mine then stop, otherwise doing avoiding.
  uint16_t direction_now = 0;
  uint16_t counter = 0;
  while (counter < 2) {
    counter++;
    Sampling();
    Do_FFT();
    max_frequency = My_MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY, 20);
    //mag_sum / nsamples
    if (abs(max_frequency - max_frequency_last) >= 2 * FREQUENCY_DIFF || P_value < 0.9 * mag_average) {
      //if not detected max_freq
      max_frequency = target_freq;
    }
    max_frequency_last = max_frequency;
    if (abs(max_frequency - max_target) < FREQUENCY_DIFF * 2 && max(P_value, mag_average) >= 500 * 0.7) {
      //find the last mine and stop avoiding
      return true;
    }
    if (abs(max_frequency - target_freq) < FREQUENCY_DIFF * 2 && mag_average >= 600 * 0.7) {
      //find the target mine and stop avoiding
      return false;
    }

    Rotate(-39); //right rotate
    direction_now--;
    delay(500);
    if (!Straight_avoid(30, RIGHT_DISTANCE)) {
      continue; //situation 2
    }
    delay(500);

    Rotate(38); //left rotate
    direction_now++;
    delay(500);
    if (!Straight_avoid(30, STRAIGHT_DISTANCE)) {
      continue;//situation 3
    }
    delay(500);

    Rotate(38); //left rotate
    direction_now++;
    delay(500);
    if (!Straight_avoid(30, LEFT_DISTANCE)) {
      continue;//situation 4
    }
    else {
      //turning to the orgin direction
      delay(500);
      switch (direction_now % 4) {
        case 1:
          Rotate(-39);
          direction_now--;
          delay(200);
          break;
        case 2:
          Rotate(-39, 720);
          direction_now -= 2;
          delay(200);
          break;
        case 3:
          Rotate(38);
          direction_now -= 4;
          delay(200);
          break;
        case 0:  break;
        default: ;
      }
      return false;
      break;
    }
  }
  return false;
}


#endif // AVOID_FIXED_H_INCLUDED
