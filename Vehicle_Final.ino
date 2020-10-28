//Main algorithm designed by all members
//Moving average By Fathur Said
//Coded by Fathurur Said(ffs221) & Xinyu Ma
//5.6.2018 by Fathurur Said: Added function: light_led(), ntoindex(),My_getmag_n()
//5.8.2018 by Fathurur Said: Added out-of-time detect
//5.9.2018 Modified by Fathurur Said: Choose_target_n
/*========Pin Map==========
   left servo      9
   right servo    10
   mic out        14
   mic ground     AGND
   mic VCC        3.3V
   trigPin        15
   echoPin        12
   led            13
   Ultrasonic GND GND
   Ultrasonic VCC 5V
*/
#include "Motion.h"
#include "MYFFT.h"
#include "get_distance.h"
#include "Avoid_fixed.h"
#define CLIP_NOISE_FREQ_n 20
#define MAX_TARGET_FREQ 9500  //SHOULD BE 9500    
extern PWMServo myservo, myservo2;//in Motion.h
double peak, peak_last = 0;//store the target mine freq
extern double P_value;//major peak mag
extern double P_value_mf; //max freq mag
extern double vReal[SAMPLES];//store the fft data
//------addition 4.27-------
int stop_int = -1/*counter, control the rotation*/;
double p_value_last, p_value_max = 0, value = 0, mag_average = 0, mag_average_max = 0;

//====addition 5.4======
int nsamples = 0/*average sample counter*/, nround = 0/*turning counter*/;
double mag_sum = 0/*sample mag sum*/;
bool state = false/*control the rotation direction*/, direction_flag = false/*false to enable finding directon*/;
uint16_t Target_mine_n;//0-9,nth mine
double max_frequency = 0, max_frequency_last = 0;
double dis_now;//the front distance
//=====addition 5.8======
uint16_t nsamples_all[10];
double mag_sum_all[10];
//=======LED=======
const int ledPin = 13;
void light_led(int n = 2, int dtime = 1000) {
  for (int i = 0; i < n; i++) {
    digitalWrite(ledPin, HIGH);   // set the LED on
    delay(dtime);                  // wait for a second
    digitalWrite(ledPin, LOW);    // set the LED off
    delay(dtime);                  // wait for a second
  }
}
//=======Mine&Freq========
uint16_t ntoindex(uint16_t mine_n, uint16_t samples = SAMPLES, double samplingFrequency = SAMPLING_FREQUENCY) {
  //convert nth mine into frequency index(according to fft parameters)
  return (5000 + mine_n * 500) / (samplingFrequency / samples);
}
void My_getmag_n(double *vD, uint16_t Target_mine_n) {
  //to find the mag from nth mine, store freq in peak and mag in P_value_mf
  uint16_t IndexOfY = ntoindex(Target_mine_n);
  IndexOfY = vD[IndexOfY] > vD[IndexOfY + 1] ? IndexOfY : (IndexOfY + 1);
  if (vD[IndexOfY] > vD[IndexOfY + 1] && vD[IndexOfY] > vD[IndexOfY - 1] && vD[IndexOfY] > 100) {
    peak = Get_freq(vD, SAMPLES, SAMPLING_FREQUENCY, IndexOfY);
    P_value_mf = vD[IndexOfY];
  }
  else {
    peak = 0;
    P_value_mf = 0;
  }
}
uint16_t Choose_target_n(uint16_t n_now) {
  //if could hear higher frequency, turn to that mine, otherwise choosing the next
  uint16_t i = 9;
  //init all parameters
  for (i = 0; i < 10; i++) {
    nsamples_all[i] = 0;
    mag_sum_all[i] = 0;
  }
  //measure all mines
  unsigned long milliseconds = millis();
  while (millis() < milliseconds + 1000) {
    Sampling();
    Do_FFT();
    for (i = 0; i < 10; i++) {
      My_getmag_n(vReal, i);
      if (P_value_mf >= 100 && ((round(peak) + FREQUENCY_DIFF * 2) % 500) - FREQUENCY_DIFF * 2 < FREQUENCY_DIFF * 2) {
        //if freq belongs to mine, doing average
        mag_sum_all[i] += P_value_mf;
        nsamples_all[i]++;
      }
    }
  }
  //find highest freq over threshold
  for (i = 9; i > n_now; i--) {
    if (mag_sum_all[i] / nsamples_all[i] > 200) {
      break;
    }
  }
  //reset parameters
  direction_flag = false;
  p_value_max = 0;
  //  mag_sum = 0;//sum of mag
  //  nsamples = 0;//
  nround = 0;//turning n
  mag_average_max = 0;
  mag_average = 0;
  return i;
}
void setup() {
  Motion_init();
  MYFFT_init();
  Serial.begin(115200);
  //Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //=======LED=======
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  //=======init mine=======
  Target_mine_n = 0;//the first mine
  //if could hear higher frequency, turn to that mine, otherwise choosing the next
  Target_mine_n = Choose_target_n(Target_mine_n);
}

void loop() {
  Sampling();
  Do_FFT();
  My_getmag_n(vReal, Target_mine_n);
  if (abs(peak_last - peak) < FREQUENCY_DIFF && peak > 4500 && peak < 11000) { //peak_last==peak abs(7500+20-peak) < 30
    //if the target mine is beeping, doing the average fft, otherwise rotate.
    //if (value == 0)Serial.println("----------");
    value = P_value_mf;//max(P_value_mf, p_value_last);//
    p_value_max = max(value, p_value_max);//p_value_last
    if (value > 0.2 * p_value_max && p_value_max != 0) {
      //if mag large enough
      stop_int = 5;//counter, control the rotation
      //doing averge
      nsamples++;
      mag_sum += value;
    }
    else {
      stop_int--;//counter, control the rotation,from 5 to 0
      value = 0;
    }
  }
  else {
    stop_int--;//counter, control the rotate,from 5 to 0
    value = 0;
  }
  if (!direction_flag) {//enable to finding the direction
    /*------------target to the mine------------*/
    if (stop_int == 0) {
      //if the target mine is not beeping//end of beeping
      unsigned long microseconds = micros();    //Overflows after around 70 minutes!
      if (mag_sum / nsamples < 0.85 * mag_average) {
        //doing average and changing the rotate direction
        state = !state;//turning into opposite direction
        nround++;//added the turning counter
      }
      if ((nround >= 2 && mag_sum / nsamples > 0.93 * mag_average_max && ((mag_sum / nsamples) < mag_average_max || (get_distance() <= 30))) || nround >= 10) {
        //stop condition:1.finding the highest mag 2.taking too much time
        direction_flag = true;
      }
      else {
        //not meet the stop condition, doing rotation
        if (state) Rotate(-23, 0);//26
        else Rotate(23, 0);
        while (micros() < (microseconds + 250 * 1000)) {
        }
        Motion_stop();
      }
      //doing average and find the max mean mag
      mag_average = mag_sum / nsamples;
      mag_average_max = max(mag_average, mag_average_max);
      //clear average variables
      nsamples = 0;
      mag_sum = 0;
    }
    if (stop_int < -360) {
      //if out of time
      stop_int = -1;
      microseconds = micros();
      p_value_max = 0;
      //mag_average_max=0;
      if (state)Rotate(-23, 0);
      else Rotate(23, 0);
      while (micros() < (microseconds + 250 * 1000)) {
      }
      Motion_stop();
    }
  }
  else {
    /*------------End finding the direction, doing approaching------------*/
    max_frequency = My_MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY, CLIP_NOISE_FREQ_n);
    dis_now = get_distance();
    if (dis_now <= 15) {
      Motion_stop();
      //max_frequency=My_MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY,CLIP_NOISE_FREQ_n);
      if (avoid(peak, MAX_TARGET_FREQ)) {
        /*----------have find the last mine and hit it---------*/
        Motion_stop();
        light_led(20);
      }
      else {
        /*----------have not find the last mine---------*/
        Target_mine_n += 1;
        Motion_stop();
        if (Target_mine_n > 9)light_led(20); //ended
        else light_led(2);
        Target_mine_n = Choose_target_n(Target_mine_n);
      }
    }
    else {
      //going straight with avoiding
      if (!Straight_avoid(30, 1600, 12)) {
        //there's mine on the front
        avoid(peak, MAX_TARGET_FREQ);
      }
      else {
        //there's no mine on the front
        light_led(1);
        //still finding the mine
        Target_mine_n = Choose_target_n(Target_mine_n);
      }
    }
    if (stop_int == 0) {
      //end of beeping
      mag_average = mag_sum / nsamples;
      //clear average variables
      nsamples = 0;
      mag_sum = 0;
    }
    //    if (abs(max_frequency - peak)<10 && P_value_mf > 3000 && max_frequency > 4500 && max_frequency < 11000) {
    //      //near the target mine and mag is higher than threshold
    //      Motion_stop();
    //      light_led(10);
    //    }
    if (nsamples >= 3 && (mag_sum / nsamples) < 0.75 * mag_average_max) {
      //have wrong direction
      Motion_stop();
      light_led(3);
      Target_mine_n = Choose_target_n(Target_mine_n);
    }
    max_frequency_last = max_frequency;
  }
  peak_last = peak;
  p_value_last = P_value_mf;
}
