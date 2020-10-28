//By Bomiao Chen & Fathur Said
//V2 by Fathurur Said(zx694) 4.28.2018
//5.6.2018 By Fathurur Said: Added Get_freq, My_MajorPeak, My_maxfreqPeak
#ifndef MYFFT_H_INCLUDED
#define MYFFT_H_INCLUDED

#include "arduinoFFT.h"
#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 25600//Hz, must be less than 80000 due to ADC
#define MAG_THRESHOLD 1200             //Must be a power of 2

arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
double P_value;//major peak mag
double P_value_mf;//max freq mag
void MYFFT_init() {
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}
void Sampling() {
  /*SAMPLING*/
  for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros();    //Overflows after around 70 minutes!
    vReal[i] = analogRead(0);
    vImag[i] = 0;
    while (micros() < (microseconds + sampling_period_us)) {
    }
  }
}
void Do_FFT() {
  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
}
double Get_freq(double *vD, uint16_t samples, double samplingFrequency, uint16_t Index) {
  //getting the accurate(interpolated) freq from index(if peak)
  double delta = 0.5 * ((vD[Index - 1] - vD[Index + 1]) / (vD[Index - 1] - (2.0 * vD[Index]) + vD[Index + 1]));
  double interpolatedX = ((Index + delta)  * samplingFrequency) / (samples - 1);
  if (Index == (samples >> 1)) //To improve calculation on edge values
    interpolatedX = ((Index + delta)  * samplingFrequency) / (samples);
  // returned value: interpolated frequency peak apex
  return (interpolatedX);
}
double My_MajorPeak(double *vD, uint16_t samples, double samplingFrequency, uint16_t clip_n = 1)
{
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;
  //If sampling_frequency = 2 * max_frequency in signal,
  //value would be stored at position samples/2
  for (uint16_t i = clip_n; i < ((samples >> 1) + 1); i++) {
    if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
      if (vD[i] > maxY) {
        maxY = vD[i];
        IndexOfMaxY = i;
      }
    }
  }
  P_value = maxY;
  double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
  double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples - 1);
  if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
    interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
  // returned value: interpolated frequency peak apex
  return (interpolatedX);
}

#endif // MYFFT_H_INCLUDED
