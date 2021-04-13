/*
 * AnalogInput.cpp
 *
 *  Created on: 18 nov. 2020
 *      Author: phifo
 */
#define ARM_MATH_CM4

#include <arm_math.h>

#include "main.h"

#include "AnalogInput.h"


#define TEST_LENGTH_SAMPLES  320
#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            32
#define NUM_TAPS              29

float32_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
float32_t refOutput[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES];
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

const float32_t firCoeffs32[NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;
float32_t  snr;


AnalogInput::AnalogInput ()
{
  arm_fir_instance_f32 S;
  float32_t  *inputF32, *outputF32;

   /* Initialize input and output buffer pointers */
   inputF32 = &testInput_f32_1kHz_15kHz[0];
   outputF32 = &testOutput[0];

   /* Call FIR init function to initialize the instance structure. */
   arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);


  for (int i = 0; i < ANALOG_ARRAY_SIZE; i++)
    {
      rawValuesArray_[i] = 0;
    }

  rawArrayIndex_ = 0;
  filteredValue_ = 0;
  averageValue_ = 0;

}

AnalogInput::~AnalogInput ()
{
  // TODO Auto-generated destructor stub
}

void AnalogInput::storeNewValue (uint16_t v)
{
  rawArrayIndex_ = (rawArrayIndex_ + 1) % ANALOG_ARRAY_SIZE;
  rawValuesArray_[rawArrayIndex_] = v;

  flagNewValForAverage_ = true;
  ;
  flagNewValForFilter_ = true;
}

uint16_t AnalogInput::getAverageValue (void)
{
  int32_t tmp = 0;
  if (flagNewValForAverage_)
    {
      for (int i = 0; i < ANALOG_ARRAY_SIZE; i++)
        {
          tmp += rawValuesArray_[i];
        }
      averageValue_ = tmp / ANALOG_ARRAY_SIZE;
      flagNewValForAverage_ = false;
    }

  return averageValue_;
}

float AnalogInput::getFilteredValue (void)
{
  // Calcul de la valeur filtrée, seulement si il y a une nouvelle valeur brute
  if (flagNewValForFilter_)
    {
      filteredValue_ = 0.0;
      int j = rawArrayIndex_;
      for (int i = 0; i < ANALOG_ARRAY_SIZE; i++)
        {
          j = (j + 1) % ANALOG_ARRAY_SIZE;
          filteredValue_ += ((float)rawValuesArray_[j]) * filterTap_[i];
        }
      flagNewValForFilter_ = false;
    }
  return filteredValue_;
}

// end of file

