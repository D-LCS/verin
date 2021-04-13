/*
 * AnalogInput.cpp
 *
 *  Created on: 18 nov. 2020
 *      Author: phifo
 */

#include "arm_math.h"
#include "main.h"

#include "AnalogInput.h"


AnalogInput::AnalogInput ()
{
   /* Initialize input and output buffer pointers */
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
  flagNewValForFilter_ = true;
}


//uint16_t AnalogInput::getAverageValue (void) // version originale
//{
//  int32_t tmp = 0;
//  if (flagNewValForAverage_)
//    {
// HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
//
//      for (int i = 0; i < ANALOG_ARRAY_SIZE; i++)
//        {
//          tmp += rawValuesArray_[i];
//        }
//      averageValue_ = tmp / ANALOG_ARRAY_SIZE;
//      flagNewValForAverage_ = false;
// HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
//
//    }
//  return averageValue_;
//}

uint16_t add_and_halve (uint16_t val1, uint16_t val2)
{
  uint16_t res;
  res = __SHADD8 (val1, val2);
  return res;
}

uint16_t AnalogInput::getAverageValue (void)
{
  if (flagNewValForAverage_)
    {
      //HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin); // pour la mesure du temps de calcul
      arm_mean_q15 ( (q15_t *) rawValuesArray_, ANALOG_ARRAY_SIZE, (q15_t *)& averageValue_);
      //HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
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

uint16_t AnalogInput::getValue (void)
{
  return (uint16_t)getAverageValue ();
}

void AnalogInput::periodicManagement (void)
{
  getFilteredValue ();
}

// end of file

