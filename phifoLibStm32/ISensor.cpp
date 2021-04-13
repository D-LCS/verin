/*
 * ISensor.cpp
 *
 *  Created on: 1 sept. 2020
 *      Author: phifo
 */

//#include <arm_math.h> //! \todo essayer d'utiliser les fonctionS DSP de filtrage

#include "main.h"
#include "ISensor.h"

#define NUM_TAPS CURRENT_TABLE_SIZE
#define BLOCK_SIZE 1

/*! Valeurs des coefficients du filtre
 * Les coefficients sont générés par l'application pyFDA (python Filter Design Analysis)
 * Le fichier suivant est généré par l'application
 */
const float filterTap[CURRENT_TABLE_SIZE] = {
#include "/home/phifo/testfiltre.csv"
    };

//static float32_t firStateF32[BLOCK_SIZE + CURRENT_TABLE_SIZE - 1]; //! \todo essayer d'utiliser les fonctionS DSP de filtrage


CurrentSensor::CurrentSensor (float toVolts, float offset, float sensitivity, float fSeuil)
{
  initialisation (); //initialisation de la table de valeurs brutes

  toVolts_ = toVolts;
  offset_ = offset;
  sensitivity_ = sensitivity;

  iSeuilPos_ = ((fSeuil * sensitivity) + offset) / toVolts;
  iSeuilNeg_ = ((fSeuil * sensitivity * -1.0) + offset) / toVolts;
}

void CurrentSensor::initialisation (void)
{
  for (int i = 0; i < CURRENT_TABLE_SIZE; i++)
    {
      rawCurrentValues[i] = 0;
    }
  overCurrent = false;
  rawCurrentTableIndex = 0;

  // arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs[0], (float32_t *)&firStateF32[0], blockSize);
  // arm_fir_init_f32(&S, CURRENT_TABLE_SIZE, (float32_t *)&filterTap[0], (float32_t *)&firStateF32[0], blockSize);
  // https://community.st.com/s/question/0D50X0000BrDrzwSQC/how-to-add-cmsis-dsp-libraries-in-stm32cubeide
  //https://www.youtube.com/watch?v=vCcALaGNlyw
  // https://github.com/ARM-software/CMSIS/tree/master/CMSIS/Lib/GCC
}

void CurrentSensor::setNewValue (uint16_t v)
{
  rawCurrentTableIndex = (rawCurrentTableIndex + 1) % CURRENT_TABLE_SIZE;
  rawCurrentValues[rawCurrentTableIndex] = v;

  flagNewVal = true;
}

void CurrentSensor::filterRawValues (void)
{
// Calcul de la valeur moyenne
//  int32_t tmp = 0;
//  for (int i = 0; i < CURRENT_TABLE_SIZE; i++)
//    {
//      tmp += rawCurrentValues[i];
//    }
//  averageRawCurrent = tmp / CURRENT_TABLE_SIZE;

  // Calcul de la valeur filtrée, seulement si il y a une nouvelle valeur brute
  if (flagNewVal)
    {
      // HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // pour la mesure du temps de calcul
      filteredCurrent = 0.0;
      int j = rawCurrentTableIndex;
      for (int i = 0; i < CURRENT_TABLE_SIZE; i++)
        {
          j = (j + 1) % CURRENT_TABLE_SIZE;
          filteredCurrent += rawCurrentValues[j] * filterTap[i];
        }
      flagNewVal = false;
      // HAL_GPIO_WritePin (LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
}

bool CurrentSensor::isOverCurrentDetected (void)
{
  filterRawValues ();
  if ((filteredCurrent > iSeuilPos_) || (filteredCurrent < iSeuilNeg_))
    return true;
  else
    return false;
}

float CurrentSensor::getValue (void)
{
  //  (int16_t) ((((float) raw) * RAW_TO_VOLTS - CURRENT_SENSOR_V0) / ACS712_SENSITIVITY_MILLIAMP);
  //  return ((averageRawCurrent * toVolts_) - offset_) / sensitivity_;
  filterRawValues ();
  return ((filteredCurrent * toVolts_) - offset_) / sensitivity_;
}

void CurrentSensor::periodicManagement (void)
{
  filterRawValues ();
}


// end of file

