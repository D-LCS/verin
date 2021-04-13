/*
 * AnalogInput.h
 *
 *  Created on: 18 nov. 2020
 *      Author: phifo
 */

#ifndef ANALOGINPUT_H_
#define ANALOGINPUT_H_

#define ANALOG_ARRAY_SIZE 64

class AnalogInput
{
public:
  AnalogInput ();
  AnalogInput (float *filterTap)
  {
    filterTap_ = filterTap;
  }
  ;

  virtual ~AnalogInput ();

  /*!
   *
   */
  uint16_t getValue (void);

  /*!
   *
   */
  float getFilteredValue (void);

  /*!
   *
   */
  uint16_t getAverageValue (void);

  /*!
   * Enregistrement d'une nouvelle valeur dans le tableau de valeurs brutes
   * et gestion de l'index de la table
   */
  void storeNewValue (uint16_t v);

/*!
 *
 */
 void periodicManagement (void);


private:
  uint16_t rawValuesArray_[ANALOG_ARRAY_SIZE];
  uint16_t rawArrayIndex_;
  uint16_t averageValue_;

  // const float filterTap[ANALOG_ARRAY_SIZE] = {#include "/home/phifo/testfiltre.csv"};
  float filteredValue_;
  float *filterTap_;

  volatile bool flagNewValForAverage_;
  volatile bool flagNewValForFilter_;
};

#endif /* ANALOGINPUT_H_ */
