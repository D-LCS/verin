/**
 * \brief Driver pour IFX9201
 *
 * \file IFX9201.cpp
 *
 * Adaptation à la bibliothèque STM32 HAL de fichier présent sur le github géré par Infineon
 * Le portage a consisté à traduire les "appels arduino" en appels pour assurer la compatibilité avec la HAL
 * Pour pouvoir suivre les évolutions sur le github, les appels d'origines ont été laissé en commentaire
 *
 * \Auteur : phifo
 * \Date : 27 & 28/05/2020
 *
 * \Etat : provisoire, dans l'attente d'une carte prototype
 *
 */

/**
 * IFX9201.cpp - Library for Arduino to control the IFX9201 H-Bridge.
 *
 * The IFX9201 is a general purpose 6A H-Bridge for industrial applications, home appliance and building
 * automation, power tools battery management and medical applications, designed for the control of small
 * DC motors and inductive loads.
 *
 * Have a look at the application note/datasheet for more information.
 *
 * Copyright (c) 2018 Infineon Technologies AG
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*! \file IFX9201.cpp
 *  \brief This file defines functions and predefined instances from IFX9201.h
 */
/* Paul Carpenter Sept-2019
 Add error traps and return codes to more functions
 Improve duty cycle conversion to work with any analogWriteResolution
 (requires Pull request #106 merged for getAnalogWriteMaximum)
 Try to make code more readable
 Correct Disable Pin polarity for correct operation
 */

#include "main.h"
#include "application.h"
#include "HDriver.h"

#include "IFX9201conf.h"
#include "IFX9201.h"

// Instantiate with SPI
IFX9201::IFX9201 (SPI_HandleTypeDef *hspi, const MGPIO_HandleTypeDef *pinSlaveSelect, const MPWM_HandleTypeDef *pwm,
                  const MGPIO_HandleTypeDef *pinDirection, const MGPIO_HandleTypeDef *pinDisable)
{
  // Inutile, cree par cubeMX
  // m_SPIsettings = SPISettings (1000000u, MSBFIRST, SPI_MODE1);
  //m_Mode = IFX9201Mode_SPI;
  m_Mode = IFX9201Mode_PWM;
  m_bus = hspi;
  m_PWMFreq = 0u;

  m_SlaveSelect = pinSlaveSelect;
  m_Disable = pinDisable;
  m_Direction = pinDirection;
  m_Pwm = pwm;

}

// Instantiate without SPI
IFX9201::IFX9201 (MPWM_HandleTypeDef *pwm, MGPIO_HandleTypeDef *pinDirection, MGPIO_HandleTypeDef *pinDisable)
{
  m_Mode = IFX9201Mode_PWM;
  m_PWMFreq = 0u;

  //m_SlaveSelect = pinSlaveSelect;
  m_Disable = pinDisable;
  m_Direction = pinDirection;
  m_Pwm = pwm;
  while (1)
    {
      //not yet implemented and tested
    }
}

void IFX9201::setWiringMode (HDriver_wiring_t w)
{
  if (w == HDRIVER_Flipping_Mode)
    {
      reverseWiring = true;
    }
  else
    {
      reverseWiring = false;
    }
}

void IFX9201::setAcceleration (uint8_t a)
{
  //HDRIVER_Result_Not_Implemented_Yet
}

void IFX9201::setDeceleration (uint8_t d)
{
  //HDRIVER_Result_Not_Implemented_Yet
}

void IFX9201::diagnostic (void)
{
  //HDRIVER_Result_Not_Implemented_Yet
}

// Note for SPI mode DIR, PWM and DIS must all be LOW
// Otherwise IFX9201 reverts to PWM mode
//
//void IFX9201::begin (void)
HDriver_Result_t IFX9201::initialisation (void)
{
  // Inutile, cree par cubeMX
  //pinMode (m_Disable, OUTPUT);
  //pinMode (m_Direction, OUTPUT);
  //pinMode (m_PWM, OUTPUT);

  //digitalWrite (m_PWM, LOW);
  maximumPwmValue = m_Pwm->htim->Instance->ARR;  // recuperation de la valeur dans les configs timer
  setPWM (0);
  m_forward = false;
  m_reverse = false;

  // digitalWrite (m_Direction, LOW);
  HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);

  if (m_Mode == IFX9201Mode_SPI)
    {
      // pinMode (m_SlaveSelect, OUTPUT); // Inutile, cree par cubeMX
      // digitalWrite (m_SlaveSelect, HIGH);
      HAL_GPIO_WritePin (m_SlaveSelect->port, m_SlaveSelect->pin, GPIO_PIN_SET);
      //digitalWrite (m_Disable, LOW);        // To ensure we stay in SPI mode
      HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_RESET);
      //setPWMFrequency (IFX9201__DEFAULT_PWM_FREQUENCY);
    }
  else   // PWM mode
    {
      //digitalWrite (m_Disable, HIGH);        // Disable ON - MOSFET drivers OFF
      HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_SET);
      //setPWMFrequency (IFX9201__DEFAULT_PWM_FREQUENCY);
    }

  return HDRIVER_Result_Not_Implemented_Yet;
}

// forwards SPI or default PWM mode
HDriver_Result_t IFX9201::forwards (uint8_t dutyCycle)
{
  HDriver_Result_t ret = HDRIVER_Result_Ok;

  if (m_Mode == IFX9201Mode_SPI)
    ret = setCTRLReg (IFX9201__CTRL_SIN_MSK | IFX9201__CTRL_SEN_MSK | IFX9201__CTRL_SDIR_MSK | IFX9201__CTRL_SPWM_MSK);
  else
    {
//      switch (mode)
//        {
//        case HDRIVER_stopped:
//          mode = HDRIVER_acceleration;
//          if (reverseWiring)
//            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
//          else
//            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_SET);
//          currentDutyCycle = 10;
//          HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_RESET);
//          HAL_TIM_PWM_Start (m_Pwm->htim, m_Pwm->timChannel);
//          m_reverse = false;
//          m_forward = true;
//          break;
//
//        case HDRIVER_acceleration:
//          currentDutyCycle += 2;
//          if (currentDutyCycle >= duty_cycle)
//            {
//              currentDutyCycle = duty_cycle;
//              mode = HDRIVER_cruise;
//            }
//          break;
//
//        default:
//        case HDRIVER_cruise:
//          currentDutyCycle = duty_cycle;
//          break;
//        }
//      ret = setPWM (currentDutyCycle);

      if (mode == HDRIVER_stopped)
        {
          mode = HDRIVER_cruise;
          if (reverseWiring)
            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
          else
            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_RESET);
          HAL_TIM_PWM_Start (m_Pwm->htim, m_Pwm->timChannel);
          m_forward = true;
          m_reverse = false;
        }
      ret = setPWM (dutyCycle);
    }
  return ret;
}

// backwards SPI or default PWM mode
HDriver_Result_t IFX9201::reverse (uint8_t dutyCycle)
{
  HDriver_Result_t ret = HDRIVER_Result_Ok;

  if (m_Mode == IFX9201Mode_SPI)
    ret = setCTRLReg (IFX9201__CTRL_SIN_MSK | IFX9201__CTRL_SEN_MSK | IFX9201__CTRL_SPWM_MSK);
  else
    {
//      switch (mode)
//        {
//        case HDRIVER_stopped:
//          mode = HDRIVER_acceleration;
//          if (reverseWiring)
//            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_SET);
//          else
//            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
//          currentDutyCycle = 10;
//          HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_RESET);
//          HAL_TIM_PWM_Start (m_Pwm->htim, m_Pwm->timChannel);
//          m_forward = false;
//          m_reverse = true;
//          break;
//
//        case HDRIVER_acceleration:
//          currentDutyCycle += 2;
//          if (currentDutyCycle >= duty_cycle)
//            {
//              currentDutyCycle = duty_cycle;
//              mode = HDRIVER_cruise;
//            }
//          break;
//
//        default:
//        case HDRIVER_cruise:
//          currentDutyCycle = duty_cycle;
//          break;
//        }
//      ret = setPWM (currentDutyCycle);

      if (mode == HDRIVER_stopped)
        {
          mode = HDRIVER_cruise;
          if (reverseWiring)
            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_SET);
          else
            HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_RESET);
          HAL_TIM_PWM_Start (m_Pwm->htim, m_Pwm->timChannel);
          m_reverse = true;
          m_forward = false;
        }
      ret = setPWM (dutyCycle);
    }
  return ret;
}

//// forwards PWM mode ONLY
//uint8_t IFX9201::forwards (uint8_t duty_cycle)
//{
//  uint8_t ret = IFX9201__NO_ERROR;
//
//  if (m_Mode == IFX9201Mode_SPI)
//    ret = IFX9201__ILLEGAL_OPERATION_ERROR;
//  else
//    {
//
//    }
//  return ret;
//}

//// backwards PWM mode ONLY
//uint8_t IFX9201::backwards (uint8_t duty_cycle)
//{
//  uint8_t ret = IFX9201__NO_ERROR;
//
//  if (m_Mode == IFX9201Mode_SPI)
//    ret = IFX9201__ILLEGAL_OPERATION_ERROR;
//  else
//    {
//      //digitalWrite (m_Direction, LOW);
//      HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
//      ret = setPWM (duty_cycle);
//      // digitalWrite (m_Disable, LOW);        // Disable OFF - MOSFET drivers ON
//      HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_RESET);
//    }
//  return ret;
//}

/**
 * \brief  Stop both modes
 *
 * \todo voir si il faut desactiver le timer et placer la broche en GPIO
 */
HDriver_Result_t IFX9201::brake (void)
{
  HDriver_Result_t ret = HDRIVER_Result_Ok;

  if (m_Mode == IFX9201Mode_SPI)
    ret = setCTRLReg (IFX9201__CTRL_SIN_MSK);
  else
    {
      // on laisse le dernier sens commandé
      //digitalWrite (m_Direction, LOW);
      //HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
      ret = setPWM (0);
      //digitalWrite (m_Disable, HIGH);        // Disable ON - MOSFET drivers OFF
      HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop (m_Pwm->htim, m_Pwm->timChannel);
      m_forward = false;
      m_reverse = false;
      mode = HDRIVER_stopped;
    }
  return ret;
}

/**
 * \brief  Stop both modes, disabled, output tri states
 *
 * \todo voir si il faut desactiver le timer et placer la broche en GPIO
 */
HDriver_Result_t IFX9201::freeSpin ()
{
  HDriver_Result_t ret = HDRIVER_Result_Ok;

  if (m_Mode == IFX9201Mode_SPI)
    ret = setCTRLReg (IFX9201__CTRL_SIN_MSK);
  else
    {
      // on laisse le dernier sens commandé
      //digitalWrite (m_Direction, LOW);
      //HAL_GPIO_WritePin (m_Direction->port, m_Direction->pin, GPIO_PIN_RESET);
      ret = setPWM (0);
      //digitalWrite (m_Disable, HIGH);        // Disable ON - MOSFET drivers OFF
      HAL_GPIO_WritePin (m_Disable->port, m_Disable->pin, GPIO_PIN_SET);
      HAL_TIM_PWM_Stop (m_Pwm->htim, m_Pwm->timChannel);
      mode = HDRIVER_stopped;
    }
  return ret;
}

///**
// * \todo a faire
// */
//void IFX9201::end ()
//{
////  pinMode (m_Disable, INPUT);
////  pinMode (m_SlaveSelect, INPUT);
////  pinMode (m_Direction, INPUT);
////  pinMode (m_PWM, INPUT);
//}

uint8_t IFX9201::getDIAReg ()
{
  SPItransfer (IFX9201__RD_DIA);
  return SPItransfer (IFX9201__RD_DIA);
}

uint8_t IFX9201::getREVReg ()
{
  SPItransfer (IFX9201__RD_REV);
  return SPItransfer (IFX9201__RD_REV);
}

uint8_t IFX9201::getCTRLReg ()
{
  SPItransfer (IFX9201__RD_CTRL);
  return SPItransfer (IFX9201__RD_CTRL);
}

HDriver_Result_t IFX9201::setCTRLReg (uint8_t data_out)
{
  HDriver_Result_t ret = HDRIVER_Result_Ok;

  if (m_Mode == IFX9201Mode_SPI)
    {
      SPItransfer (IFX9201__WR_CTRL | (IFX9201__CTRL_DATA_MSK & data_out));
      if ((SPItransfer (IFX9201__RD_CTRL) & IFX9201__CTRL_DATA_MSK) != (data_out & IFX9201__CTRL_DATA_MSK))
        ret = HDRIVER_Result_CommunicationProblem;
    }
  else
    ret = HDRIVER_Result_Mode_Error;

  return ret;
}

void IFX9201::resetDIAReg ()
{
  if (m_Mode == IFX9201Mode_SPI)
    SPItransfer (IFX9201__RES_DIA);
}

uint8_t IFX9201::SPItransfer (uint8_t data_out)
{
  uint8_t data_in = 0x00u;

  // m_bus->beginTransaction (m_SPIsettings);

  //digitalWrite (m_SlaveSelect, LOW);
  HAL_GPIO_WritePin (m_SlaveSelect->port, m_SlaveSelect->pin, GPIO_PIN_RESET);

//  data_in = m_bus->transfer (data_out);

  //digitalWrite (m_SlaveSelect, HIGH);
  HAL_GPIO_WritePin (m_SlaveSelect->port, m_SlaveSelect->pin, GPIO_PIN_SET);

  return data_in;
}

//
HDriver_Result_t IFX9201::setPWM (uint8_t dutyCycle)
{
  HDriver_Result_t ret = HDRIVER_Result_Ok;
  int32_t val;

  if (dutyCycle > IFX9201__MAX_DUTY_CYCLE)
    {
      ret = HDRIVER_Result_Illegal_DutyCycle;
    }
  else
    {
      if (dutyCycle == 0)
        {
          //pwmActif = false;
          //HAL_TIM_PWM_Stop (m_Pwm->htim, m_Pwm->timChannel);
          __HAL_TIM_SET_COMPARE(m_Pwm->htim, m_Pwm->timChannel, 0);
        }
      else if (dutyCycle == 100)
        {
          __HAL_TIM_SET_COMPARE(m_Pwm->htim, m_Pwm->timChannel, maximumPwmValue + 1);
        }
      else
        {
          //  if (pwmActif == false)
          //    {
          //     pwmActif = true;
          //     HAL_TIM_PWM_Start (m_Pwm->htim, m_Pwm->timChannel);
          //   }

// cas normal (PWM actif, change seulement rapport cyclique
          val = ((int32_t) dutyCycle * maximumPwmValue) / 100lu;
          __HAL_TIM_SET_COMPARE(m_Pwm->htim, m_Pwm->timChannel, val);
        }
    }
  return ret;
}

//// SPI Mode begin
//void IFX9201::begin (void)
//{
//  //m_Mode = IFX9201Mode_SPI;
//  //m_bus = &bus;
//  //m_hspi = hspi;
//
//  // Inutile, cree par cubeMX
//  //m_bus->begin ();
//  //m_bus->setBitOrder (MSBFIRST);
//  //m_bus->setClockDivider (SPI_CLOCK_DIV16);
//  //m_bus->setDataMode (SPI_MODE1);
//
////  m_SlaveSelect = pinSlaveSelect;
////  m_Disable = pinDisable;
////  m_Direction = pinDirection;
////  m_Pwm = pwm;
//
//  //begin ();
//
//  setCTRLReg (IFX9201__CTRL_SIN_MSK);
//}

//// PWM Mode begin
//void IFX9201::begin (MPWM_HandleTypeDef *pwm, MGPIO_HandleTypeDef *pinDirection, MGPIO_HandleTypeDef *pinDisable)
//{
//  m_Mode = IFX9201Mode_PWM;
//
//  m_Disable = pinDisable;
//  m_Direction = pinDirection;
//  m_Pwm = pwm;
//
//  begin ();
//}

//// Changed to return errors for invalid frequencies or PWM pin
//uint8_t IFX9201::setPWMFrequency (uint16_t freq)
//{
//  uint8_t ret = IFX9201__NO_ERROR;
//  int16_t val;
//
//  if (freq <= IFX9201__MAX_PWM_FREQ && freq > 0)
//    {
//      m_PWMFreq = freq;
//      //val = setAnalogWriteFrequency (m_PWM, m_PWMFreq);
//      //! \todo a ecrire
//      val = 0;
//
//      if (val == -1)
//        ret = IFX9201__ILLEGAL_FREQUENCY;
//      else if (val == -2)
//        ret = IFX9201__ILLEGAL_PWM_PIN;
//    }
//  else
//    ret = IFX9201__ILLEGAL_FREQUENCY;
//  return ret;
//}

