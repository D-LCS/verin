/*!
 *
 * \file IFX9201.h
 * \brief This file has to be included in projects that use Infineon's DC Motor Control Shield with IFX9201
 *
 *
 *
 * From IFX9201.h - Library for Arduino to control the IFX9201 H-Bridge.
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

/*! \todo separer le PWM du pont en H
 *
 */

#ifndef IFX9201_H
#define IFX9201_H

//! \brief  Class that represents an IFX9201 H-bridge
class IFX9201 : public HDriver
{
public:

  typedef enum IFX9201Mode
  {
    IFX9201Mode_PWM = 0x00u, IFX9201Mode_SPI = 0x01u
  } IFX9201Mode_t;

  //! \brief Enables and initializes the IFX9201 for use over SPI with custom control pins
  IFX9201 (SPI_HandleTypeDef *hspi, const MGPIO_HandleTypeDef *pinSlaveSelect, const MPWM_HandleTypeDef *pwm,
           const MGPIO_HandleTypeDef *pinDirection, const MGPIO_HandleTypeDef *pinDisable);

  //! \brief Enables and initializes the IFX9201 with PWM control pins
  IFX9201 (MPWM_HandleTypeDef *pwm, MGPIO_HandleTypeDef *pinDirection, MGPIO_HandleTypeDef *pinDisable);

  //! \brief Enables and initializes the IFX9201 with PWM control pins
  HDriver_Result_t initialisation (void);
  //void begin (void);
  //void begin (SPI_HandleTypeDef hspi, uint8_t pinSlaveSelect, uint8_t pinDirection, uint8_t pinPWM, uint8_t pinDisable);
  //void begin (MPWM_HandleTypeDef * pwm, MGPIO_HandleTypeDef * pinDirection,  MGPIO_HandleTypeDef * pinDisable);

  //! \brief Function to set PWM frequency for the IFX9201
  // uint8_t setPWMFrequency (uint16_t);

  /*! \brief Functions to rotate the motor controlled by IFX9201 forwards
   *  \param [in] dutyCycle : dutyCycle value restrained in [0..100]   (if dutyCycle = 0 or 100 ==> no PWM)
   */
  HDriver_Result_t forwards (uint8_t dutyCycle);
  //uint8_t forwards (); // without parameter ?

  /*! \brief Functions to rotate the motor controlled by IFX9201 reverse or backwards
   *  \param [in] dutyCycle : dutyCycle value restrained in [0..100]   (if dutyCycle = 0 or 100 ==> no PWM)
   */
  HDriver_Result_t reverse (uint8_t dutyCycle);
  inline HDriver_Result_t backwards (uint8_t dutyCycle)
  {
    return reverse (dutyCycle);
  }
  ; //alias
  //uint8_t backwards (); // without parameter ?

  void setWiringMode (HDriver_wiring_t w);

  /*! \brief Function stop the motor controlled by IFX9201 with short circuit on highSide power stage
   * Direction unchanged from the last PWM command
   *
   *  no param
   */
  HDriver_Result_t brake (void);

  /*! \brief Function deactivate IFX9201 power stage (disabled, outputs tristate)
   * Deactivates all outputs and disables the IFX9201
   *
   *  no param
   */
  HDriver_Result_t freeSpin (void);
  inline HDriver_Result_t end (void)
  {
    return freeSpin ();
  }
  ;  // alias

  void setAcceleration (uint8_t a);
  void setDeceleration (uint8_t d);

  inline bool isForwardMove (void)
  {
    return m_forward;
  }
  ;
  inline bool isReverseMove (void)
  {
    return m_reverse;
  }
  ;

  void diagnostic (void);

  //! \brief Get content of DIA Reg (only in SPI Mode)
  uint8_t getDIAReg ();
  //! \brief Get content of REV Reg (only in SPI Mode)
  uint8_t getREVReg ();
  //! \brief Get content of CTRL Reg (only in SPI Mode)
  uint8_t getCTRLReg ();

  //! \brief Set value of CTRL Reg (only in SPI Mode)
  HDriver_Result_t setCTRLReg (uint8_t);

  //! \brief Reset value of DIA Reg (only in SPI Mode)
  void resetDIAReg ();

private:
  IFX9201Mode_t m_Mode; //!<
  SPI_HandleTypeDef *m_bus; //!<

  const MGPIO_HandleTypeDef *m_SlaveSelect; //!<
  const MGPIO_HandleTypeDef *m_Disable; //!<
  const MGPIO_HandleTypeDef *m_Direction; //!<

  const MPWM_HandleTypeDef *m_Pwm; //!<
  uint16_t m_PWMFreq; //!<
  bool pwmActif; //!<
  bool m_forward; //!< information pour le capteur de vitesse/position
  bool m_reverse; //!< information pour le capteur de vitesse/position
  int32_t maximumPwmValue; //!<

  //! \brief Transfer a byte using SPI
  uint8_t SPItransfer (uint8_t);

  //! \brief Converts a duty cycle to suitable value for the HAL timers functions
  HDriver_Result_t setPWM (uint8_t);
};

#endif  // IFX9201_H

