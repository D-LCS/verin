// Classe abstraite d'interface pour les ponts en H

#ifndef __H_DRIVER__
#define __H_DRIVER__

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief  *
 */
typedef enum
{
  HDRIVER_Result_Ok = 0x00, /*!< Everything OK */
  HDRIVER_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
  HDRIVER_Result_CommunicationProblem, /*!< */
  HDRIVER_Result_SelftestProblem, /*!< */
  HDRIVER_Result_Mode_Error, /*!< */
  HDRIVER_Result_Illegal_operation, /*!< */
  HDRIVER_Result_Illegal_DutyCycle, /*!< */
  HDRIVER_Result_Illegal_DutyValue, /*!< */
  HDRIVER_Result_Not_Implemented_Yet /*!< */

} HDriver_Result_t;

/*!
 *
 */
typedef enum
{
  HDRIVER_Normal_Mode, HDRIVER_Flipping_Mode
} HDriver_wiring_t;

/*!
 *
 */
typedef enum
{
  HDRIVER_stopped, HDRIVER_acceleration, HDRIVER_cruise,
} HDriver_mode_t;

/*!
 * Abstact class for general interface for H driver bridge
 *
 */
class HDriver
{
public:

  /*!
   * Default constructor
   */
  HDriver (void)
  {
    reverseWiring = false;
  }
  ;

  /*!
   * Constructor
   */
  HDriver (HDriver_wiring_t mode)
  {
    reverseWiring = (mode == HDRIVER_Flipping_Mode) ? true : false;
  }
  ;

  virtual HDriver_Result_t initialisation (void) = 0;

  virtual HDriver_Result_t forwards (uint8_t f) = 0;
  virtual HDriver_Result_t reverse (uint8_t r) = 0;
  virtual HDriver_Result_t brake (void) = 0;
  virtual HDriver_Result_t freeSpin (void) = 0;

  virtual void setAcceleration (uint8_t a) = 0;
  virtual void setDeceleration (uint8_t d) = 0;

  virtual void diagnostic (void) = 0;

  virtual bool isForwardMove (void) = 0;
  virtual bool isReverseMove (void) = 0;

  virtual void setWiringMode (HDriver_wiring_t w) = 0;

  HDriver_mode_t mode = HDRIVER_stopped;
  uint8_t currentDutyCycle = 0;

protected:
  bool reverseWiring;

private:

};

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
