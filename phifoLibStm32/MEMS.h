/**
 * \brief Abstract class MEMS to interface for real IMU
 *
 * \file mems.h
 *
 *  Created on: 14 mai 2020
 *      Author: phifo
 */

#ifndef MEMS_H_
#define MEMS_H_

/**
 * \brief  MEMS result enumeration
 *
 */
typedef enum
{
  MEMS_Result_Ok = 0x00, /*!< Everything OK */
  MEMS_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
  MEMS_Result_DeviceInvalid, /*!< Connected device with address is not MPU6050 */
  MEMS_Result_ImpossibleWakeUp, /*!< Initialisation wakeup impossible */
  MEMS_Result_CommunicationProblem, /*!< */
  MEMS_Result_SelftestProblem /*!< */
} MEMS_Result_t;

/**
 * \brief  MEMS abstract class for general interface for MEMS IMU
 *
 */
class MEMS
{
public:

  /**
   * \brief MEMS physical initialisations with objet parameters
   *
   * Must be implemented in daughter class
   *
   * \retval status
   */
  virtual MEMS_Result_t initialisation (void) = 0;

  /**
   * \brief Request MEMS data
   *
   * Start hardware communication getting fresh physical value
   * Must be implemented in daughter class
   */
  virtual void requestData (void) = 0;

  /**
   * \brief Getter for accelerometer value in "g" or in "m.s-2"
   *
   * \return last read value
   */
  float getAccelerometer_X (void) const
  {
    return ((float) accelerometer_X) * accelerometer_multFactor;
  }

  /**
   * \brief Getter for accelerometer value in "g" or in "m.s-2"
   *
   * \return last read value
   */
  float getAccelerometer_Y (void) const
  {
    return ((float) accelerometer_Y) * accelerometer_multFactor;
  }

  /**
   * \brief Getter for accelerometer value in "g" or in "m.s-2"
   *
   * \return last read value
   */
  float getAccelerometer_Z (void) const
  {
    return ((float) accelerometer_Z) * accelerometer_multFactor;
  }

  /**
   * \brief Getter for gyroscope value in "dps" (Degrees Per Second)
   *
   * \return last read value
   */
  float getGyroscope_X (void) const
  {
    // return ((float) (gyroscope_X - gyroOffset_X)) * gyroscope_multFactor;
    return ((float) (gyroscope_X)) * gyroscope_multFactor;
  }
  /**
   * \brief Getter for gyroscope value in "dps" (Degrees Per Second)
   *
   * \return last read value
   */
  float getGyroscope_Y (void) const
  {
    // return ((float) (gyroscope_Y - gyroOffset_Y)) * gyroscope_multFactor;
    return ((float) (gyroscope_Y)) * gyroscope_multFactor;
  }
  /**
   * \brief Getter for gyroscope value in "dps" (Degrees Per Second)
   *
   * \return last read value
   */
  float getGyroscope_Z (void) const
  {
    // return ((float) (gyroscope_Z - gyroOffset_Z)) * gyroscope_multFactor;
    return ((float) (gyroscope_Z)) * gyroscope_multFactor;
  }

protected:
  float gyroscope_multFactor; /*!< Gyroscope corrector from raw data to "degrees/s" */
  float accelerometer_multFactor; /*!< Accelerometer corrector from raw data to "g" */

  int16_t accelerometer_X; /*!< Accelerometer raw value X axis */
  int16_t accelerometer_Y; /*!< Accelerometer raw value Y axis */
  int16_t accelerometer_Z; /*!< Accelerometer raw value Z axis */
  int16_t gyroscope_X; /*!< Gyroscope raw value X axis */
  int16_t gyroscope_Y; /*!< Gyroscope raw value Y axis */
  int16_t gyroscope_Z; /*!< Gyroscope raw value Z axis */

  int16_t accelOffset_X;
  int16_t accelOffset_Y;
  int16_t accelOffset_Z;

  int16_t gyroOffset_X;
  int16_t gyroOffset_Y;
  int16_t gyroOffset_Z;




};

#endif /* MEMS_H_ */
