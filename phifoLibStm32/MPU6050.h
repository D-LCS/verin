/**
 * \brief Driver pour le MPU6050 d'Invensense
 *
 * Hérite de la classe abstraite MEMS
 *
 *
 *
 *
 * ide     STM32CubeIde
 *
 * Code d'origine de Tilen Majerle
 * http://stm32f4-discovery.com/2014/10/library-43-mpu-6050-6-axes-gyro-accelerometer-stm32f4/
 * mais passage en objet avec héritage de l'objet MEMS
 *
 */
#ifndef __MPU6050_H__
#define __MPU6050_H__ 100

/* C++ detection */
#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * \brief MPU6050 AD0 configuration
   *
   * MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
   *
   * If you set AD0 pin to low, then this parameter should be MPU6050_Device_0,
   * but if AD0 pin is high, then you should use MPU6050_Device_1
   */
  typedef enum
  {
    MPU6050_Device_0 = 0, /*!< AD0 pin is set to low */
    MPU6050_Device_1 = 0x02 /*!< AD0 pin is set to high */
  } MPU6050_Device_t;

  /**
   * \brief  Parameters for accelerometer range
   */
  typedef enum
  {
    MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
    MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
    MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
    MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
  } MPU6050_Accelerometer_Range_t;

  /**
   * \brief  Parameters for gyroscope range
   */
  typedef enum
  {
    MPU6050_Gyroscope_250s = 0x00, /*!< Range is +- 250 degrees/s */
    MPU6050_Gyroscope_500s = 0x01, /*!< Range is +- 500 degrees/s */
    MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
    MPU6050_Gyroscope_2000s = 0x03 /*!< Range is +- 2000 degrees/s */
  } MPU6050_Gyroscope_Range_t;

  extern volatile uint8_t i2cDmaReceiveBuffer[20];

  /**
   * \brief  MPU6050
   */
  class MPU6050 : public MEMS
  {
  public:
    /**
     * \brief Constructeur
     *
     * \param handle : handle for initialized I2C communication bus
     * \param DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
     * \param AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref TM_MPU6050_Accelerometer_t enumeration
     * \param GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref TM_MPU6050_Gyroscope_t enumeration
     */
    MPU6050 (I2C_HandleTypeDef *handle, MPU6050_Device_t deviceNumber = MPU6050_Device_0,
             MPU6050_Accelerometer_Range_t accelerometerSensitivity = MPU6050_Accelerometer_16G,
             MPU6050_Gyroscope_Range_t gyroscopeSensitivity = MPU6050_Gyroscope_2000s);

    /**
     * \brief
     */
    MEMS_Result_t initialisation (void);

    /**
     * \brief
     */
    float getTemperature (void) const;

    /**
     * \brief
     */
    MEMS_Result_t selfTest (float *destination);

    /**
     * \brief
     */
    void requestData (void);
    static void endOfDMAtransfertCallback (void);

    void calibrate (float *dest1, float *dest2);

  private:
    uint8_t readByte (uint8_t reg); //<! Fonction d'interface avec le bus I2C
    HAL_StatusTypeDef readBytes (uint8_t reg, uint8_t n, uint8_t *buffer); //<! Fonction d'interface avec le bus I2C
    HAL_StatusTypeDef writeByte (uint8_t reg, uint8_t byte); //<! Fonction d'interface avec le bus I2C

    MEMS_Result_t readGyroscopesPollingMode (int16_t *x, int16_t *y, int16_t *z);
    MEMS_Result_t readAccelerometerPollingMode (int16_t *x, int16_t *y, int16_t *z);

    I2C_HandleTypeDef *hi2c;
    HAL_StatusTypeDef i2cStatus;

    uint8_t deviceAddress; /*!< I2C address of device */

    MPU6050_Accelerometer_Range_t accelerometerSensitivity;
    MPU6050_Gyroscope_Range_t gyroscopeSensitivity;

    int16_t temperature; /*!< Temperature in degrees */

    static MPU6050 *mySingleton; //!< Reference on self and control of one object only
  };

  void MadgwickQuaternionUpdate (float ax, float ay, float az, float gx, float gy, float gz, float deltat);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
