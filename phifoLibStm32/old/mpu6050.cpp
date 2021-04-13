/**
 *
 *
 */
#define _USE_MATH_DEFINES

#include <math.h>

#include "main.h"
#include "mems.h"
#include "mpu6050.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR            0xD0

/* Who I am register value */
#define MPU6050_I_AM                0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO           0x01

#define MPU6050_SELF_TEST_X         0x0D
#define MPU6050_SELF_TEST_Y         0x0E
#define MPU6050_SELF_TEST_Z         0x0F
#define MPU6050_SELF_TEST_A         0x10

#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_MOTION_THRESH       0x1F
#define MPU6050_FIFO_EN             0x23
#define MPU6050_I2C_MST_CTRL        0x24
#define MPU6050_INT_PIN_CFG         0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_DMP_INT_STATUS      0x39
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_MOT_DETECT_STATUS   0x61
#define MPU6050_SIGNAL_PATH_RESET   0x68
#define MPU6050_MOT_DETECT_CTRL     0x69
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_FIFO_COUNTH         0x72
#define MPU6050_FIFO_COUNTL         0x73
#define MPU6050_FIFO_R_W            0x74
#define MPU6050_WHO_AM_I            0x75

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250       ((float) 131.0)
#define MPU6050_GYRO_SENS_500       ((float) 65.5)
#define MPU6050_GYRO_SENS_1000      ((float) 32.8)
#define MPU6050_GYRO_SENS_2000      ((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2         ((float) 16384.0)
#define MPU6050_ACCE_SENS_4         ((float) 8192.0)
#define MPU6050_ACCE_SENS_8         ((float) 4096.0)
#define MPU6050_ACCE_SENS_16        ((float) 2048.0)

#define MPU6050_CLOCK_PLL_INT8MHZ   0x00
#define MPU6050_CLOCK_PLL_XGYRO     0x01
#define MPU6050_CLOCK_PLL_YGYRO     0x02
#define MPU6050_CLOCK_PLL_ZGYRO     0x02
#define MPU6050_CLOCK_PLL_EXT32K    0x04
#define MPU6050_CLOCK_PLL_EXT19_2   0x05

#define I2C_TIMEOUT_FLAG          35U

MPU6050 *MPU6050::mySingleton = nullptr;

//extern uint16_t SWV_espion_t;
//extern uint16_t SWV_espion_x;
//extern uint16_t SWV_espion_y;
//extern uint16_t SWV_espion_z;

/**
 * \brief
 *
 *
 */
HAL_StatusTypeDef MPU6050::writeByte (uint8_t reg, uint8_t byte)
{
  uint8_t temp[4];
  temp[0] = reg;
  temp[1] = byte;

  i2cStatus = HAL_I2C_Master_Transmit (hi2c, deviceAddress, temp, 2, 500);
  return i2cStatus;
}

/**
 * \brief
 *
 *
 */
uint8_t MPU6050::readByte (uint8_t reg)
{
  uint8_t temp[4];
  temp[0] = reg;

  i2cStatus = HAL_I2C_Mem_Read (hi2c, deviceAddress, reg, I2C_MEMADD_SIZE_8BIT, temp, 1, 500);
  return temp[0];
}

/**
 * \brief
 *
 */
HAL_StatusTypeDef MPU6050::readBytes (uint8_t reg, uint8_t n, uint8_t *buffer)
{
  i2cStatus = HAL_I2C_Mem_Read (hi2c, deviceAddress, reg, I2C_MEMADD_SIZE_8BIT, buffer, n, 500);
  return i2cStatus;
}

/**
 * \brief constructeur
 *
 * Initialisation des variables seulement : pas de création ni initialisation de la liaison I2C.
 *
 */
MPU6050::MPU6050 (I2C_HandleTypeDef *handle, MPU6050_Device_t deviceNumber, MPU6050_Accelerometer_Range_t accel,
                  MPU6050_Gyroscope_Range_t gyro)
{
  mySingleton = this;

  // Handle internal memorisation
  hi2c = handle;

  // Format I2C address
  deviceAddress = MPU6050_I2C_ADDR | (uint8_t) deviceNumber;

  // Configuration initialisation
  accelerometerSensitivity = accel;
  gyroscopeSensitivity = gyro;

  // Set sensitivities for multiplying gyro and accelerometer data
  switch (accelerometerSensitivity)
    {
    case MPU6050_Accelerometer_2G:
      accelerometer_multFactor = (float) 1.0 / MPU6050_ACCE_SENS_2;
      break;
    case MPU6050_Accelerometer_4G:
      accelerometer_multFactor = (float) 1.0 / MPU6050_ACCE_SENS_4;
      break;
    case MPU6050_Accelerometer_8G:
      accelerometer_multFactor = (float) 1.0 / MPU6050_ACCE_SENS_8;
      break;
    case MPU6050_Accelerometer_16G:
      accelerometer_multFactor = (float) 1.0 / MPU6050_ACCE_SENS_16;
    default:
      break;
    }

  switch (gyroscopeSensitivity)
    {
    case MPU6050_Gyroscope_250s:
      gyroscope_multFactor = (float) 1.0 / MPU6050_GYRO_SENS_250;
      break;
    case MPU6050_Gyroscope_500s:
      gyroscope_multFactor = (float) 1.0 / MPU6050_GYRO_SENS_500;
      break;
    case MPU6050_Gyroscope_1000s:
      gyroscope_multFactor = (float) 1.0 / MPU6050_GYRO_SENS_1000;
      break;
    case MPU6050_Gyroscope_2000s:
      gyroscope_multFactor = (float) 1.0 / MPU6050_GYRO_SENS_2000;
    default:
      break;
    }
}

MEMS_Result_t MPU6050::initialisation (void)
{
  uint8_t t;
  uint8_t temp[10];
  uint16_t timeout = 250;

  // Initialize I2C - inutile, realisé par CubeMx
  // I2C_Init(MPU6050_I2C, MPU6050_I2C_PINSPACK, MPU6050_I2C_CLOCK);

  // Check if device is connected
  if (HAL_I2C_IsDeviceReady (hi2c, deviceAddress, 2, I2C_TIMEOUT_FLAG) != HAL_OK)
    {
      GPIO_InitTypeDef GPIO_InitStruct =
        { 0 };

      HAL_I2C_MspDeInit (hi2c);

      GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init (GPIOB, &GPIO_InitStruct);

      HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      for (int i = 0; i < 16; i++)
        {
          HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
          HAL_Delay (2);
          HAL_GPIO_WritePin (GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
          HAL_Delay (2);
        }

      HAL_GPIO_DeInit (GPIOB, GPIO_PIN_5);
      HAL_GPIO_DeInit (GPIOB, GPIO_PIN_6);


      hi2c->Init.ClockSpeed = 100000;
      hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
      hi2c->Init.OwnAddress1 = 0;
      hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
      hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
      hi2c->Init.OwnAddress2 = 0;
      hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
      hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

      HAL_I2C_Init (hi2c);
      if (HAL_I2C_IsDeviceReady (hi2c, deviceAddress, 2, I2C_TIMEOUT_FLAG) != HAL_OK)
        return MEMS_Result_DeviceNotConnected; // Return error
    }

  // Check who I am
  if ((HAL_I2C_Mem_Read (hi2c, deviceAddress, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, temp, 1, timeout)) != HAL_OK)
    {
      return MEMS_Result_CommunicationProblem; // Return error
    }
  if (temp[0] != MPU6050_I_AM)
    {
      return MEMS_Result_DeviceInvalid;
    }

  // Reset general
  // writeByte (MPU6050_USER_CTRL, 0x01); // 0x6A
  writeByte (MPU6050_PWR_MGMT_1, 0x80);
  HAL_Delay (100);

  // Wakeup MPU6050
  writeByte (MPU6050_PWR_MGMT_1, 0x0);

  // GYRO_RESET = ACCEL_RESET = TEMP_RESET
  writeByte (MPU6050_SIGNAL_PATH_RESET, 0x07);
  HAL_Delay (100);

//  temp[0] = MPU6050_PWR_MGMT_1;
//  if (HAL_I2C_Master_Transmit (hi2c, deviceAddress, temp, 0, timeout) != HAL_OK)
//    {
//      return MEMS_Result_ImpossibleWakeUp;
//    }

   writeByte (MPU6050_CONFIG, 0x03); // 0x1A //DLPF_CFG = 3: Fs=1khz; bandwidth=42hz
//  writeByte (MPU6050_CONFIG, 0x00); // 0x1A

  // 500Hz sample rate ~ 2ms
//  writeByte(MPU6050_SMPLRT_DIV, 0x01); //0x19
//  writeByte (MPU6050_SMPLRT_DIV, 0x00); //0x19

  // Config accelerometer
//  t = readByte (MPU6050_ACCEL_CONFIG);
//  t = (t & 0xE7) | (uint8_t) accelerometerSensitivity << 3;
//  writeByte (MPU6050_ACCEL_CONFIG, t);

  // Config gyroscope
//  t = readByte (MPU6050_GYRO_CONFIG);
//  t = (t & 0xE7) | (uint8_t) gyroscopeSensitivity << 3;
//  writeByte (MPU6050_GYRO_CONFIG, t);

  // Sets the clock source to use the X Gyro for reference, which is slightly better than
  // the default internal clock source.
  // Pas de reset, pas de sleep, pas de cycle
  writeByte (MPU6050_PWR_MGMT_1, 0x01); //0x6B pour clock source sur gyro X
//  writeByte (MPU6050_PWR_MGMT_1, 0x00); //0x6B

  /* Return OK */
  return MEMS_Result_Ok;
}

/**
 * \brief
 *
 *
 */
void MPU6050::requestData (void)
{
  // polling mode for test only
  // readAccelerometerPollingMode (&mySingleton->accelerometer_X, &mySingleton->accelerometer_Y, &mySingleton->accelerometer_Z);

  // Debut de communication I2C et lecture par DMA avec interruption de fin de reception
  HAL_I2C_Mem_Read_DMA (hi2c, deviceAddress, MPU6050_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*) i2cDmaReceiveBuffer,
                        14);
}

/**
 * \brief Fin de reception et stockage des résultats
 *
 *
 */
void MPU6050::endOfDMAtransfertCallback (void)
{
  mySingleton->accelerometer_X = (int16_t) (i2cDmaReceiveBuffer[0] << 8 | i2cDmaReceiveBuffer[1]);
  mySingleton->accelerometer_Y = (int16_t) (i2cDmaReceiveBuffer[2] << 8 | i2cDmaReceiveBuffer[3]);
  mySingleton->accelerometer_Z = (int16_t) (i2cDmaReceiveBuffer[4] << 8 | i2cDmaReceiveBuffer[5]);

  mySingleton->temperature = (int16_t) (i2cDmaReceiveBuffer[6] << 8 | i2cDmaReceiveBuffer[7]);

  mySingleton->gyroscope_X = (int16_t) (i2cDmaReceiveBuffer[8] << 8 | i2cDmaReceiveBuffer[9]);
  mySingleton->gyroscope_Y = (int16_t) (i2cDmaReceiveBuffer[10] << 8 | i2cDmaReceiveBuffer[11]);
  mySingleton->gyroscope_Z = (int16_t) (i2cDmaReceiveBuffer[12] << 8 | i2cDmaReceiveBuffer[13]);
}

/**
 * \brief
 *
 *
 */
MEMS_Result_t MPU6050::readAccelerometerPollingMode (int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t data[6];

  /* Read accelerometer data */
  // i2cStatus = HAL_I2C_Mem_Read (hi2c, deviceAddress, MPU6050_ACCEL_XOUT_H, 6, data, 6, 100);
  i2cStatus = readBytes (MPU6050_ACCEL_XOUT_H, 6, data);
  if (i2cStatus != HAL_OK)
    {
      return MEMS_Result_CommunicationProblem;
    }

  // Format raw data
  *x = accelerometer_X = (int16_t) (data[0] << 8 | data[1]);
  *y = accelerometer_Y = (int16_t) (data[2] << 8 | data[3]);
  *z = accelerometer_Z = (int16_t) (data[4] << 8 | data[5]);

  return MEMS_Result_Ok;
}

/**
 * \brief
 *
 *
 */
MEMS_Result_t MPU6050::readGyroscopesPollingMode (int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t data[6];

  /* Read gyroscope data */
  //i2cStatus = HAL_I2C_Mem_Read (hi2c, deviceAddress, MPU6050_GYRO_XOUT_H, 6, data, 6, 100);
  i2cStatus = readBytes (MPU6050_GYRO_XOUT_H, 6, data);
  if (i2cStatus != HAL_OK)
    {
      return MEMS_Result_CommunicationProblem;
    }

  // Format raw data
  *x = gyroscope_X = (int16_t) (data[0] << 8 | data[1]);
  *y = gyroscope_Y = (int16_t) (data[2] << 8 | data[3]);
  *z = gyroscope_Z = (int16_t) (data[4] << 8 | data[5]);

  return MEMS_Result_Ok;
}

//TM_MPU6050_Result_t TM_MPU6050_ReadAll(TM_MPU6050_t* DataStruct) {
//	uint8_t data[14];
//	int16_t temp;
//
//	/* Read full raw data, 14bytes */
//	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);
//
//	/* Format accelerometer data */
//	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
//	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
//	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
//
//	/* Format temperature */
//	temp = (data[6] << 8 | data[7]);
//	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
//
//	/* Format gyroscope data */
//	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
//	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
//	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);
//
//	/* Return OK */
//	return TM_MPU6050_Result_Ok;
//}

//MPU6050_Result_t MPU6050::readAllPollingMode (void)
//{
//  uint8_t rawData[14];
//  int16_t temp;
//
//  /* Read full raw data, 14bytes */
////    HAL_I2C_Master_Receive_DMA(hi2c, DevAddress, pData, Size)
//  while (HAL_I2C_Master_Receive_DMA (MPU6050_I2C, (uint16_t) DataStruct->Address, (uint8_t*) rawData, 14) != HAL_OK)
//    {
//      /* Error_Handler() function is called when Timout error occurs.
//       When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
//       Master restarts communication */
//      if (HAL_I2C_GetError (MPU6050_I2C) != HAL_I2C_ERROR_AF)
//        {
//          Error_Handler ();
//        }
//    }
//
//  /* Format accelerometer data */
//  DataStruct->Accelerometer_X = (int16_t) (rawData[0] << 8 | rawData[1]);
//  DataStruct->Accelerometer_Y = (int16_t) (rawData[2] << 8 | rawData[3]);
//  DataStruct->Accelerometer_Z = (int16_t) (rawData[4] << 8 | rawData[5]);
//
//  /* Format temperature */
//  temp = (rawData[6] << 8 | rawData[7]);
//  DataStruct->Temperature = (float) ((float) ((int16_t) temp) / (float) 340.0 + (float) 36.53);
//
//  /* Format gyroscope data */
//  DataStruct->Gyroscope_X = (int16_t) (rawData[8] << 8 | rawData[9]);
//  DataStruct->Gyroscope_Y = (int16_t) (rawData[10] << 8 | rawData[11]);
//  DataStruct->Gyroscope_Z = (int16_t) (rawData[12] << 8 | rawData[13]);
//
//  /* Return OK */
//  return TM_MPU6050_Result_Ok;
//}

//void I2C_DMA_Read (uint8_t slaveAddr, uint8_t readAddr, uint8_t sensor)
//{
//  /* Disable DMA channel*/
//  DMA_Cmd (MPU6050_DMA_Channel, DISABLE);
//  /* Set current data number again to 14 for MPu6050, only possible after disabling the DMA channel */
//  DMA_SetCurrDataCounter (MPU6050_DMA_Channel, 14);
//
//  /* While the bus is busy */
//  while (I2C_GetFlagStatus (MPU6050_I2C, I2C_FLAG_BUSY))
//    ;
//
//  /* Enable DMA NACK automatic generation */
//  I2C_DMALastTransferCmd (MPU6050_I2C, ENABLE);                    //Note this one, very important
//
//  /* Send START condition */
//  I2C_GenerateSTART (MPU6050_I2C, ENABLE);
//
//  /* Test on EV5 and clear it */
//  while (!I2C_CheckEvent (MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
//    ;
//
//  /* Send MPU6050 address for write */
//  I2C_Send7bitAddress (MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);
//
//  /* Test on EV6 and clear it */
//  while (!I2C_CheckEvent (MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//    ;
//
//  /* Clear EV6 by setting again the PE bit */
//  I2C_Cmd (MPU6050_I2C, ENABLE);
//
//  /* Send the MPU6050's internal address to write to */
//  I2C_SendData (MPU6050_I2C, readAddr);
//
//  /* Test on EV8 and clear it */
//  while (!I2C_CheckEvent (MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//    ;
//
//  /* Send STRAT condition a second time */
//  I2C_GenerateSTART (MPU6050_I2C, ENABLE);
//
//  /* Test on EV5 and clear it */
//  while (!I2C_CheckEvent (MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
//    ;
//
//  /* Send MPU6050 address for read */
//  I2C_Send7bitAddress (MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);
//
//  /* Test on EV6 and clear it */
//  while (!I2C_CheckEvent (MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//    ;
//
//  /* Start DMA to receive data from I2C */
//  DMA_Cmd (MPU6050_DMA_Channel, ENABLE);
//  I2C_DMACmd (MPU6050_I2C, ENABLE);
//
//  // When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
//  //now go back to the main routine
//}

//void EXTI4_IRQHandler (void)
//{
//  if (EXTI_GetITStatus (MPU6050_INT_Exti))            //MPU6050_INT
//    {
//      EXTI_ClearITPendingBit (MPU6050_INT_Exti);
//#ifndef USE_I2C_DMA
//      Prepare_Gyro_Data ();    //Read out the accel and gyro data whenever interrupt occurs.
//#else
//      I2C_DMA_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,MPU6050);
//    #endif
//    }
//}

//void DMA1_Channel7_IRQHandler (void)
//{
//  if (DMA_GetFlagStatus (DMA1_FLAG_TC7))
//    {
//      /* Clear transmission complete flag */
//      DMA_ClearFlag (DMA1_FLAG_TC7);
//
//      I2C_DMACmd (MPU6050_I2C, DISABLE);
//      /* Send I2C1 STOP Condition */
//      I2C_GenerateSTOP (MPU6050_I2C, ENABLE);
//      /* Disable DMA channel*/
//      DMA_Cmd (MPU6050_DMA_Channel, DISABLE);
//
//      //Read Accel data from byte 0 to byte 2
//      for (i = 0; i < 3; i++)
//        AccelGyro[i] = ((s16) ((u16) I2C_Rx_Buffer[2 * i] << 8) + I2C_Rx_Buffer[2 * i + 1]);
//      //Skip byte 3 of temperature data
//      //Read Gyro data from byte 4 to byte 6
//      for (i = 4; i < 7; i++)
//        AccelGyro[i - 1] = ((s16) ((u16) I2C_Rx_Buffer[2 * i] << 8) + I2C_Rx_Buffer[2 * i + 1]);
//    }
//}

//void dma_i2c_config (void) // ne sert pas "normalement"
//{
//  NVIC_InitTypeDef NVIC_InitStructure;
//  DMA_InitTypeDef DMA_InitStructure;
//
//  DMA_DeInit (MPU6050_DMA_Channel); //reset DMA1 channe1 to default values;
//
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C_DR_Address; //=0x40005410 : address of data reading register of I2C1
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) I2C_Rx_Buffer; //variable to store data
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for peripheral to memory transfer
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;    //setting normal mode (non circular)
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;    //medium priority
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;    //Location assigned to peripheral register will be source
//  DMA_InitStructure.DMA_BufferSize = 14;    //number of data to be transfered
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    //automatic memory increment enable for memory
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;    //source peripheral data size = 8bit
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;    //destination memory data size = 8bit
//  DMA_Init (MPU6050_DMA_Channel, &DMA_InitStructure);
//  DMA_ITConfig (MPU6050_DMA_Channel, DMA_IT_TC, ENABLE);
//
//  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn; //I2C1 connect to channel 7 of DMA1
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init (&NVIC_InitStructure);
//}
//
//float MPU6050::getAccelerometer_X (void) const
//{
//  float f;
//  f = ((float) accelerometer_X) * accelerometer_multFactor;
//  return f;
//}
//
//float MPU6050::getAccelerometer_Y (void) const
//{
//  float f;
//  f = ((float) accelerometer_Y) * accelerometer_multFactor;
//  return f;
//}
//
//float MPU6050::getAccelerometer_Z (void) const
//{
//  float f;
//  f = ((float) accelerometer_Z) * accelerometer_multFactor;
//  return f;
//}
//
//float MPU6050::getGyroscope_X (void) const
//{
//  float f;
//  f = ((float) gyroscope_X) * gyroscope_multFactor;
//  return f;
//}
//
//float MPU6050::getGyroscope_Y (void) const
//{
//  float f;
//  f = ((float) gyroscope_Y) * gyroscope_multFactor;
//  return f;
//}
//
//float MPU6050::getGyroscope_Z (void) const
//{
//  float f;
//  f = ((float) gyroscope_Z) * gyroscope_multFactor;
//  return f;
//}
//
float MPU6050::getTemperature (void) const
{
  float f;
  f = ((float) temperature) / (float) 340.0 + (float) 36.53;
  return f;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
//https://github.com/kriswiner/MPU6050/blob/master/STM32F401/MPU6050.h
MEMS_Result_t MPU6050::selfTest (float *destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[4] =
    { 0, 0, 0, 0 };
  uint8_t selfTest[6];
  float factoryTrim[6];
  MEMS_Result_t result = MEMS_Result_Ok;

  // Configure the accelerometer for self-test
  writeByte (MPU6050_ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
  writeByte (MPU6050_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  HAL_Delay (250); // Delay a while to let the device execute the self-test

  rawData[0] = readByte (MPU6050_SELF_TEST_X); // X-axis self-test results
  rawData[1] = readByte (MPU6050_SELF_TEST_Y); // Y-axis self-test results
  rawData[2] = readByte (MPU6050_SELF_TEST_Z); // Z-axis self-test results
  rawData[3] = readByte (MPU6050_SELF_TEST_A); // Mixed-axis self-test results

  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4; // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2; // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0; // ZA_TEST result is a five-bit unsigned integer

  // Extract the gyration test results first
  selfTest[3] = rawData[0] & 0x1F; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1] & 0x1F; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2] & 0x1F; // ZG_TEST result is a five-bit unsigned integer

  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0f * 0.34f) * (pow ((0.92f / 0.34f), (((float) selfTest[0] - 1.0f) / 30.0f))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (4096.0f * 0.34f) * (pow ((0.92f / 0.34f), (((float) selfTest[1] - 1.0f) / 30.0f))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (4096.0f * 0.34f) * (pow ((0.92f / 0.34f), (((float) selfTest[2] - 1.0f) / 30.0f))); // FT[Za] factory trim calculation
  factoryTrim[3] = (25.0f * 131.0f) * (pow (1.046f, ((float) selfTest[3] - 1.0f)));   // FT[Xg] factory trim calculation
  factoryTrim[4] = (-25.0f * 131.0f) * (pow (1.046f, ((float) selfTest[4] - 1.0f)));  // FT[Yg] factory trim calculation
  factoryTrim[5] = (25.0f * 131.0f) * (pow (1.046f, ((float) selfTest[5] - 1.0f)));   // FT[Zg] factory trim calculation

  //  Output self-test results and factory trim calculation if desired
  //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
  //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
  //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
  //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++)
    {
      destination[i] = 100.0f + 100.0f * (selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
      if (destination[i] > 14)
        result = MEMS_Result_SelftestProblem;
    }

  return result;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
//https://github.com/kriswiner/MPU6050/blob/master/STM32F401/MPU6050.h
void MPU6050::calibrate (float *dest1, float *dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] =
    { 0, 0, 0 }, accel_bias[3] =
    { 0, 0, 0 };

// reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte (MPU6050_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay (100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte (MPU6050_PWR_MGMT_1, 0x01);
  writeByte (MPU6050_PWR_MGMT_2, 0x00);
  HAL_Delay (200);

// Configure device for bias calculation
  writeByte (MPU6050_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte (MPU6050_FIFO_EN, 0x00);      // Disable FIFO
  writeByte (MPU6050_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte (MPU6050_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte (MPU6050_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte (MPU6050_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay (15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte (MPU6050_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte (MPU6050_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte (MPU6050_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte (MPU6050_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
  uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte (MPU6050_USER_CTRL, 0x40);   // Enable FIFO
  writeByte (MPU6050_FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  HAL_Delay (80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte (MPU6050_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes (MPU6050_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t) data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++)
    {
      int16_t accel_temp[3] =
        { 0, 0, 0 }, gyro_temp[3] =
        { 0, 0, 0 };
      readBytes (MPU6050_FIFO_R_W, 12, &data[0]); // read data for averaging
      accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
      accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
      accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
      gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
      gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
      gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

      accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
      accel_bias[1] += (int32_t) accel_temp[1];
      accel_bias[2] += (int32_t) accel_temp[2];
      gyro_bias[0] += (int32_t) gyro_temp[0];
      gyro_bias[1] += (int32_t) gyro_temp[1];
      gyro_bias[2] += (int32_t) gyro_temp[2];

    }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0] /= (int32_t) packet_count;
  gyro_bias[1] /= (int32_t) packet_count;
  gyro_bias[2] /= (int32_t) packet_count;

  if (accel_bias[2] > 0L)
    {
      accel_bias[2] -= (int32_t) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
  else
    {
      accel_bias[2] += (int32_t) accelsensitivity;
    }

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

// Push gyro biases to hardware registers
//  writeByte (deviceAddress, XG_OFFS_USRH, data[0]);
//  writeByte (deviceAddress, XG_OFFS_USRL, data[1]);
//  writeByte (deviceAddress, YG_OFFS_USRH, data[2]);
//  writeByte (deviceAddress, YG_OFFS_USRL, data[3]);
//  writeByte (deviceAddress, ZG_OFFS_USRH, data[4]);
//  writeByte (deviceAddress, ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] =
    { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases

//  readBytes (deviceAddress, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//  accel_bias_reg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];
//  readBytes (deviceAddress, YA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];
//  readBytes (deviceAddress, ZA_OFFSET_H, 2, &data[0]);
//  accel_bias_reg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] =
    { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++)
    {
      if (accel_bias_reg[ii] & mask)
        mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
  dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
  dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

//https://github.com/kriswiner/MPU6050/blob/master/STM32F401/MPU6050.h
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
float q[4] =
  { 1.0f, 0.0f, 0.0f, 0.0f };         //<! vector to hold quaternion

float GyroMeasError = M_PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt (3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = M_PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt (3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

void MadgwickQuaternionUpdate (float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz;
  float gbiasx = 0.0f, gbiasy = 0.0f, gbiasz = 0.0f;  // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
//            float _2q1q3 = 2.0f * q1 * q3;
//            float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt (ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt (hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  //           gx -= gbiasx;
  //           gy -= gbiasy;
  //           gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 - (beta * hatDot1)) * deltat;
  q2 += (qDot2 - (beta * hatDot2)) * deltat;
  q3 += (qDot3 - (beta * hatDot3)) * deltat;
  q4 += (qDot4 - (beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt (q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

