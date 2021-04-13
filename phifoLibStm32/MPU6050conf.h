/*
 * mpu6050_conf.h
 *
 *  Created on: 29 mai 2020
 *      Author: phifo
 */

#ifndef MPU6050_CONF_H_
#define MPU6050_CONF_H_


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




#endif /* MPU6050_CONF_H_ */
