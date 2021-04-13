/*
 * application.cpp
 *
 *  Created on: Jun 19, 2020
 *      Author: phifo
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <array>

#include "main.h"
#include "application.h"

#include "Led.h"
#include "Serial.h"
#include "MEMS.h"
#include "MPU6050.h"
#include "HDriver.h"
#include "ISensor.h"
#include "IFX9201.h"
#include "DcMotor.h"
#include "AnalogInput.h"
#include "Corrector.h"
#include "PID.h"
#include "ServoMotor.h"

#include "communication.h"
#include "motion_fx.h"

// Voies analogiques converties par l'ADC et enregistrées par le DMA
volatile uint16_t rawAdcValuesForDMA[20];

#define ANALOG_AVERAGE_DIVIDER   7
#define ANALOG_TABLE_SIZE   (128)

//volatile uint16_t rawAdcValuesTemperature[ANALOG_TABLE_SIZE];
//volatile uint16_t averageRawTemperature;
//volatile uint16_t lastRawTemperature;
//volatile int rawAdcTableIndex = 0;

AnalogInput temperature;

// Configuration des capteurs de courant
#define RAW_TO_VOLTS    (3.3F/4096.0F)
#define ACS712_SENSITIVITY_AMP (0.1) // ACS712-20
//#define ACS712_SENSITIVITY_MILLIAMP 0.0001 // ACS712-20
CurrentSensor currentA (RAW_TO_VOLTS, 1.63F, ACS712_SENSITIVITY_AMP, 5.1);
CurrentSensor currentB (RAW_TO_VOLTS, 1.62F, ACS712_SENSITIVITY_AMP, 2.1);

// Définitions des broches pour les entrées/sorties "standards"
const MGPIO_HandleTypeDef LED_PIN = { LED_GPIO_Port, LED_Pin };
Led led (&LED_PIN, REVERSE_LOGIC);

// Définition des broches pour la liaison série et création de l'objet
MGPIO_HandleTypeDef DIR_RS485 = { DIR_485_GPIO_Port, DIR_485_Pin };
Serial serial (&huart2, &DIR_RS485);

// Création et configuration de l'objet MEMS
volatile uint8_t i2cDmaReceiveBuffer[20];
MPU6050 imu (&hi2c1, MPU6050_Device_0, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s);

// Définitions des broches et des timers pour le moteur du verin A
const MGPIO_HandleTypeDef MOTOR_A_DIR = { DIR_A_GPIO_Port, DIR_A_Pin };
const MGPIO_HandleTypeDef MOTOR_A_DIS = { DIS_A_GPIO_Port, DIS_A_Pin };
const MGPIO_HandleTypeDef MOTOR_A_CSN = { CSN_A_GPIO_Port, CSN_A_Pin };
MPWM_HandleTypeDef PWM_MOTOR_A = { PWM_A_GPIO_Port, PWM_A_Pin, &htim1, TIM_CHANNEL_1 };
IFX9201 H_DRIVER_MOTOR_A (&hspi2, &MOTOR_A_CSN, &PWM_MOTOR_A, &MOTOR_A_DIR, &MOTOR_A_DIS);
DcMotor motorA (&PWM_MOTOR_A, &H_DRIVER_MOTOR_A);

// Définitions des retours de position et du PID pour le vérin A
volatile int32_t ilsValueA = 0;
AnalogInput analogPositionA;
PID correctorA;

// Création de l'objet verin A
ServoMotor cylinderA (&motorA, &correctorA, &ilsValueA, &analogPositionA, &currentA);

const MGPIO_HandleTypeDef MOTOR_B_DIR = { DIR_B_GPIO_Port, DIR_B_Pin };
const MGPIO_HandleTypeDef MOTOR_B_DIS = { DIS_B_GPIO_Port, DIS_B_Pin };
const MGPIO_HandleTypeDef MOTOR_B_CSN = { CSN_B_GPIO_Port, CSN_B_Pin };
MPWM_HandleTypeDef PWM_MOTOR_B = { PWM_B_GPIO_Port, PWM_B_Pin, &htim5, TIM_CHANNEL_1 };
IFX9201 H_DRIVER_MOTOR_B (&hspi2, &MOTOR_B_CSN, &PWM_MOTOR_B, &MOTOR_B_DIR, &MOTOR_B_DIS);
DcMotor motorB (&PWM_MOTOR_B, &H_DRIVER_MOTOR_B);
PID correctorB;
volatile int32_t ilsValueB = 0;
AnalogInput analogPositionB;

ServoMotor cylinderB (&motorB, &correctorB, &ilsValueB, &analogPositionB, &currentB);

uint8_t boardAddress = 4; //!<
MFX_input_t data_in;
MFX_output_t data_out;
int32_t inclinaison_X, inclinaison_Y;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*!
 * Convert ADC raw value to temperature in °C
 *
 * From reference manual RM0401 Rev3 p227
 *
 */
#define TEMPERATURE_SENSOR_V25       0.76  //!< Table72 - Datasheet p114 - DoclD028094 (dec 2017)
#define TEMPERATURE_SENSOR_AVG_SLOPE 0.0025 //!< Table72 - Datasheet p114 - DoclD028094 (dec 2017)
int16_t readTemperature (uint16_t raw)
{
  int16_t temperature = (int16_t) (((((float) raw) * RAW_TO_VOLTS - TEMPERATURE_SENSOR_V25)
      / TEMPERATURE_SENSOR_AVG_SLOPE) + 25.0F);

  //uint16_t *TS_CAL1ptr = (uint16_t*) 0x1FFF7A2C;
  //uint16_t *TS_CAL2ptr = (uint16_t*) 0x1FFF7A2E;
  //sprintf (msgTmp, "temperature = %d, TS_CAL1 = %d, TS_CAL2 = %d\r\n", temperature, *TS_CAL1ptr, *TS_CAL2ptr);
  //serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));

  return temperature;
}

/*! \brief Lecture des dipSwitchs de la carte et affectation de la variable globale contenant l'adresse
 *
 * Note : sur le schéma de la carte, les adresses ne correspondent pas à l'orientation du bloc
 * d'interrupteur. D'où une adaptation dans cette fonction.
 *
 * \todo : inverser les noms sur le schéma ou souder le bloc d'interrupteur dans l'autre sens, et corriger
 * cette fonction
 */
void readBoardAddress (void)
{
  uint8_t adresse;

  adresse = (HAL_GPIO_ReadPin (ADD_0_GPIO_Port, ADD_0_Pin) == GPIO_PIN_SET) ? 0 : 64;
  adresse += (HAL_GPIO_ReadPin (ADD_1_GPIO_Port, ADD_1_Pin) == GPIO_PIN_SET) ? 0 : 32;
  adresse += (HAL_GPIO_ReadPin (ADD_2_GPIO_Port, ADD_2_Pin) == GPIO_PIN_SET) ? 0 : 16;
  adresse += (HAL_GPIO_ReadPin (ADD_3_GPIO_Port, ADD_3_Pin) == GPIO_PIN_SET) ? 0 : 8;
  adresse += (HAL_GPIO_ReadPin (ADD_4_GPIO_Port, ADD_4_Pin) == GPIO_PIN_SET) ? 0 : 4;

  boardAddress = adresse;
}

/*!
 * \brief Interruption timer
 *
 *      Gestion de la base de temps de la boucle principale
 *      Déclenchement d'une lecture des mesures de l'IMU
 */
volatile uint16_t timer11Extension = 0;
volatile bool flagTimeBaseElapsed = true; //<! Flag for main loop time base
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if (htim == &htim11)
    {
      flagTimeBaseElapsed = true;
      timer11Extension++;
    }
//  else if (htim == &htim11)
//    {
//      memsUnit.requestData ();
//    }
}

/*!
 * \brief fonction d'interruption de fin de reception I2C en DMA
 *
 * a voir : USE_HAL_I2C_REGISTER_CALLBACKS
 *
 */
void HAL_I2C_MemRxCpltCallback (I2C_HandleTypeDef *hi2c)
{
  MPU6050::endOfDMAtransfertCallback ();
}

/*!
 *  \brief callback de fin de conversion ADC (Buffer DMA Plein)
 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
  currentA.setNewValue (rawAdcValuesForDMA[0]);
  currentB.setNewValue (rawAdcValuesForDMA[1]);
  analogPositionA.storeNewValue (rawAdcValuesForDMA[2]);
  analogPositionB.storeNewValue (rawAdcValuesForDMA[3]);

  temperature.storeNewValue (rawAdcValuesForDMA[4]);
  //lastRawTemperature = rawAdcValuesForDMA[4];
  //rawAdcValuesTemperature[rawAdcTableIndex] = rawAdcValuesForDMA[4];
  //rawAdcTableIndex = (rawAdcTableIndex + 1) % ANALOG_TABLE_SIZE;
}

void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef *hadc)
{
  currentA.setNewValue (rawAdcValuesForDMA[0]);
  currentB.setNewValue (rawAdcValuesForDMA[1]);
  analogPositionA.storeNewValue (rawAdcValuesForDMA[2]);
  analogPositionB.storeNewValue (rawAdcValuesForDMA[3]);
  temperature.storeNewValue (rawAdcValuesForDMA[4]);

//  lastRawTemperature = rawAdcValuesForDMA[4];
//  rawAdcValuesTemperature[rawAdcTableIndex] = rawAdcValuesForDMA[4];
//  rawAdcTableIndex = (rawAdcTableIndex + 1) % ANALOG_TABLE_SIZE;
}

/*!
 * \brief Interruption liés aux ILS des vérins
 */
uint32_t ilsSpeedMeasurementA = 0;
uint32_t ilsSpeedMeasurementB = 0;
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  word32Byte_t newVal;
  newVal.h = timer11Extension;
  newVal.l = htim11.Instance->CNT;

  switch (GPIO_Pin)
    {
    case ILS_A_Pin:
      static uint32_t lastValA = 0;
      //if (motorA.getCurrentMove () == DcMotor::DCMOTOR_MOVE_FORWARD)
      //if (HAL_GPIO_ReadPin (DIR_A_GPIO_Port, DIR_A_Pin) == GPIO_PIN_RESET)
      //{
      //}
      //else
      //  {
      //    ilsValueA--;
      //  }
      ilsValueA = ilsValueA + cylinderA.getSensorIncrement ();
      ilsSpeedMeasurementA = newVal.val - lastValA;
      lastValA = newVal.val;
      break;

    case ILS_B_Pin:
      static uint32_t lastValB = 0;
      ilsValueB = ilsValueB + cylinderB.getSensorIncrement ();
      //if (motorB.getCurrentMove () == DcMotor::DCMOTOR_MOVE_FORWARD)
      //if (HAL_GPIO_ReadPin (DIR_B_GPIO_Port, DIR_B_Pin) == GPIO_PIN_RESET)
//        {
//          ilsValueB++;
//        }
//      else
//        {
//          ilsValueB--;
//        }
      ilsSpeedMeasurementB = newVal.val - lastValB;
      lastValB = newVal.val;
      break;

    default:
      break;
    }
}

///*!
// *
// *
// */
//uint16_t getAverageTemperature (void)
//{
//  uint32_t average = 0;
//  for (int i = 0; i < 16; i++)
//    {
//      average += rawAdcValuesTemperature[i];
//    }
//  return (uint16_t) (average >> ANALOG_AVERAGE_DIVIDER);
//}

/*! \brief Cylinder periodic management = asservissement
 *
 * selon l'adresse, soit 2 vérins indépendants, soit 2 vérins asservis en parallèle
 */
void cylindersPeriodicManagement (uint8_t address)
{
  if ((address & PHOENIX_PARALLEL_MASK) != 0)
    {
      // voir schéma de l'asservissement dans la doc
      correctorFloat_t parallelFactor;
      correctorFloat_t parallelFactorCoefficient = 0.5;
      parallelFactor = parallelFactorCoefficient * (cylinderA.getError () - cylinderB.getError ());
      cylinderA.setConsigne (cylinderA.getConsigne () - parallelFactor);
      cylinderB.setConsigne (cylinderB.getConsigne () + parallelFactor);
    }

  cylinderA.periodicManagement ();
  cylinderB.periodicManagement ();
}

/*!
 *
 */
void imuPeriodicManagement (float dT)
{
  // Get acceleration X/Y/Z in g
  data_in.acc[0] = imu.getAccelerometer_X ();
  data_in.acc[1] = imu.getAccelerometer_Y ();
  data_in.acc[2] = imu.getAccelerometer_Z ();

  // Get angular rate X/Y/Z in dps
  data_in.gyro[0] = imu.getGyroscope_X ();
  data_in.gyro[1] = imu.getGyroscope_Y ();
  data_in.gyro[2] = imu.getGyroscope_Z ();

  // Run Sensor Fusion algorithm
  MotionFX_propagate (&data_out, &data_in, &dT);
  MotionFX_update (&data_out, &data_in, &dT, NULL);
  inclinaison_X = (int32_t) (data_out.gravity_6X[0] * 16384.0);
  inclinaison_Y = (int32_t) (data_out.gravity_6X[1] * 16384.0);
  imu.requestData ();  // Demande de lecture des valeurs brutes retard d'un cycle ?
}

/*!
 *
 *
 */
void applicationSetup (void)
{
  // initialisation de la led
  led.on ();

  // Initialisation de la liaison série
  serial.initialisation ();
#ifdef SERIAL_DEBUG_MESSAGES
  serial.sendBuffer (msgStart);
#endif

  // Configuration et initialisation et  de l'IMU
  // Initialisation de la liaison I2C inutile car faite par cubeMX dans le main.c
  MEMS_Result_t status = imu.initialisation ();
  if (status != MEMS_Result_Ok)
    {
      while (1)
        {
          HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
          HAL_Delay (50);
        }
    }
#ifdef SERIAL_DEBUG_MESSAGES
  else
    {
      serial.sendBuffer (msgImuOk);
    }
#endif

  // Initialisation Sensor Fusion algorithm
    {
#ifdef SERIAL_DEBUG_MESSAGES
#define MFX_STR_LENG 35
      char lib_version[MFX_STR_LENG];
#endif
      MFX_knobs_t iKnobs;

      MotionFX_initialize (); // Sensor Fusion API initialization function
#ifdef SERIAL_DEBUG_MESSAGES
      MotionFX_GetLibVersion (lib_version); // Optional: Get version
#endif
      MotionFX_getKnobs (&iKnobs);

      /* Modify knobs settings & set the knobs */
      MotionFX_setKnobs (&iKnobs);
      MotionFX_enable_6X (MFX_ENGINE_ENABLE);

#ifdef SERIAL_DEBUG_MESSAGES
      HAL_Delay (50);
      serial.sendBuffer (lib_version);
      HAL_Delay (50);
      serial.sendBuffer (msgLibMotionOk);
#endif
    }

  // Initialisation des mesures analogiques
  HAL_ADC_Start_DMA (&hadc1, (uint32_t*) rawAdcValuesForDMA, 16);
  //HAL_TIM_OC_Start (&htim1, TIM_CHANNEL_2);

  // Initialisation des interfaces liées aux vérins
  // Si verin avec capteur analogique alors attendre mesure correcte
  if ((boardAddress & PHOENIX_LA31_MASK) != 0)
    {
      HAL_Delay (250);
    }
  // Mise à 1 de la broche commune VSO (supprimer la mise en veille)
  HAL_GPIO_WritePin (VSO_GPIO_Port, VSO_Pin, GPIO_PIN_SET);

  motorA.initialisation ();
  cylinderA.initialisation (); // ! Attention l'ordre des 2 lignes suivantes a de l'importance
  correctorA.initialisation (1.0, 0.01);

  motorB.initialisation ();
  cylinderB.initialisation (); // ! Attention l'ordre des 2 lignes suivantes a de l'importance
  correctorB.initialisation (1.0, 0.01);

  // Initialisation de la base de temps et activation de l'it timer
  HAL_TIM_Base_Start_IT (&htim11);

}

void tests_integration (void);
/*!
 *
 *
 */
extern "C" void application (void)
{
  int compteur = 0;
  char msgTmp[100];

  tests_integration ();
  applicationSetup ();

  while (1)
    {
      // Gestion base de temps et tâches immédiates
      while (!flagTimeBaseElapsed)
        {
          if (serial.isDataReady ())
            {
              serialDecodeMessageSlave (serial.read (), boardAddress);
            }
        }
      flagTimeBaseElapsed = false;

      // Gestion des tâches périodiques
      led.periodicManagement (5, 100);
      readBoardAddress ();

      imuPeriodicManagement (0.01);

      if (++compteur % 100 == 0)
        {
          sprintf (msgTmp, "Compteur = %d, adresse = %d, X = %d, Y = %d", compteur, boardAddress,
                   (int16_t) (data_in.gyro[0] * 100.0F), (int16_t) (data_in.gyro[1] * 100.0F));
          serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));
        }
    }
}

// end of file

