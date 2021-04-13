//======================================================================================================================
// Tests

/*!
 *  tests intégration et tests câblage
 *
 *
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

// Variable ou objets globaux externes (-la plupart du temps crées dans application.cpp
extern volatile bool flagTimeBaseElapsed;
extern uint8_t boardAddress;
extern CurrentSensor currentA;
extern CurrentSensor currentB;
extern AnalogInput temperature;
extern volatile uint16_t rawAdcValuesForDMA [20];
extern Led led;
extern Serial serial;
extern DcMotor motorA;
extern DcMotor motorB;
extern volatile int32_t ilsValueA;
extern volatile int32_t ilsValueB;
extern AnalogInput analogPositionA;
extern AnalogInput analogPositionB;
extern ServoMotor cylinderA;
extern ServoMotor cylinderB;
extern PID correctorA;
extern PID correctorB;

extern MPU6050 imu;
extern MFX_input_t data_in;
extern MFX_output_t data_out;
extern int32_t inclinaison_X, inclinaison_Y;


// Fonctions développées dans application.cpp
int16_t readTemperature (uint16_t raw);
void readBoardAddress (void);
void cylindersPeriodicManagement (uint8_t address);
void imuPeriodicManagement (float dT);


#ifdef SERIAL_DEBUG_MESSAGES
const char msgStart[] = { "\r\n--- Start application ---\r\n" };
const char msgImuOk[] = { "IMU ....... OK\r\n" };
const char msgLibMotionOk[] = { " ....... OK\r\n" };
#endif


/*! Premier test pour verifier les liens de debug
* Le cablage du connecteur de debug
* Le clignotement de la led
* La broche SWO
*/
void tc_ledAddressDebug (void)
{
  printf ("\nDebut de test cablage led, cablage lecture adresse carte\n"); // affichage sur SWO
  // Initialisation de la led
  led.on ();

  // Initialisation de la base de temps et activation de l'it timer
  HAL_TIM_Base_Start_IT (&htim11);

  while (1)
    {
      // Gestion base de temps et tâches immédiates
      while (!flagTimeBaseElapsed)
        {
        }
      flagTimeBaseElapsed = false;

      // Gestion des tâches périodiques
      led.periodicManagement (5, 100);
      readBoardAddress ();

      printf ("Time %lu, Adresse carte : %d\n", HAL_GetTick (), boardAddress);
    }
}

/*! \brief Fonction permettant le test de la liaison serie RS485
 *
 */
void tc_usart (void)
{
  int compteur = 0;
  char msgTmp[100];

  printf ("\nDebut de test cablage liaison serie RS485 \n"); // affichage sur SWO
  led.on (); // Initialisation de la led
  serial.initialisation (); // Initialisation de la liaison série

  while (1)
    {
      HAL_Delay (10);
      led.periodicManagement (5, 100);
      led.sendMorseLetter ('V');
      readBoardAddress ();
      if (serial.isDataReady ())
        {
          sprintf (msgTmp, "Reception de %02X", serial.read ());
          serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));
          printf ("%s", msgTmp);
        }

      if (++compteur % 100 == 0)
        {
          sprintf (msgTmp, "Compteur = %d, adresse = %d", compteur, boardAddress);
          serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));
          printf ("%s", msgTmp);
        }
    }
}

/*! \brief Test cablage des ponts en H, capteurs ILS et mesure de courant
 *
 */
void tc_moteurs (void)
{
  int cmptCycle = -1;
  char msgTmp[250];

  printf ("\nDebut de test cablage pont en H \n"); // affichage sur SWO

  led.on ();
  serial.initialisation ();

  // Initialisation des mesures analogiques
  HAL_ADC_Start_DMA (&hadc1, (uint32_t*) rawAdcValuesForDMA, 16);

  // Mise à 1 de la broche commune VSO (supprimer la mise en veille)
  HAL_GPIO_WritePin (VSO_GPIO_Port, VSO_Pin, GPIO_PIN_SET);

  motorA.initialisation ();
  motorB.initialisation ();

  while (1)
    {
      HAL_Delay (10);
      led.periodicManagement (50, 100);

      switch (++cmptCycle)
        {
        // @formatter:off
          case 000: motorA.forwards (33); break;
          case 200: motorA.freeSpin (); break;
          case 250: motorA.reverse (100); break;
          case 310: motorA.brake (); break;

          case 100: motorB.forwards (33); break;
          case 300: motorB.freeSpin (); break;
          case 380: motorB.reverse (100);break;
          case 440: motorB.brake (); break;
          case 499: cmptCycle = -1; break;
          default: break;
          // @formatter:on
}      if (cmptCycle % 20 == 0)
        {
          sprintf (msgTmp, "ADC : %03d, %03d, %03d, %03d, %03d, ILS : %ld, %ld", rawAdcValuesForDMA[0],
                   rawAdcValuesForDMA[1], rawAdcValuesForDMA[2], rawAdcValuesForDMA[3], rawAdcValuesForDMA[4],
                   ilsValueA, ilsValueB);
          serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));
          printf ("%s", msgTmp);
        }
    }
}


/*! \brief Test cablage centrale inertielle
 *
 */
void tc_imu (void)
{
  int compteur = 0;
  char msgTmp[100];

  printf ("\nDebut de test lecture valeurs brutes IMU \n"); // affichage sur SWO

  led.on ();
  serial.initialisation ();

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

  // Initialisation des mesures analogiques
  HAL_ADC_Start_DMA (&hadc1, (uint32_t*) rawAdcValuesForDMA, 16);

  // Mise à 1 de la broche commune VSO (supprimer la mise en veille)
  HAL_GPIO_WritePin (VSO_GPIO_Port, VSO_Pin, GPIO_PIN_SET);

  motorA.initialisation ();
  motorB.initialisation ();

  // Initialisation de la base de temps et activation de l'it timer
  HAL_TIM_Base_Start_IT (&htim11);

  while (1)
    {
      while (!flagTimeBaseElapsed)
        {
        }
      flagTimeBaseElapsed = false;

      // Gestion des tâches périodiques
      led.periodicManagement (5, 100);

      if (++compteur % 100 == 0)
        {
          imu.requestData ();

          // Get acceleration X/Y/Z in g
          data_in.acc[0] = imu.getAccelerometer_X ();
          data_in.acc[1] = imu.getAccelerometer_Y ();
          data_in.acc[2] = imu.getAccelerometer_Z ();

          // Get angular rate X/Y/Z in dps
          data_in.gyro[0] = imu.getGyroscope_X ();
          data_in.gyro[1] = imu.getGyroscope_Y ();
          data_in.gyro[2] = imu.getGyroscope_Z ();

          sprintf (msgTmp, "Compteur = %d, X = %d, Y = %d", compteur, (int16_t) (data_in.gyro[0] * 100.0F),
                   (int16_t) (data_in.gyro[1] * 100.0F));
          serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));
          printf ("%s", msgTmp);
        }
    }
}

/*!
 *
 *
 */
void ti_adc (void)
{
  int cmptCycle = -1;
  char msgTmp[100];

  while (1)
    {
      HAL_Delay (10);
      led.periodicManagement (5, 100);

      if (++cmptCycle == 1000 || cmptCycle == 0)
        {
          cmptCycle = 0;
          motorA.forwards (33);
        }
      else if (cmptCycle == 400)
        motorA.freeSpin ();
      else if (cmptCycle == 500)
        motorA.reverse (100);
      else if (cmptCycle == 620)
        motorA.brake ();

      if (cmptCycle % 25 == 0)
        {
          //int16_t temperature = readTemperature (getAverageTemperature ());
          int16_t temperatureTemp = readTemperature (temperature.getAverageValue ());
          //int16_t currentA = readCurrent (getAverageCurrentA ());
          //int16_t currentB = readCurrent (getAverageCurrentB ());

          sprintf (msgTmp, "Compteur = %d, temperature = %d, A = %d, B = %d, ILS = %ld\r\n", cmptCycle, temperatureTemp,
                   (int) (currentA.getValue () * 1000.0), (int) (currentB.getValue () * 1000.0), ilsValueA);
          serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));
        }
    }
}

/*!
 *
 *
 */
int16_t valCurrentA_forTi;
int16_t valCurrentB_forTi;

void ti_asservissement_position (void)
{
  int cmptCycle = -1;
  int etat = 0;

  while (1)
    {
      HAL_Delay (10);
      led.periodicManagement (5, 100);

      switch (etat)
        {
        case 0:
          // cylinderA.moveMotorToPosition (50, true);
          cylinderA.moveMotorToPosition (50);
          //if (cylinderB.moveMotorToPosition (250, false) != ServoMotor_Positionning)
          if (cylinderB.moveMotorToPosition (250) != ServoMotor_Positionning)
            {
              motorA.freeSpin ();
              etat = 1;
              cmptCycle = 0;
            }
          break;

        case 1:
          if (cmptCycle > 500)
            {
              etat = 2;
            }
          break;

        case 2:
          //cylinderA.moveMotorToPosition (250, false);
          cylinderA.moveMotorToPosition (250);
          // if (cylinderB.moveMotorToPosition (50, true) != ServoMotor_Positionning)
          if (cylinderB.moveMotorToPosition (50) != ServoMotor_Positionning)
            {
              motorA.freeSpin ();
              etat = 3;
              cmptCycle = 0;
            }
          break;

        case 3:
          if (cmptCycle > 500)
            {
              etat = 0;
            }
          break;
        }

      if (++cmptCycle % 5 == 0)
        {
          valCurrentA_forTi = (int16_t) (currentA.getValue () * 1000.0);  // use global for SWV Data trace timeline
          valCurrentB_forTi = (int16_t) (currentB.getValue () * 1000.0);
          // char msgTmp[100];
          //sprintf (msgTmp, "Compteur = %d, Etat = %d, CurrentA = %d, ILS_A = %ld, CurrentB = %d, ILS_B = %ld\r\n", cmptCycle, etat,
          //         valCurrentA, ilsValueA, valCurrentB, ilsValueB);
          //serial.sendBuffer ((uint8_t*) msgTmp, strlen (msgTmp));

          printf ("Compteur : %d, Etat : %d, CurrentA : %d, ILS_A : %ld, CurrentB : %d, ILS_B : %ld\r\n", cmptCycle,
                  etat, valCurrentA_forTi, ilsValueA, valCurrentB_forTi, ilsValueB);
        }
    }
}

/*!
 *
 *
 */
void ti_decodageMessage (void)
{
  std::array<uint8_t, 8> const message
    { ASCII_SOH, '3', 'A', 'L', TMP_CRC_H, TMP_CRC_L, ASCII_EOT };

  for (auto const element : message)
    {
      serialDecodeMessageSlave (element, 0x3A);
    }

  while (1)
    ;
}

/*!
 *
 */
void simulateurDePosition (void)
{
  static int cmptCyclePosition = 1;
  static bool littleFlag = false;
  if (--cmptCyclePosition == 0)
    {
      cmptCyclePosition = 20;
      if (littleFlag)
        {
          if (++ilsValueA > 99)
            littleFlag = false;
          if (--ilsValueB < 0)
            ilsValueB = 0;
        }
      else
        {
          if (++ilsValueB > 99)
            littleFlag = true;
          if (--ilsValueA < 0)
            ilsValueA = 0;
        }
    }
}

/*!
 *
 *
 */
void ti_communication (void)
{
  // initialisation de la led
  led.on ();

  // Initialisation de la liaison série
  serial.initialisation ();

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

  while (1)
    {
      // Gestion base de temps et tâches immédiates
      while (!flagTimeBaseElapsed)            // actuellement 10ms
        {
          if (serial.isDataReady ())
            {
              serialDecodeMessageSlave (serial.read (), boardAddress);
            }
        }
      flagTimeBaseElapsed = false;

      // Gestion des tâches périodiques
      readBoardAddress ();
      cylindersPeriodicManagement (boardAddress);
      //simulateurDePosition ();

      //imu.requestData ();
      led.periodicManagement (5, 100);
      readBoardAddress ();
    }
}

/*!
 *
 *
 */

void ti_communication_partie2 (void)
{
  float dT;

  while (1)
    {
      // Gestion base de temps et tâches immédiates
      while (!flagTimeBaseElapsed)            // actuellement 10ms
        {
          if (serial.isDataReady ())
            {
              serialDecodeMessageSlave (serial.read (), boardAddress);
            }
        }
      flagTimeBaseElapsed = false; // Acquitement interruption flag

      // Gestion des tâches périodiques
#ifdef VERIN_A_TYPE_LA31
        analogPositionFiltering ();
#endif

#ifdef VERIN_A_TYPE_LA31
        analogPositionFiltering ();
#endif

      cylinderA.periodicManagement ();
      cylinderB.periodicManagement ();

      led.periodicManagement (5, 100);
      readBoardAddress ();

      dT = 0.01; // In seconds

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
      imu.requestData (); // retard d'un cycle ?
    }
}

/*!
 *
 */
void ti_instrumentation (void)
{
  int cmpt = 0;
  char buffer[100];
  // initialisation de la led
  led.on ();

  // Initialisation de la liaison série
  serial.initialisation ();

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
  MFX_knobs_t iKnobs;

  MotionFX_initialize (); // Sensor Fusion API initialization function
  MotionFX_getKnobs (&iKnobs);

  /* Modify knobs settings & set the knobs */
  MotionFX_setKnobs (&iKnobs);
  MotionFX_enable_6X (MFX_ENGINE_ENABLE);

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
  correctorA.initialisation (2.0, 0.01);

  motorB.initialisation ();
  cylinderB.initialisation (); // ! Attention l'ordre des 2 lignes suivantes a de l'importance
  correctorB.initialisation (3.0, 0.01);

  // Initialisation de la base de temps et activation de l'it timer
  HAL_TIM_Base_Start_IT (&htim11);

  while (1)
    {
      // Gestion base de temps et tâches immédiates
      while (!flagTimeBaseElapsed) // actuellement 10ms
        {
          if (serial.isDataReady ())
            {
              serialDecodeMessageSlave (serial.read (), boardAddress);
            }
        }
      flagTimeBaseElapsed = false; // Acquitement flag interruption

      // Gestion des tâches périodiques
      led.periodicManagement (5, 100);
      readBoardAddress ();
      cylindersPeriodicManagement (boardAddress);
      imuPeriodicManagement (0.01);

      sprintf (buffer, "%d;%d;%d\n\r", cmpt++, (int16_t) cylinderA.getConsigne (), analogPositionA.getValue ());
      serial.sendBuffer (buffer);
      if (cmpt == 100)
        {
          cylinderB.moveMotorToPosition (0x200);
        }

      if (cmpt == 1000)
        {
          cylinderB.moveMotorToPosition (0x500);
        }

      if (cmpt == 1999)
        {
          cmpt = 0;
        }
    }
}

/*!
 *
 */
void tests_integration (void)
{
//  ti_instrumentation ();
//  ti_communication_partie2 (); // plus complète
  ti_communication ();
//ti_decodageMessage ();
//ti_asservissement_position ();
//ti_adc ();

  //tc_imu ();
  tc_moteurs ();
  //tc_usart ();
  //tc_ledAddressDebug ();
}
