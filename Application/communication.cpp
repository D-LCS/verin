/*!
 * communication.cpp
 *
 *
 * Liste des commandes acceptées
 * Get position (P)
 * Move to position (M)
 * Get tilt (T) = at the beginning : inclinaison (I)
 * Move forwards (F)
 * Move Backwards (R)
 * Stop Move (S)
 * Led (L)
 * Diagnostic (D)
 *
 * Grammaire des commandes
 *
 * SOH, ADDR_H, ADDR_L, MESSAGE_TYPE, PARAMETRES, CRC_H, CRC_L, EOT
 *
 * Commande P, I, S, L et D : pas de paramètre
 * Commande M : paramètres en pas de capteur [0000..9999] ou en valeur équivalente ADC [0..4096]
 * F et R : paramètres [000..100] = valeur du rapport cyclique du PWM (sans compter les accélération et freinage)
 *
 * Note : si le paramètre attendu est sur 4 chiffres (3 digit) il faut transmettre les zéros non significatifs
 *
 *  Created on: Apr 17, 2020
 *      Author: phifo
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <array>

#include "main.h"
#include "application.h"
#include "Led.h"
#include "Serial.h"

#include "ISensor.h"
#include "AnalogInput.h"
#include "Corrector.h"
#include "HDriver.h"
#include "DcMotor.h"
#include "ServoMotor.h"

#include "motion_fx.h"

#include "communication.h"

extern Led led;
extern ServoMotor cylinderA;
extern ServoMotor cylinderB;
extern Serial serial;
extern int32_t inclinaison_X, inclinaison_Y;
extern uint8_t boardAddress;

bool adressePaire = true;

char usartTxBuffer [100];

enum decodeMessageMachineState
{
  START,
  STOP,
  MSB_HIGH,
  MSB_LOW,
  LSB_HIGH,
  LSB_LOW,
  ADDRESS_H,
  ADDRESS_L,
  ERROR_STATE,
  MESSAGE_TYPE,
  CRC_HIGH,
  CRC_LOW,
  DO_COMMAND
};

/*! \brief Classe Command : définitions des objets contenant les differentes commandes possibles
 *
 */
class Command
{
public:
  Command (char l, enum decodeMessageMachineState ns, enum decodeMessage (*f) (uint16_t p))
    : letter (l), nextState (ns), traitement (f)
  {
  }

  char letter; //!< le type de commande
  enum decodeMessageMachineState nextState; //<! Le premier état après la reconnaissance de la commande reçu
  enum decodeMessage (*traitement) (uint16_t p); //!< La méthode lié à la commande

  union
  {
    int32_t s32;
    int16_t s16;
    int8_t s8;
  } val;
};

// Prototypes des fonctions liées aux commandes
enum decodeMessage commandGetPosition (uint16_t p);
enum decodeMessage commandSetPosition (uint16_t p);
enum decodeMessage commandMoveToposition (uint16_t p);
enum decodeMessage commandGetTilt (uint16_t p);
enum decodeMessage commandMoveForwards (uint16_t p);
enum decodeMessage commandMoveReverse (uint16_t p);
enum decodeMessage commandStopMove (uint16_t p);
enum decodeMessage commandDiagnostic (uint16_t p);
enum decodeMessage commandLed (uint16_t p);
enum decodeMessage commandConfig (uint16_t p);

uint8_t calculateCRC8 (uint8_t oldCRC, uint8_t dataInput);


#ifdef APPLICATION_IS_SLAVE
// @formatter:off
std::array<Command, 10> const commandsTable
  {
  Command ('P', CRC_HIGH, commandGetPosition),
  Command ('I', MSB_HIGH, commandSetPosition),
  Command ('T', CRC_HIGH, commandGetTilt),
  Command ('S', CRC_HIGH, commandStopMove),
  Command ('L', CRC_HIGH, commandLed),
  Command ('D', CRC_HIGH, commandDiagnostic),
  Command ('M', MSB_HIGH, commandMoveToposition),
  Command ('F', LSB_HIGH, commandMoveForwards),
  Command ('R', LSB_HIGH, commandMoveReverse),
  Command ('C', LSB_HIGH, commandConfig)
  };
// @formatter:on
#endif

/**
 * \brief Traduction d'une valeur 8 bits en code ASCII hexadécimal (2 caractères)
 *
 * \param v [in] valeur à convertir
 * \param highNibble [inout] code ASCII du poids fort de la valeur traduite en hexadécimal
 * \param lowNibble [inout] code ASCII du poids faible de la valeur traduite en hexadécimal
 *
 */
void int2asciiHex (uint8_t v, uint8_t *highNibble, uint8_t *lowNibble)
{
  *highNibble = ((v >> 4) & 0x0F) + '0';
  if (*highNibble > '9')
    *highNibble += 'A' - ('9' + 1);

  *lowNibble = (v & 0x0F) + '0';
  if (*lowNibble > '9')
    *lowNibble += 'A' - ('9' + 1);
}

/**
 * \brief Traduction d'une valeur codée ASCII hexadécimal (2 caractères) en valeur 8 bits non signée
 *
 * \param highNibble [inout] code ASCII du poids fort de la valeur traduite en hexadécimal
 * \param lowNibble [inout] code ASCII du poids faible de la valeur traduite en hexadécimal
 * \return la valeur convertie
 */
uint8_t asciiHex2int (uint8_t highNibble, uint8_t lowNibble)
{
  uint8_t result;

  if (highNibble >= 'A')
    result = highNibble - 'A' + 10;
  else
    result = highNibble - '0';
  result = (result << 4) & 0xF0;

  if (lowNibble >= 'A')
    result += lowNibble - 'A' + 10;
  else
    result += lowNibble - '0';
  return result;
}

/**
 * \brief Hexa
 *
 */
bool isAsciiHex (uint8_t v)
{
  if (((v >= '0') && (v <= '9')) || ((v >= 'A') && (v <= 'F')))
    return true;
  else
    return false;
}

/**
 * \brief Traduction d'un tableau de valeurs 8 bits en tableau de valeur 6 bits transmissible
 *
 * \param highNibble [inout] code ASCII du poids fort de la valeur traduite en hexadécimal
 * \param lowNibble [inout] code ASCII du poids faible de la valeur traduite en hexadécimal
 * \return la valeur convertie
 */
uint8_t binToBase64 (uint8_t size, uint8_t *tableIn, uint8_t *tableOut)
{
  uint8_t iTableIn, iTableOut;

  iTableOut = 0;
  iTableIn = 0;
  while (iTableIn < size)
    {
      switch (iTableOut % 4)
        {
        case 0:
          tableOut[iTableOut] = ((tableIn[iTableIn] >> 2) & 0x3F) + '0';
          break;
        case 1:
          tableOut[iTableOut] = (tableIn[iTableIn] << 4) & 0x30;
          tableOut[iTableOut] = (tableOut[iTableOut] | ((tableIn[++iTableIn] >> 4) & 0x0F)) + '0';
          break;
        case 2:
          tableOut[iTableOut] = ((tableIn[iTableIn] << 2) & 0x3C);
          tableOut[iTableOut] = (tableOut[iTableOut] | ((tableIn[++iTableIn] >> 6) & 0x03)) + '0';
          break;
        case 3:
          tableOut[iTableOut] = (tableIn[iTableIn++] & 0x3F) + '0';
          break;
        default:
          tableOut[iTableOut] = 0xff;
        }
      iTableOut = iTableOut + 1;
    }
  return iTableOut - 1;
}

/**
 * \brief Traduction d'un tableau de valeurs 8 bits en tableau de valeur 6 bits transmissible
 *
 * \param highNibble [inout] code ASCII du poids fort de la valeur traduite en hexadécimal
 * \param lowNibble [inout] code ASCII du poids faible de la valeur traduite en hexadécimal
 * \return la valeur convertie
 */
uint8_t base64ToBin (uint8_t c, uint8_t *index, bool reinit)
{
  static uint8_t state;
  static uint8_t next;
  //uint8_t iTableIn, iTableOut;
  uint8_t output = 0;
  uint8_t val;

  if (reinit == true)
    {
      *index = 0;
      state = 0;
    }
  else
    {
      val = c - '0';
      switch (state % 4)
        {
        case 0:
          next = (val << 2) & 0xFC;
          output = 0;
          break;
        case 1:
          output = next | ((val >> 4) & 0x03);
          *index += 1;
          next = (val << 4) & 0xF0;
          break;
        case 2:
          output = next | ((val >> 2) & 0x0F);
          *index += 1;
          next = (val << 6) & 0xc0;
          break;
        case 3:
          output = next | (val & 0x03F);
          *index += 1;
          break;
        default:
          output = 0xff;
        }
      state += 1;
    }

  return output;
}

/**
 * \brief Test base 64
 *
 */
bool isBase64 (uint8_t v)
{
  if ((v >= '0') && (v < '0' + 64))
    return true;
  else
    return false;
}

/**
 * \brief Decode le message reçu par la liaison RS485
 *
 * Machine à états pour la réception de la valeur sur la liaison série
 *
 */

#ifdef APPLICATION_IS_MASTER
uint8_t serialDecodeMessageMaster (uint8_t c)
{
  enum decodeMessageMachineState state = ERROR_STATE;
//  static uint16_t result;
  //static uint8_t tmpHigh;
  static uint8_t index;
  uint8_t output = 0;
  uint8_t crc;
  //uint8_t v;

  union
  {
    uint8_t b[16];
    float f[4];
  } quaternionInputTable;

  switch (state)
    {
    case START:
      if (c == ASCII_SOH)
        {
          state = MESSAGE_TYPE;
        }
      break;

    case MESSAGE_TYPE:
      if (c == 'Q')
        {
          index = 0;
          state = QUATERNION;
          base64ToBin (c, &index, true);
        }
      else if (c == 'G')
        {
          index = 0;
          state = GRAVITY;
          base64ToBin (c, &index, true);
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case QUATERNION:
      if (isBase64 (c))
        {
          quaternionInputTable.b[index] = base64ToBin (c, &index, false);
          if (index == 16)
            state = CRC1;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case CRC1:
      if (isBase64 (c))
        {
          crc = base64ToBin (c, &index, false);
          state = CRC2;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case CRC2:
      if (isBase64 (c))
        {
          crc = base64ToBin (c, &index, false);
          uint8_t crcCalculated = 0;
          for (uint8_t i = 0; i < 16; i++)
            {
              crcCalculated += calculateCRC8 (crcCalculated, quaternionInputTable.b[i]);
            }
          if (crc == crcCalculated)
            {
              state = STOP;
            }
          else
            {
              goto ERROR_STATE;
            }
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case STOP:
      if (c == ASCII_EOT)
        {
          output = 1;
          state = START;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    default:
      ERROR_STATE: if (c == ASCII_SOH)
        {
          //result = 0;
          state = MESSAGE_TYPE;
          output = -1;
        }
      else
        {
          state = START;
          output = 0;
        }
      break;
    }
  return output;
}
#endif

#ifdef APPLICATION_IS_SLAVE

/*!
 * \decode message esclave
 *
 * \todo Ajouter le contrôle du CRC
 *
 */
static const Command *recognizedCommand = NULL;

enum decodeMessage serialDecodeMessageSlave (uint8_t c, uint8_t adresse)
{
  enum decodeMessageMachineState static state = ERROR_STATE;
  static uint8_t tmp;
  static uint32_t parametre;
  enum decodeMessage output = NO_MSG;
  uint8_t crc;

  switch (state)
    {
    case START:
      if (c == ASCII_SOH)
        {
          recognizedCommand = NULL;
          state = ADDRESS_H;
        }
      break;

    case ADDRESS_H:
      if (isAsciiHex (c))
        {
          tmp = c;
          state = ADDRESS_L;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case ADDRESS_L:
      if (isAsciiHex (c))
        {
          uint8_t addressMsg = asciiHex2int (tmp, c);
          adressePaire = ((addressMsg & 0x01) == 0) ? true : false;
          if ((addressMsg & 0xFE) == adresse)
            {
              state = MESSAGE_TYPE;
            }
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case MESSAGE_TYPE:
      for (auto const & element : commandsTable)
        {
          if (c == element.letter)
            {
              recognizedCommand = & element;
              tmp = 0;
              parametre = 0;
              state = recognizedCommand->nextState;
              break; // Commande reconnue ==> sortie du "for"
            }
        }
      if (recognizedCommand == NULL)
        {
          goto ERROR_STATE;
        }
      break;

    case MSB_HIGH:
      if (isAsciiHex (c))
        {
          tmp = c;
          state = MSB_LOW;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case MSB_LOW:
      if (isAsciiHex (c))
        {
          parametre = asciiHex2int (tmp, c) << 8;
          state = LSB_HIGH;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case LSB_HIGH:
      if (isAsciiHex (c))
        {
          tmp = c;
          state = LSB_LOW;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case LSB_LOW:
      if (isAsciiHex (c))
        {
          parametre += asciiHex2int (tmp, c);
          state = CRC_HIGH;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case CRC_HIGH:
      if (c == TMP_CRC_H)
        {
          state = CRC_LOW;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case CRC_LOW:
      crc = c;
      if (crc == TMP_CRC_L)
        {
          state = DO_COMMAND;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    case DO_COMMAND:
      if (c == ASCII_EOT)
        {
          output = recognizedCommand->traitement (parametre);
          state = START;
        }
      else
        {
          goto ERROR_STATE;
        }
      break;

    default:
      ERROR_STATE: if (c == ASCII_SOH)
        {
          state = ADDRESS_H;
        }
      else
        {
          state = START;
        }
      break;
    }
  return output;
}
#endif

/**
 * \brief Reçoit une valeur par la liaison série
 *
 */
//void serialReceiveValue (uint16_t *value, uint16_t timeoutVal)
//{
//  uint16_t readVal;
//  int8_t status;
//  static uint16_t timeOut;
////  eusart_status_t rxStatus;
//
////  rxStatus = EUSART_get_last_status ();
////  if (rxStatus.status != 0)
//    {
//
//    }
////  else if (--timeOut == 0)
//    {
//
//    }
////  else
//    {
//      //    while (EUSART_is_rx_ready ())
//        {
//          //        status = serialDecodeMessage (EUSART_Read (), &readVal);
//          if (status == 0) // si reception en cours
//            {
//              timeOut = 3;
//            }
//          else if (status == 1) // reception message correct
//            {
//              *value = readVal;
//              timeOut = timeoutVal;
//              //              EUSART_Write (ASCII_ACK);
//            }
//          else
//            {
//              timeOut = timeoutVal;
//              //              EUSART_Write (ASCII_NACK);
//            }
//        }
//    }
//}
/*!
 * \brief Envoyer un quaternion de position de vérin par la liaison RS485
 *
 * Fonction d'émission du message en mode polling
 *      la valeur est codée sur un entier sur 16bits non signé
 *      La valeur est convertie en hexadecimal (4 caractères)
 *      Le message commence par un "Start Of Heading"
 *      Le message se termine par le caractère ASCII "End Of Transmission"
 *
 * Calcul : 1 message = 6 caractères, à 9600 bauds (~ 1 char par ms) ==> durée message = 6 ms
 *
 * \param value [in] Valeur à envoyer
 */
//#define SLAVE_SIZE_TRANSMISSION 37 // 1+1+(4*4*2)+2+1
void prepareToSendQuaternion (float *q, uint8_t adresse, uint16_t *size, char *transmissionBuffer)
{
  uint8_t p = 0;
  uint8_t i, j;
  uint8_t crc = 0;
  floatToByte_t f;
  uint8_t high;
  uint8_t low;

  transmissionBuffer[p++] = ASCII_SOH; // début message
  transmissionBuffer[p++] = 'q';  // type du message 'Q' comme quaternion

  int2asciiHex (adresse, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

  for (i = 0; i < MFX_QNUM_AXES; i++)
    {
      f.f = q[i];
      for (j = 0; j < __FLOAT_WIDTH__; j++)
        {
          int2asciiHex (f.b[j], &high, &low);
          transmissionBuffer[p++] = high;
          transmissionBuffer[p++] = low;
        }
    }
  for (i = 0; i < p; i++)
    crc += calculateCRC8 (crc, transmissionBuffer[i]);

  int2asciiHex (crc, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

  transmissionBuffer[p++] = ASCII_EOT; // fin message
  transmissionBuffer[p++] = '\n';
  transmissionBuffer[p++] = '\r';

  *size = p - 1;
}

/*!
 * \brief
 *
 */
void prepareToSendGravity (float *g, uint8_t adresse, uint16_t *size, char *transmissionBuffer)
{
  uint8_t p = 0;
  uint8_t i;
  uint8_t crc = 0;
  uint8_t high;
  uint8_t low;
  float f;
  word16Byte_t wb;

  transmissionBuffer[p++] = ASCII_SOH; // début message
  transmissionBuffer[p++] = 'g';  // type du message 'G' comme "Gravity"

  int2asciiHex (adresse, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

  for (i = 0; i < 2; i++)
    {
      f = g[i] * 16384.0;
      wb.val = (int16_t) f;
      int2asciiHex (wb.h, &high, &low);
      transmissionBuffer[p++] = high;
      transmissionBuffer[p++] = low;
      int2asciiHex (wb.l, &high, &low);
      transmissionBuffer[p++] = high;
      transmissionBuffer[p++] = low;
    }
  for (i = 0; i < p; i++)
    crc += calculateCRC8 (crc, transmissionBuffer[i]);

  int2asciiHex (crc, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

  transmissionBuffer[p++] = ASCII_EOT; // fin message
  transmissionBuffer[p++] = '\n';
  transmissionBuffer[p++] = '\r';

  *size = p - 1;
}

//void prepareToSendPosition (int32_t a, int32_t b, uint8_t adresse, uint16_t *size, char *transmissionBuffer)
//{
//  uint8_t p = 0;
//  //uint8_t crc = 0;
//  uint8_t high;
//  uint8_t low;
//  word32Byte_t wb;
//
//  transmissionBuffer[p++] = ASCII_SOH; // début message
//
//  int2asciiHex (adresse, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;
//  transmissionBuffer[p++] = 'p';  // type du message 'P' comme "Position"
//
//  //wb.val = 0x1234; // a
//  wb.val = a;
//  int2asciiHex (wb.lh, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;
//  int2asciiHex (wb.ll, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;
//
//  //wb.val = 0xCDEF;  //b
//  wb.val = b;
//  int2asciiHex (wb.lh, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;
//  int2asciiHex (wb.ll, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;
//
////  for (i = 0; i < p; i++)
////    crc += calculateCRC8 (crc, transmissionBuffer[i]);
////
////  int2asciiHex (crc, &high, &low);
////  transmissionBuffer[p++] = high;
////  transmissionBuffer[p++] = low;
//
//  transmissionBuffer[p++] = TMP_CRC_H;
//  transmissionBuffer[p++] = TMP_CRC_L;
//
//  transmissionBuffer[p++] = ASCII_EOT; // fin message
//  transmissionBuffer[p++] = '\n';
//  transmissionBuffer[p++] = '\r';
//
//  *size = p - 1;
//}


/*!
 *
 */
void prepareToSend1xInt32 (int32_t a, uint8_t adresse, char ack, uint16_t *size, char *transmissionBuffer)
{
  uint8_t p = 0;
  //uint8_t crc = 0;
  uint8_t high;
  uint8_t low;
  word32Byte_t wb;

  transmissionBuffer[p++] = ASCII_SOH; // début message

  int2asciiHex (adresse, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;
  transmissionBuffer[p++] = ack;  // type du message

  wb.val = a;
  int2asciiHex (wb.lh, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;
  int2asciiHex (wb.ll, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

//  for (i = 0; i < p; i++)
//    crc += calculateCRC8 (crc, transmissionBuffer[i]);
//
//  int2asciiHex (crc, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;

  transmissionBuffer[p++] = TMP_CRC_H;
  transmissionBuffer[p++] = TMP_CRC_L;

  transmissionBuffer[p++] = ASCII_EOT; // fin message
  transmissionBuffer[p++] = '\n';
  transmissionBuffer[p++] = '\r';

  *size = p - 1;
}

/*!
 *
 */
void prepareToSend2xInt32 (int32_t a, int32_t b, uint8_t adresse, char ack, uint16_t *size, char *transmissionBuffer)
{
  uint8_t p = 0;
  //uint8_t crc = 0;
  uint8_t high;
  uint8_t low;
  word32Byte_t wb;

  transmissionBuffer[p++] = ASCII_SOH; // début message

  int2asciiHex (adresse, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;
  transmissionBuffer[p++] = ack;  // type du message

  wb.val = a;
  int2asciiHex (wb.lh, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;
  int2asciiHex (wb.ll, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

  wb.val = b;
  int2asciiHex (wb.lh, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;
  int2asciiHex (wb.ll, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;

//  for (i = 0; i < p; i++)
//    crc += calculateCRC8 (crc, transmissionBuffer[i]);
//
//  int2asciiHex (crc, &high, &low);
//  transmissionBuffer[p++] = high;
//  transmissionBuffer[p++] = low;

  transmissionBuffer[p++] = TMP_CRC_H;
  transmissionBuffer[p++] = TMP_CRC_L;

  transmissionBuffer[p++] = ASCII_EOT; // fin message
  transmissionBuffer[p++] = '\n';
  transmissionBuffer[p++] = '\r';

  *size = p - 1;
}

/*!
 *
 */
void prepareToSendAcknoledgement (uint8_t adresse, char ack, uint16_t *size, char *transmissionBuffer)
{
  uint8_t p = 0;
  uint8_t high;
  uint8_t low;

  transmissionBuffer[p++] = ASCII_SOH; // début message

  int2asciiHex (adresse, &high, &low);
  transmissionBuffer[p++] = high;
  transmissionBuffer[p++] = low;
  transmissionBuffer[p++] = ack;  // type du message acknoledge

  transmissionBuffer[p++] = TMP_CRC_H;
  transmissionBuffer[p++] = TMP_CRC_L;

  transmissionBuffer[p++] = ASCII_EOT; // fin message
  transmissionBuffer[p++] = '\n';
  transmissionBuffer[p++] = '\r';

  *size = p - 1;

}


bool serialdataInBuffer (void)
{
  return false;
}

uint8_t getDataInSerialBuffer (void)
{
  return '0';
}



/*!
 *
 */
enum decodeMessage commandGetPosition (uint16_t p)
{
  uint16_t size;
  int16_t val;

  if (adressePaire)
    {
      val = cylinderA.getPosition ();
      prepareToSend1xInt32 (val, boardAddress, 'p', &size, usartTxBuffer);
    }
  else
    {
      val = cylinderB.getPosition ();
      prepareToSend1xInt32 (val, boardAddress+1, 'p', &size, usartTxBuffer);
    }

  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return NO_MSG;
}

/*!
 *
 */
enum decodeMessage commandSetPosition (uint16_t p)
{
   uint16_t size;
   if (adressePaire)
     {
       cylinderA.setPosition (p);
       prepareToSendAcknoledgement (boardAddress, 'i', &size, usartTxBuffer);
     }
   else
     {
       cylinderB.setPosition (p);
       prepareToSendAcknoledgement (boardAddress+1, 'i', &size, usartTxBuffer);
     }
   serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
   return MSG_OK;
 }


/*!
 *
 */
enum decodeMessage commandGetTilt (uint16_t p)
{
  uint16_t size;

  prepareToSend2xInt32 (inclinaison_X, inclinaison_Y, boardAddress, 't', &size, usartTxBuffer);
  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}


/*!
 *
 */
enum decodeMessage commandMoveToposition (uint16_t p)
{
  uint16_t size;
  if (adressePaire)
    {
      cylinderA.moveMotorToPosition (p);
      if ((boardAddress & PHOENIX_PARALLEL_MASK) != 0)
        {
          cylinderB.moveMotorToPosition (p);
        }
      prepareToSendAcknoledgement (boardAddress, 'm', &size, usartTxBuffer);
    }
  else
    {
      cylinderB.moveMotorToPosition (p);
      prepareToSendAcknoledgement (boardAddress+1, 'm', &size, usartTxBuffer);
    }
  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}


/*!
 *
 */
enum decodeMessage commandMoveForwards (uint16_t p)
{
  uint16_t size;

  if (adressePaire)
    {
      cylinderA.moveMotor (true, p);
      prepareToSendAcknoledgement (boardAddress, 'f', &size, usartTxBuffer);
    }
  else
    {
      cylinderB.moveMotor (true, p);
      prepareToSendAcknoledgement (boardAddress+1, 'f', &size, usartTxBuffer);
    }

  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}

/*!
 *
 */
enum decodeMessage commandMoveReverse (uint16_t p)
{
  uint16_t size;

  if (adressePaire)
    {
      cylinderA.moveMotor (false, p);
      prepareToSendAcknoledgement (boardAddress, 'r', &size, usartTxBuffer);
    }
  else
    {
      cylinderB.moveMotor (false, p);
      prepareToSendAcknoledgement (boardAddress+1, 'r', &size, usartTxBuffer);
    }
  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}

/*!
 *
 */
enum decodeMessage commandStopMove (uint16_t p)
{
  uint16_t size;

  if (adressePaire)
    {
      cylinderA.stopMotor ();
      prepareToSendAcknoledgement (boardAddress, 's', &size, usartTxBuffer);
    }
  else
    {
      cylinderB.stopMotor ();
      prepareToSendAcknoledgement (boardAddress+1, 's', &size, usartTxBuffer);
    }
  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}

/*!
 * HDRIVER_Normal_Mode,
 */
enum decodeMessage commandConfig (uint16_t p)
{
  uint16_t size;
  uint8_t a = boardAddress;

  if (adressePaire)
    {
      if ((p & 0x01) == 0)
        cylinderA.setWiringMode (HDRIVER_Normal_Mode);
      else
        cylinderA.setWiringMode (HDRIVER_Flipping_Mode);

      if ((p & 0x02) == 0)
        cylinderA.setIlsInversion (false);
      else
        cylinderA.setIlsInversion (true);
    }
  else
    {
      a = a+1;
      if ((p & 0x01) == 0)
        cylinderB.setWiringMode (HDRIVER_Normal_Mode);
      else
        cylinderB.setWiringMode (HDRIVER_Flipping_Mode);

      if ((p & 0x02) == 0)
        cylinderB.setIlsInversion (false);
      else
        cylinderB.setIlsInversion (true);

    }

  prepareToSendAcknoledgement (a, 'c', &size, usartTxBuffer);
  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}
/*!
 *
 */
enum decodeMessage commandDiagnostic (uint16_t p)
{
  return NO_MSG;
}

/*!
 * \brief command Led
 *
 * Traitement du message de commande LED
 * Active ou désactive la led de vie de la carte
 *
 * Le paramètre 'p' existe pour la compatibilité des fonctions de traitement mais n'est pas utilisé ici
 */
enum decodeMessage commandLed (uint16_t p)
{
  uint16_t size;
  led.toggleCycleLed ();
  prepareToSendAcknoledgement (boardAddress, 'l', &size, usartTxBuffer);
  serial.sendBuffer ((uint8_t *)usartTxBuffer, size);
  return MSG_OK;
}

// end of file



