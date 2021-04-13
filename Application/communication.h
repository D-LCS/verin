/*
 * communication.h
 *
 *  Created on: Apr 17, 2020
 *      Author: phifo
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#define ASCII_SOH   '!' //!< \todo : à corriger (plus simple lors des tests manuels)
#define ASCII_EOT   '$' //!< \todo : à corriger (plus simple lors des tests manuels)

//#define ASCII_SOH   0x01
//#define ASCII_EOT   0x04
#define ASCII_ACK   0x06
#define ASCII_NACK  0x15

#define TMP_CRC_H 0x2D  //!< \todo : à corriger (implementation du CRC)
#define TMP_CRC_L 0x2D  //!< \todo : à corriger (implementation du CRC)

#define __FLOAT_WIDTH__ 4


//void int2asciiHex (uint8_t v, uint8_t *highNibble, uint8_t *lowNibble);
//uint8_t asciiHex2int (uint8_t highNibble, uint8_t lowNibble);


void prepareToSendQuaternion (float *q, uint8_t adresse, uint16_t *size, char *transmissionBuffer);

void prepareToSendGravity (float *g, uint8_t adresse, uint16_t *size, char *transmissionBuffer);

enum decodeMessage
{
  REQUEST_MSG,
  NO_MSG,
  MSG_OK,
  ERROR_MSG
};

uint8_t serialDecodeMessageMaster (uint8_t c);
enum decodeMessage serialDecodeMessageSlave (uint8_t c, uint8_t adresse);

//bool serialdataInBuffer (void);

//uint8_t getDataInSerialBuffer (void);


#endif /* COMMUNICATION_H_ */
