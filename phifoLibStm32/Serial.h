/*
 * serial.h
 *
 *  Created on: Jun 8, 2020
 *      Author: phifo
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#define IT_RX_BUFFER_SIZE 64


/**
 * \brief  MEMS abstract class for general interface for MEMS IMU
 *
 */
class Serial
{
public:
  /**
   * \brief Constructeurs
   *
   *
   */
  Serial (UART_HandleTypeDef * h);
  Serial (UART_HandleTypeDef * h, MGPIO_HandleTypeDef * p);
  Serial (I2C_HandleTypeDef * h);


  /**
   * \brief
   *
   *
   */
  void initialisation (void);

  /**
   * \brief
   *
   *
   */
  bool isDataReady (void);
  //bool isDataReadyII (void);

  /**
   * \brief
   *
   *
   */
  uint8_t read (void);

  /**
   * \brief
   *
   *
   */
void xferDmaReadBuffer (uint8_t * buffer, int length);


  /**
   * \brief
   *
   *
   */
  void sendBuffer (uint8_t *transmissionBuffer, uint16_t msgSize);
  void sendBuffer (const char * transmissionBuffer);

  static void Handler_UART_TxCpltCallback (UART_HandleTypeDef *huart);
  static void Handler_UART_RxCpltCallback (UART_HandleTypeDef *huart);

private:
  UART_HandleTypeDef * uartHandle; //!<
  I2C_HandleTypeDef * i2cHandle; //!<
  MGPIO_HandleTypeDef * rs485pin;

  uint8_t uartRxBuffer[64];
  uint8_t uartRxtmpA[IT_RX_BUFFER_SIZE];
  uint8_t uartRxtmpB[IT_RX_BUFFER_SIZE];
  uint8_t uartRxHead = 0;
  uint8_t uartRxTail = 0;
  uint8_t uartRxCount = 0;
  bool uartFlagRxTmpA;

  void UART_RxCpltCallback (void);
  void UART_TxCpltCallback (void);

  static Serial * objectsList[5];
  static uint8_t nbObjects;

};

#endif /* SERIAL_H_ */
