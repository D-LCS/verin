/*
 * serial.cpp
 *
 *  Created on: Jun 8, 2020
 *      Author: phifo
 */

#include <string.h>

#include "main.h"
#include "application.h"

#include "Serial.h"


// Provisoire : a ratacher à l'objet
#define DMA_RX_BUFFER_SIZE 64
uint8_t uartDmaRxBuffer [DMA_RX_BUFFER_SIZE];
extern Serial serial;

/*!
 * \brief
 *
 * Explication : pour pouvoir retrouver le pointeur sur l'objet dans l'interruption, il est nécessaire
 * de créer une liste de pointeurs sur les objets.
 * Cette liste et son index (objectList et nbObjects) sont des variables de classe (statiques) initialisés ici
 */
uint8_t Serial::nbObjects = 0;
Serial *Serial::objectsList[5] = { (Serial*) nullptr, (Serial*) nullptr, (Serial*) nullptr, (Serial*) nullptr,
    (Serial*) nullptr };

/*!
 * \brief Redefinition de la fonction d'interruption du HAL
 * Appel du handler d'interruption de l'objet
 */
void HAL_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  Serial::Handler_UART_TxCpltCallback (huart);
}

/*!
 * \brief Redefinition de la fonction d'interruption du HAL
 * Appel du handler d'interruption de l'objet
 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  Serial::Handler_UART_RxCpltCallback (huart);
  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart)
{
  // __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);

  // Disable the UART Error Interrupt: (Frame error, noise error, overrun error)
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

  // The most important thing when UART framing error occur/any error is restart the RX process

  //HAL_UART_Receive_DMA (huart, (uint8_t*) uartRxtmpA, IT_RX_BUFFER_SIZE);
}

/*!
 * \brief USART IRQ Handler
 *
 * Fonctionnement général de l'usart en reception :
 *
 * Hypothèses :
 *   - La taille des paquets inconnus, donc, utilisation du DMA est compliqué puisque avec le DMA on attends un nombre
 *     d'octets définis
 *   - l'utilisation du DMA est nécessaire car ça évite une interruption par caractère (solution essayée jusqu'au 28/09/2020 avec
 *     un baudrate de 9600 bauds)
 *
 * Solution : générer une interruption USART sur détection de fin de réception, et dans cette IT, arreter le DMA manuellement,
 * récuperer les octest reçus, et reinitialiser le DMA pour le paquet suivant.
 *
 * Interrupt request raised by IDLE line detection. When Idle line detected, we need to read rx_buffer immediately, but how ?.
 * Disabling DMA will force transfert complete interrupt.\n
 *
 * Une idée essayée : utiliser la fonction callback de l'interruption USART par défaut de la lib HAL.
 * Verdict : ça ne fonctionne pas car l'it attendue est sur IDLE et donc, il n'y a pas de caractère à lire et donc
 * pas d'appel à la fonction callback.\n
 * Fonction de callback : void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
 *
 * Problème suivant : une seule et même IT pour l'emission et la reception
 * Donc l'it USART est obligatoirement reroutée ici
 *
 */
extern "C" {
void myUSART2_IRQHandler (UART_HandleTypeDef *huart);
}

void myUSART2_IRQHandler (UART_HandleTypeDef *huart)
{
  volatile uint32_t tmp;  // must be volatile to prevent optimization

//  espionUsart2Irq++;

  if ((huart->Instance->SR & UART_FLAG_IDLE)  // if IDLE Flag is set
  || (huart->Instance->SR & USART_SR_FE)) // if framing error
    {
      tmp = huart->Instance->SR; // read status register - flags reset ?
      tmp = huart->Instance->DR; // read data register - flags reset ?
      //(void *) tmp; // pour supprimer le warning (ne fonctionne pas !)
      tmp += 1;          // pour supprimer le warning :  "warning: value computed is not used [-Wunused-value]"
      hdma_usart2_rx.Instance->CR &= ~DMA_SxCR_EN;   // disabling DMA will force transfert complete interrupt if enabled

      if (huart->Instance->SR & USART_SR_FE)
        {
          __HAL_UART_CLEAR_FEFLAG(huart);
        }

      // Clear the transfer complete flag
      //regs->LIFCR = DMA_FLAG_TCIF0_4 << hdma_usart2_rx.StreamIndex;

      // Write received data for UART main buffer for manipulation later
      // memcpy (telegramsBuffer[i].data, dma_Rx_Buffer, telegramsBuffer[i].length); // Copy first part
      serial.xferDmaReadBuffer (uartDmaRxBuffer, DMA_RX_BUFFER_SIZE - hdma_usart2_rx.Instance->NDTR);

      // Prepare DMA for next transfer
      // Important! DMA stream won't start if all flags are not cleared first
      DMA_TypeDef *regs = (DMA_TypeDef*) hdma_usart2_rx.StreamBaseAddress;
      regs->LIFCR = 0x3FU << hdma_usart2_rx.StreamIndex;          // clear all interrupts
      hdma_usart2_rx.Instance->M0AR = (uint32_t) uartDmaRxBuffer; // Set memory address for DMA again
      hdma_usart2_rx.Instance->NDTR = IT_RX_BUFFER_SIZE;          // Set number of bytes to receive
      hdma_usart2_rx.Instance->CR |= DMA_SxCR_EN;          // Start DMA transfer
    }
  //else
  //  {
  //    HAL_UART_IRQHandler (&huart2);
  //  }
}

///*!
// * \brief  Rx Transfer completed callbacks (called by DMA irq handler).
// *
// * Objectif : Cette fonction est appelée par interruption.
// * Elle envoie le message reçu à la tâche de fond
// * Au début du projet, utilisation de freeRTOS et donc utilisation d'un Mail Queue (function osMailPut)
// * En final, suppression de freeRTOS, reste une copie dans la globale telegramsBuffer.
// *
// * Algo :
// *      Récupération de l'adresse du buffer DMA et de la quantité de données reçue
// *      Recherche du premier buffer libre
// *      Copie du contenu du buffer DMA dans un buffer global
// *      Reinitialisation de la réception en DMA
// *      Affectation de la variable globale telegram avec le pointeur du buffer global qui vient d'être copié
// *
// * Pour vérifier la détection de la fin de réception et l'arrivée dans l'it DMA, on envoie à la tâche "par défaut"
// * un message pour allumer la led orange quelque instants : ledMessage = message
// *
// * \param  huart  Pointer to a UART_HandleTypeDef structure that contains
// *                the configuration information for the specified UART module.
// * \retval None
// */
//void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
//{
//  // espionDMAIrq++;
//
//  DMA_TypeDef *regs = (DMA_TypeDef *) hdma_usart2_rx.StreamBaseAddress;
//  for (int i = 0; i < ENOCEAN_TELEGRAMS_BUFFER_NUMBER; i++)
//    {
//      if (telegramsBuffer[i].libre)
//        {
//          if (__HAL_DMA_GET_IT_SOURCE (&hdma_usart2_rx, DMA_IT_TC) != RESET)   // if the source is TC
//            {
//              /* Clear the transfer complete flag */
//              regs->LIFCR = DMA_FLAG_TCIF0_4 << hdma_usart2_rx.StreamIndex;
//
//              /* Get the length of the data */
//              telegramsBuffer[i].length = DMA_RX_BUFFER_SIZE - hdma_usart2_rx.Instance->NDTR;
//
//              // Write received data for UART main buffer for manipulation later
//              // memcpy (telegramsBuffer[i].data, dma_Rx_Buffer, telegramsBuffer[i].length); // Copy first part
//              for (int j = 0; j < telegramsBuffer[i].length; j++)
//                {
//                  telegramsBuffer[i].data[j] = dma_Rx_Buffer[j];
//                }
//
//              telegramsBuffer[i].libre = false;
//              telegram = &telegramsBuffer[i];
//              break;
//            }
//        }
//    }
//
//  // Prepare DMA for next transfer
//  // Important! DMA stream won't start if all flags are not cleared first
//  regs->LIFCR = 0x3FU << hdma_usart2_rx.StreamIndex; // clear all interrupts
//  hdma_usart2_rx.Instance->M0AR = (uint32_t) dma_Rx_Buffer; // Set memory address for DMA again
//  hdma_usart2_rx.Instance->NDTR = DMA_RX_BUFFER_SIZE; // Set number of bytes to receive
//  hdma_usart2_rx.Instance->CR |= DMA_SxCR_EN; // Start DMA transfer
//}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Serial::Serial (UART_HandleTypeDef *h)
{
  uartHandle = h;
  objectsList[nbObjects++] = this;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Serial::Serial (UART_HandleTypeDef *h, MGPIO_HandleTypeDef *p)
{
  uartHandle = h;
  rs485pin = p;
  objectsList[nbObjects++] = this;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Serial::Serial (I2C_HandleTypeDef *h)
{
  i2cHandle = h;
  objectsList[nbObjects++] = this;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::initialisation (void)
{
  HAL_GPIO_WritePin (rs485pin->port, rs485pin->pin, GPIO_PIN_RESET); // mode reception
  // HAL_UART_Receive_IT (uartHandle, (uint8_t*) uartRxtmpA, IT_RX_BUFFER_SIZE);

  HAL_UART_Receive_DMA (uartHandle, (uint8_t*) uartDmaRxBuffer, DMA_RX_BUFFER_SIZE);

  //__HAL_DMA_ENABLE_IT (&hdma_usart1_rx, DMA_IT_TC); // enable DMA Rx complete interrupt
  __HAL_UART_ENABLE_IT(uartHandle, UART_IT_IDLE);      // enable idle line interrupt

  uartRxCount = 0;
  uartFlagRxTmpA = true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::xferDmaReadBuffer (uint8_t * buffer, int length)
{
  for (int i = 0; i < length; i++)
    {
      uartRxBuffer[uartRxHead++] = *buffer++;
      if (sizeof(uartRxBuffer) <= uartRxHead)
        {
          uartRxHead = 0;
        }
      uartRxCount++;
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool Serial::isDataReady (void)
{
  return ((uartRxCount != 0) ? true : false);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t Serial::read (void)
{
  uint8_t readValue = 0;

  if (uartRxCount != 0)
    {
      readValue = uartRxBuffer[uartRxTail++];
      if (sizeof(uartRxBuffer) <= uartRxTail)
        {
          uartRxTail = 0;
        }

      //__HAL_UART_DISABLE_IT(uartHandle, UART_IT_RXNE);
      uartRxCount--;
      //__HAL_UART_ENABLE_IT(uartHandle, UART_IT_RXNE);
    }
  return readValue;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::sendBuffer (uint8_t *transmissionBuffer, uint16_t msgSize)
{
  HAL_Delay (1);
  HAL_GPIO_WritePin (rs485pin->port, rs485pin->pin, GPIO_PIN_SET); // mode emission
  HAL_Delay (1);
  HAL_UART_Transmit_DMA (uartHandle, transmissionBuffer, msgSize);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::sendBuffer (const char *transmissionBuffer)
{
  HAL_Delay (1);
  HAL_GPIO_WritePin (rs485pin->port, rs485pin->pin, GPIO_PIN_SET); // mode emission
  HAL_Delay (1);
  HAL_UART_Transmit_DMA (uartHandle, (uint8_t*) transmissionBuffer, strlen (transmissionBuffer));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::Handler_UART_TxCpltCallback (UART_HandleTypeDef *huart)
{
  for (int i = 0; i < nbObjects; i++)
    {
      if (huart == objectsList[i]->uartHandle)
        {
          objectsList[i]->UART_TxCpltCallback ();
          break;
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
inline void Serial::UART_TxCpltCallback (void)
{
  HAL_GPIO_WritePin (rs485pin->port, rs485pin->pin, GPIO_PIN_RESET); // mode reception
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::Handler_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  for (int i = 0; i < nbObjects; i++)
    {
      if (huart == objectsList[i]->uartHandle)
        {
          objectsList[i]->UART_RxCpltCallback ();
          break;
        }
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Serial::UART_RxCpltCallback (void)
{
  //uartRxBuffer[uartRxHead++] = (uint8_t) (uartHandle->Instance->DR & (uint16_t) 0x00FF);
//  for (int i = 0; i < IT_RX_BUFFER_SIZE; i++)
//    {
//      uartRxBuffer[uartRxHead++] = uartRxtmp[i];
//    }
//  HAL_UART_Receive_IT (uartHandle, (uint8_t*) uartRxtmp, IT_RX_BUFFER_SIZE);
//  if (sizeof(uartRxBuffer) <= uartRxHead)
//    {
//      uartRxHead = 0;
//    }
//  uartRxCount++;
  while (1)
    {
      HAL_GPIO_TogglePin (LED_GPIO_Port, LED_Pin);
      HAL_Delay (10);
    }
}

//bool Serial::isDataReadyII (void)
//{
//  uint16_t RxXferCountTmp;
//  uint8_t *pBufferTmp;
//  bool ret = false;
//
////  uint32_t tmp1 = __HAL_UART_GET_FLAG(uartHandle, UART_FLAG_ORE);
////  if (tmp1 != RESET)
////    {
////      __HAL_UART_CLEAR_OREFLAG(uartHandle);
////    }
//  if (uartHandle->ErrorCode != RESET)
//    {
////      while (1);
//    }
//
//  if (uartRxCount != 0)
//    ret = true;
//
//  if (uartHandle->RxXferCount != uartHandle->RxXferSize)
//    {
//      __HAL_UART_DISABLE_IT(uartHandle, UART_IT_RXNE);
//      uartHandle->pRxBuffPtr = (uartFlagRxTmpA) ? uartRxtmpB : uartRxtmpA;
//      uartFlagRxTmpA = (uartFlagRxTmpA) ? false : true;
//      RxXferCountTmp = uartHandle->RxXferCount;
//      uartHandle->RxXferCount = IT_RX_BUFFER_SIZE;
//      uartHandle->RxXferSize = IT_RX_BUFFER_SIZE;
//      __HAL_UART_ENABLE_IT(uartHandle, UART_IT_RXNE);
//
//      pBufferTmp = (uartFlagRxTmpA) ? uartRxtmpB : uartRxtmpA;
//      for (int i = RxXferCountTmp; i < IT_RX_BUFFER_SIZE; i++)
//        {
//          uartRxBuffer[uartRxHead++] = *pBufferTmp++;
//          if (sizeof(uartRxBuffer) <= uartRxHead)
//            {
//              uartRxHead = 0;
//            }
//          uartRxCount++;
//        }
//      ret = true;
//    }
//  return ret;
//}

/* end of file */

