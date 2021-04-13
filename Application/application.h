/*
 * application.h
 *
 *  Created on: Jun 19, 2020
 *      Author: phifo
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_


#define APPLICATION_IS_SLAVE 1
//#define APPLICATION_IS_MASTER 1


#define PHOENIX_LA31_MASK  0x40
#define PHOENIX_PARALLEL_MASK  0x20

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Liste des périphériques créés par CubeMX

extern ADC_HandleTypeDef hadc1;

extern CRC_HandleTypeDef hcrc;

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim11;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Types fréquemment utilisés

/*!
 *
 */
typedef struct __MGPIO_HandleTypeDef__
{
  GPIO_TypeDef * port;
  uint16_t pin;
} MGPIO_HandleTypeDef;

/*!
 *
 */
typedef struct __MPWM_HandleTypeDef__
{
  GPIO_TypeDef * port;
  uint16_t pin;
  TIM_HandleTypeDef * htim;
  uint32_t timChannel;
} MPWM_HandleTypeDef;

/*!
 *
 */
typedef union
{
  float f;
  uint16_t w[2];
  uint8_t b[4];
} floatToByte_t;

/*!
 *
 */
typedef union
{
  uint16_t val;
  struct
  {
    uint8_t l;
    uint8_t h;
  };
} word16Byte_t;

/*!
 *
 */
typedef union
{
  uint32_t val;
  struct
  {
    uint16_t l;
    uint16_t h;
  };
  struct
  {
    uint8_t ll;
    uint8_t lh;
    uint8_t hl;
    uint8_t hh;
  };
} word32Byte_t;


/*!
 * Interface entre STM32CubeMX et application
 */
#ifdef __cplusplus
extern "C" {
#endif

void application (void);

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_H_ */
