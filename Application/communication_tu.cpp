/*
 * communication_tu.c
 *
 *  Created on: 21 avr. 2020
 *      Author: phifo
 */

#include <stdint.h>
#include <stdio.h>

#include "gcov_support_stm32.h"
#include "communication.h"

extern "C" void initialise_monitor_handles(void);

/**
 * @brief  The assert_param macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls assert_failed function
 *         which reports the name of the source file and the source
 *         line number of the call that failed.
 *         If expr is true, it returns no value.
 * @retval None
 */
#define assert_unitTest(cas,expr) ((expr) ? (void)0U : my_assert_failed((uint8_t *)__FILE__, __LINE__, cas))

void my_assert_failed (uint8_t *file, uint32_t line, int cas)
{
  char buffer[255];
  sprintf (buffer, "PB ligne %lu, cas num %d, file %s", line, cas, file);
  while (1)
    ;
}


extern "C" void mainUnitTest (void);


uint8_t binToBase64 (uint8_t size, uint8_t *tableIn, uint8_t *tableOut);
uint8_t base64ToBin (uint8_t c, uint8_t *index, bool reinit);

void binToBase64_tu (void)
{
  uint8_t input1[5] =
    { 0xD2, 0x00, 0x55, 0xAA, 0 };
  uint8_t output1[6] =
    { 0, 0, 0, 0, 0, 0 };
  uint8_t attendu1[6] =
    { 100, 80, 49, 69, 90, 80 };
  binToBase64 (4, input1, output1);
  for (int i = 0; i < 6; i++)
    assert_unitTest(i, output1[i] == attendu1[i]);

  uint8_t input2[16] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
  uint8_t output2[22] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uint8_t attendu2[22] =
    { 48, 64, 52, 49, 48, 64, 52, 49, 48, 64, 52, 49, 48, 64, 52, 49, 48, 64,
        52, 49, 48, 64 };
  binToBase64 (16, input2, output2);
  for (int i = 0; i < 21; i++)
    assert_unitTest(i, output2[i] == attendu2[i]);

}

void base64ToBin_tu (void)
{
  uint8_t input1[6] =
    { 100, 80, 49, 69, 90, 80 };
  uint8_t output1[5] =
    { 0, 0, 0, 0, 0 };
  uint8_t attendu1[5] =
    { 0xD2, 0x00, 0x55, 0xAA, 0 };
  uint8_t index;
  base64ToBin (0, &index, true);
  for (int i = 0; i < 6; i++)
    output1[index] = base64ToBin (input1[i], &index, false);

  for (int i = 0; i < 4; i++)
    assert_unitTest(i, output1[i] == attendu1[i]);

  uint8_t output2[16] =
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uint8_t attendu2[16] =
    { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
  uint8_t input2[22] =
    { 48, 64, 52, 49, 48, 64, 52, 49, 48, 64, 52, 49, 48, 64, 52, 49, 48, 64,
        52, 49, 48, 64 };
  base64ToBin (0, &index, true);
  for (int i = 0; i < 22; i++)
    output2[index] = base64ToBin (input2[i], &index, false);

  for (int i = 0; i < 16; i++)
    assert_unitTest(i, output2[i] == attendu2[i]);
}

void binToBinbyBase64_ti (void)
{
  uint8_t input[16];
  uint8_t output[16];
  uint8_t tmp[30];
  uint8_t index;

  for (int n = 0; n < 255; n++)
    {
      for (int i = 0; i < 16; i++)
        {
          input[i] = n;
        }
      binToBase64 (16, input, tmp);
      base64ToBin (0, &index, true);
      for (int i = 0; i < 22; i++)
        output[index] = base64ToBin (tmp[i], &index, false);

      for (int i = 0; i < 16; i++)
        assert_unitTest(i, output[i] == n);
    }
}

void mainUnitTest (void)
{
  initialise_monitor_handles(); // lib rdimon
  gcov_init ();

//  binToBinbyBase64_ti ();

  base64ToBin_tu ();
  //binToBase64_tu ();

 gcov_write ();

  while (1)
    ; // all test OK
}

