


#include "main.h"
#include "application.h"
#include "Led.h"

#define DOT_VALUE 50

const char morseTable[] = { "A .-,"
    "B -...,"
    "C -.-.,"
    "D -..,"
    "E .,"
    "F ..-.,"
    "G --.,"
    "H ....,"
    "I ..,"
    "J .---,"
    "K -.-,"
    "L .-..,"
    "M --,"
    "N -.,"
    "O ---,"
    "P .--.,"
    "Q --.-,"
    "R .-.,"
    "S ...,"
    "T -,"
    "U ..-,"
    "V ...-,"
    "W .--,"
    "X -..-,"
    "Y -.--,"
    "Z --..#" };

Led::Led (const MGPIO_HandleTypeDef * gpio, wiredLogic_t logic)
{
  port = gpio->port;
  pin = gpio->pin;

  if (logic == DIRECT_LOGIC)
    {
      led_on = GPIO_PIN_SET;
      led_off = GPIO_PIN_RESET;
    }
  else // REVERSE_LOGIC
    {
      led_on = GPIO_PIN_RESET;  // inversé parce que la led est connectée au +
      led_off = GPIO_PIN_SET;  // idem
    }
}

void Led::periodicManagement (uint8_t tOn, uint8_t tTotal)
{
  if (morseInUse == true)
    {
      sendMorseLetter ('#');
    }
  else if (cycleLed == false)
    {
      HAL_GPIO_WritePin (port, pin, led_on);
    }
  else if (--cmptLed == 0)
    {
      HAL_GPIO_WritePin (port, pin, led_off);
      cmptLed = tTotal;
    }
  else if (cmptLed == tOn)
    {
      HAL_GPIO_WritePin (port, pin, led_on);
    }
}

void Led::sendMorseLetter (char letter)
{
  static const char *ptr;
  static bool space = false;
  static bool test = false;

  if (!morseInUse)
    {
      if (letter >= 'A' && letter <= 'Z')
        {
          ptr = &morseTable[0];
          while (*ptr++ != letter && *ptr != '#'); // recherche de la lettre dans la table
          if (*ptr++ != ' ')
            {
              return;
            }
          else
            {
              cmptLed = 1;
              morseInUse = true;
              space = true;
            }
        }
      else
        {
          return;
        }
    }

  if (--cmptLed == 0)
    {
      if (!space)
        {
          HAL_GPIO_WritePin (port, pin, led_off);
          space = true;
          cmptLed = DOT_VALUE;
        }
      else
        {
          switch (*ptr)
            {
            case '.':
              HAL_GPIO_WritePin (port, pin, led_on);
              cmptLed = DOT_VALUE;
              ptr++;
              space = false;
              test = false;
              break;

            case '-':
              HAL_GPIO_WritePin (port, pin, led_on);
              cmptLed = 3 * DOT_VALUE;
              ptr++;
              space = false;
              test = false;
              break;

            case ',': // fin de la lettre
              if (test)
                {
                  morseInUse = false;
                }
              else
                {
                  cmptLed = 2 * DOT_VALUE;
                  test = true;
                }
              break;

            default:

              break;
            }
        }
    }
}

/*!
 *
 */
void Led::sendMorse (char *sentence)
{

}

