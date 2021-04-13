#ifndef __LED_H__
#define __LED_H__

/**
 * \brief
 */
typedef enum
{
  DIRECT_LOGIC, //!< Pin = 1 ==> Led On
  REVERSE_LOGIC //!< Pin = 0 ==> Led On
} wiredLogic_t;

class Led
{
public:

  /*! Constructeur
   *
   * Paramètres : le port GPIO qui est relié à la led, le type de connexion
   */
  Led (const MGPIO_HandleTypeDef * gpio, wiredLogic_t logic);

  /*!
   * \brief Gestion led de debug
   *
   * Clignotement de led avec un allumage de tOn pour un cycle complet de tTotal
   * Cette fonction est prévue pour être appelée cycliquement, cadencée par une base de temps
   *
   * \param tOn [in] Durée de l'allumage (est multiplié par la base de temps de la boucle principale)
   * \param tTotal [in] Durée de la période complète (est multiplié par la base de temps principale)
   */
  void periodicManagement (uint8_t tOn, uint8_t tTotal);

  /*!
   *
   */
  inline void on (void)
  {
    HAL_GPIO_WritePin (port, pin, led_on);
  }

  /*!
   *
   */
  inline void off (void)
  {
    HAL_GPIO_WritePin (port, pin, led_off);
  }

  /*!
   *
   */
  inline void toggleCycleLed (void)
  {
    cycleLed = !cycleLed;
  }

  /*!
   *
   */
  void sendMorseLetter (char letter);

  /*!
   *
   */
  void sendMorse (char * sentence);



private:
  GPIO_TypeDef *port;
  uint16_t pin;

  bool cycleLed = true; // accessible par une requete serielle (permet de vérifier le fonctionnement des communications)
  uint8_t cmptLed = 1;
  bool morseInUse = false;

  GPIO_PinState led_on;
  GPIO_PinState led_off;
};

#endif /* __LED_H__ */

