
/*!
 *
 */
typedef enum
{
  ServoMotor_Ok = 0x00, /*!< Everything OK */
  ServoMotor_Positionning, ServoMotor_PositionReached, ServoMotor_OverCurrentDetected, ServoMotor_ManualStopped
} ServoMotor_Result_t;


#define TIMEOUT_OVERCURRENT 1000 //!< valeur en millisecondes

/*!
 *
 *
 */
class ServoMotor
{
public:

  /*!
   *
   */
  ServoMotor (DcMotor *m, Corrector *c, volatile int32_t *p)
  {
    motor_ = m;
    corrector_ = c;
    ilsPosition_ = p;
  }
  ;
  ServoMotor (DcMotor *m, Corrector *c, volatile int32_t *p, AnalogInput *a, CurrentSensor *i)
  {
    motor_ = m;
    corrector_ = c;
    ilsPosition_ = p;
    analogPosition_ = a;
    currentSensor_ = i;
  }
  ;

  /*! \brief fonction d'initialisation
   *
   * Appelle toutes les fonctions d'initialisation des objets qui composent le servoMoteur (capteur, asservissement, etc)
   *
   */
  void initialisation (void);

  /*! \brief  application de la correction calculée en tenant compte de la saturation
   *
   * Desactivation des PWM si offset = 0
   *
   * @param consigne [in] : position demandée
   * @param endPositionDetection [in] : recherche de la fin de course, et autorisation de modification de la variable globale
   *
   */
  //ServoMotor_Result_t moveMotorToPosition (int32_t consigne, bool endPositionDetection);
  ServoMotor_Result_t moveMotorToPosition (int32_t newConsigne);

  /*!
   *
   */
  void setWiringMode (HDriver_wiring_t w);


  /*!
   *
   */
  int32_t getSensorIncrement (void);

  /*!
   *
   */
void setIlsInversion (bool b) {ilsInversion_ = b;};

  /*! \brief Commande de mouvement du servo à une vitesse déterminée
   *
   *
   * Utilisation des butées NO_CONSIGNE_XXX pour continuer à utiliser les fonctions d'asservissement
   *
   * \param [in] directionForward : booleen pour indique la direction du mouvement
   * \param [in] speed : vitesse cible
   */
  ServoMotor_Result_t moveMotor (bool directionForward, int vitesse);

  /*! \brief Commande d'arret du moteur avec frein
   *
   *  Commande d'arret du moteur. Ne fonctionne que si le servo est en mouvement :
   *  soit un mouvement vers une position déterminée, soit un mouvement à une vitesse demandée
   */
  ServoMotor_Result_t stopMotor (void);


  /*! \brief continu le mouvement après un ordre donné ou ne fait rien lorsque la dernière commande est terminée
   *
   */
  void periodicManagement (void);

  /*! \brief
   *
   */
  int32_t getError (void);

  /*! \brief
    *
    */
  int32_t getConsigne (void);

  /*! \brief
    *
    */
  void setConsigne (correctorFloat_t c);

  /*!
   *
   */
  int32_t getPosition (void);
  void setPosition (int32_t p);


private:

  /*! detection d'absence de mouvement
   *
   */
  bool noMouvementDetected (int32_t p);

  /*!
   *
   */
  bool reachedPosition (int32_t consigne, int32_t actual);

  /*!
   *
   */
  float doAccelerationRamp (float consigne, float actual);

  /*!
   *
   */
  int32_t getActualPosition (uint8_t address);

  // pointeurs sur les objets qui composent le servoMoteur
  DcMotor *motor_; //!< le moteur
  Corrector *corrector_; //!< le correcteur
  CurrentSensor *currentSensor_; //!< le capteur de courant
  uint32_t timeoutOvercurrent_; //!< Date de fin de timeout

  volatile int32_t *ilsPosition_; //!< la valeur de la position actuelle si LA28 (ILS)
  bool ilsInversion_;
  volatile int32_t lastIlsIncrement_; //!<
  AnalogInput *analogPosition_; //!< la valeur de la position actuelle si LA31 (analogique)

  // Variables de l'objet
  int32_t consigne_; //!< dernière position de consigne demandée
  int32_t rampConsigne_; //!< Valeur temporaire permettant de faire une rampe d'acceleration
  int32_t rampDelta_; //!< Valeur du pas d'incrémentation pour la rampe
  ServoMotor_Result_t lastStatus_; //!< dernier état constaté
  int32_t lastPosition_;
  int32_t lackOfMovementCmpt_;
  uint8_t maxPWMDutyCycle_;
};

// end of file

