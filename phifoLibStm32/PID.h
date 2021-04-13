


#ifndef __CORRECTOR_PID_H__
#define __CORRECTOR_PID_H__



class PID : public Corrector
{
public:

  /*! Constructeur
   *
   */
  PID (void);

  /*!
   *
   */
  void initialisation (void);
  void initialisation (correctorFloat_t coeffP, correctorFloat_t Te);
  void initialisation (correctorFloat_t coeffP, correctorFloat_t coeffI, correctorFloat_t Te);
  void initialisation (correctorFloat_t coeffP, correctorFloat_t coeffI, correctorFloat_t coeffD, correctorFloat_t Te);

  void zieglerNicholsCoefficientsCalculations (correctorFloat_t Ku, correctorFloat_t Tu, correctorFloat_t Te);

  /*!
   *
   */
  correctorFloat_t cyclicManagement (correctorFloat_t error);


private:
  correctorFloat_t Kp;
  correctorFloat_t Ki;
  correctorFloat_t Kd;

  correctorFloat_t samplingPeriod;

  correctorFloat_t errorsSum = 0.0; //!< Somme des erreurs pour l'intégrateur (pour le calcul integral - I)
  correctorFloat_t previousError = 0.0; //!< memoire pour le calcul de la dérivée - D

};

#endif /* __CORRECTOR_PID_H__ */
