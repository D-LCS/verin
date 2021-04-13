
#include "Corrector.h"
#include "PID.h"

PID::PID (void)
{
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
}

void PID::initialisation (void)
{
  Kp = 0.0;
  Ki = 0.0;
  Kd = 0.0;
}

void PID::initialisation (correctorFloat_t coeffP, correctorFloat_t te)
{
  Kp = coeffP;
  Ki = 0.0;
  Kd = 0.0;
  samplingPeriod = te;
  errorsSum = 0.0;
  previousError = 0.0;
}

void PID::initialisation (correctorFloat_t coeffP, correctorFloat_t coeffI, correctorFloat_t te)
{
  initialisation (coeffP, te);

  Ki = coeffI;
  errorsSum = 0.0;
}

void PID::initialisation (correctorFloat_t coeffP, correctorFloat_t coeffI, correctorFloat_t coeffD, correctorFloat_t te)
{
  initialisation (coeffP, coeffI, te);
  Kd = coeffD;
  previousError = 0.0;
}



void PID::zieglerNicholsCoefficientsCalculations (correctorFloat_t Ku, correctorFloat_t Tu, correctorFloat_t Te)
{
  // Parametres issus de l'expérimentation (nécessaires pour le calculs des coefficients de Ziegler-Nichols)
//  const myFloat_t Ku = 15.0; //!< Gain minimum pour l'entretien de l'oscillation
//  const myFloat_t Tu = 3.5; //!< Période d'oscillation mesurée (en s)
//  const myFloat_t Te = 0.01; //!< Période d'échantillonage

// Calculs des coefficients
  Kp = (Ku * 0.6); //!< Coefficient proportionnel : Kp = Ku * 0.6
  Ki = (Kp / (Tu / 2.0)); //!< Coefficient intégrateur : Ki = Kp / Tu/2
  Kd = ((Kp * Tu / 8.0) / Te); //!< Coefficient dérivateur : Kd = 1/Te * Kp * Tu / 8
}

/*! \brief Calcul du PID
 *
 * \param [in] erreur : l'ecart entre la valeur actuelle et la consigne
 *
 * \output : la valeur de la nouvelle commande
 *
 */
correctorFloat_t PID::cyclicManagement (correctorFloat_t error)
{
  correctorFloat_t command;
  correctorFloat_t deltaError;

  deltaError = error - previousError; // calcul de la dérivée (approximation)
  previousError = error; // memorisation pour le calcul à la periode suivante

  errorsSum += error * samplingPeriod; // calcul de l'intégrale (approximation)

  // PID : calcul de la commande
  command = (Kp * error) + (Ki * errorsSum) + (Kd * deltaError);

  return command;
}

//correctorFloat_t PID::cyclicManagementPOnly (correctorFloat_t error)
//{
//  correctorFloat_t commande;
//
//  // PID : calcul de la commande
//  commande = (Kp * error);
//
//  return commande;
//}

