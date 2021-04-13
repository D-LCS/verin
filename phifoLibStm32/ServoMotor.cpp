#include <math.h>

#include "main.h"
#include "application.h"

#include "HDriver.h"
#include "DcMotor.h"
#include "Corrector.h"
#include "PID.h"
#include "ISensor.h"
#include "AnalogInput.h"

#include "ServoMotor.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define SEUIL_POSITION 5

#define NO_CONSIGNE_FORWARD 99999
#define NO_CONSIGNE_REVERSE -99999

extern uint8_t boardAddress;

void ServoMotor::initialisation (void)
{
  motor_->initialisation ();
  corrector_->initialisation ();
  lastStatus_ = ServoMotor_PositionReached;
  lackOfMovementCmpt_ = 100;
  ilsInversion_ = false;
  lastIlsIncrement_ = 1;
  lastPosition_ = getPosition ();
  consigne_ = getPosition ();
  maxPWMDutyCycle_ = 100;
}

void ServoMotor::setWiringMode (HDriver_wiring_t w)
{
  motor_->setWiring (w);
}

// if motor stopped, return last increment
int32_t ServoMotor::getSensorIncrement (void)
{
  if (ilsInversion_ == false)
    {
      if (motor_->getCurrentMove () == DcMotor::DCMOTOR_MOVE_FORWARD)
        lastIlsIncrement_ = 1;
      else if (motor_->getCurrentMove () == DcMotor::DCMOTOR_MOVE_REVERSE)
        lastIlsIncrement_ = -1;
    }
  else
    {
      if (motor_->getCurrentMove () == DcMotor::DCMOTOR_MOVE_FORWARD)
        lastIlsIncrement_ = -1;
      else if (motor_->getCurrentMove () == DcMotor::DCMOTOR_MOVE_REVERSE)
        lastIlsIncrement_ = 1;
    }
  return lastIlsIncrement_;
}

int32_t ServoMotor::getConsigne (void)
{
  return consigne_;
}

void ServoMotor::setConsigne (correctorFloat_t c)
{
  consigne_ = (int32_t) c;
}

int32_t ServoMotor::getError (void)
{
  return consigne_ - getPosition ();
}

int32_t ServoMotor::getPosition (void)
{
  int32_t position;

  if ((boardAddress & PHOENIX_LA31_MASK) != 0)
    {
      position = analogPosition_->getValue ();
    }
  else
    {
      position = *ilsPosition_;
    }
  return position;
}

void ServoMotor::setPosition (int32_t p)
{
  if ((boardAddress & PHOENIX_LA31_MASK) == 0)
    {
      *ilsPosition_ = p;
      consigne_ = p;
    }
}

bool ServoMotor::noMouvementDetected (int32_t actualPosition)
{
  bool ret = false;

  if (lastPosition_ == actualPosition)
    {
      if (lackOfMovementCmpt_-- <= 0)
        {
          ret = true;
        }
    }
  else
    {
      lastPosition_ = actualPosition;
      lackOfMovementCmpt_ = 100;
    }
  return ret;
}

bool ServoMotor::reachedPosition (int32_t consigne, int32_t actual)
{
  bool ret = false;

  if (consigne > actual)
    {
      if ((consigne - actual) < SEUIL_POSITION)
        ret = true;
    }
  else
    {
      if ((actual - consigne) < SEUIL_POSITION)
        ret = true;
    }
  return ret;
}

float ServoMotor::doAccelerationRamp (float consigne, float actual)
{
  float error;

  rampConsigne_ += rampDelta_;
  error = consigne - actual;
  if (error > 0)
    {
      if (rampConsigne_ < consigne_) // else return normal error
        {
          error = (float (rampConsigne_)) - actual;
        }
    }
  else
    {
      if (rampConsigne_ > consigne_) // else return normal error
        {
          error = (float (rampConsigne_)) - actual;
        }
    }
  return error;
}

ServoMotor_Result_t ServoMotor::moveMotorToPosition (int32_t newConsigne)
{
  if (lastStatus_ == ServoMotor_PositionReached)
    {
      lastStatus_ = ServoMotor_Positionning;
      consigne_ = newConsigne;

      rampDelta_ = (consigne_ > getPosition ()) ? +2 : -2;
      rampConsigne_ = getPosition () + rampDelta_;

      lackOfMovementCmpt_ = 100;
      maxPWMDutyCycle_ = 100;
      periodicManagement ();
    }

  return lastStatus_;
}

/*
 * Bizarre - on utilise pas la vitesse. Du coup, je doute qu'elle fonctionne !!!
 * \todo : à verifier
 *
 */
ServoMotor_Result_t ServoMotor::moveMotor (bool directionForward, int speed)
{
  if (lastStatus_ == ServoMotor_PositionReached)
    {
      lastStatus_ = ServoMotor_Positionning;
      consigne_ = (directionForward) ? NO_CONSIGNE_FORWARD : NO_CONSIGNE_REVERSE;

      rampDelta_ = (consigne_ > getPosition ()) ? +2 : -2;
      rampConsigne_ = getPosition () + rampDelta_;

      lackOfMovementCmpt_ = 100;
      maxPWMDutyCycle_ = speed;
      periodicManagement ();
    }
  return lastStatus_;
}

ServoMotor_Result_t ServoMotor::stopMotor (void)
{
  if (lastStatus_ == ServoMotor_Positionning)
    {
      lastStatus_ = ServoMotor_PositionReached;
      motor_->brake ();
      periodicManagement ();
    }

  return lastStatus_;
}

void ServoMotor::periodicManagement (void)
{
  int32_t position;

  if ((boardAddress & PHOENIX_LA31_MASK) != 0)
    {
      analogPosition_->periodicManagement ();
    }
  position = getPosition ();
  currentSensor_->periodicManagement ();

  if (lastStatus_ == ServoMotor_OverCurrentDetected)
    {
      if (timeoutOvercurrent_ <= HAL_GetTick ())
        {
          lastStatus_ = ServoMotor_PositionReached;
        }
    }

  else if (lastStatus_ == ServoMotor_Positionning)
    {
      if (noMouvementDetected (position))
        {
          lastStatus_ = ServoMotor_PositionReached;
          motor_->freeSpin ();
        }
      else if (reachedPosition (consigne_, position))
        {
          lastStatus_ = ServoMotor_PositionReached;
          motor_->freeSpin ();
        }
      else if (currentSensor_->isOverCurrentDetected ())
        {
          lastStatus_ = ServoMotor_OverCurrentDetected;
          timeoutOvercurrent_ = HAL_GetTick () + TIMEOUT_OVERCURRENT;
          motor_->freeSpin ();
        }
      else // normal movement
        {
          lastStatus_ = ServoMotor_Positionning;
          float erreur_f = doAccelerationRamp ((float) consigne_, (float) position);
          int correction = (int) corrector_->cyclicManagement (erreur_f);
          if (correction > 0)
            {
              motor_->forwards (constrain(correction, 0, maxPWMDutyCycle_));
            }
          else
            {
              motor_->reverse (constrain(fabs (correction), 0, maxPWMDutyCycle_));
            }
        }
    }
}

// end of file

