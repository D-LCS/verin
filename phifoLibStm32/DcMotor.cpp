#include "main.h"
#include "application.h"

#include "HDriver.h"

#include "DcMotor.h"

void DcMotor::initialisation (void)
{
  setWiring (HDRIVER_Normal_Mode);
  hdriver->initialisation ();

  //maximumPwmValue = pwm->htim->Instance->ARR;  // recuperation de la valeur dans les configs timer
}

void DcMotor::setWiring (HDriver_wiring_t w)
{
  hdriver->setWiringMode(w);
}

void DcMotor::brake (void)
{
  hdriver->brake ();
}

void DcMotor::freeSpin (void)
{
  hdriver->freeSpin ();
}

void DcMotor::forwards (int dutyCycle)
{
  hdriver->forwards (dutyCycle);
}

void DcMotor::reverse (int dutyCycle)
{
  hdriver->reverse (dutyCycle);
}

DcMotor::dcMotorState_t DcMotor::getCurrentMove (void)
{
  if (hdriver->isForwardMove ())
    return DCMOTOR_MOVE_FORWARD;
  else if (hdriver->isReverseMove ())
    return DCMOTOR_MOVE_REVERSE;
  else
    return DCMOTOR_STOPPED;
}

// end of file
