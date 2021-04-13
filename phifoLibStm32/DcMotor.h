




class DcMotor
{
public:

  enum  dcMotorState_t {DCMOTOR_STOPPED, DCMOTOR_MOVE_FORWARD, DCMOTOR_MOVE_REVERSE } ;

  DcMotor (MPWM_HandleTypeDef * p, HDriver * h) {timer = p; hdriver = h;};

  void initialisation (void);
  void brake (void);
  void freeSpin (void);

  void forwards (int);
  void reverse (int);

  float getPosition (void);
  float getSpeed (void);
  float getCurrent (void);

  int32_t getRawPosition (void);
  int32_t getRawSpeed (void);
  int32_t getRawCurrent (void);

  dcMotorState_t getCurrentMove (void);

  void setWiring (HDriver_wiring_t w);

private:
  MPWM_HandleTypeDef * timer;
  HDriver * hdriver;
  uint8_t status;

};

