


#ifndef __CORRECTOR_H__
#define __CORRECTOR_H__



typedef float correctorFloat_t;


class Corrector
{
public:

  /*! Constructeur
   *
   */
  // Corrector () {};

  /*!
   *
   */
  virtual void initialisation (void) = 0;

  /*!
   *
   */
  virtual correctorFloat_t cyclicManagement (correctorFloat_t error) = 0;


private:

};

#endif /* __CORRECTOR_H__ */

