/*
 * PWMClass.h - Libreria para la agrupacion y uso de componentes controlados mediante PWM
 * Creado por Álvaro Palmero, 22 Mayo 2018
 */

 #ifndef PWMCLASS_H
 #define PWMCLASS_H

 #include "Arduino.h"


 class PWMClass{
  private:
    int pin;
    int potencia;
  public:
    PWMClass();
  
    void setPin(int a);
    void setPotencia(int a);

    void enciende();
    void apaga();
 };

 #endif
