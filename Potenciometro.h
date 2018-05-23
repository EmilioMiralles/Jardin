/*
  Motor.h - Librería para el uso de los motores
  Creado por Álvaro Palmero y Emilio Miralles, 27 Abril 2018
 */

#ifndef POTENCIOMETRO_H
#define POTENCIOMETRO_H

#include "Arduino.h"

class Potenciometro{
  private:
    float limite_inferior;
    float limite_superior;
    float angulo;
    float porcent_seguridad;
    int pin;
    float valor;
    
  public:
    void setLimites(float a, float b);
    float getAngulo();
    void setPorcent_seguridad(float a);
    void setpin(int a);
    void setValor(float a);
};


#endif
