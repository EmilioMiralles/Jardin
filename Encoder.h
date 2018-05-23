/*
  Encoder.h - Libreria para el uso del encoder en el motor
  Creado por Alvaro Palmero, 11 marzo 2018
*/
#ifndef ENCODER_H
#define ENCODER_H

#include "Arduino.h"

class encoder{
private:
  int memoria=0;
  int vueltas=0;
  int band=0;
  int pin;
  int i;      //Usado para que no cuente más vueltas de las debidas en contadorMemoria();
public:
  void contadorMemoria();
  void restadorMemoria();
  void reset();
  void setPin(int p);
  void setBandera();
  void imprimir();
  int getMemoria();
  int getVueltas();
  int setVueltas(int a);
};

#endif
