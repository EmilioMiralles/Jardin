/*
  Motor.h - Librería para el uso de los motores
  Creado por Álvaro Palmero, 11 marzo 2018
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Encoder.h"
#include "Potenciometro.h"

class motor{
private:
  bool tipo;                    //Para el motor cartesiano con encoder tipo = 0 y para motor con potenciometro tipo = 1;
  int pin_ad, pin_at, pin_pwm;
  int pin_endstop;
  float posicion;
  float angulo;
  float avance;       
  int velocidad;      //Valor pensado para PWM entre 0 y 255
public:
  encoder encod;
  Potenciometro pot;

  motor();
  void setTipo(bool a);
  bool getTipo();
  void setPines(int a, int b, int c);
  void setAvance(float a);                //El valor que introduciremos de a será el valor del avance del motor por 1 vuelta
  float getFeedback();                    //En milimetros
  void setVelocidad(float a);
  void setEndstop(int a);
  
  void avanzar();
  void retroceder();
  void parar();
  void endstop();
  
  void imprimirFeedback();
  void imprimirVel();
  void imprimirAv();
 };

#endif
