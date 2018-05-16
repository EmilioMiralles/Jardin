/*
  Motor.cpp - Librería para el uso de los motores
  Creado por Álvaro Palmero y Emilio Miralles, 27 Abril 2018
 */

 #include "Potenciometro.h"

 void Potenciometro::setLimites(float a, float b){
  limite_inferior = a;
  limite_superior = b;
 }

void Potenciometro::setPorcent_seguridad(float a){
  porcent_seguridad = a;
}

void Potenciometro::setpin(int a){
  pin = a;
}

float Potenciometro::getAngulo(){
  float lectura = analogRead(pin);
  angulo = ((lectura-limite_inferior)/(limite_superior-limite_inferior))*155 + 12.5;
  return angulo;
}

