/*
  Motor.cpp - Librería para el uso de los motores
  Creado por Álvaro Palmero y Emilio Miralles, 27 Abril 2018
 */

 #include "Potenciometro.h"

void Potenciometro::setPorcent_seguridad(float a){
  porcent_seguridad = a;
}

void Potenciometro::setpin(int a){
  pinMode(a, INPUT);
  pin = a;
}

float Potenciometro::getAngulo(){
  float lectura = analogRead(pin);
  angulo = ((lectura)/(1023))*155 + valor;
  
  return angulo;
}

void Potenciometro::setValor(float a){
  valor = a;
}

