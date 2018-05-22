/*
  PWMClass.cpp - Libreria para la agrupacion y uso de componentes controlados mediante PWMLibrería para el uso de los motores
  Creado por Álvaro Palmero, 22 Mayo 2018
 */

 #include "PwmClass.h"


PWMClass::PWMClass(){
  potencia = 120;
}
 

 void PWMClass::setPin(int a){
  pin = a;
  pinMode(pin, OUTPUT);
 }


void PWMClass::setPotencia(int a){
  potencia = a;
  if (potencia >= 255){
    potencia = 255;
  }
  else if (potencia <= 0){
    potencia = 0;
  }
}


void PWMClass::enciende(){
  analogWrite(pin, potencia);
}


void PWMClass::apaga(){
  analogWrite(pin, 0);
}

