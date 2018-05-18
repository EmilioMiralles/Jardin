/*
  Interfaz.h - Librería para el uso de la interfaz con la que se controlará el robot
  Creado por Álvaro Palmero, 12 marzo 2018
 */

#ifndef INTERFAZ_H
#define INTERFAZ_H

#define pi 3.1416

#include "Arduino.h"
#include "Motor.h"
#include "Math.h"

#define L1 252
#define L2 200
#define L3 100

struct coordenadas{
  float x;
  float y;
  float z;
};

struct referencia{
  float Rx;
  float Ang1;
  float Ang2;
  float Ang3;
};

class Interfaz{
private:
  coordenadas posicion;     //Actualmente la coordenada se trata de un único dato puesto que se está modelando un unico motor
                            //Más adelante se podrá modificar esta variable de forma que coordenadas se trate de una estructura de 3 coordenadas
  referencia ref;
public:
  void imprimirInterfaz(motor a, motor b);
  void interaccionInterfaz(motor a, motor b, motor c);
  coordenadas getPosicion(motor a, motor b, motor c);
  referencia cinInversa(float x, float y, float z);
  
};
  
#endif
