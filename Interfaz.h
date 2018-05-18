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

#define DISTJARDIN  190

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
  coordenadas posicion_final;
  referencia ref;
  
  coordenadas p[2];         //Puntos del movimiento que se tomarán en la trayectoria entre dos puntos.
  int bandera[1];           //Indica en que posicion se encuentra dentro de la trayectoria. Los valores (0, 0) , (0, 1) , (1, 0) y (1, 1) indican la posicion de la que parten en la trayectoria

  float margen = 0.05;      //Margen de error para que se alcance la posicion
public:
  void imprimirInterfaz(motor a, motor b);          //Imprime la interfaz
  void interaccionInterfaz(motor a, motor b);       //Lee los datos introducidos por Serial
  
  coordenadas getPosicion(motor a, motor b, motor c);
  referencia getFeedback(motor a, motor b, motor c);
  referencia cinInversa(coordenadas coor);

  void Trayectoria(motor a, motor b, motor c);
  bool mismonivel(float y);

  void mueve(motor a, motor b, motor c);
};
  
#endif
