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
#include "Servo.h"

#define L1 252
#define L2 200
#define L3 100

#define vel_min 90
#define vel_max 255

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
  motor m2;                 //Motor que mueve la primera articulacion del robot
  motor m1;                 //Motor que mueve la parte cartesiana del robot
  motor m3;                 //Motor que miueve la segunda articulacion del robot

  Servo servo1, servo2;
  int herramienta;
  
  const int tiempoAntirebote = 10;

  int bandera[1];           //Indica en que posicion se encuentra dentro de la trayectoria. Los valores (0, 0) , (0, 1) , (1, 0) y (1, 1) indican la posicion de la que parten en la trayectoria

  float margen = 0.05;      //Margen de error para que se alcance la posicion

public:
  void imprimirInterfaz();          //Imprime la interfaz
  void interaccionInterfaz();       //Lee los datos introducidos por Serial
  void inicializar();
  void endstop();

  void InicializarServos (int pin_servo1, int pin_servo2);
  void cambiarHerramienta ();
  void MovServo();
  void MovEjex();
  void MovEje1();
  void MovEje2();
  void MovEje3();
  void Parada();
  void SetPosicion(coordenadas punto_f);
  void SetPosicion(float x, float y);
  
  coordenadas getPosicion();
  coordenadas cinDirecta(referencia r);

  referencia getFeedback();
  referencia cinInversa(coordenadas coor);

  void Trayectoria();
  bool mismonivel(float y);
  float calculoAltura(float a);
  void calculoVelocidades(referencia po, referencia pf);
  
  void mueve();
  void mueve(referencia a);
};
  
#endif
