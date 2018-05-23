/*
  Interfaz.h - Librería para el uso de la interfaz con la que se controlará el robot
  Creado por Álvaro Palmero, 12 marzo 2018
 */

#ifndef INTERFAZ_H
#define INTERFAZ_H

#include "Arduino.h"
#include "Motor.h"
#include "Math.h"
#include "Servo.h"
#include "PwmClass.h"

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
  coordenadas posicion_cons;
  
  referencia referencia_final;
  referencia referencia_actual;
  referencia punto_seguro;
  
  motor m2;                 //Motor que mueve la primera articulacion del robot
  motor m1;                 //Motor que mueve la parte cartesiana del robot
  motor m3;                 //Motor que miueve la segunda articulacion del robot

  int pin_fdc;

  Servo servo1, servo2;

  int ref_herramienta;

  int herramienta;

  PWMClass bomba;
  PWMClass corte;
  PWMClass ventilador;
  PWMClass leds;
  
  const int tiempoAntirebote = 10;

  bool bandera[1];           //Indica en que posicion se encuentra dentro de la trayectoria. Los valores (0, 0) , (0, 1) , (1, 0) y (1, 1) indican la posicion de la que parten en la trayectoria

  float margen = 0.01;         //Margen de error para que se alcance la posicion

  float DISTJARDIN  = 190;

  float L1 = 252;
  float L2 = 200;
  float L3 = 100;
  
  float pi = 3.1416;
  
  float vel_min = 130;
  float vel_max = 250;

public:
  void imprimirInterfaz();          //Imprime la interfaz
  void interaccionInterfaz();       //Lee los datos introducidos por Serial
  void inicializar();
  void setPunto_seguro(float a, float b);
  
  void cambiarHerramienta (int a);
  void setRef_herramienta(int a);
  void mueveHerramienta();

  void mueve();

  void InicializarServos (int pin_servo1, int pin_servo2);
  void SetPosicion(coordenadas punto_f);
  void SetPosicion(float a, float b);
  
  coordenadas getPosicion();
  coordenadas cinDirecta(referencia r);
  void actualizaPosicion();

  referencia getFeedback();
  referencia cinInversa(coordenadas coor);

  bool mismonivel(float y);
  float calculoAltura(float a);
  void calculoVelocidades(referencia po, referencia pf);

  void homing();
  void finaldecarrera();
  void setPin_fdc(int a);

  void corrigeAngulo();

  void imprimeMierda();
};
  
#endif
