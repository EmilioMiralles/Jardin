/*
  Programa de Arduino creado para el control de un robot jardinero (brazo robótico)
      que consta de varios grados de libertad, y de varias herramientas diferentes.
  Tanto el programa como el robot han sido  diseñados y construidos  por alumnos de
      la Escuela  Tecnica Superior de  Ingenieria y  Diseño  Industrial (ETSIDI) de
      la Universidad Politécnica de Madrid.
      
  Autores:
    Alvaro Palmero Martinez       52091
    Ignacio Rodriguez Naranjo     51773
    Emilio Miralles Rivera        51749
       
  Fecha:   Marzo 2018
*/

#include "Motor.h"
#include "Interfaz.h"

motor m1;                 //Motor que mueve la parte cartesiana del robot
motor m2;                 //Motor que mueve el primer eslabon angular del brazo robotico
motor m3;
Interfaz interfaz;
int i=0;

void setup() {
  Serial.begin(9600);

  m1.setTipo(false);
  m1.setPines(22,23,2);       //pines del primer motor
  m1.setVelocidad(90);         //Señal PWM transmitida al controlador L298N
  m1.setAvance(44);           //Seteamos el avance del motor en una vuelta para calcular la posicion
  m1.encod.setPin(A0);        //Pin del sensor del encoder
  m1.encod.setBandera();
  m2.setTipo(true);
  m2.setPines(24,25,3);
  m2.setVelocidad(120);
  m2.pot.setpin(A1);
  m2.pot.setLimites(0, 1023);
  interfaz.imprimirInterfaz(m1, m2);         //Imprimimos la interfaz con la que se trabajará
  //interfaz.setCoordenadas(90);
}

void loop() {

  interfaz.interaccionInterfaz(m1, m2);
  m1.encod.contadorMemoria();
  interfaz.getPosicion(m1, m2, m3);
}
