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

#include "Interfaz.h"

Interfaz interfaz;

void setup() {
  Serial.begin(9600);

  interfaz.inicializar();
  interfaz.imprimirInterfaz();         //Imprimimos la interfaz con la que se trabajará
}

void loop() {
  interfaz.interaccionInterfaz();
  interfaz.mueve();
}
