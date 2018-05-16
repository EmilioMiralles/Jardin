/*
  Interfaz.cpp - Librería para el uso de la interfaz con la que se controlará el robot
  Creado por Álvaro Palmero, 12 marzo 2018
 */

#include "Arduino.h"
#include "Interfaz.h"

void Interfaz::imprimirInterfaz(motor a, motor b){
  Serial.println("Interfaz para el control del robot IRON PLANT (v 0.1). A continuación");
  Serial.println("se muestran los valores disponibles: ");
  Serial.println();
  Serial.print  (" - V1 (Velocidad del motor 1) = "); a.imprimirVel();
  Serial.print  (" - A1 (Avance del motor 1)    = "); a.imprimirAv();
  Serial.print  (" - P1 (Posicion del motor 1)  = "); a.imprimirFeedback();
  Serial.print  (" - V2 (Velocidad del motor 2) = "); b.imprimirVel();
  Serial.print  (" - T2 (Angulo del motor 2)    = "); b.imprimirFeedback();
  Serial.println();
  Serial.println("Para la modificacion de los valores mostrados introducir en la  linea de");
  Serial.println("entrada el formato (V,A)=VALOR. A continuación se  muestran más comandos");
  Serial.println("disponibles: ");
  Serial.println();
  Serial.println(" - R  (Establece el home en el sitio)");
  Serial.println(" - C  (Para introducir una coordenada de la forma C=VALOR)");
  Serial.println();
  //Serial.println("I  (Para mostrar la interfaz de nuevo)");
}

void Interfaz::interaccionInterfaz(motor a, motor b){
  if(Serial.available() > 0){
      char pal = Serial.read();
      int v = Serial.parseInt();
  
      switch(pal){
        case 'v':
        case 'V':  a.setVelocidad(v);
                   Serial.print("La velocidad se ha establecido en:  ");
                   a.imprimirVel();
                   Serial.println();
                   break;
        case 'a':
        case 'A':  a.setAvance(v);
                   Serial.print("El avance del motor se ha establecido en ");
                   a.imprimirAv();
                   Serial.println();
                   break;
        case 'r':
        case 'R':  a.encod.reset();
                   a.getFeedback();
                   Serial.println("La posición actual se ha establecido como home.");
                   Serial.println();
                   break;
        /*case 'c':
        case 'C':  coordenadas = v;
                   Serial.println("Coordenadas actualizadas");
                   Serial.println();
                   break;*/
        case 'p':   
        case 'P':  Serial.print("La memoria y las vueltas del encoder son:  ");
                   a.encod.imprimir();
                   Serial.print("Y, por tanto, la posición es:  ");
                   a.imprimirFeedback();
                   Serial.println();
                   break;
        case 't':
        case 'T':  imprimirInterfaz(a, b);
                   break;
        default:   Serial.println("ERROR. El comando introducido no es correcto.");
                   Serial.println();
      }
    }
}

coordenadas Interfaz::getPosicion(motor a, motor b, motor c){
  posicion.x = a.getFeedback();
  posicion.y = L1*cos(b.getFeedback()) + L2*cos(b.getFeedback()+c.getFeedback()-90) + L3;
  posicion.z = L1*sin(b.getFeedback()) + L2*sin(b.getFeedback()+c.getFeedback()-90) + 163;

  return posicion;
}

