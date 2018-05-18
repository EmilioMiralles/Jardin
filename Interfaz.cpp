/*
  Interfaz.cpp - Librería para el uso de la interfaz con la que se controlará el robot
  Creado por Álvaro Palmero, 12 marzo 2018
 */

#include "Arduino.h"
#include "Interfaz.h"

void Interfaz::imprimirInterfaz(motor a, motor b){
  Serial.println("Interfaz para el control del robot IRON PLANT (v 1.1). A continuación");
  Serial.println("se  muestran   los   diferentes  parámetros  disponibles.   Para   la");
  Serial.println("de  los  valores  mostrados  introducir  en  la  linea de  entrada el");
  Serial.println("($,A)=VALOR. ");
  Serial.println("");
  Serial.println("TAREAS:");
  Serial.println(" - $1: Regar");
  Serial.println(" - $2: Sembrar");
  Serial.println(" - $3: Arar");
  Serial.println(" - $4: Cortar");
  Serial.println(" - $5: Demo");
  Serial.println(" - $6: Activar Luces");
  Serial.println(" - $7: Parada");
  Serial.println("");
  Serial.println("TAREAS AVANZADAS:");
  Serial.println(" - $11: Mover eje x");
  Serial.println(" - $12: Mover eje R1");
  Serial.println(" - $13: Mover eje R2");
  Serial.println(" - $14: Mover eje R3");
  Serial.println(" - $15: Mover herramienta");
  Serial.println(" - $16: Ir a posicion (x, y, z)");
  Serial.println(" - $17: Ir a posicion (x, y)");
  Serial.println(" - $20: Cambiar herramienta");
  Serial.println("");
  Serial.println("CONFIGURACION:");
  Serial.print  (" - $31: Velocidad motor 1 = ");   a.imprimirVel();
  Serial.print  (" - $32: Velocidad motor 2 = ");   b.imprimirVel();
  Serial.print  (" - $33: Velocidad motor 3 = ");
  Serial.println("");                                                                           //INTRODUCIR VELOCIDAD DE MOTOR 3 CUANDO SE INCLUYA EN LA INTERFAZ
  Serial.print  (" - $34: Avance del motor 1 = ");  a.imprimirAv();
}

void Interfaz::interaccionInterfaz(motor a, motor b, motor c){
  if(Serial.available() > 0){
      char pal = Serial.read();
      int v = Serial.parseInt();
  
  if (pal == '$')

      switch(v){
        case v = 1: REGAR; break;
        case v = 2: SEMBRAR; break;
        case v = 3: ARAR; break;
        case v = 4: CORTAR; break;
        case v = 5: DEMO; break;
        case v = 6: LUCES; break;
        case v = 7: REARME/PARADA; break;
        case v = 10: MOVER; break; 
        case v = 11: MOVER X; break; a.avanzar();
        case v = 12: ROTAR R1; break;
        case v = 13: ROTAR R2; break;
        case v = 14: ROTAR R3; break;
        case v = 15: GIRAR TOOL; break;
        case v = 16: IR A POSICION (X,Y,Z); break;
        case v = 17: IR A POSICION (X,Y); break;
        case v = 20: CAMBIAR TOOL; break;
        case v = 30: VELOCIDAD MOTORES; break;
        }
  
}

coordenadas Interfaz::getPosicion(motor a, motor b, motor c){
  posicion.x = a.getFeedback();
  posicion.y = L1*cos(b.getFeedback()) + L2*cos(b.getFeedback()+c.getFeedback()-90) + L3;
  posicion.z = L1*sin(b.getFeedback()) + L2*sin(b.getFeedback()+c.getFeedback()-90) + 163;

  return posicion;
}


referencia Interfaz::cinInversa (float x, float y, float z){
  float Theta1, Theta2;
  float Alpha1, Alpha2;     //Los angulos theta representaran los resultados positivos de la ecuacion y los alpha los negativos
                            //Se tomará como óptimo el ángulo del primero brazo(Theta1 o Alpha1) que sea mayor;

  float q2;
  float y_r = y - 100;
  float z_r = z - 163;

  float qa = ((y_r*y_r + z_r*z_r - L1*L1 - L2*L2) / 2*L1*L2);         // Se usa una variable auxiliar qa que representa cos(q2)

  Theta2 = atan2(sqrt(1-pow(qa,2)) , qa);
  Alpha2 = atan2(-sqrt(1-pow(qa,2)) , qa);

  if (Theta2 >= Alpha2){
    q2 = Theta2 - pi/2;

    Theta1 = atan2(z_r , y_r) - atan2(L2*sin(q2) , (L1 + L2*cos(q2)));

    ref.Rx = x;
    ref.Ang1 = Theta1 * 180 / pi;
    ref.Ang2 = q2 * 180 / pi;
    ref.Ang3 = Theta1 *180 / pi - Theta2 * 180 / pi -90;

    return ref;
  }

  else if (Theta2 < Alpha2){
    q2 = Alpha2 - pi/2;

    Alpha1 = atan2(z_r , y_r) - atan2(L2*sin(q2) , (L1 + L2*cos(q2)));

    ref.Rx = x;
    ref.Ang1 = Alpha1 * 180 / pi;
    ref.Ang2 = q2 * 180 / pi;
    ref.Ang3 = Alpha1 *180 / pi - Alpha2 * 180 / pi -90;

    return ref;
  }
}

