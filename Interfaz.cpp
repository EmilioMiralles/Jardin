/*
  Interfaz.cpp - Librería para el uso de la interfaz con la que se controlará el robot
  Creado por Álvaro Palmero, 12 marzo 2018
 */

#include "Arduino.h"
#include "Interfaz.h"

void Interfaz::imprimirInterfaz(){
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
  Serial.print  (" - $31: Velocidad motor 1 = ");   m1.imprimirVel();
  Serial.print  (" - $32: Velocidad motor 2 = ");   m2.imprimirVel();
  Serial.print  (" - $33: Velocidad motor 3 = ");   m3.imprimirVel();
  Serial.print  (" - $34: Avance del motor 1 = ");  m1.imprimirAv();
}

void Interfaz::interaccionInterfaz(){
  if(Serial.available() > 0){
      char pal = Serial.read();
      int v = Serial.parseInt();
  
      /*switch(pal){
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
        case 'c':
        case 'C':  coordenadas = v;
                   Serial.println("Coordenadas actualizadas");
                   Serial.println();
                   break;
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
      }*/
    }
}

coordenadas Interfaz::getPosicion(){
  coordenadas pos;
  
  pos.x = m1.getFeedback();
  pos.y = L1*cos(m2.getFeedback()) + L2*cos(m2.getFeedback()+m3.getFeedback()-90) + L3;
  pos.z = L1*sin(m2.getFeedback()) + L2*sin(m2.getFeedback()+m3.getFeedback()-90) + 163;

  return pos;
}


referencia Interfaz::cinInversa (coordenadas coord){
  float Theta1, Theta2;
  float Alpha1, Alpha2;     //Los angulos theta representaran los resultados positivos de la ecuacion y los alpha los negativos
                            //Se tomará como óptimo el ángulo del primero brazo(Theta1 o Alpha1) que sea mayor;

  referencia refe;
  
  float q2;
  float y_r = coord.y - 100;
  float z_r = coord.z - 163;

  float qa = ((y_r*y_r + z_r*z_r - L1*L1 - L2*L2) / 2*L1*L2);         // Se usa una variable auxiliar qa que representa cos(q2)

  Theta2 = atan2(sqrt(1-pow(qa,2)) , qa);
  Alpha2 = atan2(-sqrt(1-pow(qa,2)) , qa);

  if (Theta2 >= Alpha2){
    q2 = Theta2 - pi/2;

    Theta1 = atan2(z_r , y_r) - atan2(L2*sin(q2) , (L1 + L2*cos(q2)));

    refe.Rx = coord.x;
    refe.Ang1 = Theta1 * 180 / pi;
    refe.Ang2 = q2 * 180 / pi;
    refe.Ang3 = Theta1 *180 / pi - Theta2 * 180 / pi -90;

    return refe;
  }

  else if (Theta2 < Alpha2){
    q2 = Alpha2 - pi/2;

    Alpha1 = atan2(z_r , y_r) - atan2(L2*sin(q2) , (L1 + L2*cos(q2)));

    refe.Rx = coord.x;
    refe.Ang1 = Alpha1 * 180 / pi;
    refe.Ang2 = q2 * 180 / pi;
    refe.Ang3 = Alpha1 *180 / pi - Alpha2 * 180 / pi -90;

    return refe;
  }
}

void Interfaz::Trayectoria(){ 
  if (Interfaz::mismonivel(posicion_final.y)) {
    p[0]=p[1]=p[2]=posicion;
  }

  else if(!(Interfaz::mismonivel(posicion_final.y))) {
    p[0] = posicion;
    p[1] = p[0];
    p[1].z = p[0].z + 150;
    p[2] = p[1];
    p[2].z = p[1].z - 50;
    p[2].y = posicion_final.y;
  }
}


bool Interfaz::mismonivel(float y){
  if ((posicion.y >= DISTJARDIN) && (posicion.y <= (DISTJARDIN + 142))){
    if ((y >= DISTJARDIN) && (y <= (DISTJARDIN + 142))){
      return true;
    }
    else {
      return false;
    }
  }
  else if ((posicion.y > (DISTJARDIN + 142)) && (posicion.y <= (DISTJARDIN + 142*2))){
    if ((y > (DISTJARDIN + 142)) && (y <= (DISTJARDIN + 142*2))){
      return true;
    }
    else {
      return false;
    }
  }
  else if ((posicion.y > (DISTJARDIN + 142*2)) && (posicion.y <= (DISTJARDIN + 142*3))){
    if ((y > (DISTJARDIN + 142*2)) && (y <= (DISTJARDIN + 142*3))){
      return true;
    }
    else {
      return false;
    }
  }
  else {
    return true;
  }
}

void Interfaz::inicializar(){
  m1.setTipo(false);
  m1.setPines(22,23,2);       //pines del primer motor
  m1.setVelocidad(90);        //Señal PWM transmitida al controlador L298N
  m1.setAvance(44);           //Seteamos el avance del motor en una vuelta para calcular la posicion
  m1.encod.setPin(A0);        //Pin del sensor del encoder
  m1.encod.setBandera();
  m2.setTipo(true);
  m2.setPines(24,25,3);
  m2.setVelocidad(120);
  m2.pot.setpin(A1);
  m2.pot.setLimites(0, 1023);
}

