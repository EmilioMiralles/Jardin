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

    if (Serial.available() > 0){
  char pal = Serial.read();
  switch (pal)
  {
    case 'q': ref_herramienta = 1;
              break;
    case 'w': ref_herramienta = 2;
              break;
    case 'e': ref_herramienta = 3;
              break;
    case 'r': ref_herramienta = 4;
              break;
    /*case 'd':
        referencia_final.Rx +=50;
        referencia_final.Ang1 =50;
        referencia_final.Ang2 =30;
        break;
      case 'a':
        referencia_final.Rx -=50;
        referencia_final.Ang1 =100;
        referencia_final.Ang2 =50;
        break; 
        case 's':
        analogWrite(8, 200);
        break;*/
  }
//posicion_final = Serial.readBytes (6,3);
referencia_final = Interfaz::cinInversa (posicion_final);
}
}

coordenadas Interfaz::getPosicion(){
  coordenadas pos;
  
  pos.x = m1.getFeedback();
  pos.y = L1*cos(m2.getFeedback()*PI/180) + L2*cos(m2.getFeedback()*PI/180+m3.getFeedback()*PI/180-PI/2) + L3;
  pos.z = L1*sin(m2.getFeedback()*PI/180) + L2*sin(m2.getFeedback()*PI/180+m3.getFeedback()*PI/180-PI/2) + 163;

  return pos;
}

referencia Interfaz::getFeedback(){
  referencia refe;

  refe.Rx = m1.getFeedback();
  refe.Ang1 = m2.getFeedback();
  refe.Ang2 = m3.getFeedback();
  refe.Ang3 = servo1.read();

  return refe;
}

referencia Interfaz::cinInversa (coordenadas coord){
  float Theta1, Theta2;
  float Alpha1, Alpha2;     //Los angulos theta representaran los resultados positivos de la ecuacion y los alpha los negativos
                            //Se tomará como óptimo el ángulo del primero brazo(Theta1 o Alpha1) que sea mayor;

  referencia refe;
  
  float q2;
  float y_r = coord.y - 100;
  float z_r = coord.z - 163;

  float qa = ((y_r*y_r + z_r*z_r - L1*L1 - L2*L2) / (2*L1*L2));         // Se usa una variable auxiliar qa que representa cos(q2)

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
  m1.setTipo(false);                    //Motor controlador por encoder
  m1.setPines(23,22,2);                 //pines del primer motor
  m1.setAvance(38);                     //Seteamos el avance del motor en una vuelta para calcular la posicion
  m1.encod.setPin(A0);                  //Pin del sensor del encoder
  m1.encod.setBandera();                //Inicializamos la bandera
  m1.setPosicion(5000);                 //Inicializamos una posicion de 5000 para que se realice un homing
  m2.setTipo(true);                     //Motor controlado por potenciometro
  m2.setPines(24,25,3);                 //Pines del segundo motor
  m2.pot.setpin(A1);                    //Pin del potenciometro
  m2.pot.setValor(12.5);
  m3.setTipo(true);                     //Motor controlado por potenciometro
  m3.setPines(27,26,4);                 //Pines del tercer motor
  m3.pot.setpin(A2);                    //Pin del potenciometro
  m3.pot.setValor(-24.5);
  Interfaz::InicializarServos(9,10);    //Se inicializan los pines en los que se controlan los servos
  Interfaz::setPin_fdc(30);             //Pin del final de carrera usado para el homing
  Interfaz::setRef_herramienta(1);
}

void Interfaz::mueve(){
if(referencia_final.Rx > (referencia_actual.Rx + 0.5)){
    m1.avanzar();
  }
  else if(referencia_final.Rx < (referencia_actual.Rx - 0.5)){
    m1.retroceder();
  }
  else if((referencia_final.Rx <= (referencia_actual.Rx + 0.5)) && (referencia_final.Rx >= (referencia_actual.Rx-0.5))){
    m1.parar();
  }

  if(referencia_final.Ang1 > (referencia_actual.Ang1 + 2)){
    digitalWrite(24, HIGH);
    digitalWrite(25, LOW);
  }
  else if(referencia_final.Ang1 < (referencia_actual.Ang1 - 2)){
    digitalWrite(24, LOW);
    digitalWrite(25, HIGH);
  }
  else if((referencia_final.Ang1 >= (referencia_actual.Ang1 - 2)) && (referencia_final.Ang1 <= (referencia_actual.Ang1 + 2))){
    digitalWrite(24, LOW);
    digitalWrite(25, LOW);
  }

  if(referencia_final.Ang2 > (referencia_actual.Ang2 + 2)){
    digitalWrite(26, HIGH);
    digitalWrite(27, LOW);
  }
  else if(referencia_final.Ang2 < (referencia_actual.Ang2 - 2)){
    digitalWrite(26, LOW);
    digitalWrite(27, HIGH);
  }
  else if((referencia_final.Ang2 >= (referencia_actual.Ang2 - 2)) && (referencia_final.Ang2 <= (referencia_actual.Ang2 + 2))){
    digitalWrite(26, LOW);
    digitalWrite(27, LOW);
}
}


void Interfaz::InicializarServos (int pin_servo1, int pin_servo2) {
  servo1.attach(pin_servo1);
  servo1.write(0);
  servo2.attach(pin_servo2);
  servo2.write(0);
}


void Interfaz::cambiarHerramienta (int a) {
  switch (a) {
    case 1: 
      Interfaz::setRef_herramienta(1);
      break;
    case 2: 
      Interfaz::setRef_herramienta(2);
      break;
    case 3: 
      Interfaz::setRef_herramienta(3);
      break;
    case 4: 
      Interfaz::setRef_herramienta(4);
      break;
    default:
      Interfaz::setRef_herramienta(ref_herramienta);
      break;
    }
}


void Interfaz::SetPosicion(coordenadas punto_f) {
  punto_f.x = Serial.parseFloat();
  punto_f.y = Serial.parseFloat();
  punto_f.z = Serial.parseFloat();
  posicion_final = punto_f;  
  referencia_final = cinInversa(posicion_final);
} 

void Interfaz::SetPosicion(float a, float b){
  float z;
  float x;
  float y;
  
  x = a;
  y = b;
  z = Interfaz::calculoAltura(y);
  posicion_final.x = x;
  posicion_final.y = y;
  posicion_final.z = z;
}

float Interfaz::calculoAltura(float a){
  float z;
  
  if ((a >= DISTJARDIN) && (a <= (DISTJARDIN + 142))){
    z = 110;
    return z;
  }
  else if ((a > (DISTJARDIN + 142)) && (a <= (DISTJARDIN + 142*2))){
    z = 160;
    return z;
  }
  else if ((a > (DISTJARDIN + 142*2)) && (a <= (DISTJARDIN + 142*3))){
    z = 210;
    return z;
  }
  else {
    z = 210;
    return z;
  }
}


coordenadas Interfaz::cinDirecta(referencia r){
  coordenadas pos;
  
  pos.x = r.Rx;
  pos.y = L1*cos(r.Ang1*PI/180) + L2*cos(r.Ang1*PI/180+r.Ang2*PI/180-PI/2) + L3;
  pos.z = L1*sin(r.Ang1*PI/180) + L2*sin(r.Ang1*PI/180+r.Ang2*PI/180-PI/2) + 163;

  return pos;
}


void Interfaz::homing(){
referencia_final.Rx = 0;
referencia_final.Ang1 = 90;
referencia_final.Ang2 = 40;
}

void Interfaz::setPin_fdc(int a){
  pin_fdc = a;
  pinMode(pin_fdc, INPUT);
}

void Interfaz::finaldecarrera(){
}


void Interfaz::corrigeAngulo(){
  float a =90 +(referencia_actual.Ang1 - (referencia_actual.Ang2));

  servo1.write(a);
}


void Interfaz::setRef_herramienta(int a){
  ref_herramienta = a;
}


void Interfaz::mueveHerramienta(){
  switch(ref_herramienta){
    case 1:
      servo2.write(0);
      break;
    case 2:
      servo2.write(60);
      break;
    case 3:
      servo2.write(120);
      break;
    case 4:
      servo2.write(180);
      break; 
  }
}


void Interfaz::actualizaPosicion(){
  posicion = Interfaz::getPosicion();
  referencia_actual = Interfaz::getFeedback();
}

/*void Interfaz::setPuntoseguro(){
  punto_seguro.Rx = referencia_actual.Rx;
  punto_seguro.Ang1 = 90;
  punto_seguro.Ang2 = 45;
}*/

void Interfaz::imprimeMierda(){
  Serial.print(m1.getFeedback());
  Serial.print("\t");
  Serial.print(m2.getFeedback());
  Serial.print("\t");
  Serial.print(m3.getFeedback());
  Serial.print("\t");
  Serial.print(referencia_final.Rx);
  Serial.print("\t");
  Serial.print(referencia_final.Ang1);
  Serial.print("\t");
  Serial.println(referencia_final.Ang2);
  
}

