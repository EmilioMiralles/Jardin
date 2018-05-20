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
  char pal;
  int v;
  
  if(Serial.available() > 0){
      pal = Serial.read();
      v = Serial.parseInt();
  }
  if (pal == '$')
  {
    switch(v){
        case '1':
            break;
        case '2':
            break;
        case '3':
            break;
        case '4':
            break;
        case '5':
            break;
        case '6':
            break;
        case '7':
            Parada(); 
            break;
        case '11': 
            MovEje1 (); 
            break;
        case '12': 
            MovEje2 (); 
            break;
        case '13': 
            MovEje3 ();
            break;
        case '14': 
            MovServo();
            break;
        case '15':
            //MovServo();           //IMPLEMENTAR SEGUNDO SERVO
            break;
        case '16': 
            coordenadas aux;
            SetPosicion (aux);
            bandera[0]=false;
            bandera[1]=false; 
            break;
        case '17': 
            float a, b;
            SetPosicion (a, b);
            bandera[0]=false;
            bandera[1]=false;
            break;
        case '20': 
            cambiarHerramienta(); 
            break;
        case '31': 
            m1.imprimirVel();
            break;
        case '32': 
            m2.imprimirVel();
            break;
        case '33': 
            m3.imprimirVel();
            break;
        case '34': 
            m1.imprimirAv();
            break;
        }
   }   
}

coordenadas Interfaz::getPosicion(){
  coordenadas pos;
  
  pos.x = m1.getFeedback();
  pos.y = L1*cos(m2.getFeedback()) + L2*cos(m2.getFeedback()+m3.getFeedback()-90) + L3;
  pos.z = L1*sin(m2.getFeedback()) + L2*sin(m2.getFeedback()+m3.getFeedback()-90) + 163;

  return pos;
}

referencia Interfaz::getFeedback(){
  referencia refe;

  refe.Rx = m1.getFeedback();
  refe.Ang1 = m2.getFeedback();
  refe.Ang2 = m3.getFeedback();
  refe.Ang3 = 0;                      //MODIFICAR LO QUE SE OBTIENE CUANDO SE IMPLEMENTE EL SERVOMOTOR

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
  m1.setAvance(44);           //Seteamos el avance del motor en una vuelta para calcular la posicion
  m1.encod.setPin(A0);        //Pin del sensor del encoder
  m1.encod.setBandera();
  m2.setTipo(true);
  m2.setPines(24,25,3);
  m2.pot.setpin(A1);
  m2.pot.setLimites(0, 1023);
  m3.setTipo(true);
  m3.setPines(26,27,4);
  m3.pot.setpin(A2);
  m3.pot.setLimites(0,1023);
  m3.setEndstop(A3);
  Interfaz::InicializarServos(9,10);
}

void Interfaz::mueve(){

  referencia p_o = Interfaz::getFeedback();
  referencia p_f;

  bool b1, b2, b3;

  if ((!bandera[0]) && (!bandera[1])){
    p_f = Interfaz::cinInversa(p[1]);
    
    Interfaz::calculoVelocidades(p_o, p_f);

    if(p_o.Rx < p_f.Rx*(1-margen))                                            {m1.avanzar();    b1=false;}
    else if (p_o.Rx > p_f.Rx*(1+margen))                                      {m1.retroceder(); b1=false;}
    else if ((p_o.Rx >= p_f.Rx*(1-margen)) && (p_o.Rx <= p_f.Rx*(1+margen)))   {m1.parar();     b1=true;}

    if(p_o.Ang1 < p_f.Ang1*(1-margen))                                                {m2.avanzar();    b2=false;}
    else if (p_o.Ang1 > p_f.Ang1*(1+margen))                                          {m2.retroceder(); b2=false;}
    else if ((p_o.Ang1 >= p_f.Ang1*(1-margen)) && (p_o.Ang1 <= p_f.Ang1*(1+margen)))   {m2.parar();     b2=true;}

    if(p_o.Ang2 < p_f.Ang2*(1-margen))                                                {m3.avanzar();    b3=false;}
    else if (p_o.Ang2 > p_f.Ang2*(1+margen))                                          {m3.retroceder(); b3=false;}
    else if ((p_o.Ang2 >= p_f.Ang2*(1-margen)) && (p_o.Ang2 <= p_f.Ang2*(1+margen)))   {m3.parar();     b3=true;}

    if((b1)&&(b2)&&(b3)){
      bandera[0]=false;
      bandera[1]=true;
      b1 = false;
      b2 = false;
      b3 = false;
    }
  }

  else if ((!bandera[0]) && (bandera[1])){
    p_f = Interfaz::cinInversa(p[2]);
    
    Interfaz::calculoVelocidades(p_o, p_f);

    if(p_o.Rx < p_f.Rx*(1-margen))                                            {m1.avanzar();    b1=false;}
    else if (p_o.Rx > p_f.Rx*(1+margen))                                      {m1.retroceder(); b1=false;}
    else if ((p_o.Rx >= p_f.Rx*(1-margen)) && (p_o.Rx <= p_f.Rx*(1+margen)))   {m1.parar();     b1=true;}

    if(p_o.Ang1 < p_f.Ang1*(1-margen))                                                {m2.avanzar();    b2=false;}
    else if (p_o.Ang1 > p_f.Ang1*(1+margen))                                          {m2.retroceder(); b2=false;}
    else if ((p_o.Ang1 >= p_f.Ang1*(1-margen)) && (p_o.Ang1 <= p_f.Ang1*(1+margen)))   {m2.parar();     b2=true;}

    if(p_o.Ang2 < p_f.Ang2*(1-margen))                                                {m3.avanzar();    b3=false;}
    else if (p_o.Ang2 > p_f.Ang2*(1+margen))                                          {m3.retroceder(); b3=false;}
    else if ((p_o.Ang2 >= p_f.Ang2*(1-margen)) && (p_o.Ang2 <= p_f.Ang2*(1+margen)))   {m3.parar();     b3=true;}

    if((b1)&&(b2)&&(b3)){
      bandera[0]=true;
      bandera[1]=false;
      b1 = false;
      b2 = false;
      b3 = false;
    }
  }

  else if ((bandera[0]) && (!bandera[1])){
    p_f = Interfaz::cinInversa(posicion_final);
    
    Interfaz::calculoVelocidades(p_o, p_f);

    if(p_o.Rx < p_f.Rx*(1-margen))                                            {m1.avanzar();    b1=false;}
    else if (p_o.Rx > p_f.Rx*(1+margen))                                      {m1.retroceder(); b1=false;}
    else if ((p_o.Rx >= p_f.Rx*(1-margen)) && (p_o.Rx <= p_f.Rx*(1+margen)))   {m1.parar();     b1=true;}

    if(p_o.Ang1 < p_f.Ang1*(1-margen))                                                {m2.avanzar();    b2=false;}
    else if (p_o.Ang1 > p_f.Ang1*(1+margen))                                          {m2.retroceder(); b2=false;}
    else if ((p_o.Ang1 >= p_f.Ang1*(1-margen)) && (p_o.Ang1 <= p_f.Ang1*(1+margen)))   {m2.parar();     b2=true;}

    if(p_o.Ang2 < p_f.Ang2*(1-margen))                                                {m3.avanzar();    b3=false;}
    else if (p_o.Ang2 > p_f.Ang2*(1+margen))                                          {m3.retroceder(); b3=false;}
    else if ((p_o.Ang2 >= p_f.Ang2*(1-margen)) && (p_o.Ang2 <= p_f.Ang2*(1+margen)))   {m3.parar();     b3=true;}

    if((b1)&&(b2)&&(b3)){
      bandera[0]=true;
      bandera[1]=true;
      b1 = false;
      b2 = false;
      b3 = false;
    }
  }

  else if ((bandera[0]) && (bandera[1])){
    m1.parar();
    m2.parar();
    m3.parar();
  }
}


void Interfaz::mueve(referencia a){

  referencia p_o = Interfaz::getFeedback();
  referencia p_f = a;

  Interfaz::calculoVelocidades(p_o, p_f);

  if(p_o.Rx < p_f.Rx*(1-margen))                                            {m1.avanzar();}
  else if (p_o.Rx > p_f.Rx*(1+margen))                                      {m1.retroceder();}
  else if ((p_o.Rx >= p_f.Rx*(1-margen)) && (p_o.Rx <= p_f.Rx*(1+margen)))  {m1.parar();}

  if(p_o.Ang1 < p_f.Ang1*(1-margen))                                                {m2.avanzar();}
  else if (p_o.Ang1 > p_f.Ang1*(1+margen))                                          {m2.retroceder();}
  else if ((p_o.Ang1 >= p_f.Ang1*(1-margen)) && (p_o.Ang1 <= p_f.Ang1*(1+margen)))  {m2.parar();}

  if(p_o.Ang2 < p_f.Ang2*(1-margen))                                                {m3.avanzar();}
  else if (p_o.Ang2 > p_f.Ang2*(1+margen))                                          {m3.retroceder();}
  else if ((p_o.Ang2 >= p_f.Ang2*(1-margen)) && (p_o.Ang2 <= p_f.Ang2*(1+margen)))  {m3.parar();}
}


void Interfaz::calculoVelocidades(referencia po, referencia pf){
  float vel_aux;
  float dif_ang1, dif_ang2, rel_vel;

  float distancia_x = pf.Rx - po.Rx;
  if (distancia_x < 0)  distancia_x = -distancia_x;

  if (distancia_x > 500){
    m1.setVelocidad(vel_max);
  }
  
  else if (distancia_x <=500){
    vel_aux = (distancia_x/500)*255;
    if (vel_aux <= vel_min) vel_aux = vel_min;
    m1.setVelocidad(vel_aux);
  }

  dif_ang1 = pf.Ang1 - po.Ang1;
    if(dif_ang1 < 0)  dif_ang1 = -dif_ang1;
  dif_ang2 = pf.Ang2 - po.Ang2;
    if(dif_ang2 < 0)  dif_ang2 = -dif_ang2;
  rel_vel = dif_ang1 / dif_ang2;

  if(dif_ang1 > 45){
    m2.setVelocidad(vel_max);
    vel_aux = vel_max / (rel_vel * 3.33);
    if (vel_aux <= vel_min) vel_aux = vel_min;
    m3.setVelocidad(vel_aux);
  }
  
  else if (dif_ang1 <= 45){
    vel_aux = (dif_ang1/45)*255;
    if (vel_aux <= vel_min) vel_aux = vel_min;
    m2.setVelocidad(vel_aux);
    vel_aux = vel_aux / (rel_vel * 3.33);
    if (vel_aux <= vel_min) vel_aux = vel_min;
    m3.setVelocidad(vel_aux);
  }
}


void Interfaz::endstop(){
  m1.endstop();
}


void Interfaz::InicializarServos (int pin_servo1, int pin_servo2) {
  servo1.attach(pin_servo1);
  servo1.write(0);
  servo2.attach(pin_servo2);
  servo2.write(0);
}


void Interfaz::cambiarHerramienta () {
  herramienta = Serial.parseInt();
  switch (herramienta) {
    case '1': 
      servo2.write(0); 
      break;
    case '2': 
      servo2.write(60); 
      break;
    case '3': 
      servo2.write(120); 
      break;
    case '4': 
      servo2.write(180); 
      break;
    default:
      servo2.write(0);
    }
}

void Interfaz::MovServo () {
  int valor;
  valor = Serial.parseInt();
  servo1.write(valor);
 }

void Interfaz::MovEje1(){
  referencia aux;
  coordenadas p_f;
  aux = Interfaz::getFeedback();

  aux.Ang1 += 5;
  
  p_f = cinDirecta(aux);
  posicion_final = p_f;
  
  bandera[0]=true;
  bandera[1]=false;
}

void Interfaz::MovEje2(){
  referencia aux;
  coordenadas p_f;
  aux = Interfaz::getFeedback();

  aux.Ang2 += 5;
  
  p_f = cinDirecta(aux);
  posicion_final = p_f;
  
  bandera[0]=true;
  bandera[1]=false;
}

void Interfaz::MovEjex(){
  referencia aux;
  coordenadas p_f;
  aux = Interfaz::getFeedback();

  aux.Rx += 20;
  
  p_f = cinDirecta(aux);
  posicion_final = p_f;
  
  bandera[0]=true;
  bandera[1]=false;
}

void Interfaz::MovEje3(){
  referencia aux;
  aux = Interfaz::getFeedback();

  aux.Ang3 += 5;
  servo1.write(aux.Ang3);
}

void Interfaz::Parada(){
  posicion_final = Interfaz::cinDirecta(Interfaz::getFeedback());
}

void Interfaz::SetPosicion(coordenadas punto_f) {
  punto_f.x = Serial.parseFloat();
  punto_f.y = Serial.parseFloat();
  punto_f.z = Serial.parseFloat();
  posicion_final = punto_f;  
} 

void Interfaz::SetPosicion(float x, float y){
  float z;
  
  x = Serial.parseFloat();
  y = Serial.parseFloat();
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
  pos.y = L1*cos(r.Ang1) + L2*cos(r.Ang1+r.Ang2-90) + L3;
  pos.z = L1*sin(r.Ang1) + L2*sin(r.Ang1+r.Ang2-90) + 163;

  return pos;
}

