/*
  Encoder.cpp - Libreria para el uso del encoder en el motor
  Creado por Alvaro Palmero, 11 marzo 2018
*/
#include "Encoder.h"
#include "Arduino.h"


void encoder::setPin(int p){
  pin = p;
  pinMode (pin, INPUT);
}

void encoder::setBandera(){
  band = digitalRead(pin);
}

void encoder::contadorMemoria(){
  if((digitalRead(pin))&&(!band)){
    band = 1;
    memoria++;
    i=1;
  }
  else if((!digitalRead(pin))&&(band)){
    band = 0;
    memoria++;
    i=1;
  }
  if((memoria%32==0)&&(memoria!=0)&&(i==1)){
    vueltas++;
    i=0;
  }
}

void encoder::restadorMemoria(){
  if((digitalRead(pin))&&(!band)){
    band = 1;
    memoria--;
    i=1;
  }
  else if((!digitalRead(pin))&&(band)){
    band = 0;
    memoria--;
    i=1;
  }
  if (memoria < 0) memoria = -memoria;
  if((memoria%32==0)&&(memoria!=0)&&(i==1)){
    vueltas--;
    i=0;
  }
}

void encoder::reset(){
    memoria = 0;
    vueltas = 0;
}

void encoder::imprimir(){
  Serial.print(memoria);
  Serial.print("\t");
  Serial.println(vueltas);
}

int encoder::getMemoria(){
  return memoria;
}

int encoder::getVueltas(){
  return vueltas;
}

void encoder::setVueltas(int a){
  vueltas = a;
}

