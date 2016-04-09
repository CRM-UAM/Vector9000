/*
  Vector9000.c - Library for control Vector9000 robot
  Created by Victor Uceda, Feb 2016.
  Released into the public domain.
*/

#include "Vector9000.h"
#include <QTRSensors.h>
//#include "EnableInterrupt.h"


const uint8_t Vector9000::M_IZQ_PWM_PIN = 5;
const uint8_t Vector9000::M_IZQ_DIR1_PIN = 10;
const uint8_t Vector9000::M_IZQ_DIR2_PIN = 12;
const uint8_t Vector9000::M_DER_DIR1_PIN = 11;
const uint8_t Vector9000::M_DER_DIR2_PIN = 9;
const uint8_t Vector9000::M_DER_PWM_PIN = 6;
const uint8_t Vector9000::LED = 13;
const uint8_t Vector9000::NUM_IR_SENSORS = 8;
const int Vector9000::TIMEOUT = 2000;

const uint8_t Vector9000::IR1 = 4;
const uint8_t Vector9000::IR2 = 8;
const uint8_t Vector9000::IR3 = A5;
const uint8_t Vector9000::IR4 = A4;
const uint8_t Vector9000::IR5 = A3;
const uint8_t Vector9000::IR6 = A2;
const uint8_t Vector9000::IR7 = A1;
const uint8_t Vector9000::IR8 = A0;

// FIXME: colocar los pines de los encoders correctos
const uint8_t Vector9000::ENC_IZQ_PIN = 2;
const uint8_t Vector9000::ENC_DER_PIN = 3;
const uint8_t Vector9000::PULSOS_POR_REVOLUCION = 50;


 

/********* PLACA SENSORES QTR-8RC **********/
QTRSensorsRC qtrrc((unsigned char[]) {
   Vector9000::IR1, Vector9000::IR2, Vector9000::IR3, Vector9000::IR4, Vector9000::IR5, Vector9000::IR6, Vector9000::IR7, Vector9000::IR8}
, Vector9000::NUM_IR_SENSORS, Vector9000::TIMEOUT, QTR_NO_EMITTER_PIN); //emisor siempre encendido




Vector9000::Vector9000( double KP=2200, double KD=0.06, double KI=0 ) {

  _kp = KP;
  _kd = KD;
  _ki = KI;
  _DerivativeErrorTerm = 0;
  _lastTimeExec = millis();
  _lastError = 0;

}

void Vector9000::config(){
  pinMode(Vector9000::M_DER_DIR1_PIN, OUTPUT);
  pinMode(Vector9000::M_DER_DIR2_PIN, OUTPUT);
  pinMode(Vector9000::M_DER_PWM_PIN, OUTPUT);
  pinMode(Vector9000::M_IZQ_DIR1_PIN, OUTPUT);
  pinMode(Vector9000::M_IZQ_DIR2_PIN, OUTPUT);
  pinMode(Vector9000::M_IZQ_PWM_PIN, OUTPUT);
  pinMode(Vector9000::ENC_IZQ_PIN, INPUT);
  pinMode(Vector9000::ENC_DER_PIN, INPUT);
 // enableInterrupt(Vector9000::ENC_DER_PIN, aumentarCuentaDerecha, CHANGE);
  //enableInterrupt(Vector9000::ENC_IZQ_PIN, aumentarCuentaIzquierda, CHANGE);

}

void Vector9000::setRSpeed( int s ){
    if(s > 0) {// Hacia delante
        digitalWrite(Vector9000::M_DER_DIR2_PIN, LOW);
        digitalWrite(Vector9000::M_DER_DIR1_PIN, HIGH);
        analogWrite(Vector9000::M_DER_PWM_PIN , s);
    }else if(s < 0) {// Hacia atras
        digitalWrite(Vector9000::M_DER_DIR2_PIN, HIGH);
        digitalWrite(Vector9000::M_DER_DIR1_PIN, LOW);
        analogWrite(Vector9000::M_DER_PWM_PIN , -s);
    }else {//Parado
        digitalWrite(Vector9000::M_DER_PWM_PIN, LOW);// Motor apagado
    }
}

void Vector9000::setLSpeed( int s ){
    if(s > 0) {// Hacia delante
        digitalWrite(Vector9000::M_IZQ_DIR2_PIN, LOW);
        digitalWrite(Vector9000::M_IZQ_DIR1_PIN, HIGH);
        analogWrite(Vector9000::M_IZQ_PWM_PIN , s);
    }else if(s < 0) {// Hacia atras
        digitalWrite(Vector9000::M_IZQ_DIR2_PIN, HIGH);
        digitalWrite(Vector9000::M_IZQ_DIR1_PIN, LOW);
        analogWrite(Vector9000::M_IZQ_PWM_PIN , -s);
    }else {//Parado
        digitalWrite(Vector9000::M_IZQ_PWM_PIN, LOW);// Motor apagado
    }
}

void Vector9000::calibrateIR( int time, bool printflag ){
  int i;
  ledOn();
  for (i = 0; i < 40*time; i++)
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  ledOff();
  //print results
  if(printflag)Serial.println("Resultados\nMinimos:");
  for (i = 0; i < Vector9000::NUM_IR_SENSORS; i++){
    if(printflag)Serial.print(qtrrc.calibratedMinimumOn[i]);
    if(printflag)Serial.print(" ");
  }
  if(printflag)Serial.println();
  if(printflag)Serial.println("Maximos:");
  for (i = 0; i < Vector9000::NUM_IR_SENSORS; i++){
    if(printflag)Serial.print(qtrrc.calibratedMaximumOn[i]);
    if(printflag)Serial.print(" ");
  }
  if(printflag)Serial.println();
}


void Vector9000::getIRRaw( unsigned int *values ){
    qtrrc.read(values);
}

unsigned int Vector9000::readLine( void ){
    unsigned int values[Vector9000::NUM_IR_SENSORS];
    return qtrrc.readLine(values);
}

double Vector9000::getErrorLine( void ){
    unsigned int values[Vector9000::NUM_IR_SENSORS];
    int error = qtrrc.readLine(values) - (Vector9000::NUM_IR_SENSORS-1)*500; //Centrar el error en el 0
    unsigned long currentTime=micros();

    if(currentTime - _lastTimeExec > 1000){//Genero el error derivativo
        _DerivativeErrorTerm = _kd*(error-_lastError)/(currentTime-_lastTimeExec);
        _lastTimeExec = currentTime;
        _lastError = error;
    }

      /*Si el error ha cambiado mucho en proporcion aunque no haya pasado suficiente tiempo actualizo la parte derivativa
       if( (error/lastError)>2 || (error/lasError)<-2){
       Derror=kd*(error-lastError)/(currentTime-lastTime);
       lastTime=currentTime;
       lastError=error;
       }
       */
    return _kp*error+_DerivativeErrorTerm;
}
