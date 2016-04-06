#include <Arduino.h>
//#include <EnableInterrupt.h>
#include "Vector9000.h"

#define PIN_BOTON 7

#define DIFF_ENCODERS_RECT 10
#define TIME_MAX_IN_RECTA 500 //tiempo máximo que está el robot en recta antes de frena. En ms.
#define TIME_MAX_FRENADA 100 //tiempo maximo de duracion de frenada tras recta
#define TICK_ENC_MAX_RECTA 500 //ticks máximos que dura la recta antes de frenar. 50ticks por vuelta de rueda.
#define TICK_ENC_MAX_FRENADA 100 //ticks máximos que dura la frenada

#define INTERVAL_RECT_TASK 100
#define MIN_VEL_EN_RECTA 100


#define VEL_BASE_RECTA 70
#define VEL_BASE_CURVA 50
#define VEL_BASE_FRENO 0
int VEL_BASE=65;
Vector9000 robot = Vector9000(0.0481,2750.283,0);//(kp,kd, ki);


boolean schedulerVELBASEOn=false;
unsigned long nextTaskTime=4967295;
unsigned long nextEndDerTaskCount=4294967295;
unsigned long nextEndIzqTaskCount=4294967295;
void (*callback)(void);


boolean enRecta=false;
unsigned long nextCheckRectTask=0;
unsigned long lastEncDer=0;
unsigned long lastEncIzq=0;

void printTelemetria(float err){
    Serial.print("pT ");
    Serial.print(micros()/1000000.0);
    Serial.print("/");
    Serial.print(err);
    Serial.print("|");
    Serial.print(robot.readLine());
    Serial.print("|");
    Serial.print(robot._kp*1000);
    Serial.print(" ");
    Serial.print(robot._kd);
    Serial.print(" ");
    Serial.println(robot._ki);
}

bool boton_pulsado() {
  return !digitalRead(PIN_BOTON);
}

volatile unsigned long cuentaEncoderIzquierdo = 0; 
volatile unsigned long cuentaEncoderDerecho = 0;
void aumentarCuentaIzquierda()
{
  cuentaEncoderIzquierdo++;
}

void aumentarCuentaDerecha()
{
  cuentaEncoderDerecho++;
}

void setup(){
    robot.config();
    Serial.begin(19200);
    pinMode(PIN_BOTON, INPUT_PULLUP);
    delay(20);
    
    robot.calibrateIR( 5, true );
    
    while(!boton_pulsado()) delay(10);
    while(boton_pulsado()) delay(10);
    
    attachInterrupt(digitalPinToInterrupt(Vector9000::ENC_DER_PIN), aumentarCuentaDerecha, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(Vector9000::ENC_IZQ_PIN), aumentarCuentaIzquierda, CHANGE);
}

long sumErr=0;
int lastErr=0;
inline double PID(int errLine){
  int err=errLine-3500;
  sumErr+=err;
  double pid = (robot._kp * err*1.0 + robot._kd * (err-lastErr)*1.0 + robot._ki * sumErr*1.0 ) / (100 * 3500.);
  //Serial.print(err);Serial.print(" - ");Serial.println(pid);
  lastErr=err;
  return pid;
}

void inline activarCurvas(){
  VEL_BASE=VEL_BASE_CURVA;
  robot.config();
}
void inline activarFreno(){
  enRecta=false;
  VEL_BASE=VEL_BASE_FRENO;
  robot.config();
  //Configurar duracion de frenada
  schedulerVELBASEOn=true;
  callback=&activarCurvas;
  nextTaskTime=millis()+TIME_MAX_FRENADA;
  nextEndDerTaskCount=cuentaEncoderDerecho+TICK_ENC_MAX_FRENADA;
  nextEndIzqTaskCount=cuentaEncoderIzquierdo+TICK_ENC_MAX_FRENADA;
}

void inline activarRecta(){
  enRecta=true;
  VEL_BASE= VEL_BASE_RECTA;
  
  //Configurar el punto de frenada
  schedulerVELBASEOn=true;
  callback=&activarFreno;
  nextTaskTime=millis()+TIME_MAX_IN_RECTA;
  nextEndDerTaskCount=cuentaEncoderDerecho+TICK_ENC_MAX_RECTA;
  nextEndIzqTaskCount=cuentaEncoderIzquierdo+TICK_ENC_MAX_RECTA;
}


void loop() {
    if(boton_pulsado()) {
      robot.setSpeed(0,0);
      delay(1000);
      setup();
      return;
    }
    if(schedulerVELBASEOn && (millis()>nextTaskTime)) {// || cuentaEncoderDerecho>nextEndDerTaskCount || cuentaEncoderIzquierdo>nextEndIzqTaskCount) ){ //planificador por tiempo para acelerar/frenar en recta
      schedulerVELBASEOn=false;
      callback();
    }
    //int err = robot.readLine();
    double errDif = robot.getErrorLine();//PID(err);
    //printTelemetria(errDif);
    robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );

    if(enRecta) robot.ledOn();
    else robot.ledOff();

    if(millis()>nextCheckRectTask){
      nextCheckRectTask=millis()+INTERVAL_RECT_TASK;
      unsigned long velIzq=(cuentaEncoderIzquierdo-lastEncIzq);
      unsigned long velDer=(cuentaEncoderDerecho-lastEncDer);
      long diferencia = (velIzq > velDer) ? (velIzq - velDer) : (velDer - velIzq);
      if( !enRecta && diferencia < DIFF_ENCODERS_RECT && velIzq > MIN_VEL_EN_RECTA){
        activarRecta();
      }
      lastEncIzq = cuentaEncoderIzquierdo;
      lastEncDer = cuentaEncoderDerecho;
     Serial.print(velDer);Serial.print("  ");Serial.print(velIzq);Serial.print("  "); Serial.println(diferencia);
    }
    
}
/*void loop(){
  Serial.println(cuentaEncoderIzquierdo);
  delay(500);
}*/

