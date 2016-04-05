#include <Arduino.h>
//#include <EnableInterrupt.h>
#include "Vector9000.h"

#define DIFF_ENCODERS_RECT 50
#define TIME_MAX_IN_RECTA 2000 //tiempo máximo que está el robot en recta antes de frena. En ms.
#define TIME_MAX_FRENADA 500 //tiempo maximo de duracion de frenada tras recta
#define TICK_ENC_MAX_RECTA 500 //ticks máximos que dura la recta antes de frenar. 50ticks por vuelta de rueda.
#define TICK_ENC_MAX_FRENADA 100 //ticks máximos que dura la frenada

#define INTERVAL_RECT_TASK 500
#define VEL_BASE_RECTA 70
#define VEL_BASE_CURVA 65
#define VEL_BASE_FRENO 40
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
    Serial.begin(19200);
    delay(20);
    robot.calibrateIR( 5, false );
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
  schedulerVELBASEOn=false;
}
void inline activarFreno(){
  VEL_BASE= VEL_BASE_FRENO;
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


/*void loop() {
    if(schedulerVELBASEOn && (millis()>nextTaskTime || robot.cuentaEncoderDerecho>nextEndDerTaskCount || robot.cuentaEncoderIzquierdo>nextEndIzqTaskCount) ){ //planificador por tiempo para acelerar/frenar en recta
      schedulerVELBASEOn=false;
      callback();
    }
    //int err = robot.readLine();
    double errDif = robot.getErrorLine();//PID(err);
    //printTelemetria(errDif);
    robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );

    if(millis()>nextCheckRectTask){
      nextCheckRectTask=millis()+INTERVAL_RECT_TASK;
      if( !enRecta && abs((robot.cuentaEncoderIzquierdo-lastEncIzq) - (robot.cuentaEncoderDerecho-lastEncDer)) < DIFF_ENCODERS_RECT ){
        activarRecta();
      }
    }
    
}*/

void loop(){
  Serial.println(cuentaEncoderIzquierdo);
  delay(500);
}

