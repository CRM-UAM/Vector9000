#include <Arduino.h>
#include "Vector9000.h"

#define TIME_IGNORE_AFTER_BIF 200
#define TIME_IGNORE_AFTER_SIGNAL 50



#define DIF_B_SIGCONT 10

#define PIN_BOTON 7

#define DIFF_ENCODERS_RECT 50
#define TIME_MAX_IN_RECTA 2000 //tiempo máximo que está el robot en recta antes de frena. En ms.
#define TIME_MAX_FRENADA 500 //tiempo maximo de duracion de frenada tras recta
#define TICK_ENC_MAX_RECTA 500 //ticks máximos que dura la recta antes de frenar. 50ticks por vuelta de rueda.
#define TICK_ENC_MAX_FRENADA 100 //ticks máximos que dura la frenada


int VEL_BASE=62;//50+5;
#define TIME_CHECK_SIG 50 //tiempo en ms para volver a checkear si la senal leida se mantiene
#define VEL_BASE_PRE_INTERSECCION -10//(40+5)
#define VEL_BASE_POST_BIFURCACION 15//(34+5)
#define VEL_BASE_PRE_BIFURCACION 40//(34+5)

Vector9000 robot = Vector9000(0.04,3800.283,0);//(kp,kd, ki);




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


void setup(){
    robot.config();
    Serial.begin(19200);
    robot.setSpeed( 0,0 );
    pinMode(PIN_BOTON, INPUT_PULLUP);
    delay(20);
    robot.calibrateIR( 5, true );
    while(!boton_pulsado()) delay(10);
    while(boton_pulsado()) delay(10);
}




unsigned int sigCont[20]={0};
void loop(){
  //if(boton_pulsado())setup();
  int sig=NOSIG;
  double errDif = robot.readErrLineWithSignals( &sig );
  robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );
  sigCont[sig]++;
  if( sigCont[SIG_DER]>sigCont[SIG_IZQ]+DIF_B_SIGCONT && sigCont[SIG_DER]>sigCont[SIG_DOUBLE]+DIF_B_SIGCONT){
    sig=SIG_DER;
  }else if( sigCont[SIG_IZQ]>sigCont[SIG_DER]+DIF_B_SIGCONT && sigCont[SIG_IZQ]>sigCont[SIG_DOUBLE]+DIF_B_SIGCONT){
    sig=SIG_IZQ;
  }else if( sigCont[SIG_DOUBLE]>sigCont[SIG_DER]+DIF_B_SIGCONT && sigCont[SIG_DOUBLE]>sigCont[SIG_IZQ]+DIF_B_SIGCONT){
    sig=SIG_IZQ;
  }else{
    sig=NOSIG;
  }
  if(sig!=NOSIG){
    int sigcont0=0;
    int sig2null=NOSIG;
    while(sigcont0 <  DIF_B_SIGCONT ){
      double errDif2 = robot.readErrLineWithSignals( &sig2null );
      robot.setSpeed( VEL_BASE_PRE_INTERSECCION - errDif2, VEL_BASE_PRE_INTERSECCION + errDif2 );
      if(sig2null==NOSIG)sigcont0++;
    }
    digitalWrite(Vector9000::LED,HIGH);
    boolean bif=false;
    while(!bif){
      double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
      robot.setSpeed( VEL_BASE_PRE_BIFURCACION-2 - errEnBifurcacion, VEL_BASE_PRE_BIFURCACION-2 + errEnBifurcacion );
      
    }
    digitalWrite(Vector9000::LED,HIGH);
    unsigned long time2ignore=millis()+TIME_IGNORE_AFTER_BIF;
    while(millis() < time2ignore){
      double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
      robot.setSpeed( VEL_BASE_POST_BIFURCACION+2 - errEnBifurcacion, VEL_BASE_POST_BIFURCACION+2 + errEnBifurcacion );
    }
    digitalWrite(Vector9000::LED,LOW);
    //reset variables
    sigCont[NOSIG]=0;sigCont[SIG_IZQ]=0;sigCont[SIG_DER]=0;sigCont[SIG_DOUBLE]=0;
  }
}
/*
void loop(){
  int sig=1;
  boolean bif=false;
  double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
  robot.setSpeed( VEL_BASE_BIFURCACION - errEnBifurcacion, VEL_BASE_BIFURCACION + errEnBifurcacion );
  if(bif)digitalWrite(Vector9000::LED,HIGH);
  else digitalWrite(Vector9000::LED,LOW);
}*/

