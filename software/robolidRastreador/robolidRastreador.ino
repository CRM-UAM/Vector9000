#include <Arduino.h>
//#include <EnableInterrupt.h>
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


int VEL_BASE=80;//50+5;
#define TIME_CHECK_SIG 50 //tiempo en ms para volver a checkear si la senal leida se mantiene
#define VEL_BASE_PRE_INTERSECCION 30//(40+5)
#define VEL_BASE_BIFURCACION 25//(34+5)

Vector9000 robot = Vector9000(0.04,3800.283,0);//(kp,kd, ki);


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



/*unsigned long timeUltimaSig=0;
int sig=0;
void loop() {
    digitalWrite(Vector9000::LED,LOW);
    //Seguir linea sin senal detactada hasta ahora
    double errDif = robot.readErrLineWithSignals( &sig );
    robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );
    //sig = detectarSignals(); //devuelve 0=no signal, 1=signal derecha, -1=signal izq, 2=signal doble
    while(sig!=0){ //senal detectada, seguir linea hasta encontrar inteseccion
        digitalWrite(Vector9000::LED,HIGH);
        int sigtemp;
        errDif = robot.readErrLineWithSignals(&sigtemp); //Lee el error de la linea sin verse afectado por las senales
        robot.setSpeed( VEL_BASE_PRE_INTERSECCION - errDif, VEL_BASE_PRE_INTERSECCION + errDif );
        boolean bifurcacion = robot.detectarBifurcacion(); //detecta si hay mas de una linea
        while(bifurcacion != 0){
            double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bifurcacion); //Lee el error de la linea sin verse afectado por las senales
            robot.setSpeed( VEL_BASE_BIFURCACION - errEnBifurcacion, VEL_BASE_BIFURCACION + errEnBifurcacion );
            //bifurcacion = robot.detertarBifuercacion(); //detecta si hay mas de una linea
            if(!bifurcacion){ //volvemos a empezar el loop despues de haber realizado la interseccion completa
                //sigDef=0;
                sig=0;
                timeUltimaSig=0;
            }
        }
    }

}*/
unsigned int sigCont[3]={0};
void loop(){
  //if(boton_pulsado())setup();
  int sig=0;
  double errDif = robot.readErrLineWithSignals( &sig );
  robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );
  if(sig!=0){
    if(sig<0){
      sigCont[0]++;
    }else{
      sigCont[sig]++;
    }
  }
  if(sigCont[0]>sigCont[1]+DIF_B_SIGCONT && sigCont[0]>sigCont[2]+DIF_B_SIGCONT){
    sig=-1;
  }else if(sigCont[1]>sigCont[0]+DIF_B_SIGCONT && sigCont[1]>sigCont[2]+DIF_B_SIGCONT){
    sig=1;
  }else if(sigCont[2]>sigCont[0]+DIF_B_SIGCONT && sigCont[2]>sigCont[1]+DIF_B_SIGCONT){
    sig=2;
  }else{
    sig = 0;
  }
  if(sig!=0){
    int sigcont0=0;
    int sig2null=0;
    while(sigcont0 <  DIF_B_SIGCONT ){
      double errDif2 = robot.readErrLineWithSignals( &sig2null );
      robot.setSpeed( VEL_BASE_PRE_INTERSECCION - errDif2, VEL_BASE_PRE_INTERSECCION + errDif2 );
      if(sig2null==0)sigcont0++;
    }
    digitalWrite(Vector9000::LED,HIGH);
    boolean bif=false;
    while(!bif){
      double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
      robot.setSpeed( VEL_BASE_BIFURCACION-2 - errEnBifurcacion, VEL_BASE_BIFURCACION-2 + errEnBifurcacion );
      
    }
    digitalWrite(Vector9000::LED,HIGH);
    unsigned long time2ignore=millis()+TIME_IGNORE_AFTER_BIF;
    while(millis() < time2ignore){
      double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
      robot.setSpeed( VEL_BASE_BIFURCACION+2 - errEnBifurcacion, VEL_BASE_BIFURCACION+2 + errEnBifurcacion );
    }
    digitalWrite(Vector9000::LED,LOW);
    //reset variables
    sigCont[0]=0;sigCont[1]=0;sigCont[2]=0;
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

