#include <Arduino.h>
#include <EnableInterrupt.h>
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
#define TIME_CHECK_SIG 50 //tiempo en ms para volver a checkear si la senal leida se mantiene
#define VEL_BASE_PRE_INTERSECCION 50
#define VEL_BASE_BIFURCACION 40

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


void setup(){
    robot.config();
    Serial.begin(19200);
    delay(20);
    robot.calibrateIR( 5, false );
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

  //Configurar duracion de frenada
  schedulerVELBASEOn=true;
  callback=&activarCurvas;
  nextTaskTime=millis()+TIME_MAX_FRENADA;
  nextEndDerTaskCount=robot.cuentaEncoderDerecho+TICK_ENC_MAX_FRENADA;
  nextEndIzqTaskCount=robot.cuentaEncoderIzquierdo+TICK_ENC_MAX_FRENADA;
}

void inline activarRecta(){
  enRecta=true;
  VEL_BASE= VEL_BASE_RECTA;

  //Configurar el punto de frenada
  schedulerVELBASEOn=true;
  callback=&activarFreno;
  nextTaskTime=millis()+TIME_MAX_IN_RECTA;
  nextEndDerTaskCount=robot.cuentaEncoderDerecho+TICK_ENC_MAX_RECTA;
  nextEndIzqTaskCount=robot.cuentaEncoderIzquierdo+TICK_ENC_MAX_RECTA;
}

unsigned long timeUltimaSig=0;
int sig=0;
/*void loop() {

    //Seguir linea sin senal detactada hasta ahora
    double errDif = robot.readErrLineWithSignals( &sig );
    robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );
    //sig = detectarSignals(); //devuelve 0=no signal, 1=signal derecha, -1=signal izq, 2=signal doble
    while(sig!=0){ //senal detectada, seguir linea hasta encontrar inteseccion
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

void loop(){
   int errDif = robot.readPosLineWithSignals( &sig );
   Serial.print(errDif);
   Serial.print(": ");
   Serial.println(sig);
   delay(200);
}
