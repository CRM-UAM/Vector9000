#include <Arduino.h>
#include "Vector9000.h"

#define TIME_IGNORE_AFTER_BIF 300

#define NUM_SECTORES_VUELTA 4

#define DIF_B_SIGCONT 6
#define PIN_BOTON 7



int VEL_BASE=60;//50+5;
int VEL_BASE_IN_SIGNAL=-10;//(40+5)
int VEL_BASE_POST_BIFURCACION=15;//(34+5)
int VEL_BASE_PRE_BIFURCACION=10;//(34+5)

Vector9000 robot = Vector9000(0.04,3800.283,0);//(kp,kd, ki);

void setNewSector(int sector){
  /*switch(sector){
    /case 0:
      VEL_BASE=60;//50+5;
      VEL_BASE_IN_SIGNAL=-15;//(40+5)
      VEL_BASE_POST_BIFURCACION=20;//(34+5)
      VEL_BASE_PRE_BIFURCACION=30;//(34+5)
      break;
    case 3:
      VEL_BASE=60;//50+5;
      VEL_BASE_IN_SIGNAL=10;//(40+5)
      VEL_BASE_POST_BIFURCACION=40;//(34+5)
      VEL_BASE_PRE_BIFURCACION=30;//(34+5)
    default:
      VEL_BASE=70;
      VEL_BASE_IN_SIGNAL=-5;
      VEL_BASE_POST_BIFURCACION=30;
      VEL_BASE_PRE_BIFURCACION=30;
      break;
  }*/
}


void printTelemetria(float err){
    Serial.print("pT ");
    Serial.print(micros()/1000000.0);
    Serial.print("/");
    Serial.println(err);
    /*Serial.print("|");
    Serial.print(robot.readLine());
    Serial.print("|");
    Serial.print(robot._kp*1000);
    Serial.print(" ");
    Serial.print(robot._kd);
    Serial.print(" ");
    Serial.println(robot._ki);*/
}

bool boton_pulsado() {
  return !digitalRead(PIN_BOTON);
}


void setup(){
    pinMode(PIN_BOTON, INPUT_PULLUP);
    robot.config();
    Serial.begin(19200);
    robot.setSpeed( 0,0 );
    delay(1000);
    
    robot.calibrateIR( 5, true );
    while(!boton_pulsado()){
      delay(10);
    }
    while(boton_pulsado()){
      delay(10);
    }
}


long numSector=0;
unsigned int sigCont[20]={0};
void loop(){
  //if(boton_pulsado())setup();
  int sig=NOSIG;
  
  double errDif = robot.readErrLineWithSignals( &sig );
  //printTelemetria(robot._last_value);
  robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );
  if(sig<20 && sig>0)sigCont[sig]++;
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
      robot.setSpeed( VEL_BASE_IN_SIGNAL - errDif2, VEL_BASE_IN_SIGNAL + errDif2 );
      if(sig2null==NOSIG)sigcont0++;
    }
    digitalWrite(Vector9000::LED,HIGH);
    boolean bif=false;
    while(!bif){
      double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
      //printTelemetria(robot._last_value);
      robot.setSpeed( VEL_BASE_PRE_BIFURCACION - errEnBifurcacion, VEL_BASE_PRE_BIFURCACION + errEnBifurcacion );
      
    }
    //robot.setP(0.032);
    digitalWrite(Vector9000::LED,HIGH);
    unsigned long time2ignore=millis()+TIME_IGNORE_AFTER_BIF;
    while(millis() < time2ignore){
      double errEnBifurcacion = robot.readErrLineBifurcacion(sig, &bif); //Lee el error de la linea sin verse afectado por las senales
      //printTelemetria(robot._last_value);
      robot.setSpeed( VEL_BASE_POST_BIFURCACION - errEnBifurcacion, VEL_BASE_POST_BIFURCACION + errEnBifurcacion );
    }
    digitalWrite(Vector9000::LED,LOW);
    //reset variables
    //robot.setP(0.04);
    sigCont[NOSIG]=0;sigCont[SIG_IZQ]=0;sigCont[SIG_DER]=0;sigCont[SIG_DOUBLE]=0;
    numSector++;
    numSector =  numSector % NUM_SECTORES_VUELTA;
    Serial.println("sector: ");Serial.println(numSector);
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

