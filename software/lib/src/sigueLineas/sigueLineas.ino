#include <Arduino.h>
#include "Vector9000.h"



int VEL_BASE=60;
Vector9000 robot = Vector9000(0.065,3000,0);//(500,30, 0);


String ultimoComando = "";
void leerComando (){
  int numLetras = Serial.available ();

  for(int i = 0; i<numLetras; i++){
    char letra = (char)Serial.read();

    if (letra == '='){
      ultimoComando = "" ;

    }
    else if (letra != '+'){
      ultimoComando = ultimoComando + letra;
    }
  } 
} 

int updateParams(){
  char cmd[30];
  ultimoComando.toCharArray(cmd, 30);
  ultimoComando="";
  switch(cmd[0]){
    case 'P':
      robot.setP(atof(cmd+1));
      return 1;
    case 'D':
      robot.setD(atof(cmd+1));
      return 1;
   case 'I':
      robot.setI(atof(cmd+1));
      return 1;
    case 'B': 
      VEL_BASE=atof(cmd+1);
      return 1;
  }
  return 0;
}

bool inMeta(){
  int count=0;
  unsigned int val[8];
  robot.getIRRaw( val );
    for (int i=0;i<8;i++){
      if(val[i] > 1400){
        count++;
      }
    }
  return count > 5; 

}

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


int contMeta=-1;
unsigned long timer=0;
unsigned long timerM=0;
void loop() {
    
    //int err = robot.readLine();
    double errDif = robot.getErrorLine();//PID(err);
    //printTelemetria(errDif);
    
    robot.setSpeed( VEL_BASE - errDif, VEL_BASE + errDif );


    if(inMeta() && millis()>timerM+200){
      timerM=millis();
      contMeta++;
      //Serial.print("META ");Serial.println(contMeta);
      
      int i=0;
      for(i=0;i<3;i++){
        leerComando();
        if(updateParams()){
          contMeta=0;
          timer=millis();
        }
      }
      
      if(contMeta>=3){
        Serial.println(millis()-timer);
        timer=millis();
        contMeta=0;
      }
    } 
    
}
