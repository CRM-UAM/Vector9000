#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "speedController.h"

#include "Vector9000.h"

//public
/**
 * Control variables
 */
int targetSpeedL;
int targetSpeedR;

/**
 * Output variable. To be readed
 */
  long encoderCount;
  float curSpeedL;
  float curSpeedR;
  float accX;
  float decX;
  float accW;
  float decW;

/**
 * Tune variables
 */
  float ir_weight;
  float kpL, kdL;
  float kpR, kdR; //used in straight





//private
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float pidInputL = 0;
float pidInputR = 0;
float posErrorL = 0;
float posErrorR = 0;
float oldPosErrorL = 0;
float oldPosErrorR = 0;

int posPwmL = 0;
int posPwmR = 0;



  
long encoderChange=0;
unsigned long leftEncoderChange=0;
unsigned long rightEncoderChange=0;
unsigned long leftEncoderOld = 0;
unsigned long rightEncoderOld = 0;
long leftEncoder = 0;
long  rightEncoder = 0;
long leftEncoderCount = 0;
long rightEncoderCount = 0;
    
int leftBaseSpeed = 0;
int rightBaseSpeed = 0;


void speedProfile(void *a){ 
    getEncoderStatus(); 
    updateCurrentSpeed(); 
    calculateMotorPwm(); 
}
extern unsigned long cuentaEncoderIzquierdo; 
extern unsigned long cuentaEncoderDerecho;

void getEncoderStatus()
{
    unsigned long leftEncoder = cuentaEncoderIzquierdo;//robot.getEncLeftCount(); //leftEncoder = TIM2->CNT;//read current encoder ticks from register of 32 bit general purpose timer 2
    unsigned long rightEncoder = cuentaEncoderDerecho;//robot.getEncRightCount(); //rightEncoder = TIM5->CNT;//read current encoder ticks from register of 32 bit general purpose timer 5

    leftEncoderChange = leftEncoder - leftEncoderOld;
    rightEncoderChange = rightEncoder - rightEncoderOld;

    leftEncoderOld = leftEncoder;
    rightEncoderOld = rightEncoder;

    leftEncoderCount += leftEncoderChange;
    rightEncoderCount += rightEncoderChange;
    encoderCount =  (leftEncoderCount+rightEncoderCount)/2;
}



void updateCurrentSpeed(void)
{
    if(curSpeedL < targetSpeedL && curSpeedL >= 0){
        curSpeedL += accX;
        if(curSpeedL > targetSpeedL)
            curSpeedL = targetSpeedL;
    }else if(curSpeedL > targetSpeedL && curSpeedL > 0){
        curSpeedL -= decX;
        if(curSpeedL < targetSpeedL)
            curSpeedL = targetSpeedL;
    }else if(curSpeedL < targetSpeedL && curSpeedL < 0){
        curSpeedL += decX;
        if(curSpeedL > targetSpeedL)
            curSpeedL = targetSpeedL;
    }else if(curSpeedL > targetSpeedL && targetSpeedL <= 0){
        curSpeedL -= accX;
        if(curSpeedL < targetSpeedL)
            curSpeedL = targetSpeedL;
    }
    
    if(curSpeedR < targetSpeedR && curSpeedR >= 0){
        curSpeedR += accX;
        if(curSpeedR > targetSpeedR)
            curSpeedR = targetSpeedR;
    }else if(curSpeedR > targetSpeedR && curSpeedR > 0){
        curSpeedR -= decX;
        if(curSpeedR < targetSpeedR)
            curSpeedR = targetSpeedR;
    }else if(curSpeedR < targetSpeedR && curSpeedR < 0){
        curSpeedR += decX;
        if(curSpeedR > targetSpeedR)
            curSpeedR = targetSpeedR;
    }else if(curSpeedR > targetSpeedR && targetSpeedR <= 0){
        curSpeedR -= accX;
        if(curSpeedR < targetSpeedR)
            curSpeedR = targetSpeedR;
    }
    
}


void calculateMotorPwm(void) // encoder PD controller
{

  
  
    posErrorL += curSpeedL - leftEncoderChange;
    posErrorR += curSpeedR - rightEncoderChange;
   


    Serial.print("Time: ");
    Serial.println(micros());
    Serial.print("SppedL: ");
    Serial.println(curSpeedL);
    Serial.print("SppedR: ");
    Serial.println(curSpeedR);
    Serial.print("EncoderFedbackL: ");
    Serial.println(leftEncoderChange);
    Serial.print("EncoderFedbackR: ");
    Serial.println(rightEncoderChange);
    



    posPwmL = kpL * posErrorL + kdL * (posErrorL - oldPosErrorL);
    posPwmR = kpR * posErrorR + kdR * (posErrorR - oldPosErrorR); 

//    Serial.print("OutputSpeedL");
//    Serial.println(posPwmL);
//    Serial.print("OutputSpeedR");
//    Serial.println(posPwmR);

    oldPosErrorL = posErrorL;
    oldPosErrorR = posErrorR;

    robot.setSpeed(posPwmL,posPwmR);
    
//    if(p_telemetria<SIZE_TELEMETRIA){
//      telemetria[p_telemetria][0]=micros();
//      telemetria[p_telemetria][1]=curSpeedL;
//      telemetria[p_telemetria][2]=encoderFeedbackX;
//      //telemetria[p_telemetria][3]=curSpeedR;
//      telemetria[p_telemetria][3]=curSpeedR;
//      telemetria[p_telemetria][4]=rotationalFeedback;
//      telemetria[p_telemetria][5]=posErrorWir;
//      //telemetria[p_telemetria][6]=leftEncoderChange;
//      p_telemetria++;
//    }
    
}





void resetSpeedProfile(void)
{
    //reset Everything and reTune parameters;


    robot.setSpeed(0,0);//setLeftPwm(0);//setRightPwm(0);

  //Tune parameters
   ir_weight = 0.0;
   kpL = 20; kdL = 100;
   kpR = 20; kdR = 100;
  

   accX = 2;// 2ticks * 100s^-1 * (30mm * pi)/(100 ticks / rev) = 188.49 mm/s^2  [ejecutando la rutina cada 10ms]
   decX = 3; // -282.74 mm/s^

   


//reset variables
    
    curSpeedL = 0;
    curSpeedR = 0;
    targetSpeedL = 0;
    targetSpeedR = 0;
    posErrorL = 0;
    posErrorR = 0;
    oldPosErrorL = 0;
    oldPosErrorR = 0;
    leftEncoderOld = 0;
    rightEncoderOld = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    encoderCount = 0;
    leftBaseSpeed = 0;
    rightBaseSpeed = 0;

    //robot.resetEncoders();//reset left encoder count //TIM5->CNT = 0;//reset right encoder count
    cuentaEncoderIzquierdo=0; 
    cuentaEncoderDerecho=0;
}
