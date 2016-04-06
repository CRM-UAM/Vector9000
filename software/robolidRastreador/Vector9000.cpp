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
//const uint8_t Vector9000::ENC_IZQ_PIN = 7;
//const uint8_t Vector9000::ENC_DER_PIN = 8;
const uint8_t Vector9000::PULSOS_POR_REVOLUCION = 50;

volatile unsigned long cuentaEncoderIzquierdo = 0;
volatile unsigned long cuentaEncoderDerecho = 0;


/********* PLACA SENSORES QTR-8RC **********/
QTRSensorsRC qtrrc((unsigned char[]) {
   Vector9000::IR1, Vector9000::IR2, Vector9000::IR3, Vector9000::IR4, Vector9000::IR5, Vector9000::IR6, Vector9000::IR7, Vector9000::IR8}
, Vector9000::NUM_IR_SENSORS, Vector9000::TIMEOUT, QTR_NO_EMITTER_PIN); //emisor siempre encendido

void aumentarCuentaIzquierda()
{
  cuentaEncoderIzquierdo++;
}

void aumentarCuentaDerecha()
{
  cuentaEncoderDerecho++;
}


Vector9000::Vector9000( double KP=2200, double KD=0.06, double KI=0 ) {

  _kp = KP;
  _kd = KD;
  _ki = KI;
  _DerivativeErrorTerm = 0;
  _lastTimeExec = millis();
  _lastError = 0;


}

void Vector9000::config( void ){
  pinMode(Vector9000::M_DER_DIR1_PIN, OUTPUT);
  pinMode(Vector9000::M_DER_DIR2_PIN, OUTPUT);
  pinMode(Vector9000::M_DER_PWM_PIN, OUTPUT);
  pinMode(Vector9000::M_IZQ_DIR1_PIN, OUTPUT);
  pinMode(Vector9000::M_IZQ_DIR2_PIN, OUTPUT);
  pinMode(Vector9000::M_IZQ_PWM_PIN, OUTPUT);
  //enableInterrupt(Vector9000::ENC_DER_PIN, aumentarCuentaDerecha, CHANGE);
  //enableInterrupt(Vector9000::ENC_IZQ_PIN, aumentarCuentaIzquierda, CHANGE);
}

void Vector9000::setRSpeed( int s ){
    if(s > 0) {// Hacia delante
        digitalWrite(Vector9000::M_DER_DIR2_PIN, LOW);
        digitalWrite(Vector9000::M_DER_DIR1_PIN, HIGH);
        if(s<=255)analogWrite(Vector9000::M_DER_PWM_PIN , s);
    }else if(s < 0) {// Hacia atras
        digitalWrite(Vector9000::M_DER_DIR2_PIN, HIGH);
        digitalWrite(Vector9000::M_DER_DIR1_PIN, LOW);
        if(-s<=255)analogWrite(Vector9000::M_DER_PWM_PIN , -s);
    }else {//Parado
        digitalWrite(Vector9000::M_DER_PWM_PIN, LOW);// Motor apagado
    }
}

void Vector9000::setLSpeed( int s ){
    if(s > 0) {// Hacia delante
        digitalWrite(Vector9000::M_IZQ_DIR2_PIN, LOW);
        digitalWrite(Vector9000::M_IZQ_DIR1_PIN, HIGH);
        if(s<=255)analogWrite(Vector9000::M_IZQ_PWM_PIN , s);
    }else if(s < 0) {// Hacia atras
        digitalWrite(Vector9000::M_IZQ_DIR2_PIN, HIGH);
        digitalWrite(Vector9000::M_IZQ_DIR1_PIN, LOW);
        if(-s<=255)analogWrite(Vector9000::M_IZQ_PWM_PIN , -s);
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

/**
 * Devuelve el error diferencial (procesado con PID) siguiendo la linea ignorando las senales
 * @param  sig : puntero para devolver las senales detectadas (sig=0 no senales, sig=1 senal a la der, sig=-1 senal a la izq, sig=2 dos senales )
 * @return    Error diferencial para colocar la velocidad de las ruedas
 */
int Vector9000::readErrLineWithSignals( int *sig ){
    int error = readPosLineWithSignals(sig) - (Vector9000::NUM_IR_SENSORS-1)*500; //Centrar el error en el 0
    unsigned long currentTime=micros();

    if(currentTime - _lastTimeExec > 1000){//Genero el error derivativo
        _DerivativeErrorTerm = _kd*(error-_lastError)/(currentTime-_lastTimeExec);
        _lastTimeExec = currentTime;
        _lastError = error;
    }
    return _kp*error+_DerivativeErrorTerm;
}

int calcularMediaSensores(unsigned int *values){
    int i;
    unsigned long avg=0; // this is for the weighted total, which is long before division
    unsigned int sum=0; // this is for the denominator which is <= 64000
    //Serial.print("DLimpio -> ");
    for(i=0;i<Vector9000::NUM_IR_SENSORS;i++){ //media de los sensores ya limpios para sacar el error de la linea
        // only average in values that are above a noise threshold
        int value = values[i];
        //Serial.print(" ");
        //Serial.print(value);
        if(value > 50) {
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }
    //Serial.println("");
    //Serial.print("savg=");Serial.print(avg);Serial.print("sum=");SSerial.println(sum);
    return avg/sum;
}

int readMultipleLines(unsigned int *values, int *lines, float *centerOfLine, float *numSensorsOfLine){
    int i,j;
    qtrrc.readCalibrated(values);
    int cont_line=-1;
    int lastValueSensor=0;
    //Serial.print("D -> ");
    for(i=0;i<Vector9000::NUM_IR_SENSORS;i++) {
        int value = values[i];
        //Serial.print(" ");
        //Serial.print(value);
        if(value > 300){
            if(lastValueSensor<300){
                lines[++cont_line]=i;
            }
            centerOfLine[cont_line]+=i;
            numSensorsOfLine[cont_line]++;

        }
        lastValueSensor=value;
    }
    cont_line++;
    //sSerial.println("");
    return cont_line;
}
/**
 * Devuelve la posicion de la linea a seguir ignorando otras senales a los lados
 * @param  sig : puntero para devolver las senales detectadas (sig=0 no senales, sig=1 senal a la der, sig=-1 senal a la izq, sig=2 dos senales )
 * @return  Devuelve la posicion de a linea [0,7000]
 */
int Vector9000::readPosLineWithSignals( int *sig ){
    int i,j,h;
    unsigned int values[Vector9000::NUM_IR_SENSORS];
    //static int _last_value=0; // assume initially that the line is left.
    int lines[5]={0};
    float centerOfLine[5]={0};
    float numSensorsOfLine[5]={0};

    int cont_line = readMultipleLines(values, lines,centerOfLine,numSensorsOfLine);

    /*Serial.print("DEBUG ");Serial.print(cont_line); Serial.print("-");
    for(h=0;h<cont_line;h++){
        Serial.print(lines[h]);
        Serial.print("-");
        Serial.print(centerOfLine[h]);
        Serial.print("-");
        Serial.print(numSensorsOfLine[h]);
        Serial.print("_");
    }
    Serial.println("");*/

    if(cont_line<=0){
        *sig=0; //no sinales
        if(_last_value < (Vector9000::NUM_IR_SENSORS-1)*1000/2){
            return 0; // If it last read to the left of center, return 0.
        }
        else{
            return (Vector9000::NUM_IR_SENSORS-1)*1000; // If it last read to the right of center, return the max.
        }
    }else if(cont_line > 1){ //hay mas de una linea, seguir la mas cercana a la detectada anteriormente (_last_value)
        //Serial.println("ALERTA SIGNAL");
        double min_dist_line=9000;
        int min_ind=0;
        for(j=0;j<cont_line;j++){ //busco la linea que mas cercana esta al _last_value. Esta es la linea a seguir, las demas son señales
            double dist= abs( (_last_value*1.0f)/1000.0f - centerOfLine[j]/numSensorsOfLine[j]);
            if(dist<min_dist_line){
                min_dist_line=dist;
                min_ind=j;
            }
        }
        if(min_ind==0) *sig=1; //linea elegida para seguir a la izq por tanto senal a la derecha
        else if(min_ind==1 && cont_line==2) *sig=-1; //linea elegida para seguir a la der por tanto senal a la izq
        else if(min_ind==1 && cont_line==3) *sig=2; //linea elegida para seguir en el centro y con dos senales a los lados
        else *sig=-10; //formato de signales detactado erroneo
        //Serial.print(">>min_dis=");Serial.print(min_dist_line);Serial.print("  ind_min=");Serial.println(min_ind);
        //pongo a 0 los sensores activos que no han sido elegidos como la linea a seguir

        for(j=0;j<cont_line;j++){
            if(j!=min_ind)//limpio las lineas no cercanas
                for(h=lines[j];values[h]>300;h++)
                    values[h]=0;
        }

    }else{
        *sig=0; //no signales solo una linea para seguir
    }

    _last_value = calcularMediaSensores(values);
    return _last_value;
}

boolean Vector9000::detectarBifurcacion( void ){
    int i;
    unsigned int values[Vector9000::NUM_IR_SENSORS];
    int lines[5]={0};
    float centerOfLine[5]={0};
    float numSensorsOfLine[5]={0};
    int cont_line = readMultipleLines(values, lines, centerOfLine, numSensorsOfLine);
    if(cont_line > 1) return true;

    for(i=0;i<cont_line;i++){
      if(numSensorsOfLine[i]>=3)
        return true;
    }
    return false;
}
boolean _previus_bif=false;
double Vector9000::readErrLineBifurcacion( int sig, boolean *bifurcacion){
    int error = readPosLineBifurcacion(sig, bifurcacion) - (Vector9000::NUM_IR_SENSORS-1)*500; //Centrar el error en el 0
    if(_previus_bif==false && *bifurcacion==true){
      error = (error + _lastError)/2;
    }
    Serial.print(error);Serial.print(" : ");Serial.print(*bifurcacion);Serial.print(" | ");Serial.println(sig);
    unsigned long currentTime=micros();
    if(currentTime - _lastTimeExec > 1000){//Genero el error derivativo
        _DerivativeErrorTerm = _kd*(error-_lastError)/(currentTime-_lastTimeExec);
        _lastTimeExec = currentTime;
        _lastError = error;
    }
    _previus_bif=*bifurcacion;
    return _kp*error+_DerivativeErrorTerm;
}

int Vector9000::readPosLineBifurcacion( int sig, boolean *bifurcacion){
    int i,j;
    unsigned int values[Vector9000::NUM_IR_SENSORS];
    //static int _last_value=0; // assume initially that the line is left.
    int lines[5]={0};
    float centerOfLine[5]={0};
    float numSensorsOfLine[5]={0};

    int cont_line = readMultipleLines(values, lines, centerOfLine, numSensorsOfLine);

    int ind_line_viva=0;
    if(cont_line==0){
        *bifurcacion=false; //no bifurcacion
        if(_last_value < (Vector9000::NUM_IR_SENSORS-1)*1000/2){
            _last_value=0;
            return 0; // If it last read to the left of center, return 0.
        }
        else{
            _last_value=(Vector9000::NUM_IR_SENSORS-1)*1000;
            return _last_value; // If it last read to the right of center, return the max.
        }
    }else if(cont_line > 1){ //hay mas de una linea, seguir la que corresponde con la bifurcacion
        *bifurcacion=true; // bifurcacion
        if(sig==1){ // quiero girar a la derecha limpio todas menos la ultima linea
            for(i=0;i<cont_line-1;i++){
                for(j=lines[i];values[j]>300;j++)
                    values[j]=0;
            }
            ind_line_viva=cont_line-1;
        }else if(sig==-1){ // quiero girar a la izq limpio todas menos la primera linea
            for(i=1;i<cont_line;i++){
                for(j=lines[i];values[j]>300;j++)
                    values[j]=0;
            }
            ind_line_viva=0;
        }else if(sig==2){ //quiero seguir defrente
            _last_value= (Vector9000::NUM_IR_SENSORS-1)*1000 / 2; //seguir de frente posicion  de linea centrada
            return _last_value;
        }
    }else{ //solo una linea para seguir.
        if(numSensorsOfLine[0]<=3) *bifurcacion=false;
        else *bifurcacion=true;
        ind_line_viva=0;
    }

    double lastPos=(_last_value*1.0f)/1000.0f;
    double dist= abs( lastPos - centerOfLine[ind_line_viva]/numSensorsOfLine[ind_line_viva]);
    
    if( sig==-1 && dist > 2 && dist > lastPos  /*&& (sig==-1 || (lastPos < 1 && sig!=1))*/){ //realmente la linea mas cercana se ha perdido por el 0
         *bifurcacion=true;
         _last_value=0;
         Serial.print(lastPos);Serial.print(" return 0 ");Serial.println(dist);
         return _last_value; // If it last read to the left of center, return 0.
    }
    if( sig==1 && dist > 2 && dist > (Vector9000::NUM_IR_SENSORS-1)-lastPos /*&& (sig==1 || (lastPos>(Vector9000::NUM_IR_SENSORS-1) && sig!=-1))*/){//realmente la linea mas cercana se ha perdido por el 7000
         *bifurcacion=true;
         _last_value = (Vector9000::NUM_IR_SENSORS-1)*1000;
        Serial.print(lastPos);Serial.print(" return 7000 ");Serial.println(dist);
        return _last_value; // If it last read to the right of center, return the max.
    }
    //He limpiado y solo queda una linea, limpio todos los sensores activos menos los dos mas cercanos a donde quiero ir
    if(numSensorsOfLine[ind_line_viva]>3){
        if(sig==1)
            for(i=lines[ind_line_viva];values[i]>300 && numSensorsOfLine[ind_line_viva]>2;i++ , numSensorsOfLine[ind_line_viva]--){
                values[i]=0;
            }
        else if(sig==-1)
            for(i=lines[ind_line_viva]+2;values[i]>300 && numSensorsOfLine[ind_line_viva]>2;i++ , numSensorsOfLine[ind_line_viva]--){
                values[i]=0;
            }
        else{} //Si sig=2 (seguir de frente) no limpio sensores ya que luego coge la media
    }


    _last_value= calcularMediaSensores(values);
    return _last_value;

}

