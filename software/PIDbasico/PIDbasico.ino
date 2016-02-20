#include <EEPROM.h>
#include <QTRSensors.h>
#include <Arduino.h>
#include <math.h>



#define KD 2200
#define KP 0.06
#define VEL_BASE 50
#define VEL_RECTA 50
#define VEL_FRENO 50

unsigned long initialTime,lastLogTime;

#define LED 13       // Pin para el LED
#define M_INPUT_1 11  // Pines para el driver de motores L298
#define M_INPUT_2 9
#define M_INPUT_3 10
#define M_INPUT_4 12
#define M_ENABLE_A 6
#define M_ENABLE_B 5


#define LIM_MOTORES 200

#define NUM_CNY70 4

#define QTR_0 8
#define QTR_1 4
#define QTR_2 3
#define QTR_3 2
#define QTR_4 A1
#define QTR_5 7
#define QTR_6 A2
#define QTR_7 A0
#define NUM_SENSORS 8
#define LONG_RECTA 500000 //1 seg
#define TIMEOUT 2000  // waits for 2500 us for sensor outputs to go low
#define /*EMITTER_PIN 13 */EMITTER_PIN QTR_NO_EMITTER_PIN    // Controlar el emiso IR con el pin 13 como salida (Cuidado LED) o siempre encendido QTR_NO_EMITTER_PIN

#define INTER_ERR_D 10000 //Intervalo de tiempo para generar la parte derivativa del errorr (en uSeg) [a 2m/s en 10000us avanza 2cm]

#define PULSADOR A5   //Pin para el pulsador


#define MUX_S0 A1
#define MUX_S1 A2
#define MUX_S2 A3
#define MUX_Z A4

// ID relativo al MUX
#define IR_4 1
#define IR_3 2
#define IR_2 3
#define IR_1 0
#define LDR_1 4
#define LDR_2 5
#define LDR_2 6
#define LDR_2 7

#define RUIDO_LDR 100

#define LDR_FRONT 4
#define LDR_RIGHT 5
#define LDR_BACK 6
#define LDR_LEFT 7

/*^^^^^^^^^^^^^^^^^^^^^^^^^ PROTOTIPOS .H ^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/**** MOTORES ****/
void inicia_pines_driver_motores();
/*Funciones para el control de los motores
 Toman un parametro que va de -255 a 255 que indica la velocidad y direccion de rotacion*/
void motor_izq(int velocidad);
void motor_der(int velocidad);


/*** SENSORES QTR-8RC ***/

void calibraIR(int seg);
/*Ejecutar solo una vez [ en setup() ] imprime por el puerto serie*/

void prueba_sensoreIR(unsigned int *sensorValues);
/*Realiza lectura de los sensores y de linea imprime por el puerto serie*/


int getErrorPD(unsigned int *sensorValues,double kp, double kd);
/*Devuelve el error que tenemos que aplicar a los motores directamente, la salida del control PD:
  La parte derivativa se ejecuta solo si ha pasado INTER_ERR_D uSeg desde alguna llamada anterior
  retorno: [-3500*kp-7000*kd , 3500*kp+7000*kd]
 */
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/**********VARIABLES GLOBALES ***********************/
int minIRSensor[4];
int maxIRSensor[4];
int minLDRSensor[4];
int maxLDRSensor[4];




unsigned int sensorValues[NUM_SENSORS];
int errArray[200];
int i=-1;
int parpadeoled=0;
unsigned long rectaTime=0;

/*^^^^^^^^^^^^^^^^^^^^^^^^^ FUNCIONES BASICAS .c ^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*********  MOTORES  *********/
void inicia_pines_driver_motores() {
  // Inicia los pines como salidas
  pinMode(M_INPUT_1, OUTPUT);
  pinMode(M_INPUT_2, OUTPUT);
  pinMode(M_INPUT_3, OUTPUT);
  pinMode(M_INPUT_4, OUTPUT);
  pinMode(M_ENABLE_A, OUTPUT);
  pinMode(M_ENABLE_B, OUTPUT);

  // Los inicia apagados
  digitalWrite(M_INPUT_1, LOW);
  digitalWrite(M_INPUT_2, LOW);
  digitalWrite(M_INPUT_3, LOW);
  digitalWrite(M_INPUT_4, LOW);
  digitalWrite(M_ENABLE_A, LOW);
  digitalWrite(M_ENABLE_B, LOW);
}

void motor_der(int velocidad) {
  if(velocidad > 0) {// Hacia delante
    digitalWrite(M_INPUT_2, LOW);// Direccion de giro
    digitalWrite(M_INPUT_1, HIGH);

    analogWrite(M_ENABLE_A, velocidad);// Velocidad (siempre un parametro positivo para PWM)

  }
  else if(velocidad < 0) {// Hacia atras
    digitalWrite(M_INPUT_2, HIGH);// Direccion de giro
    digitalWrite(M_INPUT_1, LOW);

    analogWrite(M_ENABLE_A, -velocidad);// Velocidad (siempre un parametro positivo para PWM)

  }
  else {//Parado
    digitalWrite(M_ENABLE_A, LOW);// Motor apagado
  }
}

void motor_izq(int velocidad) {
  if(velocidad > 0) {// Hacia delante
    digitalWrite(M_INPUT_4, LOW);// Direccion de giro
    digitalWrite(M_INPUT_3, HIGH);

    analogWrite(M_ENABLE_B, velocidad);// Velocidad (siempre un parametro positivo para PWM)

  }
  else if(velocidad < 0) {//Hacia atras
    digitalWrite(M_INPUT_4, HIGH);// Direccion de giro
    digitalWrite(M_INPUT_3, LOW);

    analogWrite(M_ENABLE_B, -velocidad);// Velocidad (siempre un parametro positivo para PWM)

  }
  else {//Parado
    digitalWrite(M_ENABLE_B, LOW);// Motor apagado
  }
}

/*******************************************/
int comprobarValoresEnRango(int *array,int num,int min, int max){
  int j;
  int contador=0;
  for(j=0;j<num;j++){
    if(array[j]<max && array[j]>min)
      contador++;
  }
  return contador;
}

/********** MUX ****************/
void configuraPinesMUX() {
  int j;
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  for(j=0;j<4;j++){
    minIRSensor[j]=2000;
    maxIRSensor[j]=-1;
  }
  for(j=4;j<8;j++){
    minLDRSensor[j-4]=2000;
    maxLDRSensor[j-4]=-1;
  }
}

int leePinMuxAnalogico(int pinID) {
  switch(pinID){
    case 0:
      digitalWrite( MUX_S0,0);
      digitalWrite( MUX_S1,0);
      digitalWrite( MUX_S2,0);
      break;
      case 1:
      digitalWrite( MUX_S0, 1 );
      digitalWrite( MUX_S1,0 );
      digitalWrite( MUX_S2, 0 );
      break;
     case 2:
      digitalWrite( MUX_S0, 0 );
      digitalWrite( MUX_S1,1 );
      digitalWrite( MUX_S2, 0 );
      break;
     case 3:
      digitalWrite( MUX_S0, 1 );
      digitalWrite( MUX_S1,1 );
      digitalWrite( MUX_S2, 0 );
      break;
     case 4:
      digitalWrite( MUX_S0, 0 );
      digitalWrite( MUX_S1,0 );
      digitalWrite( MUX_S2, 1 );
      break;
     case 5:
      digitalWrite( MUX_S0, 1 );
      digitalWrite( MUX_S1,0 );
      digitalWrite( MUX_S2, 1 );
      break;
     case 6:
      digitalWrite( MUX_S0, 0 );
      digitalWrite( MUX_S1,1);
      digitalWrite( MUX_S2, 1 );
      break;
     case 7:
      digitalWrite( MUX_S0,1);
      digitalWrite( MUX_S1,1);
      digitalWrite( MUX_S2,1);
      break;
     default:
      break;

  }
  /*digitalWrite( MUX_S0, bitRead(pinID,0) ); // Configuramos los pines de selección
  digitalWrite( MUX_S1, bitRead(pinID,1) );
  digitalWrite( MUX_S2, bitRead(pinID,2) );*/
  delayMicroseconds(2); // Esto no debería ser necesario ya que el MUX reacciona en <500ns
  return analogRead(MUX_Z);

}
int readPinMuxIRCalibrated(int pinID){
  int lectura;
  lectura=1023-leePinMuxAnalogico(pinID);


  if(lectura>maxIRSensor[pinID]){
    return 1000;
  }
  else if(lectura<minIRSensor[pinID]){
    return 0;
  }
  else{
    int valor = map(lectura,minIRSensor[pinID],maxIRSensor[pinID],0,1000);
    //int valor=( (lectura-minIRSensor[pinID])/(maxIRSensor[pinID]-minIRSensor[pinID]) )*1000;
    return valor;
  }
}
int errorLinePasado;
int readLineCNY70(){
  int j;
  int suma=0;
  int lectura;
  long errorLine=0;
  long errorLineCero=0;
  int linea=0;
  lectura=readPinMuxIRCalibrated(IR_1);
  //Serial.print("IR1: ");Serial.print(lectura);Serial.print("  \n");
  if(lectura>350){
    linea=1;
  }
  if(lectura>50){
    suma+=lectura;
  }
   lectura=readPinMuxIRCalibrated(IR_2);
 //  Serial.print("IR2: ");Serial.print(lectura);Serial.print("  \n");
  if(lectura>350){
    linea=1;
  }
  if(lectura>50){
    suma+=lectura;
    errorLine+=(1000*(long)lectura);
  }
  lectura=readPinMuxIRCalibrated(IR_3);
 // Serial.print("IR3: ");Serial.print(lectura);Serial.print("  \n");
  if(lectura>350){
    linea=1;
  }
  if(lectura>50){
    suma+=lectura;
    errorLine+=(2000*(long)lectura);
  }
  lectura=readPinMuxIRCalibrated(IR_4);
  //Serial.print("IR4: ");Serial.print(lectura);Serial.print("  \n");
  if(lectura>350){
    linea=1;
  }
  if(lectura>50){
    suma+=lectura;
    errorLine+=(3000*(long)lectura);
  }
  /*Serial.print("  ERROR1:  ");
  Serial.print(errorLine);*/
  errorLine/=suma;
  /*Serial.print("  Suma:  ");
  Serial.print(suma);
  Serial.print("  ERROR1:  ");
  Serial.print(errorLine);*/
  errorLineCero=errorLine-1500;
  /*Serial.print("  ERROR-Cero:  ");
  Serial.print(errorLineCero);
  Serial.println("  ");*/
  if(!linea && errorLinePasado<=0){
    errorLinePasado=-1500;
    return -1500;
  }
  else if(!linea && errorLinePasado>0){
    errorLinePasado=1500;
    return 1500;
  }
  else{
    errorLinePasado=errorLineCero;
    return errorLineCero;
  }

}

int leePinMuxDigital(int pinID) {

  digitalWrite( MUX_S0, bitRead(pinID,0) ); // Configuramos los pines de selección
  digitalWrite( MUX_S1, bitRead(pinID,1) );
  digitalWrite( MUX_S2, bitRead(pinID,2) );
  delayMicroseconds(1); // Esto no debería ser necesario ya que el MUX reacciona en <500ns
  return digitalRead(MUX_Z);

}

void sensorMuxIRCalibrar(){
  int lectura;
  lectura=1023-leePinMuxAnalogico(IR_1);
  if(lectura<minIRSensor[IR_1])
    minIRSensor[IR_1]=lectura;
  if(lectura>maxIRSensor[IR_1])
    maxIRSensor[IR_1]=lectura;
  lectura=1023-leePinMuxAnalogico(IR_2);
  if(lectura<minIRSensor[IR_2])
    minIRSensor[IR_2]=lectura;
  if(lectura>maxIRSensor[IR_2])
    maxIRSensor[IR_2]=lectura;
  lectura=1023-leePinMuxAnalogico(IR_3);
  if(lectura<minIRSensor[IR_3])
    minIRSensor[IR_3]=lectura;
  if(lectura>maxIRSensor[IR_3])
    maxIRSensor[IR_3]=lectura;
  lectura=1023-leePinMuxAnalogico(IR_4);
  if(lectura<minIRSensor[IR_4])
    minIRSensor[IR_4]=lectura;
  if(lectura>maxIRSensor[IR_4])
    maxIRSensor[IR_4]=lectura;
}

void sensorMuxLDRCalibrar(){
  int lectura;
  int j;
  for(j=0;j<4;j++){
    lectura=leePinMuxAnalogico(j+4);
    if(lectura<minLDRSensor[j])
      minLDRSensor[j]=lectura;
    if(lectura>maxLDRSensor[j])
      maxLDRSensor[j]=lectura;
  }

}

int readPinMuxLDRCalibrated(int pinID){

 int lectura;
  lectura=leePinMuxAnalogico(pinID+4);
  if(lectura>maxLDRSensor[pinID]){
    maxLDRSensor[pinID]=lectura;
    return 1000;
  }
  else if(lectura<minLDRSensor[pinID]){
    minLDRSensor[pinID]=lectura;
    return 0;
  }
  else{
    return map(lectura,minLDRSensor[pinID],maxLDRSensor[pinID],0,1000);
  }
}



/********* PLACA SENSORES QTR-8RC **********/
QTRSensorsRC qtrrc((unsigned char[]) {
  QTR_0, QTR_1, QTR_2, QTR_3, QTR_4, QTR_5, QTR_6, QTR_7}
, NUM_SENSORS, TIMEOUT, EMITTER_PIN);



void calibraIR(int seg){
  int i;
  delay(500);
  digitalWrite(LED,HIGH);
  for (i = 0; i < 40*seg; i++)
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(LED,LOW);


  //Muestro los valores de calibracion por el puerto seria

  Serial.println("Resultados\nMinimos:");
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.println("Maxmos:");
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void pruebaSensoresIR(unsigned int *sensorValues){
  unsigned int position = qtrrc.readLine(sensorValues);

  //qtrrc.read(sensorValues);


  unsigned char i;
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]*10/1001);
    Serial.print(' ');
  }
  Serial.print("    ");
  Serial.println(position);

  delay(250);
}


/**PID**/
int lastError=0;
unsigned long lastTime=0;
int Derror=0;
int getErrorPD(unsigned int *sensorValues,double kp, double kd){
  unsigned long currentTime;
  unsigned int position = qtrrc.readLine(sensorValues);
  int error=position-(NUM_SENSORS-1)*500; //Centrar el error en el 0
  errArray[i]=error;
  currentTime=micros();

  if(currentTime-lastTime > INTER_ERR_D){//Genero el error derivativo
    Derror=kd*(error-lastError)/(currentTime-lastTime);
    lastTime=currentTime;
    lastError=error;
  }

  /*Si el error ha cambiado mucho en proporcion aunque no haya pasado suficiente tiempo actualizo la parte derivativa
   if( (error/lastError)>2 || (error/lasError)<-2){
   Derror=kd*(error-lastError)/(currentTime-lastTime);
   lastTime=currentTime;
   lastError=error;
   }
   */
  return kp*error+Derror;
}
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
double kProp=KP;
double kDer=KD;


void setup(){ //11,5 seg
  int j,pulsador;
  double mod;
  Serial.begin(19200);
  delay(3000);
  pinMode(PULSADOR,OUTPUT);
  Serial.println("STARTUP");
  unsigned long time=0;
  pinMode(LED, OUTPUT);
  for(j=0;j<200;j++)
    errArray[j]=9000;//INICIALIZO EL ERROR DE ARRAY A UN VALOR: "NO RECTA"

  Serial.println("Configurando pines MUX");
  configuraPinesMUX();
  Serial.println("Configurando driver motores");
  inicia_pines_driver_motores();



  delay(1000);
  Serial.println("Calibrando CNY70...");
  delay(10);

  //2 - calibracion CNY70

  //while( (pulsador=digitalRead(PULSADOR))==1 && (micros()-time)<250000);
  //if(pulsador==1){
    digitalWrite(LED,HIGH);
    for(j=0;j<2500;j++){  //5 seg
      sensorMuxIRCalibrar();
      delay(2);
    }
    for(j=0;j<4;j++){
      EEPROM.write(j+10,minIRSensor[j]/4);
      EEPROM.write(j+20,maxIRSensor[j]/4);
      Serial.print(j);
      Serial.print(" ");
      Serial.print(minIRSensor[j]/4);
      Serial.print(" ");
      Serial.println(maxIRSensor[j]/4);

    }
    digitalWrite(LED,LOW);
  /*}else{
    for(j=0;j<NUM_SENSORS;j++){
      minIRSensor[j]=EEPROM.read(j+10)*4;
      maxIRSensor[j]=EEPROM.read(j+20)*4;
    }
  }*/
  Serial.println("CALIBRANDO IR Delanteros");
  calibraIR(5);
  Serial.println("CALIBRACION COMPLETADA. Iniciando Programa...");
  delay(10);

  digitalWrite(LED,HIGH);
  delay(250);
  digitalWrite(LED,LOW);
  delay(250);
  digitalWrite(LED,HIGH);
  delay(250);
  digitalWrite(LED,LOW);

  pinMode(PULSADOR,OUTPUT);
  Serial.println("TS\tErrD\tErrT\tAng\tVm1\tVm2");


  //LANZADOR DEL LOOP
  //pinMode(PULSADOR,INPUT);
  //while(digitalRead(PULSADOR)==1);
  //while(digitalRead(PULSADOR)==0);






  initialTime = millis();
  lastLogTime = initialTime;
  pinMode(PULSADOR,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  //pruebaSensoresIR(sensorValues);
  //delay(200);


  int m1Speed;
  int m2Speed;
  int err;


  err=getErrorPD(sensorValues,kProp, kDer); //[-280,280]

  m1Speed = VEL_BASE + err;
  m2Speed = VEL_BASE - err;

  motor_der(m1Speed);
  motor_izq(m2Speed);
}
