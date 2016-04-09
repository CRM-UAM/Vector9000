/*
  Vector9000.h - Library for control Vector9000 robot
  Created by Victor Uceda, Feb 2016.
  Released into the public domain.
*/
#define LIBCALL_ENABLEINTERRUPT

#ifndef Vector9000_h
#define Vector9000_h

#include "Arduino.h"
#include <stdint.h>
//#include "EnableInterrupt.h"
#define EI_ARDUINO_INTERRUPTED_PIN



class Vector9000
{
  public:
    Vector9000( double KP, double KD, double KI );
    void config();
    void ledOn( void );
    void ledOff( void );

    /**
     * Change robot speed
     * @param lSpeed left wheel speed [-255, 255]
     * @param rSpeed right wheel speed [-255, 255]
     */
    void setSpeed( int lSpeed, int rSpeed );
    /**
     * Change right wheel speed
     * @param s speed value [-255, 255]
     */
    void setRSpeed( int s );
    /**
     * Change left wheel speed
     * @param s speed value [-255, 255]
     */
    void setLSpeed( int s );

    /**
     * calibrate IR sensors
     * @param time duration in seconds
     */
    void calibrateIR( int time, bool printflag );

    /**
     * read IR-array values
     * @param  values pointer to hold de values readed
     */
    void getIRRaw(unsigned int *values);

    /**
     * read line with IR sensors
     * @return  line position [0, 7000]
     */
    unsigned int readLine( void );

    /**
     * Change PID values
     * @param kp     Proportional constant value
     * @param kd     Derivative constant value
     * @param ki     Integral constant value
     */
    void setPID( double KP, double KD, double KI );
    void setP( double KP);
    void setI( double KI);
    void setD( double KD);
    /**
     * Compute error line with PID controller
     * @return  error line value [-255, 255]
     */
    double getErrorLine( void );

    static const uint8_t M_IZQ_PWM_PIN, M_IZQ_DIR1_PIN, M_IZQ_DIR2_PIN, M_DER_DIR1_PIN, M_DER_DIR2_PIN, M_DER_PWM_PIN, LED, NUM_IR_SENSORS, ENC_IZQ_PIN, ENC_DER_PIN, PULSOS_POR_REVOLUCION;;
    static const int TIMEOUT;
    static const uint8_t IR1, IR2, IR3, IR4, IR5, IR6, IR7, IR8;

    double _kp;
    double _kd;
    double _ki;
 
  private:
    double _DerivativeErrorTerm;
    unsigned long _lastTimeExec;
    int _lastError;

};


inline void Vector9000::setPID( double KP, double KD, double KI ){
  _kp = KP;
  _kd = KD;
  _ki = KI;
}
inline void Vector9000::setP( double KP){
  _kp = KP;
}
inline void Vector9000::setI( double KI){
  _ki = KI;
};
inline void Vector9000::setD( double KD){
  _kd = KD;
}

inline void Vector9000::setSpeed( int lSpeed, int rSpeed ){
    setRSpeed( rSpeed );
    setLSpeed( lSpeed);
 }

inline void Vector9000::ledOn( void ){
    digitalWrite(Vector9000::LED, HIGH);
}

inline void Vector9000::ledOff( void ){
    digitalWrite(Vector9000::LED, LOW);
}

#endif
