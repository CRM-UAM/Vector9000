#include <Arduino.h>
#include "Vector9000.h"

#define VEL_BASE 50
Vector9000 robot = Vector9000(2200, 0.06, 0);

void setup(){
    Serial.begin(19200);
    robot.calibrateIR( 5 );
}

void loop() {
    Serial.println(robot.readLine());
    int err = robot.getErrorLine();
    robot.setSpeed( VEL_BASE + err, VEL_BASE - err );
}
