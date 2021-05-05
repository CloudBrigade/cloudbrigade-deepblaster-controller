#ifndef PARAMETERS_H
#define PARAMETERS_H

#define SERIAL_BAUD 115200  // Baudrate
#define FLYWHEEL_PIN 3
#define TRIGGER_PIN 4
#define XSERVO_PIN 6
#define YSERVO_PIN 7
#define X_INITIAL_THETA 90  // Initial angle of the servomotor
#define Y_INITIAL_THETA 90  // Initial angle of the servomotor
#define X_OFFSET 7 // Offset to center the servo
#define Y_OFFSET 0 // Offset to center the servo
#define FLYWHEEL_SPINUP_TIME 1000 // Time in ms for flywheels to achieve full speed
#define TRIGGER_SPEED 80 // Time in ms to run feeder motor, 3fps motor fires one ball per 80ms rotation
// Min and max values for motors
#define X_THETA_MIN 60
#define X_THETA_MAX 120
#define Y_THETA_MIN 60
#define Y_THETA_MAX 120
#define SPEED_MAX 100

// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG false

#endif
