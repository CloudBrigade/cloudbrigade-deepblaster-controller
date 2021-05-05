/* * * * * * * * * * * * * * * * * * * * * * *
 *   Cloud Brigade - Deep Blaster
 *   Code by: Chris Miller
 *   Website: https://www.cloudbrigade.com
 *   Version: 1.0
 *   Date:    Apr 21, 2021
 *   Copyright (C) 2020, Apache 2.0  License
 *
 *   Code included from these excellent projects
 *   Servo Trajectory by Simon Bluett
 *   https://github.com/chillibasket/arduino-classes/tree/master/servo-trajectory
 *   Robust Serial by Antonin Raffin
 *   https://github.com/araffin/arduino-robust-serial
 * * * * * * * * * * * * * * * * * * * * * * */
#include <Arduino.h>
#include "trajectory.h"
#include "order.h"
#include "blasterctrl.h"
#include "parameters.h"
#include <Servo.h>
bool is_connected = false; ///< True if the connection with the DeepRacer is available
/**
 * If you want the acceleration and deceleration to be the same
 * FORMAT: Trajectory(max velocity, acceleration)
 * If the acceleration and deceleration are different
 * FORMAT: Trajectory(max velocity, acceleration, deceleration)
 *
 * By default the dynamics controller turns off when it is within 0.1 units
 * of the target position. This threshold value can be changed in the declaration
 * FORMAT: Dynamics(max velocity, acceleration, deceleration, threshold)
 */
// --- Instantiate the class ---
Trajectory xservoTrajectory(60, 40, 34, 0.1);
Trajectory yservoTrajectory(60, 40, 34, 0.1);

Servo xservo;
Servo yservo;

// --- Define global variables ---
// The controller will be updated at a rate of 100Hz
#define UPDATE_FREQUENCY 100
#define UPDATE_TIME (1000 / UPDATE_FREQUENCY)
unsigned long updateTimer = 0;
int xservo_angle = (X_INITIAL_THETA + X_OFFSET);
int yservo_angle = (Y_INITIAL_THETA + Y_OFFSET);
int flywheel_engage = 0;
int trigger_engage = 0;
int fspin;

/* * * * * * * * * * * * * * * * * * * * * * *
 * SETUP
 * * * * * * * * * * * * * * * * * * * * * * */
void setup() {

	// Init Serial
  Serial.begin(SERIAL_BAUD);
	// Serial.println("Starting DeepBlaster Serial Controller");

  // Init Motors and Servos
  pinMode(FLYWHEEL_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
	// Attaches the servo pin to the servo object
  xservo.attach(XSERVO_PIN);
  yservo.attach(YSERVO_PIN);

	// Set servos at initial position
	xservo.write(X_INITIAL_THETA + X_OFFSET);
  yservo.write(Y_INITIAL_THETA + X_OFFSET);

	// By default the controller starts at 0, so we need to
	// set the starting angle as well
	xservoTrajectory.reset(X_INITIAL_THETA + X_OFFSET);
  yservoTrajectory.reset(Y_INITIAL_THETA + Y_OFFSET);

  /**
   * FORMAT: Trajectory(float maxVelocity, float acceleration, float deceleration, float threshold)
   * @param Maximum Velocity (units/second) - default = 100
   * @param Acceleration (units/second^2) - default = 50
   * @param Deceleration (units/second^2) - default = same as acceleration
   * @param Threshold (units) - default = 0.1
  // For example:
  Trajectory servoTrajectory(20, 15, 12.5, 0.01);

  // If the default threshold of 0.1 doesn't need to be changed:
  Trajectory servoTrajectory(20, 15, 12.5);

  // If you want the acceleration and deceleration to be the same:
  Trajectory servoTrajectory(20, 15);

  * If we suddenly decide we want to change the maximum velocity to 30°/s,
  * the acceleration to 15°/s^2 and deceleration to 5.3°/s^2
	//xservoTrajectory.setMaxVel(30);
	//xservoTrajectory.setAcc(15);
	//xservoTrajectory.setDec(5.3);
 
	 * To read what the current velocity and acceleration settings are
	//float maxVelocity = xservoTrajectory.getMaxVel();
	//float acceleration = xservoTrajectory.getAcc();
	//float deceleration = xservoTrajectory.getDec();
*/
}

/* * * * * * * * * * * * * * * * * * * * * * *
 * LOOP
 * * * * * * * * * * * * * * * * * * * * * * */
void loop() {
  // Update the servo position at regular intervals
	if (millis() - updateTimer >= UPDATE_TIME) {
		updateTimer += UPDATE_TIME;

		// Update the controller
    float xcurrentAngle = xservoTrajectory.update();
    float ycurrentAngle = yservoTrajectory.update();

		// Set the new servo position; the function only takes integer numbers
    xservo.write(round(xcurrentAngle));
    yservo.write(round(ycurrentAngle));

		/**
		 * For more precise servo control, you could use writeMicroseconds.
		 * The min and max PWM pulse widths which correspond to the 0° and 180°
		 * positions needs to be inserted for MIN_PWM and MAX_PWM.
		 */
		//xservo.writeMicroseconds(map(currentAngle, 0, 180, MIN_PWM, MAX_PWM));

		// Output the target position, along with the current position and velocity
		// Serial.print("Target: ");
		// Serial.print(xservoTrajectory.getTarget());
		// Serial.print(", Angle: ");
		// Serial.print(xservoTrajectory.getPos());
		// Serial.print(", Velocity: ");
		// Serial.println(xservoTrajectory.getVel());

    // Preprogrammed movemnets to demo/test the blaster controller
		// Only once the servo has reached the desired position, complete the next move
		if (xservoTrajectory.ready()) {
      xservoTrajectory.setTargetPos(xservo_angle + X_OFFSET);
		}
    if (yservoTrajectory.ready()) {
      yservoTrajectory.setTargetPos(yservo_angle + Y_OFFSET);
    }
    if (flywheel_engage == 1){
      spinup();
      fspin = 1;
    }
    if (trigger_engage == 1){
      fire();
    }
//    if ((fspin = 1) && (flywheel_engage = 0)){
//      spindown();
//    }
  }
    get_messages_from_serial();
}

void spinup(){
    digitalWrite(FLYWHEEL_PIN, HIGH);
    //Serial.println("Flywheels Engaged");
    delay(FLYWHEEL_SPINUP_TIME);
}

void spindown(){
    digitalWrite(FLYWHEEL_PIN, LOW);
    //Serial.println("Flywheels Disengaged");
}

void fire() {
    digitalWrite(TRIGGER_PIN, HIGH);
    //Serial.println("Fire!!!");
    delay(80);
    digitalWrite(TRIGGER_PIN, LOW);
    spindown();
}

void stop() {
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(FLYWHEEL_PIN, LOW);
}

int convert_to_pwm(float motor_speed) {
  // TODO: compensate the non-linear dependency speed = f(PWM_Value)
  return (int) round(abs(motor_speed)*(255./100.));
}

void get_messages_from_serial() {
  if(Serial.available() > 0)
  {
    // The first byte received is the instruction
    Order order_received = read_order();

    if(order_received == HELLO)
    {
      // If the cards haven't say hello, check the connection
      if(!is_connected)
      {
        is_connected = true;
        write_order(HELLO);
      }
      else
      {
        // If we are already connected do not send "hello" to avoid infinite loop
        write_order(ALREADY_CONNECTED);
      }
    }
    else if(order_received == ALREADY_CONNECTED)
    {
      is_connected = true;
    }
    else
    {
      switch(order_received)
      {
        case STOP:
        {
          trigger_engage = 0;
          flywheel_engage = 0;
          xservo_angle = (X_INITIAL_THETA + X_OFFSET);
          yservo_angle = (Y_INITIAL_THETA + Y_OFFSET);
          stop();
          if(DEBUG)
          {
            //write_order(STOP);
          }
          break;
        }
        case XSERVO:
        {
          xservo_angle = read_i16();
          if(DEBUG)
          {
            write_order(XSERVO);
            //write_i16(servo_angle);
          }
          break;
        }
        case YSERVO:
        {
          yservo_angle = read_i16();
          if(DEBUG)
          {
            write_order(YSERVO);
            //write_i16(servo_angle);
          }
          break;
        }
        case FLYWHEEL:
        {
          // 0 or 1
          flywheel_engage = read_i8();
          if(DEBUG)
          {
            write_order(FLYWHEEL);
            write_i8(flywheel_engage);
          }
          break;
        }
        case TRIGGER:
        {
          // 0 or 1
          trigger_engage = read_i8();
          if(DEBUG)
          {
            write_order(TRIGGER);
            write_i8(trigger_engage);
          }
          break;
        }
        // Unknown order
        default:
          write_order(ERROR);
          //write_i16(404);
          return;
      }
    }
    write_order(RECEIVED); // Confirm the reception
  }
}


Order read_order() {
  return (Order) Serial.read();
}

void wait_for_bytes(int num_bytes, unsigned long timeout) {
  unsigned long startTime = millis();
  //Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n) {
  size_t i = 0;
  int c;
  while (i < n)
  {
    c = Serial.read();
    if (c < 0) break;
    *buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
    i++;
  }
}

int8_t read_i8() {
  wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

int16_t read_i16() {
  int8_t buffer[2];
  wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
  read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32() {
  int8_t buffer[4];
  wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
  read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

void write_order(enum Order myOrder) {
  uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

void write_i8(int8_t num) {
  Serial.write(num);
}

void write_i16(int16_t num) {
  int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

void write_i32(int32_t num) {
  int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
