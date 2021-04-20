/* * * * * * * * * * * * * * * * * * * * * * *
 * * * * * * * * * * * * * * * * * * * * * * */
#include <Arduino.h>
#include "trajectory.h"
#include "order.h"
#include "blasterctrl.h"
#include "parameters.h"
#include <Servo.h>
bool is_connected = false; ///< True if the connection with the master is available

// --- Instantiate the class ---
/**
 * If you want the acceleration and deceleration to be the same
 * FORMAT: Trajectory(max velocity, acceleration)
 * If the acceleration and deceleration are different
 * FORMAT: Trajectory(max velocity, acceleration, deceleration)
 */
Trajectory xservoTrajectory(60, 40, 34);
Trajectory yservoTrajectory(60, 40, 34);

/**
 * By default the dynamics controller turns off when it is within 0.1 units
 * of the target position. This threshold value can be changed in the declaration
 * FORMAT: Dynamics(max velocity, acceleration, deceleration, threshold)
 */
//Trajectory xservoTrajectory(60, 40, 34, 0.05);

Servo xservo;
Servo yservo;

// --- Define global variables ---
// The controller will be updated at a rate of 100Hz
#define UPDATE_FREQUENCY 100
#define UPDATE_TIME (1000 / UPDATE_FREQUENCY)
unsigned long updateTimer = 0;
int moveNumber = 0;
int xservo_angle;

#define MOTOR_PIN 3
#define TRIGGER_PIN 4

/* * * * * * * * * * * * * * * * * * * * * * *
 * SETUP
 * * * * * * * * * * * * * * * * * * * * * * */
void setup() {

	// Init Serial
  Serial.begin(SERIAL_BAUD);
	// Serial.println("Starting DeepBlaster Serial Controller");

  // Init Motors and Servos
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
	// Attaches the servo on pin 9 to the servo object
  xservo.attach(XSERVO_PIN);
  yservo.attach(YSERVO_PIN);

	// Set servos at initial position
	xservo.write(98);
  yservo.write(90);

	// By default the controller starts at 0, so we need to
	// set the starting angle as well
	xservoTrajectory.reset(98);
  yservoTrajectory.reset(98);

	/**
	 * If we suddenly decide we want to change the maximum velocity to 30°/s,
	 * the acceleration to 15°/s^2 and deceleration to 5.3°/s^2
	 */
	//xservoTrajectory.setMaxVel(30);
	//xservoTrajectory.setAcc(15);
	//xservoTrajectory.setDec(5.3);

	/**
	 * To read what the current velocity and acceleration settings are
	 */
	//float maxVelocity = xservoTrajectory.getMaxVel();
	//float acceleration = xservoTrajectory.getAcc();
	//float deceleration = xservoTrajectory.getDec();

	updateTimer = millis();
}


/* * * * * * * * * * * * * * * * * * * * * * *
 * NEW MOVEMENT COMMANDS DEMO
 * * * * * * * * * * * * * * * * * * * * * * */
void nextMove() {
	switch (moveNumber) {
		case 0:
			// First we move to the 180° position as fast as possible
			xservoTrajectory.setTargetPos(118);
			break;
		case 1:
			// Then move back to 20° as fast as possible
			xservoTrajectory.setTargetPos(78);
			break;

		case 2:
			// Next move to 180°, but over the course of 5 seconds
			xservoTrajectory.setTargetPos(118);
			break;

		case 3:
			// Finally back to 20°, taking 8.5 seconds
			xservoTrajectory.setTargetPos(98);
			break;

		default:
			// If all other moves have completed, stop the program
			// Serial.println("All moves completed");
			while(1) {}
	}

	moveNumber++;
}


/* * * * * * * * * * * * * * * * * * * * * * *
 * LOOP
 * * * * * * * * * * * * * * * * * * * * * * */
void loop() {
    get_messages_from_serial();
  	// Update the servo position at regular intervals
	  if (millis() - updateTimer >= UPDATE_TIME) {
		updateTimer += UPDATE_TIME;

		// Update the controller
		float currentAngle = xservoTrajectory.update();

		// Set the new servo position; the function only takes integer numbers
		xservo.write(round(xservo_angle));

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
		//if (servoTrajectory.ready()) {
		//	nextMove();
    //  delay(500);
    //  fire();
    //  delay(500);
		//}
	}
}

void spinup(){
    digitalWrite(MOTOR_PIN, HIGH);
    // Serial.println("Flywheels Engaged");
    delay(FLYWHEEL_SPINUP_TIME);
}

void spindown(){
    digitalWrite(MOTOR_PIN, LOW);
    // Serial.println("Flywheels Disengaged");
}

void fire() {
    digitalWrite(TRIGGER_PIN, HIGH);
    // Serial.println("Fire!!!");
    delay(80);
    digitalWrite(TRIGGER_PIN, LOW);
}

void stop() {
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(MOTOR_PIN, LOW);
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
          //motor_speed = 0;
          //stop();
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
        case FLYWHEEL:
        {
          // between -100 and 100
          //motor_speed = read_i8();
          if(DEBUG)
          {
            //write_order(MOTOR);
            //write_i8(motor_speed);
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
