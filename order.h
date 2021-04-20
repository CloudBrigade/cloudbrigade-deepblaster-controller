#ifndef ORDER_H
#define ORDER_H

// Define the orders that can be sent and received
enum Order {
  HELLO = 0,
  XSERVO = 1,
  YSERVO = 2,
  FLYWHEEL = 3,
  TRIGGER = 4,
  ALREADY_CONNECTED = 5,
  ERROR = 6,
  RECEIVED = 7,
  STOP = 8,
  MOTOR = 9,
};

typedef enum Order Order;

#endif
