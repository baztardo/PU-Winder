#include <MotorController.h>

// Define the motor control pins
#define RPWM 10
#define LPWM 11
#define ENAR 8
#define ENAL 7

MotorController motor(RPWM, LPWM, ENAR, ENAL); // RPWM, LPWM, ENAR, ENAL

void setup() {
  motor.begin();
  motor.enableDirection(MotorController::CW, true);
  motor.enableDirection(MotorController::CCW, true);

  // Start ramping to 200 (non-blocking)
  motor.beginRamping(MotorController::CW, 200, 5, 20);
}

void loop() {
  motor.update(); // MUST be called to update ramping

  // Do other stuff here
  // e.g., read sensors, control LEDs, listen to serial, etc.
}
