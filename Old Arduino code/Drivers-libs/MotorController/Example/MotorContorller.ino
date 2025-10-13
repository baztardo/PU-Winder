#include "MotorController.h"

// Pin definitions (adjust to your wiring)
#define RPWM_PIN 5   // Direction pin 1
#define LPWM_PIN 6   // Direction pin 2
#define REN_PIN 9    // PWM enable pin
#define LEN_PIN 10   // PWM enable pin

// Create motor controller instance
MotorController motor(RPWM_PIN, LPWM_PIN, REN_PIN, LEN_PIN);

void setup() {
  Serial.begin(9600);
  motor.begin();
  Serial.println("MotorController test starting...");
}

void loop() {
  // Run CW at half speed
  Serial.println("CW half speed");
  motor.setMotor(MotorController::CW, 128);
  delay(2000);

  // Stop
  Serial.println("Stop");
  motor.stopAll();
  delay(1000);

  // Run CCW at full speed
  Serial.println("CCW full speed");
  motor.setMotor(MotorController::CCW, 45);
  delay(2000);

  // Stop
  Serial.println("Stop");
  motor.stopAll();
  delay(1000);

  // Ramping example (CW, 0 → 200 speed)
  Serial.println("Ramping CCW up");
  //motor.beginRamping(MotorController::CCW, 200, 2, 100);  // 2 step ever 100ms or 
  motor.beginRampingTimed(MotorController::CCW, 200, 5000); // ramp to 200 in 5 seconds
  while (true) {
    motor.update();  // must be called regularly
    delay(10);

    // When ramping is done, break
    if (!motor.isRamping()) break;
  }

  // Hold at ramped speed
  delay(2000);

  // Stop
  Serial.println("Stop after ramp");
  motor.stopAll();
  delay(3000);
}
