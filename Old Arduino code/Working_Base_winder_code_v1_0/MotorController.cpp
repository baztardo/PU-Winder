#include "MotorController.h"

MotorController::MotorController(uint8_t rpwm, uint8_t lpwm, uint8_t ren, uint8_t len) 
  : _rpwmPin(rpwm), _lpwmPin(lpwm), _renPin(ren), _lenPin(len) {}

void MotorController::begin() {
  pinMode(_rpwmPin, OUTPUT);
  pinMode(_lpwmPin, OUTPUT);
  pinMode(_renPin, OUTPUT);
  pinMode(_lenPin, OUTPUT);

  stopAll();
}

void MotorController::setMotor(Direction dir, uint8_t pwm) {
  if (dir == CW) {
    digitalWrite(_rpwmPin, HIGH);
    digitalWrite(_lpwmPin, LOW);
  } else { // CCW
    digitalWrite(_rpwmPin, LOW);
    digitalWrite(_lpwmPin, HIGH);
  }

  // Both enables always get the same PWM
  analogWrite(_renPin, pwm);
  analogWrite(_lenPin, pwm);
}

void MotorController::stopMotor(Direction dir) {
  analogWrite(_renPin, 0);
  analogWrite(_lenPin, 0);

  if (dir == CW) {
    _currentSpeedCW = 0;
  } else {
    _currentSpeedCCW = 0;
  }
  _isRamping = false;
}


void MotorController::stopAll() {
  analogWrite(_renPin, 0);
  analogWrite(_lenPin, 0);

  _currentSpeedCW = 0;   // reset ramp tracking
  _currentSpeedCCW = 0;  // reset ramp tracking
  _isRamping = false;
}


void MotorController::beginRamping(Direction dir, uint8_t targetSpeed, uint8_t step, uint16_t interval) {
  _rampDir = dir;
  _targetSpeed = targetSpeed;
  _rampStep = step;
  _rampInterval = interval;
  _lastRampUpdate = millis();
  _isRamping = true;
}

// New: ramping with total time instead of step+interval
void MotorController::beginRampingTimed(Direction dir, uint8_t targetSpeed, uint16_t timeToTargetMs) {
  _rampDir = dir;
  _targetSpeed = targetSpeed;
  _lastRampUpdate = millis();
  _isRamping = true;

  // Figure out current speed for this direction
  uint8_t currentSpeed = (dir == CW) ? _currentSpeedCW : _currentSpeedCCW;
  int distance = abs((int)_targetSpeed - (int)currentSpeed);

  if (distance == 0) {
    _isRamping = false;
    return;
  }

  // Decide step + interval automatically
  // Aim for ~50 updates per ramp (smooth), so step is small
  _rampStep = max(1, distance / 50);  
  _rampInterval = max(10, timeToTargetMs / (distance / _rampStep));
}

void MotorController::update() {
  if (!_isRamping) return;

  unsigned long now = millis();
  if (now - _lastRampUpdate < _rampInterval) return;

  _lastRampUpdate = now;

  uint8_t &currentSpeed = (_rampDir == CW) ? _currentSpeedCW : _currentSpeedCCW;

  if (currentSpeed == _targetSpeed) {
    _isRamping = false;
    return;
  }

  int delta = (_targetSpeed > currentSpeed) ? _rampStep : -_rampStep;
  int nextSpeed = currentSpeed + delta;

  // Clamp to target without overshoot
  if ((delta > 0 && nextSpeed > _targetSpeed) || (delta < 0 && nextSpeed < _targetSpeed)) {
    nextSpeed = _targetSpeed;
  }

  currentSpeed = nextSpeed;
  setMotor(_rampDir, currentSpeed);

  if (currentSpeed == _targetSpeed) {
    _isRamping = false;
  }
}
