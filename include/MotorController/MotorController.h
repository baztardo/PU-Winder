#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>


class MotorController {
  public:
    enum Direction {
      CW,
      CCW
    };
    void beginRamping(Direction dir, uint8_t targetSpeed, uint8_t step = 5, uint16_t interval = 10);
    void beginRampingTimed(Direction dir, uint8_t targetSpeed, uint16_t timeToTargetMs);


    void update(); // Must be called regularly in loop()


    MotorController(uint8_t rpwm, uint8_t lpwm, uint8_t enar, uint8_t enal);
    
    void begin();  // Initializes pins
    void enableDirection(Direction dir, bool enable);
    void setMotor(Direction dir, uint8_t pwm);
    void stopMotor(Direction dir);
    void stopAll(); // Convenience to stop both directions
    void rampToSpeed(Direction dir, uint8_t targetSpeed, uint8_t step = 5, uint16_t delayTime = 10);
    bool isRamping() const { return _isRamping; }


  private:
  private:
    uint8_t _rpwmPin;
    uint8_t _lpwmPin;
    uint8_t _renPin;
    uint8_t _lenPin;
    bool _isRamping = false;
    Direction _rampDir;
    uint8_t _currentSpeedCW = 0;
    uint8_t _currentSpeedCCW = 0;
    uint8_t _targetSpeed;
    uint8_t _rampStep;
    uint16_t _rampInterval;
    unsigned long _lastRampUpdate = 0;
    
};

#endif
