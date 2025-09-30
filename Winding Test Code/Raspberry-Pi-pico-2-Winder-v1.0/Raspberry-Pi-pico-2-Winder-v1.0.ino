/*
  Raspberry Pi Pico 2 Coil Winder v2.0
  
  Using your existing libraries:
  - AccelStepper for stepper control
  - MotorController for spindle motor
  - High-frequency PWM to eliminate motor whining
  
  Pin Mapping for Pico 2:
  Motor: GP2 (RPWM), GP3 (LPWM), GP4 (REN), GP5 (LEN)
  Stepper: GP6 (STEP), GP7 (DIR), GP8 (EN)
  Encoder: GP9 (ENC_A), GP10 (ENC_B), GP11 (ENC_Z)
  Buttons: GP12 (ESTOP), GP13 (RESUME)
  Endstop: GP14
  Sensors: GP26 (Current), GP27 (Voltage), GP28 (Temp)
*/

#include <AccelStepper.h>
#include "MotorController.h"
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ST7735S connections to Pico 1.8 inch 120x160 Pixel Lcd (Waveshare)
#define TFT_CS   17   // CS  -> GP17
#define TFT_RST  16   // RST -> GP16  
#define TFT_DC   15   // DC  -> GP15
#define TFT_DIN  19   // DIN -> GP19 (SPI MOSI)
#define TFT_CLK  18   // CLK -> GP18 (SPI SCLK)
#define TFT_BL   20   // BL  -> GP20 (backlight control)
// VCC -> 3.3V, GND -> GND

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
/* ------------------------
   Pin Configuration for Pico 2
   ------------------------ */
// Motor pins
#define RPWM 2
#define LPWM 3
#define REN  4
#define LEN  5

// Stepper pins
#define STEP_PIN 6
#define DIR_PIN  7
#define EN_PIN   8
#define ENDSTOP_PIN 14

// Encoder pins
#define ENC_A 9       // Hardware counter input
#define ENC_B 10      // Direction sensing
#define ENC_Z 11      // Index pulse interrupt

// Control buttons
#define ESTOP_PIN 12
#define RESUME_PIN 13

// Sensor pins (ADC)
#define CURRENT_SENSE_PIN 26  // ADC0
#define VOLTAGE_SENSE_PIN 27  // ADC1
#define TEMP_SENSE_PIN 28     // ADC2

/* ------------------------
   Configuration Constants
   ------------------------ */
volatile uint32_t encoderPulseCount = 0;    // Raw pulses from ENC_A
volatile uint32_t revolutionCount = 0;       // Actual revolutions
volatile uint32_t lastPulseCount = 0;        // For tracking

const uint16_t ENCODER_PULSES_PER_REV = 360;
const float wireDiameterMM = 0.0635;
const long bobbinWidth = 7;
const long targetTurns = 1000;
const int bobin_WidthMM = 12;
const int startOffsetMM = 1;
const int HOMING_BACKOFF_MM = 20;
const float HOMING_APPROACH_SPEED = 1000.0;

// Motor specs
const int MOTOR_MAX_RPM = 7000;
const float GEAR_RATIO = 3.0;
const int SPINDLE_MAX_RPM = MOTOR_MAX_RPM / GEAR_RATIO;
const int SAFE_MAX_RPM = 2000;

// Mechanics
const float leadPitchMM = 6.0;
const int stepsPerRev = 200;
const int microsteps = 16;
float stepperSpeed = 1000.0;
float stepperMaxSpeed = 2000.0;
float stepperAcceleration = 350;

/* ------------------------
   System Data Structure
   ------------------------ */
struct SystemData {
  float currentRPM = 0.0;
  float targetRPM = 2000.0;
  float motorCurrent = 0.0;
  float supplyVoltage = 12.0;
  float temperature = 25.0;
  long currentTurns = 0;
  long targetTurns = 1000;
  
  bool isRunning = false;
  bool isHomed = false;
  String systemStatus = "Ready";
  
  bool rpmOverspeed = false;
  bool currentOverload = false;
  unsigned long lastSafetyCheck = 0;
  
  unsigned long windingStartTime = 0;
  unsigned long sessionStartTime = 0;
} systemData;

/* ------------------------
   Hardware Counter Variables
   ------------------------ */
volatile uint32_t encoderCount = 0;
volatile bool newRevolution = false;
volatile bool stepperDir = true;
bool spindleRunning = false;
int initial_homing = 0;

long stepperMinSteps, stepperMaxSteps;
float stepsPerMM;
long stepperPosition = 0;
long stepperTargetPosition = 0;

/* ------------------------
   1.8" 128x160 LCD TFT Display
   ------------------------ */
void setupDisplay() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // Turn on backlight
  
  tft.initR(INITR_BLACKTAB);  // For 128x160 ST7735S
  tft.setRotation(0);         // Portrait mode
  tft.fillScreen(ST77XX_BLACK);
  
  // Startup screen
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("PICO COIL WINDER");
  tft.println("v2.0 READY");
  
  Serial.println("ST7735S display initialized");
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  static float lastRPM = -1;
  static long lastTurns = -1;
  static String lastStatus = "";
  static float lastVoltage = -1;
  static float lastCurrent = -1;
  static float lastTemp = -1;
  static bool firstRun = true;
  
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    
    // First run - draw everything with proper margins
    if (firstRun) {
      tft.fillScreen(ST77XX_BLACK);
      firstRun = false;
      
      // Header with margin
      tft.setTextColor(ST77XX_CYAN);
      tft.setTextSize(1);
      tft.setCursor(5, 5);
      tft.println("COIL WINDER v2.0");
      
      // Static labels that don't change
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(2);
      tft.setCursor(5, 45);
      tft.println("RPM:");
      
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setCursor(85, 55);
      tft.println("Target:");
    }
    
    // Update status if changed
    if (systemData.systemStatus != lastStatus) {
      lastStatus = systemData.systemStatus;
      tft.fillRect(5, 20, 118, 15, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
      tft.setTextSize(1);
      tft.setCursor(5, 25);
      tft.println("State: " + systemData.systemStatus);
    }
    
    // Update RPM if changed
    if (abs(systemData.currentRPM - lastRPM) > 1.0) {
      lastRPM = systemData.currentRPM;
      tft.fillRect(5, 60, 75, 15, ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(2);
      tft.setCursor(5, 60);
      tft.println(String(systemData.currentRPM, 1));
    }
    
    // Update target RPM (changes less often)
    tft.fillRect(85, 65, 38, 10, ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(1);
    tft.setCursor(85, 65);
    tft.println(String(systemData.targetRPM, 0));
    
    // Update turns if changed
    if (systemData.currentTurns != lastTurns) {
      lastTurns = systemData.currentTurns;
      tft.fillRect(5, 85, 118, 20, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
      tft.setCursor(5, 85);
      tft.println("Turns: " + String(systemData.currentTurns));
      tft.setCursor(5, 95);
      tft.println("Target: " + String(systemData.targetTurns));
      
      // Update progress bar
      tft.fillRect(5, 110, 118, 12, ST77XX_BLACK);
      int progress = (float)systemData.currentTurns / systemData.targetTurns * 113;
      tft.drawRect(5, 110, 115, 10, ST77XX_WHITE);
      if (progress > 0) {
        tft.fillRect(6, 111, progress, 8, ST77XX_BLUE);
      }
    }
    
    // Update system info if changed
    if (abs(systemData.supplyVoltage - lastVoltage) > 0.1 || 
        abs(systemData.motorCurrent - lastCurrent) > 0.1 || 
        abs(systemData.temperature - lastTemp) > 1.0) {
      
      lastVoltage = systemData.supplyVoltage;
      lastCurrent = systemData.motorCurrent;
      lastTemp = systemData.temperature;
      
      tft.fillRect(5, 135, 118, 20, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
      tft.setTextSize(1);
      tft.setCursor(5, 135);
      tft.println("V:" + String(systemData.supplyVoltage, 1) + 
                  " I:" + String(systemData.motorCurrent, 1) + "A");
      tft.setCursor(5, 145);
      tft.println("T:" + String(systemData.temperature, 1) + "C");
    }
  }
}

void showSplashScreen() {
  tft.fillScreen(ST77XX_BLACK);
  
  // Title
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("PRECISION");
  tft.setCursor(15, 30);
  tft.println("COIL");
  tft.setCursor(10, 50);
  tft.println("WINDER");
  
  // Version
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(85, 55);
  tft.println("v2.0");
  
  // Decorative line
  tft.drawLine(10, 75, 118, 75, ST77XX_WHITE);
  
  // Hardware info
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(15, 85);
  tft.println("Raspberry Pi");
  tft.setCursor(25, 95);
  tft.println("Pico 2");
  
  // Features
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 115);
  tft.println("25kHz PWM Drive");
  tft.setCursor(5, 125);
  tft.println("Hardware Encoding");
  tft.setCursor(5, 135);
  tft.println("Auto Wire Guide");
  
  // Status
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setCursor(30, 150);
  tft.println("INITIALIZING...");
  
  delay(3000); // Show splash for 3 seconds
}

/* ------------------------
   State Machine
   ------------------------ */
enum State { IDLE, HOMING, OFFSET, WINDING, DONE, ESTOP };
State state = IDLE;

// Objects - using your proven libraries on Pico
MotorController spindle(RPWM, LPWM, REN, LEN);                   // Spindle (bobbin)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);   // Wireguide (carriage)

// =================================================================
void setupPicoPWM() {
  Serial.println("Setting up Pico PWM...");
  
  // CRITICAL: Set PWM frequency BEFORE using analogWrite
  analogWriteFreq(25000);  // 25kHz
  analogWriteRange(255);   // 0-255 range like Arduino
  
  // Initialize all motor pins
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  
  // Test immediate PWM output
  digitalWrite(RPWM, HIGH);
  digitalWrite(LPWM, LOW);
  analogWrite(REN, 100);
  analogWrite(LEN, 100);
  
}
/* ------------------------
   Pico 2 Stepper Control
   ------------------------ */
void setupStepperPWM() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  
  // Enable stepper driver
  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  
}

void setStepperSpeed(float stepsPerSecond) {
  static unsigned long lastStepTime = 0;
  static bool stepState = false;
  
  if (abs(stepsPerSecond) < 1.0) {
    return; // Too slow, don't step
  }
  
  // Set direction
  digitalWrite(DIR_PIN, stepsPerSecond > 0 ? HIGH : LOW);
  
  // Calculate step interval
  unsigned long stepInterval = 1000000 / abs(stepsPerSecond); // microseconds
  
  // Generate steps
  if (micros() - lastStepTime >= stepInterval) {
    stepState = !stepState;
    digitalWrite(STEP_PIN, stepState);
    
    if (stepState) { // Count steps on rising edge
      if (stepsPerSecond > 0) {
        stepperPosition++;
      } else {
        stepperPosition--;
      }
    }
    
    lastStepTime = micros();
  }
}

void moveStepperTo(long targetSteps) {
  stepperTargetPosition = targetSteps;
}

void updateStepperMovement() {
  long stepsToGo = stepperTargetPosition - stepperPosition;
  
  if (abs(stepsToGo) < 2) {
    return; // Close enough, stop
  }
  
  // Calculate speed with acceleration/deceleration
  float speed = (stepsToGo > 0) ? stepperSpeed : -stepperSpeed;
  
  // Simple acceleration
  if (abs(stepsToGo) < 100) {
    speed *= 0.5; // Slow down near target
  }
  
  setStepperSpeed(speed);
}

/* ------------------------
   Motor Control using MotorController Library
   ------------------------ */
void beginSpindle(int rpm) {
  Serial.println("Starting Pico spindle with MotorController library");
  Serial.println("Target RPM: " + String(rpm));
  
  rpm = constrain(rpm, 50, SAFE_MAX_RPM);
  
  // Use the mapping that worked well on Arduino
  //uint8_t pwmValue = map(rpm, 0, 2000, 0, 255);
  uint8_t pwmValue = map(rpm, 0, 1000, 100, 255); // Start at 100 instead of 0
  Serial.println("PWM value: " + String(pwmValue) + "/255");
  
  // Use your library's ramping function
  spindle.beginRampingTimed(MotorController::CW, pwmValue, 3000);
  
  spindleRunning = true;
  systemData.isRunning = true;
  systemData.targetRPM = rpm;
  systemData.systemStatus = "Winding";
  
  Serial.println("Pico spindle ramp started (CW, 3sec ramp time)");
}

void stopAll() {
  Serial.println("Stopping all motors using MotorController");
  spindle.stopAll();  // Use your library's stop function
  stepper.setSpeed(0);
  spindleRunning = false;
  systemData.isRunning = false;
  systemData.rpmOverspeed = false;
  systemData.currentOverload = false;
  Serial.println("All motors stopped");
}

/* ------------------------
   Hardware Encoder Functions
   ------------------------ */
void encoderISR() {
  //encoderCount++;
  encoderPulseCount++;
}

void zIndexISR() {
  revolutionCount++;
  newRevolution = true;
  
  if (revolutionCount % 10 == 0) {   // 100 - 10
    Serial.println("Pico revolutions: " + String(revolutionCount));
  }
}
// Calculate revolutions in main loop (not in ISR)
void updateRevolutionCount() {
  // How many complete revolutions have we done?
  revolutionCount = encoderPulseCount / ENCODER_PULSES_PER_REV;
  
  // Optional: track remainder for sub-revolution accuracy
  uint32_t remainder = encoderPulseCount % ENCODER_PULSES_PER_REV;
}
void setupEncoderInterrupts() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), zIndexISR, FALLING);
  
}

uint32_t getTotalEncoderCount() {
  return encoderCount;
}

void resetEncoderCount() {
  noInterrupts();
  encoderCount = 0;
  revolutionCount = 0;
  interrupts();
  // Pico encoder count reset
}

/* ------------------------
   RPM Calculation (Enhanced for Pico)
   ------------------------ */
float computeRPM() {
  static uint32_t lastPulseCount = 0;
  static uint32_t lastTime = 0;
  static float rpmHistory[5] = {0};
  static int historyIndex = 0;
  
  uint32_t currentTime = millis();
  uint32_t currentPulses = encoderPulseCount;
  
  if (currentTime - lastTime >= 250) {  // Calculate every 250ms
    uint32_t pulseDelta = currentPulses - lastPulseCount;
    uint32_t timeDelta = currentTime - lastTime;
    
    if (timeDelta > 0 && pulseDelta > 0) {
      // Calculate RPM from pulses
      // pulses/ms * 1000ms/s * 60s/min / pulses_per_rev = RPM
      float rawRPM = (float)pulseDelta * 60000.0 / ((float)timeDelta * ENCODER_PULSES_PER_REV);
      
      // Smoothing filter
      rpmHistory[historyIndex] = rawRPM;
      historyIndex = (historyIndex + 1) % 5;
      
      float smoothedRPM = 0;
      for (int i = 0; i < 5; i++) {
        smoothedRPM += rpmHistory[i];
      }
      smoothedRPM /= 5.0;
      
      lastPulseCount = currentPulses;
      lastTime = currentTime;
      
      return smoothedRPM;
    }
  }
  
  return systemData.currentRPM; // Return last known value
}

/* ------------------------
   Sensor Reading (Pico 2 12-bit ADC)
   ------------------------ */
void updateSensorReadings() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();
    
    // Pico has 12-bit ADC (0-4095) vs Arduino 10-bit (0-1023)
    // Pico reference voltage is 3.3V vs Arduino 5V
    
    int voltageADC = analogRead(VOLTAGE_SENSE_PIN);
    systemData.supplyVoltage = (voltageADC / 4095.0) * 3.3 * 7.45; // Adjust multiplier for 3.3V
    
    int currentADC = analogRead(CURRENT_SENSE_PIN);
    float currentVoltage = (currentADC / 4095.0) * 3.3;
    systemData.motorCurrent = abs((currentVoltage - 1.65) / 0.066); // Adjust for 3.3V reference
    
    // NTC temperature sensor
    int tempADC = analogRead(TEMP_SENSE_PIN);
    float tempVoltage = (tempADC / 4095.0) * 3.3;
    
    if (tempADC < 50) {
      systemData.temperature = -999.0;
    } else if (tempADC > 4000) {
      systemData.temperature = 999.0;
    } else {
      // NTC calculation for 3.3V reference
      float ntcResistance = 10000.0 * tempVoltage / (3.3 - tempVoltage);
      float tempK = 1.0 / (log(ntcResistance / 10000.0) / 3950.0 + 1.0 / 298.15);
      systemData.temperature = tempK - 273.15;
    }
  }
}
/* ------------------------
   System Status and Safety
   ------------------------ */
void printSystemStatus() {
  static unsigned long lastReport = 0;
  
  if (millis() - lastReport >= 2000) {
    lastReport = millis();
    
    if (systemData.isRunning && systemData.targetRPM > 0) {
      float rpmError = abs(systemData.currentRPM - systemData.targetRPM);
      float rpmErrorPercent = (rpmError / systemData.targetRPM) * 100.0;
    }
    
    if (systemData.targetTurns > 0) {
      float progress = (float)systemData.currentTurns / systemData.targetTurns * 100.0;
      
    }
    
    float positionMM = stepperPosition / stepsPerMM;

  }
}

void syncCarriage(float rpm) {
  float feedMMs = (rpm / 60.0) * wireDiameterMM;
  float stepsPerSec = feedMMs * stepsPerMM;
  
  if (!stepperDir) stepsPerSec = -stepsPerSec;
  
  stepper.setSpeed(stepsPerSec);
  
  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 3000) {
    lastDebug = millis();
    /*
      RPM=" + (rpm, 1)
      FeedMM/s=" + (feedMMs, 3)  
      Steps/s=" + (stepsPerSec, 1)
      (stepperDir ? "Right" : "Left")
    */
  }
}

/* ------------------------
   Physical Button Control
   ------------------------ */
void checkPhysicalButtons() {
  static unsigned long lastResumePress = 0;
  static int resumePressCount = 0;
  
  if (digitalRead(RESUME_PIN) == LOW) {
    if (millis() - lastResumePress > 300) {
      lastResumePress = millis();
      resumePressCount++;
      
      if (!systemData.isHomed && state == IDLE) {
        state = HOMING;
        systemData.systemStatus = "Homing";
        resumePressCount = 0;
      }
      else if (systemData.isHomed && !systemData.isRunning && state == IDLE) {
        state = OFFSET;
        systemData.systemStatus = "Starting";
        resumePressCount = 0;
      }
      else if (systemData.isRunning && state == WINDING) {
        if (resumePressCount == 1) {
          stopAll();
          systemData.systemStatus = "Paused";
        } else if (resumePressCount >= 2) {
          stopAll();
          state = IDLE;
          systemData.systemStatus = "Stopped";
          resumePressCount = 0;
        }
      }
    }
  }
  
  if (resumePressCount > 0 && millis() - lastResumePress > 3000) {
    resumePressCount = 0;
  }
}
void showEncoderDebugInfo() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 1000) {  // Update every second
    lastUpdate = millis();
    
    // Calculate current values
    updateRevolutionCount();
    
    // Show at top of screen temporarily
    tft.fillRect(0, 0, 128, 30, ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(1);
    tft.setCursor(0, 0);
    tft.println("Pulses: " + String(encoderPulseCount));
    tft.println("Revs: " + String(revolutionCount));
    tft.println("Target: " + String(systemData.targetTurns));
  }
}
/* ------------------------
   Setup
   ------------------------ */
void setup() {
  Serial.begin(115200);
  setupDisplay();
  delay(2000); // Give time for serial to initialize

   // Initialize display first
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  
  // Show splash screen
  showSplashScreen();
  
  // Setup PWM for motor control (eliminates whining)
  setupPicoPWM();
  
  // Initialize motor controller library
  spindle.begin();

  // Setup stepper with AccelStepper library
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable stepper driver
  delay(5);

  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  
  // Setup encoder interrupts
  setupEncoderInterrupts();
  
  // Setup other pins
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(RESUME_PIN, INPUT_PULLUP);
  
  // Calculate mechanics
  stepsPerMM = (float)stepsPerRev * (float)microsteps / leadPitchMM;
  stepperMaxSteps = (long)round(bobin_WidthMM * stepsPerMM);
    
  systemData.sessionStartTime = millis();
  systemData.targetTurns = targetTurns;

  // Clear splash and show ready screen
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  tft.setCursor(20, 70);
  tft.println("SYSTEM READY");
  delay(1000);

}

/* ------------------------
   Main Loop
   ------------------------ */
void loop() {
  // Display update
  updateDisplay();
  showEncoderDebugInfo();  // Shows real-time encoder data
  // CRITICAL: Update MotorController ramping - must be called every loop!
  spindle.update();
  
  // Update hardware counters
  systemData.currentRPM = computeRPM();
  systemData.currentTurns = revolutionCount;
  
  // Check buttons
  checkPhysicalButtons();
  
  // Update sensors
  //updateSensorReadings();

  // Print status
  printSystemStatus();

  // Emergency stop
  if (digitalRead(ESTOP_PIN) == LOW) {
    stopAll();
    state = ESTOP;
    systemData.systemStatus = "E-STOP";
    
    while (digitalRead(ESTOP_PIN) == LOW) {
      delay(100);
    }
    state = IDLE;
    systemData.systemStatus = "Ready";
  }
  
  // State machine
  switch (state) {
    case IDLE:
      break;
      
    case HOMING:
      {
        static int homingStep = 1; // 1=approach, 2=backup, 3=complete
        static long switchPosition = 0;
        static bool movingToTarget = false;

        switch (homingStep) {
          case 1: // Approaching switch
            stepper.setSpeed(HOMING_APPROACH_SPEED);
            stepper.runSpeed();

            if (digitalRead(ENDSTOP_PIN) == LOW) {
              stepper.stop();
              switchPosition = stepper.currentPosition();
              systemData.systemStatus = "Backing Off";
              homingStep = 2;
              movingToTarget = false;
            }
            break;

          case 2: // Backing off from switch
            if (!movingToTarget) {
              long backoffSteps = (long)round(HOMING_BACKOFF_MM * stepsPerMM);
              long targetPosition = switchPosition - backoffSteps;
              
              stepper.moveTo(targetPosition);
              movingToTarget = true;
            }

            if (stepper.distanceToGo() != 0) {
              stepper.run();
            } else {
             homingStep = 3;
            }
            break;

          case 3: // Set home and complete
            stepper.setCurrentPosition(0);
            stepperMinSteps = 0;
            stepperMaxSteps = (long)round(bobin_WidthMM * stepsPerMM);

            initial_homing = 1;
            systemData.isHomed = true;
            systemData.systemStatus = "Homed - Ready";
            
            homingStep = 1;
            movingToTarget = false;
            state = IDLE;
            break;
        }
      }
      break;
      
case OFFSET:
  {
    static bool moving = false;
    long targetSteps = startOffsetMM * stepsPerMM;

    if (!moving) {
      stepper.moveTo(targetSteps);
      moving = true;
      systemData.systemStatus = "Moving to Start";
    }

    if (stepper.distanceToGo() != 0) {
      stepper.run();
    } else {
      moving = false;
      
      // ✅ RESET ENCODERS PROPERLY
      noInterrupts();
      encoderPulseCount = 0;
      revolutionCount = 0;
      interrupts();
      
      systemData.currentTurns = 0;
      systemData.windingStartTime = millis();
      
      beginSpindle(systemData.targetRPM);
      state = WINDING;
    }
  }
  break;

      
case WINDING:
  {
    // SAFETY: Stop if hitting endstop
    if (digitalRead(ENDSTOP_PIN) == LOW) {
      stopAll();
      state = ESTOP;
      break;
    }
    
    // ✅ CRITICAL: Update revolution count from pulses
    updateRevolutionCount();
    systemData.currentTurns = revolutionCount;
    
    // Update RPM
    systemData.currentRPM = computeRPM();
    
    if (spindleRunning) {
      syncCarriage(systemData.currentRPM);
    }
    
    // Check limits and reverse direction
    long pos = stepper.currentPosition();
    
    if (pos <= stepperMinSteps && !stepperDir) {
      stepperDir = true;
      syncCarriage(systemData.currentRPM);
    } else if (pos >= stepperMaxSteps && stepperDir) {
      stepperDir = false;
      syncCarriage(systemData.currentRPM);
    }
    
    // Check if target reached
    if (revolutionCount >= systemData.targetTurns) {
      systemData.systemStatus = "Complete";
      state = DONE;
      
      // Show completion on LCD
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_GREEN);
      tft.setTextSize(2);
      tft.setCursor(10, 50);
      tft.println("COMPLETE!");
      tft.setTextSize(1);
      tft.setCursor(10, 80);
      tft.println("Total: " + String(systemData.currentTurns) + " turns");
      tft.setCursor(10, 95);
      
      unsigned long windingTime = (millis() - systemData.windingStartTime) / 1000;
      tft.println("Time: " + String(windingTime) + "s");
    }
  }
  break;
      
  case DONE:
    {
      stopAll();
      systemData.systemStatus = "DONE - " + String(systemData.currentTurns) + " turns";
      
      unsigned long windingTime = (millis() - systemData.windingStartTime) / 1000;
      /*
      PICO WINDING COMPLETE!
        Total turns:
        Winding time:
        Average RPM:
        Total encoder pulses:
      */
      state = IDLE;
      break;
    } 
    case ESTOP:
      break;
  }
  
  // Always run stepper - CRITICAL for AccelStepper
  stepper.run();
  
  delay(1); // Small delay for stability
}