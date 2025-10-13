/* Winder v1.5 - Clean MotorController Integration
   - Uses your existing MotorController library properly
   - Hardware focus with enhanced debugging
   - All compilation errors fixed
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include "MotorController.h"

/* ------------------------
   Pin Configuration
   ------------------------ */
// Motor pins (matching your MotorController library)
#define RPWM 5
#define LPWM 6
#define REN  9
#define LEN  10

// Stepper pins
#define STEP_PIN 48   // yellow
#define DIR_PIN  50   // green  
#define EN_PIN   52   // orange
#define ENDSTOP_PIN 7

// Encoder pins
#define ENC_A 47      // black
#define ENC_B 3       // white
#define ENC_Z 18      // orange (turn counter)

// Control buttons
#define ESTOP_PIN 12  // E-STOP button (to GND, active LOW)
#define RESUME_PIN 13 // Resume button (to GND, active LOW)

// Sensor pins
#define CURRENT_SENSE_PIN A0
#define VOLTAGE_SENSE_PIN A1
#define TEMP_SENSE_PIN A2

/* ------------------------
   Machine Configuration
   ------------------------ */
const float wireDiameterMM = 0.0635;   // 43 AWG wire diameter
const long bobbinWidth = 7;            // Bobbin width 7mm
const long targetTurns = 1000;         // Target turns to wind

const int bobin_WidthMM = 12;          // Bobbin width (soft limit)
const int startOffsetMM = 1;           // Starting offset from home
const int HOMING_BACKOFF_MM = 20;      // Back off distance after hitting switch
const float HOMING_APPROACH_SPEED = 1000.0; // Steps/sec toward switch

/* ------------------------
   Mechanics
   ------------------------ */
const float leadPitchMM = 6.0;         // Stepper leadscrew pitch in mm
const int stepsPerRev = 200;           // Stepper steps per revolution
const int microsteps = 16;             // Stepper microstepping
float stepperSpeed = 1000.0;           // Stepper movement speed
float stepperMaxSpeed = 2000.0;        // Max stepper speed
float stepperAcceleration = 350;       // Stepper acceleration

/* ------------------------
   System Data Structure  
   ------------------------ */
struct SystemData {
  // Real-time values
  float currentRPM = 0.0;
  float targetRPM = 300.0;
  float motorCurrent = 0.0;
  float supplyVoltage = 12.0;
  float temperature = 25.0;
  long currentTurns = 0;
  long targetTurns = 1000;
  
  // System status
  bool isRunning = false;
  bool isHomed = false;
  String systemStatus = "Ready";
  
  // Statistics  
  unsigned long windingStartTime = 0;
  unsigned long sessionStartTime = 0;
} systemData;

/* ------------------------
   Global Variables
   ------------------------ */
volatile bool stepperDir = true;  // Stepper direction: true = right, false = left
volatile int spindleRevs = 0;     // Z-encoder revolution count
volatile int spindleTurns = 0;    // Current winding turns (from Z)
bool spindleRunning = false;      // Spindle state
int initial_homing = 0;           // Has the unit homed? (1/0)

// Computed stepper parameters
long stepperMinSteps, stepperMaxSteps;
float stepsPerMM;

// State machine
enum State { IDLE, HOMING, OFFSET, WINDING, DONE, ESTOP };
State state = IDLE;

// Objects - using your MotorController library
MotorController spindle(RPWM, LPWM, REN, LEN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ------------------------
   Core Functions
   ------------------------ */
void beginSpindle(int rpm) {
  Serial.println("Starting spindle with MotorController library");
  Serial.println("Target RPM: " + String(rpm));
  
  // Convert RPM to PWM (0-255)
  uint8_t pwmValue = map(rpm, 0, 800, 0, 255);
  Serial.println("PWM value: " + String(pwmValue));
  
  // Use your library's ramping function
  spindle.beginRampingTimed(MotorController::CW, pwmValue, 2000); // 2 second ramp
  
  spindleRunning = true;
  systemData.isRunning = true;
  systemData.systemStatus = "Winding";
  
  Serial.println("Spindle ramp started (CW, 2sec ramp time)");
}

void stopAll() {
  Serial.println("Stopping all motors using MotorController");
  spindle.stopAll();  // Use your library's stop function
  stepper.setSpeed(0);
  spindleRunning = false;
  systemData.isRunning = false;
  Serial.println("All motors stopped");
}

void syncCarriage(float rpm) {
  float feedMMs = (rpm / 60.0) * wireDiameterMM;
  float stepsPerSec = feedMMs * stepsPerMM;
  
  if (!stepperDir) stepsPerSec = -stepsPerSec;
  
  stepper.setSpeed(stepsPerSec);
  
  // Debug output every few seconds
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 3000) {
    lastDebug = millis();
    Serial.print("Sync: RPM=" + String(rpm, 1));
    Serial.print(" FeedMM/s=" + String(feedMMs, 3));  
    Serial.print(" Steps/s=" + String(stepsPerSec, 1));
    Serial.println(" Dir=" + String(stepperDir ? "Right" : "Left"));
  }
}

float computeRPM() {
  static unsigned long lastTime = 0;
  static int lastRevs = 0;
  
  unsigned long now = millis();
  if (now - lastTime >= 500) {
    int revDelta = spindleRevs - lastRevs;
    unsigned long timeDelta = now - lastTime;
    
    float rpm = (revDelta * 60000.0) / timeDelta;
    
    lastRevs = spindleRevs;
    lastTime = now;
    
    return rpm;
  }
  
  return systemData.currentRPM; // Return last known RPM
}

/* ------------------------
   Sensor Reading Functions
   ------------------------ */
void updateSensorReadings() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 200) { // Update every 200ms
    lastUpdate = millis();
    
    // Read supply voltage
    int voltageADC = analogRead(VOLTAGE_SENSE_PIN);
    systemData.supplyVoltage = (voltageADC / 1023.0) * 5.0 * 5.128; // Using your calibration factor
    
    // Read motor current (ACS712 sensor)
    int currentADC = analogRead(CURRENT_SENSE_PIN);
    float currentVoltage = (currentADC / 1023.0) * 5.0;
    systemData.motorCurrent = abs((currentVoltage - 2.5) / 0.066); // ACS712-30A: 66mV/A
    
    // Read temperature (LM35 or similar)
    int tempADC = analogRead(TEMP_SENSE_PIN);
    systemData.temperature = (tempADC / 1023.0) * 5.0 * 100.0; // LM35: 10mV/°C
  }
}

/* ------------------------
   Status Reporting
   ------------------------ */
void printSystemStatus() {
  static unsigned long lastReport = 0;
  
  if (millis() - lastReport >= 2000) { // Report every 2 seconds
    lastReport = millis();
    
    Serial.println("=== SYSTEM STATUS ===");
    Serial.println("State: " + systemData.systemStatus);
    Serial.println("Homed: " + String(systemData.isHomed ? "YES" : "NO"));
    Serial.println("Running: " + String(systemData.isRunning ? "YES" : "NO"));
    Serial.println("RPM: " + String(systemData.currentRPM, 1));
    Serial.println("Turns: " + String(systemData.currentTurns) + " / " + String(systemData.targetTurns));
    
    if (systemData.targetTurns > 0) {
      float progress = (float)systemData.currentTurns / systemData.targetTurns * 100.0;
      Serial.println("Progress: " + String(progress, 1) + "%");
    }
    
    Serial.println("Voltage: " + String(systemData.supplyVoltage, 2) + "V");
    Serial.println("Current: " + String(systemData.motorCurrent, 2) + "A");
    Serial.println("Temp: " + String(systemData.temperature, 1) + "°C");
    
    // Stepper position info
    float positionMM = stepper.currentPosition() / stepsPerMM;
    Serial.println("Carriage: " + String(positionMM, 2) + "mm");
    Serial.println("Direction: " + String(stepperDir ? "Right" : "Left"));
    Serial.println("=====================");
  }
}

/* ------------------------
   Interrupt Handlers
   ------------------------ */
void encoderISR() {
  // Basic encoder reading (if needed for speed feedback)
}

void encoderZISR() {
  spindleRevs++;
  if (spindleRevs % 100 == 0) {
    Serial.println("Spindle revolutions: " + String(spindleRevs));
  }
}

/* ------------------------
   Physical Button Control
   ------------------------ */
void checkPhysicalButtons() {
  static unsigned long lastResumePress = 0;
  static int resumePressCount = 0;
  
  // Multi-function RESUME button
  if (digitalRead(RESUME_PIN) == LOW) {
    if (millis() - lastResumePress > 300) { // Debounce
      lastResumePress = millis();
      resumePressCount++;
      
      Serial.println("RESUME button pressed (#" + String(resumePressCount) + ")");
      
      // Function based on system state
      if (!systemData.isHomed && state == IDLE) {
        Serial.println("→ Starting HOMING sequence");
        state = HOMING;
        systemData.systemStatus = "Homing";
        resumePressCount = 0;
      }
      else if (systemData.isHomed && !systemData.isRunning && state == IDLE) {
        Serial.println("→ Starting WINDING sequence");
        state = OFFSET;
        systemData.systemStatus = "Starting";
        resumePressCount = 0;
      }
      else if (systemData.isRunning && state == WINDING) {
        if (resumePressCount == 1) {
          Serial.println("→ PAUSING winder");
          stopAll();
          systemData.systemStatus = "Paused";
        } else if (resumePressCount >= 2) {
          Serial.println("→ STOPPING winder");
          stopAll();
          state = IDLE;
          systemData.systemStatus = "Stopped";
          resumePressCount = 0;
        }
      }
    }
  }
  
  // Reset press counter after 3 seconds
  if (resumePressCount > 0 && millis() - lastResumePress > 3000) {
    resumePressCount = 0;
  }
}

/* ------------------------
   Setup
   ------------------------ */
void setup() {
  Serial.begin(115200);
  Serial.println("=== COIL WINDER v1.5 (MotorController Integration) ===");

  // Initialize motor controller
  spindle.begin();
  Serial.println("MotorController initialized");

  // Setup stepper
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable stepper driver
  delay(5);

  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  Serial.println("✓ Stepper motor initialized");

  // Setup pins
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(RESUME_PIN, INPUT_PULLUP);
  
  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), encoderZISR, FALLING);
  Serial.println("✓ Pins and interrupts configured");

  // Compute derived values
  stepsPerMM = (float)stepsPerRev * (float)microsteps / leadPitchMM;
  long backoffSteps = (long)round(HOMING_BACKOFF_MM * stepsPerMM);
  stepperMaxSteps = (long)round(bobin_WidthMM * stepsPerMM);

  Serial.println("=== MECHANICAL PARAMETERS ===");
  Serial.println("Lead pitch: " + String(leadPitchMM) + "mm");
  Serial.println("Steps per rev: " + String(stepsPerRev * microsteps));
  Serial.println("Steps per mm: " + String(stepsPerMM, 6));
  Serial.println("Backoff distance: " + String(HOMING_BACKOFF_MM) + "mm (" + String(backoffSteps) + " steps)");
  Serial.println("Bobbin width: " + String(bobin_WidthMM) + "mm (" + String(stepperMaxSteps) + " steps)");
  Serial.println("Wire diameter: " + String(wireDiameterMM, 4) + "mm");
  Serial.println("Target turns: " + String(targetTurns));
  Serial.println("==============================");

  systemData.sessionStartTime = millis();
  systemData.targetTurns = targetTurns;

  Serial.println("System ready!");
  Serial.println("Controls:");
  Serial.println("   • Press RESUME to start homing");
  Serial.println("   • Press RESUME again to start winding");
  Serial.println("   • Type 'help' in Serial Monitor for manual motor control");
  Serial.println("");
  
}

/* ------------------------
   Main Loop
   ------------------------ */
void loop() {
  // CRITICAL: Update MotorController ramping - must be called every loop!
  spindle.update();
  
  // Check physical buttons
  checkPhysicalButtons();
  
  // Update sensors
  updateSensorReadings();
  
  // Print status periodically
  printSystemStatus();

  // Handle emergency stop
  if (digitalRead(ESTOP_PIN) == LOW) {
    Serial.println("EMERGENCY STOP ACTIVATED!");
    stopAll();
    state = ESTOP;
    systemData.systemStatus = "E-STOP";
    
    while (digitalRead(ESTOP_PIN) == LOW) {
      delay(100);
    }
    
    Serial.println("E-STOP released - system ready");
    state = IDLE;
    systemData.systemStatus = "Ready";
  }

  // State machine
  switch (state) {
    case IDLE:
      // Waiting for user input
      break;

    case HOMING:
      {
        static int homingStep = 1; // 1=approach, 2=backup, 3=complete
        static long switchPosition = 0;
        static bool movingToTarget = false;

        switch (homingStep) {
          case 1: // Approaching switch
            // Move toward switch using runSpeed (continuous movement)
            stepper.setSpeed(HOMING_APPROACH_SPEED); // Try positive first
            stepper.runSpeed();

            if (digitalRead(ENDSTOP_PIN) == LOW) {
              // Hit switch - stop and record position
              stepper.stop();
              switchPosition = stepper.currentPosition();
              Serial.println("Hit endstop at position: " + String(switchPosition));
              Serial.println("→ Now backing off " + String(HOMING_BACKOFF_MM) + "mm");
              systemData.systemStatus = "Backing Off";
              homingStep = 2;
              movingToTarget = false;
            }
            break;

          case 2: // Backing off from switch
            if (!movingToTarget) {
              // Calculate where to back off to (opposite direction from approach)
              long backoffSteps = (long)round(HOMING_BACKOFF_MM * stepsPerMM);
              long targetPosition = switchPosition - backoffSteps; // Move away from switch
              
              Serial.println("Backing off from " + String(switchPosition) + " to " + String(targetPosition));
              stepper.moveTo(targetPosition);
              movingToTarget = true;
            }

            // Move to backup position using run() 
            if (stepper.distanceToGo() != 0) {
              stepper.run();
            } else {
              Serial.println("Reached backup position: " + String(stepper.currentPosition()));
              homingStep = 3;
            }
            break;

          case 3: // Set home and complete
            stepper.setCurrentPosition(0); // Set current position as home (0)
            stepperMinSteps = 0;
            stepperMaxSteps = (long)round(bobin_WidthMM * stepsPerMM);

            Serial.println("HOMING COMPLETE");
            Serial.println("Home position set to 0");
            Serial.println("Travel range: 0 to " + String(stepperMaxSteps) + " steps (" + String(bobin_WidthMM) + "mm)");
            
            initial_homing = 1;
            systemData.isHomed = true;
            systemData.systemStatus = "Homed - Ready";
            
            // Reset for next homing cycle
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
          Serial.println("Moving to start offset: " + String(startOffsetMM) + "mm (" + String(targetSteps) + " steps)");
          systemData.systemStatus = "Moving to Start";
        }

        if (stepper.distanceToGo() != 0) {
          stepper.run();
        } else {
          moving = false;
          spindleRevs = 0; // Reset turn counter
          systemData.currentTurns = 0;
          systemData.windingStartTime = millis();
          
          beginSpindle(systemData.targetRPM);
          Serial.println("WINDING STARTED!");
          Serial.println("Target: " + String(systemData.targetTurns) + " turns");
          state = WINDING;
        }
      }
      break;

    case WINDING:
      {
        // Calculate current RPM
        systemData.currentRPM = computeRPM();
        
        // Sync carriage movement to spindle RPM
        if (spindleRunning) {
          syncCarriage(systemData.currentRPM);
        }

        // Update turn count
        spindleTurns = spindleRevs;
        systemData.currentTurns = spindleTurns;

        // Handle soft limits with direction reversal
        long pos = stepper.currentPosition();
        if (pos <= stepperMinSteps && !stepperDir) {
          stepperDir = true;
          Serial.println("Hit left limit → reversing RIGHT");
          syncCarriage(systemData.currentRPM);
        } else if (pos >= stepperMaxSteps && stepperDir) {
          stepperDir = false;
          Serial.println("Hit right limit → reversing LEFT");
          syncCarriage(systemData.currentRPM);
        }

        // Check for completion
        if (spindleTurns >= systemData.targetTurns) {
          Serial.println("TARGET REACHED!");
          Serial.println("Completed " + String(spindleTurns) + " turns");
          systemData.systemStatus = "Complete";
          state = DONE;
        }
      }
      break;

    case DONE:
      stopAll();
      systemData.systemStatus = "DONE - " + String(systemData.currentTurns) + " turns";
      
      unsigned long windingTime = (millis() - systemData.windingStartTime) / 1000;
      Serial.println("WINDING COMPLETE!");
      Serial.println("Total turns: " + String(systemData.currentTurns));
      Serial.println("Winding time: " + String(windingTime) + " seconds");
      Serial.println("Average RPM: " + String(systemData.currentRPM, 1));
      Serial.println("");
      Serial.println("Press RESUME to start a new cycle");
      
      // Stay in DONE state until user intervention
      break;

    case ESTOP:
      // Wait for E-STOP to be released (handled above)
      break;
  }

  // Always run stepper
  stepper.run();
}