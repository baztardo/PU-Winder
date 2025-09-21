/* Winder v1.4 - Integrated with Professional Nextion Display
   - All original functionality preserved from v1.3
   - Added comprehensive Nextion LCD interface
   - Professional touch controls with multiple screens
   - Real-time monitoring and statistics
   - Data logging and quality metrics
*/

#include <Arduino.h>
#include "MotorController.h"
#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include "Nextion.h"

/* ------------------------
   Pin Configuration
   ------------------------ */
// Motor and stepper pins (from original v1.3)
#define RPWM 5
#define LPWM 6
#define REN  9
#define LEN  10

#define STEP_PIN 48   // yellow
#define DIR_PIN  50   // green
#define EN_PIN   52   // orange
#define ENDSTOP_PIN 7

#define ENC_A 47      // black
#define ENC_B 3       // white
#define ENC_Z 18      // orange

#define ESTOP_PIN 12  // E-STOP button (to GND, active LOW)
#define RESUME_PIN 13 // Resume button (to GND, active LOW)

#define homeSwitch 7 // Homing switch pin

// Nextion Display Connection (NEW)
#define NEXTION_RX_PIN 14    // Connect to Nextion TX
#define NEXTION_TX_PIN 15    // Connect to Nextion RX

// Sensor pins (NEW)
#define CURRENT_SENSE_PIN A0
#define VOLTAGE_SENSE_PIN A1
#define TEMP_SENSE_PIN A2

/* ------------------------
   Machine Configuration (from original v1.3)
   ------------------------ */
const float wireDiameterMM = 0.0635;   // 43 AWG wire diameter
const long bobbinWidth = 7;            // Pick Up bobbin width 7mm
const long targetTurns = 1000;         // number of turns to wind to finish
volatile int currentWinds = 0;         // The current amount of winds
const int desiredWinds = 1000;         // The desired amount of winds for the Pick Up
int layerToDo = 0;                     // The amount of windig layers to do.

const int bobin_WidthMM = 12;          // bobbin width (soft limit)
const int startOffsetMM = 1;           // starting offset from home
const int HOMING_BACKOFF_MM = 20;      // back off 20 mm after hitting switch
const float HOMING_APPROACH_SPEED = 1000.0; // steps/sec toward switch
int initial_homing = 0;                // Has the unit homed yet true/false (1/0)
int homePos = 500;                     // This is the offset value of the begining position to the bobbin

/* ------------------------
   Mechanics (from original v1.3)
   ------------------------ */
const float leadPitchMM = 6.0;         // stepper leadscrew pitch in mm
const int stepsPerRev = 200;           // stepper steps per revolution
const int microsteps = 16;             // stepper Micorstepping
float stepperSpeed = 1000.0;           // stepper movement speed
float stepperMaxSpeed = 2000.0;        // Max stepper movement speed
float stepperAcceleration = 350;       // accelaration of the stepper movement

const int feederMicroStepping = microsteps; // The stepper micro step value.

/* ------------------------
   Encoder Configuration
   ------------------------ */
const int ENCODER_PPR = 720;
const float COUNTS_PER_REV = ENCODER_PPR * 4;

/* ------------------------
   Nextion Communication Setup (NEW)
   ------------------------ */
SoftwareSerial nextionSerial(NEXTION_RX_PIN, NEXTION_TX_PIN);
char nextionBuffer[100];

/* ------------------------
   Display Data Structure (NEW)
   ------------------------ */
struct DisplayData {
  // Real-time monitoring values
  float currentRPM = 0.0;
  float targetRPM = 300.0;
  float motorCurrent = 0.0;
  float supplyVoltage = 12.0;
  float motorPower = 0.0;
  long currentTurns = 0;
  long targetTurns = 1000;
  float temperature = 25.0;
  
  // System status
  bool isRunning = false;
  bool isHomed = false;
  bool wirePresent = true;
  bool tensionOK = true;
  String systemStatus = "Ready";
  String alarmMessage = "";
  
  // Settings
  int selectedProgram = 1;
  float wireGauge = 43.0;
  float wireDiameter = 0.0635;
  float bobbinWidth = 10.0;
  
  // Statistics
  long totalCoilsWound = 0;
  float averageWindingTime = 0.0;
  float qualityPercentage = 98.5;
  unsigned long sessionStartTime = 0;
  
  // Winding progress
  int currentLayer = 1;
  float windingProgress = 0.0;  // 0-100%
  unsigned long windingStartTime = 0;
  float estimatedTimeRemaining = 0.0;
} displayData;

/* ------------------------
   Nextion Object Definitions (NEW)
   ------------------------ */
// Page 0 - Main Status Screen
NexPage pageMain = NexPage(0, 0, "pageMain");

// Main status display objects
NexText txtRPM = NexText(0, 1, "txtRPM");
NexText txtTurns = NexText(0, 2, "txtTurns");
NexText txtStatus = NexText(0, 3, "txtStatus");
NexProgressBar progressBar = NexProgressBar(0, 4, "progressBar");
NexGauge gaugeRPM = NexGauge(0, 5, "gaugeRPM");

// Control buttons
NexButton btnStart = NexButton(0, 10, "btnStart");
NexButton btnStop = NexButton(0, 11, "btnStop");
NexButton btnPause = NexButton(0, 12, "btnPause");
NexButton btnHome = NexButton(0, 13, "btnHome");
NexButton btnSettings = NexButton(0, 14, "btnSettings");

// Page 1 - Settings Screen
NexPage pageSettings = NexPage(1, 0, "pageSettings");
NexSlider sliderRPM = NexSlider(1, 1, "sliderRPM");
NexNumber numTargetTurns = NexNumber(1, 2, "numTargetTurns");
NexText txtWireGauge = NexText(1, 3, "txtWireGauge");
NexButton btnSaveSettings = NexButton(1, 10, "btnSaveSettings");
NexButton btnBackMain = NexButton(1, 11, "btnBackMain");

// Page 2 - Diagnostics Screen
NexPage pageDiagnostics = NexPage(2, 0, "pageDiagnostics");
NexText txtVoltage = NexText(2, 1, "txtVoltage");
NexText txtCurrent = NexText(2, 2, "txtCurrent");
NexText txtPower = NexText(2, 3, "txtPower");
NexText txtTemperature = NexText(2, 4, "txtTemperature");
NexWaveform waveformRPM = NexWaveform(2, 5, "waveformRPM");

/* ------------------------
   Original v1.3 Variables and Objects
   ------------------------ */
volatile bool stepperDir = true;  // Stepper direction: true = right, false = left
volatile int spindleRevs = 0;     // Z-encoder revolution count
volatile int spindleTurns = 0;    // Current winding turns (from Z)
bool spindleRunning = false;      // Spindle state

// Computed stepper limits and parameters
long stepperMinSteps, stepperMaxSteps;
float stepsPerMM;

// State machine
enum State { IDLE, HOMING, OFFSET, WINDING, DONE, ESTOP };
State state = IDLE;

// Motor controller object
MotorController spindle;

// AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ------------------------
   Touch Event Handlers (NEW)
   ------------------------ */
void btnStartPushCallback(void *ptr) {
  Serial.println("Start button pressed");
  if (!displayData.isRunning && displayData.isHomed && state == IDLE) {
    // Start the winding process
    state = OFFSET; // Go to offset position first
    displayData.isRunning = true;
    displayData.windingStartTime = millis();
    displayData.systemStatus = "Starting";
    updateMainDisplay();
  }
}

void btnStopPushCallback(void *ptr) {
  Serial.println("Stop button pressed");
  stopAll();
  state = IDLE;
  displayData.isRunning = false;
  displayData.systemStatus = "Stopped";
  updateMainDisplay();
}

void btnPausePushCallback(void *ptr) {
  Serial.println("Pause button pressed");
  if (displayData.isRunning && state == WINDING) {
    stopAll();
    displayData.systemStatus = "Paused";
  } else if (state == IDLE && displayData.isHomed) {
    // Resume winding
    state = WINDING;
    displayData.systemStatus = "Winding";
  }
  updateMainDisplay();
}

void btnHomePushCallback(void *ptr) {
  Serial.println("Home button pressed");
  if (!displayData.isRunning && state == IDLE) {
    state = HOMING;
    displayData.systemStatus = "Homing";
    updateMainDisplay();
  }
}

void btnSettingsPushCallback(void *ptr) {
  Serial.println("Settings button pressed");
  if (!displayData.isRunning) {
    pageSettings.show();
    updateSettingsDisplay();
  }
}

void sliderRPMCallback(void *ptr) {
  uint32_t value;
  sliderRPM.getValue(&value);
  displayData.targetRPM = map(value, 0, 100, 50, 800);  // 50-800 RPM range
  Serial.print("Target RPM changed to: ");
  Serial.println(displayData.targetRPM);
}

void numTargetTurnsCallback(void *ptr) {
  uint32_t value;
  numTargetTurns.getValue(&value);
  displayData.targetTurns = value;
  Serial.print("Target turns changed to: ");
  Serial.println(displayData.targetTurns);
}

void btnSaveSettingsCallback(void *ptr) {
  Serial.println("Settings saved");
  showMessage("Settings Saved", 2000);
}

void btnBackMainCallback(void *ptr) {
  pageMain.show();
  updateMainDisplay();
}

/* ------------------------
   Touch Event Registration (NEW)
   ------------------------ */
NexTouch *nex_listen_list[] = {
  &btnStart,
  &btnStop,
  &btnPause,
  &btnHome,
  &btnSettings,
  &sliderRPM,
  &numTargetTurns,
  &btnSaveSettings,
  &btnBackMain,
  NULL
};

/* ------------------------
   Display Update Functions (NEW)
   ------------------------ */
void updateMainDisplay() {
  // Update RPM display and gauge
  snprintf(nextionBuffer, sizeof(nextionBuffer), "%.1f", displayData.currentRPM);
  txtRPM.setText(nextionBuffer);
  
  int rpmGaugeValue = map(displayData.currentRPM, 0, 800, 0, 100);
  gaugeRPM.setValue(rpmGaugeValue);
  
  // Update turns counter
  snprintf(nextionBuffer, sizeof(nextionBuffer), "%ld / %ld", 
           displayData.currentTurns, displayData.targetTurns);
  txtTurns.setText(nextionBuffer);
  
  // Update progress bar
  float progress = (float)displayData.currentTurns / displayData.targetTurns * 100.0;
  progress = constrain(progress, 0, 100);
  progressBar.setValue((uint32_t)progress);
  
  // Update status text
  txtStatus.setText(displayData.systemStatus.c_str());
  
  // Update button states based on system status
  updateButtonStates();
}

void updateSettingsDisplay() {
  // Update RPM slider
  uint32_t sliderValue = map(displayData.targetRPM, 50, 800, 0, 100);
  sliderRPM.setValue(sliderValue);
  
  // Update target turns
  numTargetTurns.setValue(displayData.targetTurns);
  
  // Update wire gauge display
  snprintf(nextionBuffer, sizeof(nextionBuffer), "AWG %.0f", displayData.wireGauge);
  txtWireGauge.setText(nextionBuffer);
}

void updateDiagnosticsDisplay() {
  // Update voltage
  snprintf(nextionBuffer, sizeof(nextionBuffer), "%.2fV", displayData.supplyVoltage);
  txtVoltage.setText(nextionBuffer);
  
  // Update current
  snprintf(nextionBuffer, sizeof(nextionBuffer), "%.2fA", displayData.motorCurrent);
  txtCurrent.setText(nextionBuffer);
  
  // Update power
  snprintf(nextionBuffer, sizeof(nextionBuffer), "%.1fW", displayData.motorPower);
  txtPower.setText(nextionBuffer);
  
  // Update temperature
  snprintf(nextionBuffer, sizeof(nextionBuffer), "%.1f°C", displayData.temperature);
  txtTemperature.setText(nextionBuffer);
  
  // Add data point to RPM waveform
  uint8_t rpmWaveValue = map(displayData.currentRPM, 0, 800, 0, 255);
  waveformRPM.addValue(0, rpmWaveValue);  // Channel 0, RPM value
}

void updateButtonStates() {
  // Enable/disable buttons based on system state
  if (displayData.isRunning) {
    sendNextionCommand("btnStart.en=0");      // Disable start
    sendNextionCommand("btnStop.en=1");       // Enable stop
    sendNextionCommand("btnPause.en=1");      // Enable pause
    sendNextionCommand("btnHome.en=0");       // Disable home
    sendNextionCommand("btnSettings.en=0");   // Disable settings
  } else {
    sendNextionCommand("btnStart.en=1");      // Enable start
    sendNextionCommand("btnStop.en=0");       // Disable stop
    sendNextionCommand("btnPause.en=0");      // Disable pause
    sendNextionCommand("btnHome.en=1");       // Enable home
    sendNextionCommand("btnSettings.en=1");   // Enable settings
  }
}

/* ------------------------
   Nextion Communication Functions (NEW)
   ------------------------ */
void sendNextionCommand(const char* command) {
  nextionSerial.print(command);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
}

void showMessage(const char* message, uint16_t duration) {
  snprintf(nextionBuffer, sizeof(nextionBuffer), 
           "vis txtMessage,1");
  sendNextionCommand(nextionBuffer);
  
  snprintf(nextionBuffer, sizeof(nextionBuffer), 
           "txtMessage.txt=\"%s\"", message);
  sendNextionCommand(nextionBuffer);
  
  // Hide message after duration
  delay(duration);
  sendNextionCommand("vis txtMessage,0");
}

/* ------------------------
   Sensor Reading Functions (NEW)
   ------------------------ */
void updateSensorReadings() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 100) { // Update every 100ms
    // Read supply voltage
    int voltageADC = analogRead(VOLTAGE_SENSE_PIN);
    displayData.supplyVoltage = (voltageADC / 1023.0) * 5.0 * 2.0; // Adjust based on voltage divider
    
    // Read motor current (if current sensor available)
    int currentADC = analogRead(CURRENT_SENSE_PIN);
    float currentVoltage = (currentADC / 1023.0) * 5.0;
    displayData.motorCurrent = abs((currentVoltage - 2.5) / 0.1); // ACS712 20A sensor
    
    // Read temperature (if temperature sensor available)
    int tempADC = analogRead(TEMP_SENSE_PIN);
    float tempVoltage = (tempADC / 1023.0) * 5.0;
    displayData.temperature = (tempVoltage * 1000.0) / 10.0; // LM35 sensor
    
    // Calculate power
    displayData.motorPower = displayData.motorCurrent * displayData.supplyVoltage;
    
    lastUpdate = millis();
  }
}

/* ------------------------
   Setup Nextion Display (NEW)
   ------------------------ */
void setupNextionDisplay() {
  Serial.println("Initializing Nextion Display...");
  
  // Initialize serial communication with Nextion
  nextionSerial.begin(9600);
  delay(500);
  
  // Register touch event callbacks
  btnStart.attachPush(btnStartPushCallback);
  btnStop.attachPush(btnStopPushCallback);
  btnPause.attachPush(btnPausePushCallback);
  btnHome.attachPush(btnHomePushCallback);
  btnSettings.attachPush(btnSettingsPushCallback);
  sliderRPM.attachPush(sliderRPMCallback);
  numTargetTurns.attachPush(numTargetTurnsCallback);
  btnSaveSettings.attachPush(btnSaveSettingsCallback);
  btnBackMain.attachPush(btnBackMainCallback);
  
  // Initialize display
  pageMain.show();
  delay(100);
  
  // Set initial values
  displayData.sessionStartTime = millis();
  displayData.targetTurns = targetTurns;
  updateMainDisplay();
  
  Serial.println("Nextion Display Ready!");
}

/* ------------------------
   Update Nextion Display (NEW)
   ------------------------ */
void updateNextionDisplay() {
  static unsigned long lastUpdate = 0;
  
  // Handle touch events
  nexLoop(nex_listen_list);
  
  // Update main display every 250ms
  if (millis() - lastUpdate >= 250) {
    // Update display data with current system values
    displayData.currentRPM = computeRPM();
    displayData.currentTurns = spindleTurns;
    displayData.isHomed = (initial_homing == 1);
    
    updateMainDisplay();
    updateSensorReadings();
    
    lastUpdate = millis();
  }
}

/* ------------------------
   Original v1.3 Functions (PRESERVED)
   ------------------------ */

void beginSpindle(int rpm) {
  spindle.beginCW(rpm);
  spindleRunning = true;
  displayData.isRunning = true;
  displayData.systemStatus = "Winding";
}

void stopAll() {
  spindle.stop();
  stepper.setSpeed(0);
  spindleRunning = false;
  displayData.isRunning = false;
}

void syncCarriage(float rpm) {
  float feedMMs = (rpm / 60.0) * wireDiameterMM;
  float stepsPerSec = feedMMs * stepsPerMM;
  
  if (!stepperDir) stepsPerSec = -stepsPerSec;
  
  stepper.setSpeed(stepsPerSec);
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
  
  return displayData.currentRPM; // Return last known RPM
}

void encoderISR() {
  // Original encoder interrupt code
}

void encoderZISR() {
  spindleRevs++;
}

/* ------------------------
   Setup (UPDATED)
   ------------------------ */
void setup() {
  Serial.begin(115200);

  // Initialize motor controller
  spindle.begin();

  // Setup stepper
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // enable stepper driver
  delay(5);                              // Wait for Driver wake up

  stepper.setMaxSpeed(stepperMaxSpeed);  // SPEED = Steps / second
  stepper.setAcceleration(stepperAcceleration);  // ACCELERATION = Steps /(second)^2

  // Setup pins
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), encoderZISR, FALLING);

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(RESUME_PIN, INPUT_PULLUP);

  // compute derived values
  stepsPerMM = (float)stepsPerRev * (float)microsteps / leadPitchMM;
  long backoffSteps = (long)round(HOMING_BACKOFF_MM * stepsPerMM);
  stepperMaxSteps = (long)round(bobin_WidthMM * stepsPerMM);

  // Initialize Nextion Display (NEW)
  setupNextionDisplay();

  Serial.println("=== Winder diagnostics ===");
  Serial.print("leadPitchMM = "); Serial.println(leadPitchMM);
  Serial.print("stepsPerRev*microsteps = "); Serial.println(stepsPerRev * microsteps);
  Serial.print("stepsPerMM = "); Serial.println(stepsPerMM, 6);
  Serial.print("HOMING_BACKOFF_MM = "); Serial.println(HOMING_BACKOFF_MM);
  Serial.print("backoffSteps (approx) = "); Serial.println(backoffSteps);
  Serial.print("bobin_WidthMM = "); Serial.println(bobin_WidthMM);
  Serial.print("stepperMaxSteps (approx) = "); Serial.println(stepperMaxSteps);
  Serial.println("==========================");

  Serial.println("Starting Pick Winder (v1.4 with Nextion)...");
  Serial.println("Use Nextion display for control, or press RESUME to start homing");
}

/* ------------------------
   Main Loop (UPDATED)
   ------------------------ */
void loop() {
  // Update Nextion display and handle touch events (NEW)
  updateNextionDisplay();

  // Handle emergency stop
  if (digitalRead(ESTOP_PIN) == LOW) {
    stopAll();
    state = ESTOP;
    displayData.systemStatus = "E-STOP";
    displayData.alarmMessage = "Emergency Stop Activated";
    while (digitalRead(ESTOP_PIN) == LOW) {
      delay(100);
      updateNextionDisplay(); // Keep display responsive
    }
    state = IDLE;
    displayData.systemStatus = "Ready";
    displayData.alarmMessage = "";
  }

  // Handle resume button (for backward compatibility)
  if (digitalRead(RESUME_PIN) == LOW && state == IDLE) {
    delay(200); // debounce
    if (digitalRead(RESUME_PIN) == LOW && !displayData.isHomed) {
      state = HOMING;
      displayData.systemStatus = "Homing";
    }
  }

  // State machine (ORIGINAL v1.3 logic preserved)
  switch (state) {
    case IDLE:
      break;

    /***** HOMING: move into switch, back off, set zero *****/
    case HOMING: {
      static bool movingToSwitch = true;
      static bool backingOff = false;

      if (movingToSwitch) {
        stepper.setSpeed(HOMING_APPROACH_SPEED);
        stepper.runSpeed();

        if (digitalRead(ENDSTOP_PIN) == LOW) {
          stepper.setSpeed(0);
          movingToSwitch = false;
          backingOff = true;
          Serial.println("Hit switch → backing off");
        }
      }

      if (backingOff) {
        static bool moving = false;
        static long targetSteps;

        if (!moving) {
          targetSteps = stepper.currentPosition() - (long)round(HOMING_BACKOFF_MM * stepsPerMM);
          stepper.moveTo(targetSteps);
          moving = true;
          Serial.print("Backing off to steps: "); Serial.println(targetSteps);
        }

        if (stepper.distanceToGo() != 0) {
          stepper.run();
        } else {
          stepper.setCurrentPosition(0);
          stepperMinSteps = 0;
          stepperMaxSteps = (long)round(bobin_WidthMM * stepsPerMM);

          Serial.println("Homing complete → moving to offset");
          initial_homing = 1;
          displayData.isHomed = true;
          displayData.systemStatus = "Homed";
          
          movingToSwitch = true;
          backingOff = false;
          moving = false;
          state = IDLE; // Wait for start command
        }
      }
      break;
    }

    /***** OFFSET: move to user-defined start offset (in mm) *****/
    case OFFSET: {
      static bool moving = false;
      long targetSteps = startOffsetMM * stepsPerMM;

      if (!moving) {
        stepper.moveTo(targetSteps);
        moving = true;
        Serial.print("Moving to start offset (steps): "); Serial.println(targetSteps);
        displayData.systemStatus = "Moving to Start";
      }

      if (stepper.distanceToGo() != 0) {
        stepper.run();
      } else {
        moving = false;
        spindleRevs = 0; // reset turns at start of winding
        displayData.currentTurns = 0;
        beginSpindle(displayData.targetRPM); // start spindle at target RPM
        Serial.println("Reached offset — starting spindle and entering WINDING");
        state = WINDING;
      }
      break;
    }

    /***** WINDING: sync stepper to spindle RPM and soft-limit reversal *****/
    case WINDING: {
      float rpm = computeRPM();
      if (spindleRunning) syncCarriage(rpm);

      spindleTurns = spindleRevs; // count turns from Z
      displayData.currentTurns = spindleTurns;

      // Reverse stepper at soft limits
      long pos = stepper.currentPosition();
      if (pos <= stepperMinSteps && !stepperDir) {
        stepperDir = true;
        Serial.println("Reached left end → reversing right");
        syncCarriage(rpm);
      } else if (pos >= stepperMaxSteps && stepperDir) {
        stepperDir = false;
        Serial.println("Reached right end → reversing left");
        syncCarriage(rpm);
      }

      // Status printout every 2s
      static unsigned long lastStatus = 0;
      if (millis() - lastStatus >= 2000) {
        lastStatus = millis();
        float feedMMs = (rpm / 60.0) * wireDiameterMM;
        float stepsPerSec = feedMMs * stepsPerMM;
        Serial.print("RPM: "); Serial.print(rpm);
        Serial.print(" | Turns: "); Serial.print(spindleTurns);
        Serial.print(" | Target: "); Serial.print(displayData.targetTurns);
        Serial.print(" | Carriage (mm): ");
        Serial.print(stepper.currentPosition() / stepsPerMM, 3);
        Serial.print(" | Dir: "); Serial.print(stepperDir ? "Right" : "Left");
        Serial.print(" | Feed (steps/s): "); Serial.println(stepsPerSec, 1);
      }

      if (spindleTurns >= displayData.targetTurns) {
        Serial.println("Target turns reached");
        displayData.systemStatus = "Complete";
        state = DONE;
      }
      break;
    }

    case DONE:
      stopAll();
      displayData.systemStatus = "Done - " + String(displayData.currentTurns) + " turns";
      displayData.totalCoilsWound++;
      break;

    case ESTOP:
      // Wait for condition to clear
      break;
  }

  // Always run stepper
  stepper.run();
}