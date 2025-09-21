/* Winder v1.4 - Integrated with Nextion Display (Fixed)
   - All original functionality preserved from v1.3
   - Added Nextion LCD interface using basic serial communication
   - No external Nextion library required
   - Works with standard Arduino libraries only

   CAUTION !! dont forget tot add a Ground GnD from the power supply to the MPU GnD

   // Current and voltage hardware -------------------------
   Voltage Sensor Module:
      1.Voltage input range: DC 0-25V.
      2.Voltage detection range: DC 0.02445V-25V.
      3.Voltage Analog Resolution: 0.00489V.
      4.DC input connector: Red terminal positive with VCC, negative with GND.
      5.Output interface: "+" connected 5/3.3V, "-" connected GND, "s" connected the for Arduino AD pins.

     * PIN CONNECTIONS VOLTAGE SENSOR
          S --> A3  green
          + --> 5V  red
          - --> GND blue
           
          Calculate exact values with multimeter:
           
          Vin on adapter = 12.41V
          Vout on Arduino = 2.42
          5V on Arduino = 5.10V
          
          Vin / vOut = factor
          12.41 / 2.42 = 5.128
    --------------------
    Current Sensor Module:
      1.The current sensor chips: ACS712ELC.
      2.Current Range: 30A.
      3.Supply Power: 5V, on-board power indicator.
      4.The module can measure the positive and negative 30 amps, corresponding to the analog output 66mV / A
      5.When there is no the detection current , the output voltage is VCC / 2.
      6.Analog output: 200 mv/A.
  //  -------------------------------------------------------



*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <SoftwareSerial.h>

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

// Nextion Display Connection
#define NEXTION_RX_PIN 14    // Connect to Nextion TX
#define NEXTION_TX_PIN 15    // Connect to Nextion RX

// Sensor pins
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
   Voltage sensor
   ------------------------ */
const int voltageSensorPin = VOLTAGE_SENSE_PIN;          // sensor pin
float vIn;                                // measured voltage (3.3V = max. 16.5V, 5V = max 25V)
float vOut;
float voltageSensorVal;                   // value on pin A3 (0 - 1023)
const float factor = 5.128;               // reduction factor of the Voltage Sensor shield
const float vCC = 5.00;                   // Arduino input voltage (measurable by voltmeter)
/*
  voltageSensorVal = analogRead(voltageSensorPin);    // read the current sensor value (0 - 1023) 
  vOut = (voltageSensorVal / 1024) * vCC;             // convert the value to the real voltage on the analog pin
  vIn =  vOut * factor;                               // convert the voltage on the source by multiplying with the factor
*/
/* ------------------------
   Nextion Communication Setup
   ------------------------ */
SoftwareSerial nextionSerial(NEXTION_RX_PIN, NEXTION_TX_PIN);
char nextionBuffer[100];
String nextionResponse = "";

/* ------------------------
   Simple DC Motor Controller Class
   ------------------------ */
class SimpleMotorController {
  private:
    bool running = false;
    int currentSpeed = 0;
    
  public:
    void begin() {
      pinMode(RPWM, OUTPUT);
      pinMode(LPWM, OUTPUT);
      pinMode(REN, OUTPUT);
      pinMode(LEN, OUTPUT);
      stop();
    }
    
    void beginCW(int speed) {
      currentSpeed = constrain(speed, 0, 255);
      analogWrite(RPWM, currentSpeed);
      analogWrite(LPWM, 0);
      digitalWrite(REN, HIGH);
      digitalWrite(LEN, LOW);
      running = true;
    }
    
    void stop() {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      digitalWrite(REN, LOW);
      digitalWrite(LEN, LOW);
      running = false;
      currentSpeed = 0;
    }
    
    bool isRunning() {
      return running;
    }
};

/* ------------------------
   Display Data Structure
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
SimpleMotorController spindle;

// AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ------------------------
   Nextion Communication Functions
   ------------------------ */
void sendNextionCommand(String command) {
  nextionSerial.print(command);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  delay(10); // Small delay for command processing
}

void sendNextionText(String component, String text) {
  String command = component + ".txt=\"" + text + "\"";
  sendNextionCommand(command);
}

void sendNextionValue(String component, int value) {
  String command = component + ".val=" + String(value);
  sendNextionCommand(command);
}

void showNextionMessage(String message) {
  sendNextionText("txtMessage", message);
  sendNextionCommand("vis txtMessage,1");
  delay(2000);
  sendNextionCommand("vis txtMessage,0");
}

// Read responses from Nextion (for button presses)
void readNextionResponse() {
  if (nextionSerial.available()) {
    char c = nextionSerial.read();
    if (c == '\n' || c == '\r') {
      if (nextionResponse.length() > 0) {
        processNextionCommand(nextionResponse);
        nextionResponse = "";
      }
    } else {
      nextionResponse += c;
    }
  }
}

void processNextionCommand(String command) {
  Serial.println("Nextion command: " + command);
  
  if (command == "start") {
    if (!displayData.isRunning && displayData.isHomed && state == IDLE) {
      state = OFFSET;
      displayData.isRunning = true;
      displayData.windingStartTime = millis();
      displayData.systemStatus = "Starting";
      Serial.println("Start button pressed via Nextion");
    }
  }
  else if (command == "stop") {
    stopAll();
    state = IDLE;
    displayData.isRunning = false;
    displayData.systemStatus = "Stopped";
    Serial.println("Stop button pressed via Nextion");
  }
  else if (command == "pause") {
    if (displayData.isRunning && state == WINDING) {
      stopAll();
      displayData.systemStatus = "Paused";
    } else if (state == IDLE && displayData.isHomed) {
      state = WINDING;
      displayData.systemStatus = "Winding";
    }
    Serial.println("Pause button pressed via Nextion");
  }
  else if (command == "home") {
    if (!displayData.isRunning && state == IDLE) {
      state = HOMING;
      displayData.systemStatus = "Homing";
      Serial.println("Home button pressed via Nextion");
    }
  }
  else if (command.startsWith("rpm:")) {
    int rpm = command.substring(4).toInt();
    displayData.targetRPM = constrain(rpm, 50, 800);
    Serial.println("RPM changed to: " + String(displayData.targetRPM));
  }
  else if (command.startsWith("turns:")) {
    long turns = command.substring(6).toInt();
    displayData.targetTurns = turns;
    Serial.println("Target turns changed to: " + String(displayData.targetTurns));
  }
}

/* ------------------------
   Display Update Functions
   ------------------------ */
void updateMainDisplay() {
  // Update RPM display
  sendNextionText("txtRPM", String(displayData.currentRPM, 1));
  
  // Update RPM gauge (0-100 scale)
  int rpmGaugeValue = map(displayData.currentRPM, 0, 800, 0, 100);
  sendNextionValue("gaugeRPM", rpmGaugeValue);
  
  // Update turns counter
  String turnsText = String(displayData.currentTurns) + " / " + String(displayData.targetTurns);
  sendNextionText("txtTurns", turnsText);
  
  // Update progress bar
  float progress = (float)displayData.currentTurns / displayData.targetTurns * 100.0;
  progress = constrain(progress, 0, 100);
  sendNextionValue("progressBar", (int)progress);
  
  // Update status text
  sendNextionText("txtStatus", displayData.systemStatus);
  
  // Update button states
  updateButtonStates();
}

void updateSettingsDisplay() {
  // Update RPM slider
  int sliderValue = map(displayData.targetRPM, 50, 800, 0, 100);
  sendNextionValue("sliderRPM", sliderValue);
  
  // Update target turns
  sendNextionValue("numTargetTurns", displayData.targetTurns);
  
  // Update wire gauge display
  String gaugeText = "AWG " + String(displayData.wireGauge, 0);
  sendNextionText("txtWireGauge", gaugeText);
}

void updateDiagnosticsDisplay() {
  // Update voltage
  sendNextionText("txtVoltage", String(displayData.supplyVoltage, 2) + "V");
  
  // Update current
  sendNextionText("txtCurrent", String(displayData.motorCurrent, 2) + "A");
  
  // Update power
  sendNextionText("txtPower", String(displayData.motorPower, 1) + "W");
  
  // Update temperature
  sendNextionText("txtTemperature", String(displayData.temperature, 1) + "°C");
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
   Sensor Reading Functions
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
   Setup Nextion Display
   ------------------------ */
void setupNextionDisplay() {
  Serial.println("Initializing Nextion Display...");
  
  // Initialize serial communication with Nextion
  nextionSerial.begin(9600);
  delay(1000);
  
  // Clear any existing data
  sendNextionCommand("cls BLACK");
  delay(100);
  
  // Initialize main page
  sendNextionCommand("page 0");
  delay(100);
  
  // Set initial values
  displayData.sessionStartTime = millis();
  displayData.targetTurns = targetTurns;
  updateMainDisplay();
  
  // Send initial message
  showNextionMessage("Nextion Display Ready!");
  
  Serial.println("Nextion Display Ready!");
}

/* ------------------------
   Update Nextion Display
   ------------------------ */
void updateNextionDisplay() {
  static unsigned long lastUpdate = 0;
  
  // Handle touch events and responses
  readNextionResponse();
  
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
  spindle.beginCW(map(rpm, 0, 800, 0, 255)); // Map RPM to PWM value
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
  // Original encoder interrupt code (if needed)
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

  // Initialize Nextion Display
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
  // Update Nextion display and handle touch events
  updateNextionDisplay();

  // Handle emergency stop
  if (digitalRead(ESTOP_PIN) == LOW) {
    stopAll();
    state = ESTOP;
    displayData.systemStatus = "E-STOP";
    displayData.alarmMessage = "Emergency Stop Activated";
    sendNextionText("txtStatus", "E-STOP ACTIVE");
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
          displayData.systemStatus = "Backing Off";
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
          displayData.systemStatus = "Homed - Ready";
          
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
        Serial.print(" | Progress: "); Serial.print((float)spindleTurns/displayData.targetTurns*100, 1); Serial.print("%");
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
      displayData.systemStatus = "DONE - " + String(displayData.currentTurns) + " turns";
      displayData.totalCoilsWound++;
      showNextionMessage("Coil Complete!");
      break;

    case ESTOP:
      // Wait for condition to clear
      break;
  }

  // Always run stepper
  stepper.run();
}