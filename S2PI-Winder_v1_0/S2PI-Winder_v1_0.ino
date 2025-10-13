/*
  Raspberry Pi Pico 2 Coil Winder v2.2
  For GeeekPi Pico Breadboard Kit with 3.5" TFT (ST7796S)
  
  Display: 3.5" TFT 320x480 pixels (ST7796S)
  Touch: Capacitive (GT911) on I2C
  
  Pin Mapping from GeeekPi Kit Documentation:
  Display (SPI0): GP2(CLK), GP3(DIN), GP5(CS), GP6(DC), GP7(RST)
  Touch (I2C0): GP8(SDA), GP9(SCL)
  Buzzer: GP13
  Buttons: GP14(BTN2), GP15(BTN1)
  LEDs: GP16(D1), GP17(D2)
  RGB LED: GP12
  Joystick: ADC0(X), ADC1(Y)
  
  Motor Control (avoiding conflicts):
  Spindle Motor: GP18(RPWM), GP19(LPWM), GP20(REN), GP21(LEN)
  Stepper: GP22(STEP), GP23(DIR), GP24(EN)
  Encoder: GP0(ENC_A), GP1(ENC_B), GP4(ENC_Z)
  Endstop: GP10
  Sensors: GP26(Current), GP27(Voltage), GP28(Temp)
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7796S.h> // Your ST7796S library
#include <AccelStepper.h>
#include "MotorController.h"

// Display pins for GeeekPi Kit (from documentation)
#define TFT_CLK  2   // SPI Clock
#define TFT_MOSI 3   // SPI Data In (DIN)
#define TFT_CS   5   // Chip Select
#define TFT_DC   6   // Data/Command (RS)
#define TFT_RST  7   // Reset

// Create display object for ST7796S using software SPI
Adafruit_ST7796S tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST);

// Use kit's built-in components
#define BUZZER_PIN 13
#define BUTTON1_PIN 15  // Use as RESUME
#define BUTTON2_PIN 14  // Use as E-STOP
#define LED1_PIN 16     // Status LED
#define LED2_PIN 17     // Warning LED
#define RGB_LED_PIN 12  // For special effects

// Motor control pins (avoiding display/kit pins)
#define RPWM 18
#define LPWM 19
#define REN  20
#define LEN  21

// Stepper pins
#define STEP_PIN 22
#define DIR_PIN  23
#define EN_PIN   24

// Encoder pins (using available pins)
#define ENC_A 0   // GP0
#define ENC_B 1   // GP1  
#define ENC_Z 4   // GP4 (turn counter)

// Endstop
#define ENDSTOP_PIN 10  // GP10

// Analog sensors
#define CURRENT_SENSE_PIN 26  // ADC0
#define VOLTAGE_SENSE_PIN 27  // ADC1
#define TEMP_SENSE_PIN 28     // ADC2

// Joystick (can be used for manual speed control)
#define JOYSTICK_X A0  // ADC0 - GP26
#define JOYSTICK_Y A1  // ADC1 - GP27

/* ------------------------
   Configuration
   ------------------------ */
const uint16_t ENCODER_PULSES_PER_REV = 360;
const float wireDiameterMM = 0.0635;  // 43 AWG
const int bobbinWidthMM = 12;
const int startOffsetMM = 1;
const int HOMING_BACKOFF_MM = 20;
const float HOMING_APPROACH_SPEED = 1000.0;

// Motor specifications
const int MOTOR_MAX_RPM = 7000;
const float GEAR_RATIO = 3.0;
const int SPINDLE_MAX_RPM = MOTOR_MAX_RPM / GEAR_RATIO;
const int SAFE_MAX_RPM = 2000;

// Stepper specifications
const float leadPitchMM = 6.0;
const int stepsPerRev = 200;
const int microsteps = 16;
float stepperMaxSpeed = 2000.0;
float stepperAcceleration = 350;

// Display colors - using standard color definitions
#define BLACK   0x7BEF
#define NAVY    0x000F
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFD20



/* ------------------------
   System Data
   ------------------------ */
struct SystemData {
  float currentRPM = 0.0;
  float targetRPM = 300.0;
  float motorCurrent = 0.0;
  float supplyVoltage = 24.0;
  float temperature = 25.0;
  long currentTurns = 0;
  long targetTurns = 1000;
  
  bool isRunning = false;
  bool isHomed = false;
  String systemStatus = "Ready";
  
  unsigned long windingStartTime = 0;
} systemData;

// Global variables
volatile uint32_t encoderCount = 0;
volatile uint32_t revolutionCount = 0;
volatile bool stepperDir = true;
bool spindleRunning = false;

float stepsPerMM;
long stepperMinSteps, stepperMaxSteps;

// State machine
enum State { IDLE, HOMING, OFFSET, WINDING, DONE, ESTOP };
State state = IDLE;

// Motor objects
MotorController spindle(RPWM, LPWM, REN, LEN);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ------------------------
   Display Functions - ST7796S 480x320
   ------------------------ */
void setupDisplay() {
  Serial.println("Initializing ST7796S 3.5\" display...");
  
  // Initialize display using the init() method from your library
  tft.init(320, 480);  // Width 320, Height 480 for portrait
  tft.setRotation(1);  // Landscape mode (480x320)
  tft.fillScreen(BLACK);
  
  // Test that display is working
  tft.setCursor(10, 10);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Display Init OK");
  delay(500);
  
  showSplashScreen();
  
  Serial.println("ST7796S display initialized successfully");
}

void showSplashScreen() {
  tft.fillScreen(BLACK);
  
  // Draw fancy border using the extra screen space
  for(int i = 0; i < 3; i++) {
    tft.drawRect(i, i, 480-i*2, 320-i*2, CYAN);
  }
  
  // Large title (we have 480x320 pixels!)
  tft.setTextColor(CYAN);
  tft.setTextSize(4);
  tft.setCursor(60, 40);
  tft.println("PRECISION");
  tft.setCursor(40, 85);
  tft.println("COIL WINDER");
  
  // Version with kit info
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.setCursor(350, 130);
  tft.println("v2.2");
  
  // Separator line
  tft.drawLine(40, 140, 440, 140, WHITE);
  
  // Hardware info
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.setCursor(140, 160);
  tft.println("Raspberry Pi Pico 2");
  tft.setCursor(110, 185);
  tft.println("GeeekPi Breadboard Kit");
  
  // Features
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(120, 220);
  tft.println("3.5\" ST7796 480x320 Color Display");
  tft.setCursor(130, 235);
  tft.println("High-Speed Precision Winding");
  tft.setCursor(135, 250);
  tft.println("Automatic Wire Guide System");
  tft.setCursor(150, 265);
  tft.println("Real-time RPM Control");
  
  // Kit features indicator
  tft.setTextColor(MAGENTA);
  tft.setCursor(140, 290);
  tft.println("Buzzer + RGB LED + Joystick");
  
  // Flash the onboard LEDs for effect
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
    delay(200);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
    delay(200);
  }
  digitalWrite(LED2_PIN, LOW);
  
  delay(2000);
}

void drawMainScreen() {
  tft.fillScreen(BLACK);
  
  // Header bar
  tft.fillRect(0, 0, 480, 35, NAVY);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("COIL WINDER CONTROL");
  
  // Status indicator circle
  tft.fillCircle(440, 17, 10, systemData.isRunning ? GREEN : RED);
  
  // Main data area with nice spacing for 480x320
  // RPM Box (larger on wide screen)
  tft.drawRect(10, 45, 225, 100, WHITE);
  tft.drawRect(11, 46, 223, 98, WHITE);  // Double border
  tft.setTextSize(2);
  tft.setCursor(90, 50);
  tft.print("RPM");
  
  // Turns Box
  tft.drawRect(245, 45, 225, 100, WHITE);
  tft.drawRect(246, 46, 223, 98, WHITE);  // Double border
  tft.setCursor(315, 50);
  tft.print("TURNS");
  
  // Progress Bar area
  tft.drawRect(10, 155, 460, 50, WHITE);
  tft.setCursor(15, 160);
  tft.setTextSize(1);
  tft.print("PROGRESS");
  
  // System info area
  tft.drawRect(10, 215, 225, 90, WHITE);
  tft.setCursor(15, 220);
  tft.print("SYSTEM INFO");
  
  // Controls area
  tft.drawRect(245, 215, 225, 90, WHITE);
  tft.setCursor(250, 220);
  tft.print("CONTROLS");
  
  // Draw control instructions
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.setCursor(250, 240);
  tft.print("BTN1: START/PAUSE");
  tft.setCursor(250, 255);
  tft.print("BTN2: EMERGENCY STOP");
  tft.setCursor(250, 270);
  tft.print("JOYSTICK: SPEED ADJUST");
}

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  static bool screenDrawn = false;  // Define static variable here
  static float lastRPM = -1;
  static long lastTurns = -1;
  static String lastStatus = "";
  static int lastProgress = -1;
  
  // Draw main screen once
  if (!screenDrawn) {
    drawMainScreen();
    screenDrawn = true;
  }
  
  if (millis() - lastUpdate >= 250) {  // Update 4 times per second
    lastUpdate = millis();
    
    // Update status
    if (systemData.systemStatus != lastStatus) {
      lastStatus = systemData.systemStatus;
      tft.fillRect(200, 10, 220, 20, NAVY);
      tft.setTextColor(YELLOW);
      tft.setTextSize(2);
      tft.setCursor(210, 10);
      tft.print(systemData.systemStatus);
      
      // Update status LED
      tft.fillCircle(440, 17, 10, systemData.isRunning ? GREEN : RED);
      digitalWrite(LED1_PIN, systemData.isRunning ? HIGH : LOW);
    }
    
    // Update RPM with large display
    if (abs(systemData.currentRPM - lastRPM) > 1.0) {
      lastRPM = systemData.currentRPM;
      
      // Clear RPM area
      tft.fillRect(15, 70, 215, 70, BLACK);
      
      // Display current RPM (extra large)
      tft.setTextColor(GREEN);
      tft.setTextSize(5);
      tft.setCursor(25, 75);
      if(systemData.currentRPM < 1000) {
        tft.print(" ");  // Extra space for alignment
      }
      tft.print((int)systemData.currentRPM);
      
      // Display target RPM
      tft.setTextColor(YELLOW);
      tft.setTextSize(2);
      tft.setCursor(25, 120);
      tft.print("Target: ");
      tft.print((int)systemData.targetRPM);
    }
    
    // Update Turns with large display
    if (systemData.currentTurns != lastTurns) {
      lastTurns = systemData.currentTurns;
      
      // Clear turns area
      tft.fillRect(250, 70, 215, 70, BLACK);
      
      // Display current turns (large)
      tft.setTextColor(CYAN);
      tft.setTextSize(4);
      tft.setCursor(260, 75);
      tft.print(systemData.currentTurns);
      
      // Display target turns
      tft.setTextColor(YELLOW);
      tft.setTextSize(2);
      tft.setCursor(260, 120);
      tft.print("of ");
      tft.print(systemData.targetTurns);
    }
    
    // Update Progress Bar with percentage
    if (systemData.targetTurns > 0) {
      int progress = (float)systemData.currentTurns / systemData.targetTurns * 100;
      if (progress != lastProgress) {
        lastProgress = progress;
        
        // Clear progress area
        tft.fillRect(15, 175, 450, 25, BLACK);
        
        // Draw progress bar frame
        tft.drawRect(15, 175, 450, 25, WHITE);
        
        // Fill progress bar
        int barWidth = (446 * progress) / 100;
        if (barWidth > 0) {
          // Gradient effect with multiple colors
          if(progress < 33) {
            tft.fillRect(17, 177, barWidth, 21, RED);
          } else if(progress < 66) {
            tft.fillRect(17, 177, barWidth, 21, YELLOW);
          } else {
            tft.fillRect(17, 177, barWidth, 21, GREEN);
          }
        }
        
        // Progress text in center
        tft.setTextColor(WHITE);
        tft.setTextSize(2);
        tft.setCursor(220, 180);
        tft.print(progress);
        tft.print("%");
      }
    }
    
    // Update system info
    tft.fillRect(15, 235, 215, 65, BLACK);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    
    // Voltage
    tft.setCursor(20, 240);
    tft.print("Voltage: ");
    tft.setTextColor(systemData.supplyVoltage < 22 ? YELLOW : GREEN);
    tft.print(systemData.supplyVoltage, 1);
    tft.print("V");
    
    // Current
    tft.setTextColor(WHITE);
    tft.setCursor(20, 255);
    tft.print("Current: ");
    tft.setTextColor(systemData.motorCurrent > 3 ? YELLOW : GREEN);
    tft.print(systemData.motorCurrent, 1);
    tft.print("A");
    
    // Power
    tft.setTextColor(WHITE);
    tft.setCursor(20, 270);
    tft.print("Power: ");
    tft.print(systemData.supplyVoltage * systemData.motorCurrent, 1);
    tft.print("W");
    
    // Temperature
    tft.setCursor(20, 285);
    tft.print("Temp: ");
    tft.setTextColor(systemData.temperature > 50 ? YELLOW : GREEN);
    tft.print(systemData.temperature, 1);
    tft.print("C");
    
    // Time if running
    tft.setTextColor(WHITE);
    if (systemData.isRunning && systemData.windingStartTime > 0) {
      tft.setCursor(120, 240);
      unsigned long elapsed = (millis() - systemData.windingStartTime) / 1000;
      tft.print("Time: ");
      tft.print(elapsed / 60);
      tft.print(":");
      if ((elapsed % 60) < 10) tft.print("0");
      tft.print(elapsed % 60);
    }
  }
}

/* ------------------------
   Sound Functions using Kit's Buzzer
   ------------------------ */
void beep(int duration = 100) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void playStartSound() {
  for(int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
  }
}

void playCompleteSound() {
  for(int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

/* ------------------------
   Encoder Functions
   ------------------------ */
void encoderISR() {
  encoderCount++;
}

void encoderZISR() {
  revolutionCount++;
  systemData.currentTurns = revolutionCount;
}

void setupEncoders() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), encoderZISR, FALLING);
}

/* ------------------------
   Motor Control
   ------------------------ */
void setupPWM() {
  // Set PWM frequency to 25kHz to eliminate motor whine
  analogWriteFreq(25000);
  analogWriteRange(255);
}

void beginSpindle(int rpm) {
  rpm = constrain(rpm, 50, SAFE_MAX_RPM);
  
  uint8_t pwmValue = map(rpm, 0, SAFE_MAX_RPM, 60, 255);
  
  spindle.beginRampingTimed(MotorController::CW, pwmValue, 2000);
  
  spindleRunning = true;
  systemData.isRunning = true;
  systemData.targetRPM = rpm;
  systemData.systemStatus = "Winding";
  
  playStartSound();  // Use buzzer for feedback
  
  Serial.print("Spindle started: ");
  Serial.print(rpm);
  Serial.print(" RPM, PWM: ");
  Serial.println(pwmValue);
}

void stopAll() {
  spindle.stopAll();
  stepper.setSpeed(0);
  stepper.stop();
  
  spindleRunning = false;
  systemData.isRunning = false;
  systemData.systemStatus = "Stopped";
  
  digitalWrite(LED1_PIN, LOW);
  beep(200);  // Stop beep
  
  Serial.println("All motors stopped");
}

/* ------------------------
   Joystick Speed Control
   ------------------------ */
void checkJoystickSpeed() {
  static unsigned long lastCheck = 0;
  
  if (!systemData.isRunning) return;
  if (millis() - lastCheck < 100) return;
  
  lastCheck = millis();
  
  // Read joystick Y-axis for speed control
  int joyY = analogRead(JOYSTICK_Y);
  
  // Map joystick to speed adjustment (-100 to +100 RPM)
  int speedAdjust = map(joyY, 0, 4095, -100, 100);
  
  if (abs(speedAdjust) > 20) {  // Dead zone
    float newTarget = systemData.targetRPM + (speedAdjust / 10.0);
    newTarget = constrain(newTarget, 50, SAFE_MAX_RPM);
    
    if (abs(newTarget - systemData.targetRPM) > 10) {
      systemData.targetRPM = newTarget;
      beginSpindle(newTarget);
      Serial.print("Joystick speed adjust to: ");
      Serial.println(newTarget);
    }
  }
}

/* ------------------------
   RPM Calculation
   ------------------------ */
float computeRPM() {
  static uint32_t lastCount = 0;
  static unsigned long lastTime = 0;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= 250) {
    uint32_t countDelta = encoderCount - lastCount;
    float timeSeconds = (currentTime - lastTime) / 1000.0;
    
    if (timeSeconds > 0) {
      float pulsesPerSecond = countDelta / timeSeconds;
      float revsPerSecond = pulsesPerSecond / ENCODER_PULSES_PER_REV;
      float rpm = revsPerSecond * 60.0;
      
      systemData.currentRPM = (systemData.currentRPM * 0.7) + (rpm * 0.3);
      
      lastCount = encoderCount;
      lastTime = currentTime;
    }
  }
  
  return systemData.currentRPM;
}

/* ------------------------
   Carriage Sync
   ------------------------ */
void syncCarriage(float rpm) {
  float feedMMs = (rpm / 60.0) * wireDiameterMM;
  float stepsPerSec = feedMMs * stepsPerMM;
  
  if (!stepperDir) stepsPerSec = -stepsPerSec;
  
  stepper.setSpeed(stepsPerSec);
}

/* ------------------------
   Button Handling (Using Kit's Buttons)
   ------------------------ */
void checkButtons() {
  static unsigned long lastDebounce = 0;
  static bool lastBtn1State = HIGH;
  static bool lastBtn2State = HIGH;
  
  if (millis() - lastDebounce < 200) return;
  
  bool btn1 = digitalRead(BUTTON1_PIN);
  bool btn2 = digitalRead(BUTTON2_PIN);
  
  // Button 2 - Emergency Stop (Priority)
  if (btn2 == LOW && lastBtn2State == HIGH) {
    stopAll();
    state = ESTOP;
    systemData.systemStatus = "E-STOP";
    digitalWrite(LED2_PIN, HIGH);  // Turn on warning LED
    beep(500);  // Long beep for E-STOP
    lastDebounce = millis();
    Serial.println("EMERGENCY STOP!");
  }
  
  // Button 1 - Resume/Start/Pause
  if (btn1 == LOW && lastBtn1State == HIGH) {
    lastDebounce = millis();
    beep(50);  // Short confirmation beep
    
    if (state == ESTOP) {
      state = IDLE;
      systemData.systemStatus = "Ready";
      digitalWrite(LED2_PIN, LOW);  // Turn off warning LED
      Serial.println("E-STOP cleared");
    }
    else if (state == IDLE && !systemData.isHomed) {
      state = HOMING;
      systemData.systemStatus = "Homing";
      Serial.println("Starting homing sequence");
    }
    else if (state == IDLE && systemData.isHomed) {
      state = OFFSET;
      systemData.systemStatus = "Starting";
      Serial.println("Starting winding cycle");
    }
    else if (state == WINDING) {
      stopAll();
      state = IDLE;
      Serial.println("Winding paused");
    }
  }
  
  lastBtn1State = btn1;
  lastBtn2State = btn2;
}

/* ------------------------
   Sensor Reading
   ------------------------ */
void updateSensors() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();
    
    // Note: Joystick uses ADC0 and ADC1, so we might need to share or reassign
    // For now, using fixed values or alternate ADC channels if available
    
    // Simulated values for testing (replace with actual sensor reading)
    systemData.supplyVoltage = 24.0;
    systemData.motorCurrent = 1.5;
    systemData.temperature = 25.0 + (millis() / 60000.0);  // Slowly rising temp
  }
}

/* ------------------------
   Setup
   ------------------------ */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== GeeekPi Pico Breadboard Kit Coil Winder v2.2 ===");
  Serial.println("Initializing systems...");
  
  // Setup kit components
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RGB_LED_PIN, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  // Quick LED test
  digitalWrite(LED1_PIN, HIGH);
  delay(100);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, HIGH);
  delay(100);
  digitalWrite(LED2_PIN, LOW);
  beep(50);
  
  // Initialize display
  setupDisplay();
  
  // Setup PWM for motors
  setupPWM();
  
  // Initialize motor controller
  spindle.begin();
  Serial.println("Motor controller initialized");
  
  // Setup stepper
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  Serial.println("Stepper initialized");
  
  // Setup encoders
  setupEncoders();
  Serial.println("Encoders initialized");
  
  // Setup endstop
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  
  // Calculate mechanics
  stepsPerMM = (stepsPerRev * microsteps) / leadPitchMM;
  stepperMaxSteps = bobbinWidthMM * stepsPerMM;
  
  // Initialize system data
  systemData.targetTurns = 1000;
  
  Serial.println("System ready!");
  Serial.println("Controls:");
  Serial.println("  BTN1 (GP15): START/PAUSE");
  Serial.println("  BTN2 (GP14): EMERGENCY STOP");
  Serial.println("  JOYSTICK: Speed adjustment while running");
  
  // Ready beeps
  playStartSound();
}

/* ------------------------
   Main Loop
   ------------------------ */
void loop() {
  // Critical: Update motor ramping
  spindle.update();
  
  // Update display
  updateDisplay();
  
  // Calculate RPM
  computeRPM();
  
  // Check buttons
  checkButtons();
  
  // Check joystick for speed control
  checkJoystickSpeed();
  
  // Update sensors
  updateSensors();
  
  // State machine
  switch (state) {
    case IDLE:
      // Flash LED1 slowly when idle
      digitalWrite(LED1_PIN, (millis() / 1000) % 2);
      break;
      
    case HOMING:
      {
        static int homingPhase = 0;
        
        if (homingPhase == 0) {
          // Move toward endstop
          stepper.setSpeed(-HOMING_APPROACH_SPEED);
          
          if (digitalRead(ENDSTOP_PIN) == LOW) {
            stepper.stop();
            stepper.setCurrentPosition(0);
            homingPhase = 1;
            beep(50);
            Serial.println("Endstop reached");
          } else {
            stepper.runSpeed();
          }
        }
        else if (homingPhase == 1) {
          // Back off from endstop
          long backoffSteps = HOMING_BACKOFF_MM * stepsPerMM;
          stepper.moveTo(backoffSteps);
          homingPhase = 2;
        }
        else if (homingPhase == 2) {
          // Wait for backoff to complete
          if (stepper.distanceToGo() == 0) {
            stepper.setCurrentPosition(0);
            stepperMinSteps = 0;
            stepperMaxSteps = bobbinWidthMM * stepsPerMM;
            
            systemData.isHomed = true;
            systemData.systemStatus = "Ready";
            state = IDLE;
            homingPhase = 0;
            
            beep(100);  // Homing complete beep
            Serial.println("Homing complete");
          } else {
            stepper.run();
          }
        }
      }
      break;
      
    case OFFSET:
      {
        static bool offsetStarted = false;
        
        if (!offsetStarted) {
          long offsetSteps = startOffsetMM * stepsPerMM;
          stepper.moveTo(offsetSteps);
          offsetStarted = true;
          systemData.systemStatus = "Moving to start";
        }
        
        if (stepper.distanceToGo() == 0) {
          // Reset counters
          encoderCount = 0;
          revolutionCount = 0;
          systemData.currentTurns = 0;
          systemData.windingStartTime = millis();
          
          // Start winding
          beginSpindle(systemData.targetRPM);
          state = WINDING;
          offsetStarted = false;
          
          Serial.println("Winding started");
        } else {
          stepper.run();
        }
      }
      break;
      
    case WINDING:
      {
        // Sync carriage with spindle
        if (spindleRunning) {
          syncCarriage(systemData.currentRPM);
          stepper.runSpeed();
        }
        
        // Check carriage limits and reverse
        long pos = stepper.currentPosition();
        
        if (pos <= stepperMinSteps && !stepperDir) {
          stepperDir = true;
          beep(25);  // Quick beep on direction change
          Serial.println("Reversing: moving right");
        }
        else if (pos >= stepperMaxSteps && stepperDir) {
          stepperDir = false;
          beep(25);  // Quick beep on direction change
          Serial.println("Reversing: moving left");
        }
        
        // Check if target reached
        if (systemData.currentTurns >= systemData.targetTurns) {
          stopAll();
          state = DONE;
          systemData.systemStatus = "Complete";
          
          playCompleteSound();  // Victory sound!
          
          Serial.print("Winding complete: ");
          Serial.print(systemData.currentTurns);
          Serial.println(" turns");
        }
      }
      break;
      
    case DONE:
      {
        static bool resultShown = false;
        static bool screenDrawn = false;  // Local static for DONE state
        
        if (!resultShown) {
          unsigned long windingTime = (millis() - systemData.windingStartTime) / 1000;
          
          // Display completion screen
          tft.fillScreen(BLACK);
          tft.setTextColor(GREEN);
          tft.setTextSize(4);
          tft.setCursor(100, 50);
          tft.println("COMPLETE!");
          
          tft.setTextColor(WHITE);
          tft.setTextSize(2);
          tft.setCursor(80, 120);
          tft.print("Total Turns: ");
          tft.println(systemData.currentTurns);
          
          tft.setCursor(80, 150);
          tft.print("Time: ");
          tft.print(windingTime / 60);
          tft.print(" min ");
          tft.print(windingTime % 60);
          tft.println(" sec");
          
          tft.setCursor(80, 180);
          tft.print("Avg RPM: ");
          tft.println((int)systemData.currentRPM);
          
          tft.setTextColor(YELLOW);
          tft.setCursor(100, 240);
          tft.println("Press BTN1 to restart");
          
          Serial.println("=== WINDING COMPLETE ===");
          Serial.print("Total turns: ");
          Serial.println(systemData.currentTurns);
          Serial.print("Time: ");
          Serial.print(windingTime / 60);
          Serial.print(":");
          Serial.println(windingTime % 60);
          Serial.print("Average RPM: ");
          Serial.println(systemData.currentRPM);
          Serial.println("Press BTN1 to start new cycle");
          
          // Flash LEDs in celebration
          for(int i = 0; i < 5; i++) {
            digitalWrite(LED1_PIN, HIGH);
            digitalWrite(LED2_PIN, LOW);
            delay(100);
            digitalWrite(LED1_PIN, LOW);
            digitalWrite(LED2_PIN, HIGH);
            delay(100);
          }
          digitalWrite(LED2_PIN, LOW);
          
          resultShown = true;
        }
        
        // Flash LED1 while waiting
        digitalWrite(LED1_PIN, (millis() / 500) % 2);
        
        // Wait for button press to restart
        if (digitalRead(BUTTON1_PIN) == LOW) {
          state = IDLE;
          resultShown = false;
          screenDrawn = false;  // Force redraw of main screen
          beep(50);
          delay(500);  // Debounce
        }
      }
      break;
      
    case ESTOP:
      // Flash warning LED
      digitalWrite(LED2_PIN, (millis() / 200) % 2);
      break;
  }
  
  // Always run stepper
  stepper.run();
  
  delay(1);  // Small delay for stability
}