//====== Includes & Library ===================================//
#include <AccelStepper.h>
#include <MotorController.h>

/*
  Double BTS7960 high current (43 A) H-bridge driver;
  Using custom library MotorController design for this hardware
*/

//***======================================***
//*** Encoder 5-26V --- 360R/P A/B 720 Pulse per Revolution
//*** Bare=Gnd(Shield), Blue=0v, Brown=Vcc, Black=A, White=B, Orange=z,

//**=== Motor ==================***
const int MOTOR_RPWM = 5;
const int MOTOR_LPWM = 6;
const int MOTOR_ENAR = 9;
const int MOTOR_ENAL = 10;

volatile int currentWinds = 0;   // current count of winds
static int desiredWinds = 1000;  // default number of winds to perform
int motorPWM = 200;


//***=== Encoder ================***
const byte PulsesPerRevolution = 720;
//*** Encoder 5-26V --- 360R/P A/B 720 Pulse per Revolution
//*** Bare=Gnd(Shield), Blue=0v, Brown=Vcc, Black=A, White=B, Orange=z,
const byte ENCZ_PIN = 18;  //pin Orange
const byte ENCA_PIN = 3;  //pin Black
const byte ENCB_PIN = 47;  //pin White
volatile int8_t direction = 0;            // +1 or -1
volatile unsigned long revolutionCount = 0;
volatile unsigned long overflowCount = 0; // counts Timer5 overflows
unsigned long revs = 0;
unsigned long pulseDiff = 0;
int8_t dir = direction;
unsigned long encoderCount = 0;
unsigned long currentCount = 0;

volatile unsigned long lastDirectionChangeMicros = 0;
const unsigned long debounceDirMicros = 1000;  // 1ms debounce

volatile unsigned long lastZPulseTime = 0;
const unsigned long debounceZMicros = 2000; // 2ms debounce

unsigned long totalCount = 0;
unsigned long lastTotalCount = 0;
unsigned long lastTime = 0;
float RPM = 0;

//***=== Define Limit Switches ===***
const int homeSwitch = 34;  // Pin connected to Home Switch (MicroSwitch)
#define ENDSTOP_PIN 33   // carriage home switch
//const int endLimit = 33;
bool isAlarm = 0;

//**== Homing Variable ===========***
byte initial_homing = -1;  // Used to Home stepper at startup
byte initialMotor_Homing = -1;
bool homed = false;
long turnsDone = 0;
long turnsToDo = 5000;         // Set target turns here
// One full wire guide pass every N turns
const int turnsPerPass = 20;
long nextPassTurn = turnsPerPass;
bool wireGuideDirection = true;

// Wire guide stepper
const byte STEP_PIN = 11;
const byte Step_DIR_PIN = 7;
const byte Step_ENA_PIN = 8;
AccelStepper carriageStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
const int wireGuideTravel = 1000; // Steps across bobbin
const int wireGuideAccel = 1000;

#define LIMIT_SWITCH_PIN 6

MotorController motor(MOTOR_RPWM, MOTOR_LPWM, MOTOR_ENAR, MOTOR_ENAL);

//***======================================***
void diag_timer_enc(){
    // Diagnostic printing
    Serial.print("TCNT5: ");
    Serial.print(TCNT5);
    Serial.print(" Overflows: ");
    Serial.print(overflowCount);
    Serial.print(" TotalCount: ");
    Serial.print(totalCount);
    Serial.print(" pulseDiff: ");
    Serial.print(pulseDiff);
    Serial.print(" RPM: ");
    Serial.print(RPM, 1);
    Serial.print(" Dir: ");
    Serial.print(dir > 0 ? "Forward" : (dir < 0 ? "Reverse" : "Stopped"));
    Serial.print(" Revs: ");
    Serial.println(revs);
}

void updateRPM(){
    Serial.print("Turns: "); 
    Serial.print(revs, 1);
    Serial.print("  ");
    Serial.print(RPM, 1);
    Serial.println();
}
// ----------------------------------------
void homeWireGuide() {
  Serial.println("Homing wire guide...");

  wireGuide.setMaxSpeed(300);
  wireGuide.setAcceleration(500);
  wireGuide.moveTo(-5000);

  while (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
    wireGuide.run();
  }

  wireGuide.setCurrentPosition(0);
  homed = true;
  Serial.println("Wire guide homed.");
}
//----
void homeCarriage() {
  carriageStepper.setMaxSpeed(500);   // safe speed
  carriageStepper.setSpeed(-200);     // move left until switch

  while (digitalRead(ENDSTOP_PIN) == HIGH) {
    carriageStepper.runSpeed();
  }

  // Stop at switch
  carriageStepper.setCurrentPosition(0);  // home = 0
}

const int START_OFFSET_STEPS = 200; // adjust for wire offset

void moveToOffset() {
  carriageStepper.moveTo(START_OFFSET_STEPS);
  while (carriageStepper.distanceToGo() != 0) {
    carriageStepper.run();
  }
}

// Example: wire diameter ~0.05mm, leadscrew 1mm pitch, 200 steps/rev
// feed speed = spindle RPM * (wire_dia / lead_pitch * steps_per_rev)

void syncCarriageWithMotor(float spindleRPM) {
  float stepsPerMM = 200.0; // adjust for your leadscrew & microstepping
  float feedRateMMs = (spindleRPM / 60.0) * 0.05; // mm/sec
  float stepperSpeed = feedRateMMs * stepsPerMM;  // steps/sec

  carriageStepper.setSpeed(stepperSpeed * (carriageDir ? 1 : -1));
  carriageStepper.runSpeed();
}

// ---------------------------------------
void setupTimer5() {
  noInterrupts();

  TCCR5A = 0;  // Normal mode
  TCCR5B = 0;

  TCNT5 = 0;   // Reset count

  // External clock on T5 pin (pin 47), rising edge
  // CS52:0 = 111
  TCCR5B |= (1 << CS52) | (1 << CS51) | (1 << CS50);

  // Clear Timer on overflow flag
  TIFR5 |= (1 << TOV5);

  // Enable Timer5 overflow interrupt
  TIMSK5 |= (1 << TOIE5);

  interrupts();
}

ISR(TIMER5_OVF_vect) {
  overflowCount++;
}

void ISR_direction() {
  unsigned long now = micros();
  if (now - lastDirectionChangeMicros >= debounceDirMicros) {
    // For direction, read the state of Channel A pin (pinA)
    int stateA = digitalRead(ENCA_PIN);
    direction = (stateA == HIGH) ? +1 : -1;
    lastDirectionChangeMicros = now;
  }
}

void ISR_index() {
  unsigned long now = micros();
  if (now - lastZPulseTime >= debounceZMicros) {
    revolutionCount++;
    lastZPulseTime = now;
  }
}

unsigned long getTotalCount() {
  unsigned long overflows;
  unsigned int counter;

  noInterrupts();
  overflows = overflowCount;
  counter = TCNT5;
  // If overflow happened just now (during this code), adjust
  if ((TIFR5 & _BV(TOV5)) && (counter < 65535)) {
    overflows++;
    counter = TCNT5;
  }
  interrupts();

  return ((unsigned long)overflows << 16) + counter;
}

void updateTurns() {
  //currentCount = encoderCount;
  //turnsDone = currentCount / pulsesPerRev;
}

//***======================================***
void setup() {

  //***=== Limit Switch ===================***
  pinMode(homeSwitch, INPUT);
  pinMode(endLimit, INPUT);

  //***=== Motor ==========================***
  pinMode(MOTOR_ENAR, OUTPUT);
  pinMode(MOTOR_ENAL, OUTPUT);
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  digitalWrite(MOTOR_ENAR, LOW);
  digitalWrite(MOTOR_ENAL, LOW);
  digitalWrite(MOTOR_RPWM, LOW);
  digitalWrite(MOTOR_LPWM, LOW); 

  //***=== Endcoder =======================***
  pinMode(ENCZ_PIN, INPUT_PULLUP);
  pinMode(ENCA_PIN, INPUT_PULLUP);  
  pinMode(ENCB_PIN, INPUT_PULLUP);  // internal pullup
  digitalWrite(ENCZ_PIN, HIGH);
  digitalWrite(ENCA_PIN, HIGH);            
  digitalWrite(ENCB_PIN, HIGH);     //turn pullup resistor on

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  setupTimer5();

  attachInterrupt(digitalPinToInterrupt(ENCB_PIN), ISR_direction, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCZ_PIN), ISR_index, RISING);

  lastTime = millis();
  lastTotalCount = getTotalCount();

  //***=== Serial Com =======================***
  Serial.begin(115200);
  Serial.println("Encoder + Timer5...");
  Serial.println("Ready...");

  //***=== Set up Hardware ==================***
  motor.begin();

  // Start ramping to 200 (non-blocking)
  motor.beginRamping(MotorController::CW, motorPWM, 5, 20);

  // Setup stepper
  wireGuide.setMaxSpeed(1000);
  wireGuide.setAcceleration(wireGuideAccel);
  //motorHoming();

}

//***======================================***
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= 100) {  // every 100 ms
    noInterrupts();
    totalCount = getTotalCount();
    int8_t dir = direction;
    revs = revolutionCount;
    interrupts();

    pulseDiff = totalCount - lastTotalCount;
    lastTotalCount = totalCount;
    unsigned long interval = currentTime - lastTime;
    lastTime = currentTime;

    float pulsesPerSecond = (pulseDiff * 1000.0) / interval;
    RPM = (pulsesPerSecond / PulsesPerRevolution) * 60.0;

    diag_timer_enc();
  } 

  updateRPM();
  // ----------------------------
  motor.beginRampingTimed(MotorController::CCW, 70, 5000);
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
  // -----------------------------
}

//***======================================***
