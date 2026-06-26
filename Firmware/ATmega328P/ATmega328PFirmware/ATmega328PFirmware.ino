/*---------------------------------------------------------------------------
    Project    : Four-Quadrant DC Motor Drive
    Controller : ATmega328P (Arduino Uno)
    Author     : Harshak V P
    Year       : 2026

    Description
    ------------------------------------------------------------------------
    Educational implementation of a textbook Type-E Four-Quadrant Chopper
    using an H-Bridge and a dedicated brake chopper.

    Regenerated energy is dissipated through a braking resistor and is not
    returned to the battery.

    Power Topology
    ------------------------------------------------------------------------
    Battery → Series Diode → DC Bus → Brake Resistor
                                      ↓
                               Brake MOSFET
                                      ↓
                                     GND

    Switching Summary
    ------------------------------------------------------------------------
    Quadrant I    : Forward Motoring
                    Q1 = PWM, Q4 = ON

    Quadrant II   : Forward Dynamic Braking
                    Q2 = PWM, Brake MOSFET = ON

    Quadrant III  : Reverse Motoring
                    Q3 = PWM, Q2 = ON

    Quadrant IV   : Reverse Dynamic Braking
                    Q4 = PWM, Brake MOSFET = ON

    Stop          : All MOSFETs OFF

    Firmware Execution Flow
    ------------------------------------------------------------------------
    readInputs()
          ↓
    determineState()
          ↓
    transitionState()
          ↓
    runState()

    Notes
    ------------------------------------------------------------------------
    • Dead time (100 µs) is inserted only during state transitions.
    • Default Arduino PWM (analogWrite()) is used.
    • Hardware abstraction macros can be replaced to port the firmware to
      STM32 or another microcontroller.
---------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

/*--------------------------- HAL Shim -----------------------------------*/
// Replace these macros with STM32 HAL equivalents when porting.

// Set a digital output HIGH or LOW
#define HAL_GPIO_Write(pin, val)     digitalWrite((pin), (val))

// Read a digital input
#define HAL_GPIO_Read(pin)           digitalRead((pin))

// Write PWM duty cycle (0–255)
#define HAL_PWM_Write(pin, duty)     analogWrite((pin), (duty))

// Busy-wait for the specified number of microseconds
#define HAL_DelayUs(us)              delayMicroseconds((us))

// Millisecond timestamp — rollover-safe when used with subtraction
#define HAL_GetTickMs()              ((uint32_t)millis())

/*--------------------------- Pin Configuration --------------------------*/

// H-Bridge MOSFETs
static const uint8_t PIN_Q1_PWM      = 5;   // Timer0/OC0B
static const uint8_t PIN_Q2_PWM      = 9;   // Timer1/OC1A
static const uint8_t PIN_Q3_PWM      = 6;   // Timer0/OC0A
static const uint8_t PIN_Q4_PWM      = 10;  // Timer1/OC1B

// Brake chopper
static const uint8_t PIN_BRAKE_MOS   = 8;

// User inputs
static const uint8_t PIN_BTN_FORWARD = 2;   // Active LOW
static const uint8_t PIN_BTN_REVERSE = 3;   // Active LOW
static const uint8_t PIN_BTN_BRAKE   = 4;   // Active LOW, level-sensitive
static const uint8_t PIN_POT         = A0;  // Speed demand

/*--------------------------- Timing Constants ---------------------------*/

// Dead time between any two switching states — prevents shoot-through
static const uint32_t DEAD_TIME_US        = 100U;

// Mandatory pause when reversing motor direction
static const uint32_t DIR_CHANGE_DELAY_MS = 500U;

// Minimum stable time before a button state is accepted
static const uint32_t DEBOUNCE_MS         = 200U;

/*--------------------------- Motor State Machine ------------------------*/

// All legal operating states for the drive
typedef enum {
    STATE_STOP = 0,
    STATE_FORWARD_MOTORING,
    STATE_FORWARD_BRAKING,
    STATE_REVERSE_MOTORING,
    STATE_REVERSE_BRAKING,
    STATE_COUNT             // Sentinel — keep last
} MotorState_t;

// Human-readable names indexed by MotorState_t — used for serial logging
static const char * const STATE_NAMES[STATE_COUNT] = {
    "STOP",
    "FORWARD_MOTORING",
    "FORWARD_BRAKING",
    "REVERSE_MOTORING",
    "REVERSE_BRAKING"
};

/*--------------------------- Input Snapshot -----------------------------*/

// Holds debounced button states and speed demand, refreshed once per loop.
// The rest of the firmware reads from this snapshot rather than polling hardware directly.
typedef struct {
    bool    fwdPressed;   // True for one cycle after a valid forward button press
    bool    revPressed;   // True for one cycle after a valid reverse button press
    bool    brkHeld;      // True while the brake button is held and debounced
    uint8_t speedDemand;  // Potentiometer mapped to 0–255
} Inputs_t;

/*--------------------------- Module-Level State -------------------------*/

static MotorState_t g_currentState = STATE_STOP;
static MotorState_t g_nextState    = STATE_STOP;
static Inputs_t     g_inputs       = { false, false, false, 0U };

// Debounce timestamps — one per button
static uint32_t g_lastFwdPressMs  = 0U;
static uint32_t g_lastRevPressMs  = 0U;
static uint32_t g_lastBrkChangeMs = 0U;

// Previous raw brake level — used to detect level changes for debounce restart
static bool g_brkPrevRaw = false;

// Cached PWM duty — avoids redundant analogWrite calls when pot is not moving
static uint8_t g_lastDuty = 0U;

/*--------------------------- Forward Declarations -----------------------*/

static void readInputs(void);
static void determineState(void);
static void transitionState(void);
static void runState(void);

static void enterForwardMotoring(void);
static void enterForwardBraking(void);
static void enterReverseMotoring(void);
static void enterReverseBraking(void);
static void enterStop(void);

static void runForwardMotoring(void);
static void runForwardBraking(void);
static void runReverseMotoring(void);
static void runReverseBraking(void);

static void allMosfetsOff(void);
static void setDuty(uint8_t pin, uint8_t duty);

/*--------------------------- Setup --------------------------------------*/

// Initialise GPIO and put the drive in the STOP state.
// Outputs are driven LOW before pinMode OUTPUT is set to prevent a brief
// undefined level from reaching the gate drivers on power-up.
void setup(void)
{
    // Safe startup — pull all gate signals LOW before enabling outputs
    HAL_GPIO_Write(PIN_Q1_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q2_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q3_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q4_PWM,    LOW);
    HAL_GPIO_Write(PIN_BRAKE_MOS, LOW);

    // Configure outputs
    pinMode(PIN_Q1_PWM,    OUTPUT);
    pinMode(PIN_Q2_PWM,    OUTPUT);
    pinMode(PIN_Q3_PWM,    OUTPUT);
    pinMode(PIN_Q4_PWM,    OUTPUT);
    pinMode(PIN_BRAKE_MOS, OUTPUT);

    // Configure inputs — internal pull-up, buttons are active LOW
    pinMode(PIN_BTN_FORWARD, INPUT_PULLUP);
    pinMode(PIN_BTN_REVERSE, INPUT_PULLUP);
    pinMode(PIN_BTN_BRAKE,   INPUT_PULLUP);

    pinMode(PIN_POT, INPUT);

    Serial.begin(115200);
    Serial.println(F("[DRIVE] Initialised — STOP"));
}

/*--------------------------- Main Loop ----------------------------------*/

// Fixed execution order — no switching logic lives here.
//   1. readInputs()      — sample hardware, apply debounce
//   2. determineState()  — compute next state from debounced inputs
//   3. transitionState() — if state changed: dead time, then enter new state
//   4. runState()        — update PWM duty for the active state
void loop(void)
{
    readInputs();
    determineState();
    transitionState();
    runState();
}

/*--------------------------- Read User Inputs ---------------------------*/

// Sample all buttons and the potentiometer into g_inputs.
//
// Forward and Reverse are edge-triggered: fwdPressed / revPressed are true
// for exactly one loop cycle after the button is pressed and debounced.
//
// Brake is level-sensitive: brkHeld tracks the held state of the button
// with hysteresis provided by the debounce timer.
//
// This is the only place hardware inputs are read.
static void readInputs(void)
{
    const uint32_t now = HAL_GetTickMs();

    // Forward button — edge-triggered, debounced
    g_inputs.fwdPressed = false;
    if (HAL_GPIO_Read(PIN_BTN_FORWARD) == LOW) {
        if ((now - g_lastFwdPressMs) >= DEBOUNCE_MS) {
            g_inputs.fwdPressed = true;
            g_lastFwdPressMs    = now;
        }
    }

    // Reverse button — edge-triggered, debounced
    g_inputs.revPressed = false;
    if (HAL_GPIO_Read(PIN_BTN_REVERSE) == LOW) {
        if ((now - g_lastRevPressMs) >= DEBOUNCE_MS) {
            g_inputs.revPressed = true;
            g_lastRevPressMs    = now;
        }
    }

    // Brake button — level-sensitive, debounced
    // Any level change restarts the debounce timer; the output only updates
    // once the level has been stable for the full debounce window.
    {
        bool brkRaw = (HAL_GPIO_Read(PIN_BTN_BRAKE) == LOW);

        if (brkRaw != g_brkPrevRaw) {
            g_lastBrkChangeMs = now;
            g_brkPrevRaw      = brkRaw;
        }

        if ((now - g_lastBrkChangeMs) >= DEBOUNCE_MS) {
            g_inputs.brkHeld = brkRaw;
        }
    }

    // Potentiometer — 10-bit ADC result shifted to 8-bit for analogWrite
    g_inputs.speedDemand = (uint8_t)(analogRead(PIN_POT) >> 2);
}

/*--------------------------- Determine Next State -----------------------*/

// Compute g_nextState from g_currentState and the debounced inputs.
//
// Transition rules:
//   STOP             + fwdPressed  → FORWARD_MOTORING
//   STOP             + revPressed  → REVERSE_MOTORING
//   FORWARD_MOTORING + brkHeld     → FORWARD_BRAKING
//   FORWARD_MOTORING + revPressed  → STOP  (direction change via STOP)
//   FORWARD_BRAKING  + !brkHeld    → STOP
//   REVERSE_MOTORING + brkHeld     → REVERSE_BRAKING
//   REVERSE_MOTORING + fwdPressed  → STOP  (direction change via STOP)
//   REVERSE_BRAKING  + !brkHeld    → STOP
//
// g_nextState is only written here; gate changes happen in transitionState().
static void determineState(void)
{
    g_nextState = g_currentState;  // Default: remain in current state

    switch (g_currentState) {

        case STATE_STOP:
            if (g_inputs.fwdPressed) {
                g_nextState = STATE_FORWARD_MOTORING;
            } else if (g_inputs.revPressed) {
                g_nextState = STATE_REVERSE_MOTORING;
            }
            break;

        case STATE_FORWARD_MOTORING:
            if (g_inputs.brkHeld) {
                g_nextState = STATE_FORWARD_BRAKING;
            } else if (g_inputs.revPressed) {
                // Direction reversal: always pass through STOP
                g_nextState = STATE_STOP;
            }
            break;

        case STATE_FORWARD_BRAKING:
            if (!g_inputs.brkHeld) {
                g_nextState = STATE_STOP;
            }
            break;

        case STATE_REVERSE_MOTORING:
            if (g_inputs.brkHeld) {
                g_nextState = STATE_REVERSE_BRAKING;
            } else if (g_inputs.fwdPressed) {
                g_nextState = STATE_STOP;
            }
            break;

        case STATE_REVERSE_BRAKING:
            if (!g_inputs.brkHeld) {
                g_nextState = STATE_STOP;
            }
            break;

        default:
            g_nextState = STATE_STOP;
            break;
    }
}

/*--------------------------- Execute State Transition -------------------*/

// Apply a state transition when g_nextState differs from g_currentState.
//
// Transition sequence:
//   1. allMosfetsOff()           — disable entire bridge immediately
//   2. HAL_DelayUs(DEAD_TIME_US) — 100 µs dead time, no switching overlap
//   3. delay(DIR_CHANGE_DELAY_MS)— extra 500 ms pause for direction reversals
//   4. enter<NewState>()         — configure gate signals for the new state
//   5. Commit g_currentState     — FSM advances
//
// Dead time is applied here and nowhere else.
static void transitionState(void)
{
    if (g_nextState == g_currentState) {
        return;  // No transition — nothing to do
    }

    // Disable bridge and insert dead time before any gate reconfiguration
    allMosfetsOff();
    HAL_DelayUs(DEAD_TIME_US);

    // Extra settling time when reversing direction
    bool isDirectionChange =
        ((g_currentState == STATE_FORWARD_MOTORING ||
          g_currentState == STATE_FORWARD_BRAKING)  &&
         (g_nextState    == STATE_REVERSE_MOTORING  ||
          g_nextState    == STATE_REVERSE_BRAKING)) ||
        ((g_currentState == STATE_REVERSE_MOTORING ||
          g_currentState == STATE_REVERSE_BRAKING)  &&
         (g_nextState    == STATE_FORWARD_MOTORING  ||
          g_nextState    == STATE_FORWARD_BRAKING));

    if (isDirectionChange) {
        delay(DIR_CHANGE_DELAY_MS);
    }

    // Configure gate signals for the incoming state
    switch (g_nextState) {
        case STATE_FORWARD_MOTORING:  enterForwardMotoring();  break;
        case STATE_FORWARD_BRAKING:   enterForwardBraking();   break;
        case STATE_REVERSE_MOTORING:  enterReverseMotoring();  break;
        case STATE_REVERSE_BRAKING:   enterReverseBraking();   break;
        case STATE_STOP:              enterStop();              break;
        default:                      enterStop();              break;
    }

    // Log the transition and commit the new state
    Serial.print(F("[DRIVE] "));
    Serial.print(STATE_NAMES[g_currentState]);
    Serial.print(F(" → "));
    Serial.println(STATE_NAMES[g_nextState]);

    g_currentState = g_nextState;
    g_lastDuty     = 0xFFU;  // Invalidate cache to force first PWM write in runState()
}

/*--------------------------- Run Active State ---------------------------*/

// Dispatch to the active state's run-function every loop.
// run<State>() functions update the PWM duty cycle only.
// Gate assignments are never changed here — that is done once in enter<State>().
static void runState(void)
{
    switch (g_currentState) {
        case STATE_FORWARD_MOTORING:  runForwardMotoring();  break;
        case STATE_FORWARD_BRAKING:   runForwardBraking();   break;
        case STATE_REVERSE_MOTORING:  runReverseMotoring();  break;
        case STATE_REVERSE_BRAKING:   runReverseBraking();   break;
        case STATE_STOP:              /* Nothing to do */     break;
        default:                      enterStop();            break;
    }
}

/*--------------------------- Enter-State Functions ----------------------*/
// Called ONCE on transition — configure static gate signals here.
// The PWM pin for the state is left at 0; runState() applies the duty each loop.

// Q-I Forward Motoring
// Q4 = ON (static), Q1 = PWM, Q2 = Q3 = Brake = OFF
static void enterForwardMotoring(void)
{
    HAL_GPIO_Write(PIN_Q2_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q3_PWM,    LOW);
    HAL_GPIO_Write(PIN_BRAKE_MOS, LOW);
    HAL_GPIO_Write(PIN_Q4_PWM,    HIGH);  // Q4 static ON
    // Q1 PWM applied each loop by runForwardMotoring()
}

// Q-II Forward Dynamic Braking
// Brake MOSFET = ON, Q2 = PWM, Q1 = Q3 = Q4 = OFF
static void enterForwardBraking(void)
{
    HAL_GPIO_Write(PIN_Q1_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q3_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q4_PWM,    LOW);
    HAL_GPIO_Write(PIN_BRAKE_MOS, HIGH);  // Brake resistor connected
    // Q2 PWM applied each loop by runForwardBraking()
}

// Q-III Reverse Motoring
// Q2 = ON (static), Q3 = PWM, Q1 = Q4 = Brake = OFF
static void enterReverseMotoring(void)
{
    HAL_GPIO_Write(PIN_Q1_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q4_PWM,    LOW);
    HAL_GPIO_Write(PIN_BRAKE_MOS, LOW);
    HAL_GPIO_Write(PIN_Q2_PWM,    HIGH);  // Q2 static ON
    // Q3 PWM applied each loop by runReverseMotoring()
}

// Q-IV Reverse Dynamic Braking
// Brake MOSFET = ON, Q4 = PWM, Q1 = Q2 = Q3 = OFF
static void enterReverseBraking(void)
{
    HAL_GPIO_Write(PIN_Q1_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q2_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q3_PWM,    LOW);
    HAL_GPIO_Write(PIN_BRAKE_MOS, HIGH);  // Brake resistor connected
    // Q4 PWM applied each loop by runReverseBraking()
}

// STOP — all gates off
static void enterStop(void)
{
    allMosfetsOff();
}

/*--------------------------- Run-State Functions ------------------------*/
// Called every loop — update PWM duty only, never reconfigure gates.

// Q-I Forward Motoring — Q1 duty tracks the potentiometer
static void runForwardMotoring(void)
{
    setDuty(PIN_Q1_PWM, g_inputs.speedDemand);
}

// Q-II Forward Dynamic Braking — Q2 duty controls braking torque
// Higher duty → higher average current through brake resistor → more braking force
static void runForwardBraking(void)
{
    setDuty(PIN_Q2_PWM, g_inputs.speedDemand);
}

// Q-III Reverse Motoring — Q3 duty tracks the potentiometer
static void runReverseMotoring(void)
{
    setDuty(PIN_Q3_PWM, g_inputs.speedDemand);
}

// Q-IV Reverse Dynamic Braking — Q4 duty controls braking torque
static void runReverseBraking(void)
{
    setDuty(PIN_Q4_PWM, g_inputs.speedDemand);
}

/*--------------------------- Utility Functions --------------------------*/

// Force all bridge and brake gate signals to LOW immediately.
// Uses digitalWrite rather than analogWrite(pin, 0) to guarantee a
// synchronous LOW — analogWrite(pin, 0) can leave the AVR timer running.
// Call this before every dead-time insertion.
static void allMosfetsOff(void)
{
    HAL_GPIO_Write(PIN_Q1_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q2_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q3_PWM,    LOW);
    HAL_GPIO_Write(PIN_Q4_PWM,    LOW);
    HAL_GPIO_Write(PIN_BRAKE_MOS, LOW);
    g_lastDuty = 0xFFU;  // Invalidate cached duty so next setDuty() always writes
}

// Write a PWM duty cycle to a pin, skipping the write if the value has not changed.
// Only one pin is PWM'd at a time in this topology, so a single cached value suffices.
// This avoids redundant timer register writes on every loop iteration when the pot is still.
static void setDuty(uint8_t pin, uint8_t duty)
{
    if (duty == g_lastDuty) {
        return;
    }
    HAL_PWM_Write(pin, duty);
    g_lastDuty = duty;
}
