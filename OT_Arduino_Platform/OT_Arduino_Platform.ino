#include <Wire.h>    // I2C Communication between to LCD
#include <SerLCD.h>  // LCD over qwiic via I2C

#include <EEPROM.h>            // For storing set temperatures
#include <Adafruit_ADS1X15.h>  // For thermistor ADC

#include <SparkFun_Qwiic_Relay.h>  // For all qwiic relays

#include <AutoPID.h>  // PID Controllers for thermocouple + heaters

#include <Vector.h>
#include "Hashtable.h"  // Mapping commands to handlers

#include <avr/interrupt.h>  // For custom uint64 timer instead of default uint32
// Required to avoid problems with integer overflow after 47 days
#include <avr/wdt.h>  // Include the watchdog timer header

volatile uint64_t longTimer = 0;  // milliseconds since board was powered/last reset

#define FUNCTION_COMPLETE() Serial.println("0");
#define THROW_BAD_ARGS() \
  Serial.println("1"); \
  return;

#define CHECK_ARG(arg) \
  if (arg == -1) { \
    THROW_BAD_ARGS() \
  };

// Define LCD
SerLCD lcd;  // Initialize the library with default I2C address 0x72

// Define control objects
class RelayBoard  // Overhead over qwiic relay boards
{
private:
  Qwiic_Relay *qwiicRelay = nullptr;

public:
  uint8_t numRelays;
  uint8_t relayIdsStart;  // i.e. for the second quad relay board this is 4

  RelayBoard(Qwiic_Relay *primitiveRelay, uint8_t numRelays, uint8_t relayIdsStart) {
    qwiicRelay = primitiveRelay;
    this->numRelays = numRelays;
    this->relayIdsStart = relayIdsStart;
    begin();

    // Start with all relays off
    for (int i = 0; i < numRelays; i++) {
      qwiicRelay->turnRelayOff(i + 1);
    }
  }

  void enable(int localId) {
    if (numRelays == 1) {
      qwiicRelay->turnRelayOn();
    } else {
      qwiicRelay->turnRelayOn(localId);
    }
  }

  void disable(int localId) {
    if (numRelays == 1)
      qwiicRelay->turnRelayOff();
    else
      qwiicRelay->turnRelayOff(localId);
  }

  int getEnabled(int localId) {
    if (numRelays == 1)
      qwiicRelay->getState();
    else
      qwiicRelay->getState(localId);
  }

  bool begin() {
    return qwiicRelay->begin();
  }

  void setPWM(int localId, float dutyCycle) {
    qwiicRelay->setSlowPWM(localId, dutyCycle);
  }

  void startup() {
    if (!qwiicRelay->begin()) {
      // Display message on lcd and transmit via serial
      String err =
        "Relay " + String((int)relayIdsStart) + "-" + String((int)relayIdsStart + (int)numRelays) + " not found";

      lcd.setCursor(0, 0);
      lcd.print(err);
      Serial.println(err);
      delay(2000);
    }
  }
};

class Relay {
private:
  RelayBoard *parentBoard = nullptr;
  uint8_t numberOnBoard = 0;

public:
  Relay(RelayBoard *relayBoard, uint8_t localRelayNumber) {
    this->parentBoard = relayBoard;
    this->numberOnBoard = localRelayNumber;
  }
  void enable() {
    parentBoard->enable(numberOnBoard);
  }
  void disable() {
    parentBoard->disable(numberOnBoard);
  }
  void setPWM(float dutyCycle) {
    parentBoard->setPWM(numberOnBoard, dutyCycle);
  }
  int getState() {
    return parentBoard->getEnabled(numberOnBoard);
  }
};
using UltrasonicDriver = Relay;
using Pump = Relay;

// Constants for thermistor decoding
#define RC 10000.0
#define VCC 3.3
#define R25 10000.0
#define B2585 3977.0

class ThermistorChannel {
private:
  uint8_t channel = 0;

public:
  ThermistorChannel(uint8_t channel) {
    this->channel = channel;
  }

  double getTemp()
  {
    if (channel > 3) return -1.0;
    int16_t adc = ads.readADC_SingleEnded(this->channel); //0-1023

    double V = (adc * VCC / 1023.0);
    double R = RC / ((VCC / V) - 1);
    double Tk = 1.0 / (log(R / R25) / B2585 + (1.0 / 298.15));
    return Tk - 273.15;
  }
}

// Definitions for heater PID objects
// Min and max temperatures of the heating elements
#define HEATER_MAX 100
#define HEATER_MIN 0

class Heater {
private:
  // Thermocouple
  ThermistorChannel* tempSensor;
  // Relay for heater
  Relay *heaterRelay;

  // PID Control Loop
  AutoPID *pidLoop;
  double currentTemperature;
  double targetTemperature = 20.0;
  double pidOut;

public:
  Heater(uint8_t tcChannel, Relay *relay, float kp, float ki, float kd) {
    this->tempSensor = new ThermistorChannel(tcChannel);
    this->heaterRelay = relay;

    this->pidLoop = new AutoPID(
      &currentTemperature, &targetTemperature, &pidOut,
      HEATER_MIN, HEATER_MAX, kp, ki, kd);
    pidLoop->setBangBang(2);
    pidLoop->setTimeStep(1000);  // PID only updates when 1000ms has elapsed,
                                 // even when run() is called before 1000ms has passed
  }

  void setTemperature(float temperature) {
    targetTemperature = temperature;
  }

  double getTemperature() {
    return tempSensor->getTemp();
  }

  double getTargetTemperature() {
    return targetTemperature;
  }

  // Iterate on the PID loop
  void updateHeaterPID() {
    // Get current temperature reading
    currentTemperature = tempSensor->getTemp();

    // Iterate PID
    pidLoop->run();

    // Use pid output to update PWM on heater relay
    if (pidOut < 0) {
      heaterRelay->setPWM(0);
      return;
    }

    heaterRelay->setPWM(pidOut * 120 / HEATER_MAX);  // Calculation from Nis
  }

};

class CellBase {
public:
  Heater *heater;
  UltrasonicDriver *ultrasonic;

  CellBase(uint8_t thermoCoupleChannel, Relay *heaterRelay, float kp, float ki, float kd,
           UltrasonicDriver *ultrasonicRelay) {
    this->heater = new Heater(thermoCoupleChannel, heaterRelay, kp, ki, kd);
    this->ultrasonic = ultrasonicRelay;
  }
};

// Variables for recieving and parsing commands over Serial
String message_from_pc;
char charArray[50];
Vector<String> tokens;

// Define relay boards wired in
// Relay 0-3
#define RELAY_ADDR 0x6D  // Alternate address 0x6C
RelayBoard *quadRelay;
// Relay 4-7
// Remember this is NOT a standard I2C add - it was coded to the quad relay.
#define RELAY_ADDR2 0x6C  // 0x09
RelayBoard *quadRelay2;
// Solid state relays for 110VAC
#define RELAY_SOLIDSTATE 0x0A
RelayBoard *solidStateRelay;
// Relay 8
#define RELAY_ADDR3 0x18
RelayBoard *singleRelay3;

// Create hashtable to map commands to their associated handlers
// Handlers take no args and get other inputs from strtok directly
Hashtable<String, void (*)()> commandHandlers;

// For controlling relays set to turn off at specific times
struct RelayOnTimer {
  Relay *relay;
  uint32_t endTime;
};

Vector<RelayOnTimer *> relaysOnTimers;
RelayOnTimer *relaysOnTimersStorage[6];  // Can store 6 relays on timers

// ====================================
// Define board setup and init components
// ====================================
CellBase *bases[2] = {
  nullptr, nullptr
};

// Reinitialized in setup since RelayBoard are not yet initted
Pump pumps[6] = {
  Pump(quadRelay, 1),
  Pump(quadRelay, 2),
  Pump(quadRelay, 3),
  Pump(quadRelay, 4),
  Pump(quadRelay2, 1),
  Pump(quadRelay2, 2),
};

// ===================================
// Update the LCD with current base setpoints and temperatures
void write_to_lcd() {
  for (int i = 0; i < 2; i++) {
    lcd.setCursor((byte)(i * 8), 0);
    lcd.print(String(i) + ":");
    lcd.print(bases[i]->heater->getTemperature());
    lcd.print(" ");

    lcd.setCursor((byte)(i * 8), 1);
    lcd.print("T:");
    lcd.print(bases[i]->heater->getTargetTemperature());
    lcd.print(" ");
  }
}
// Helper used in command handlers to get handler arguments from the Serial input
template<class T>
T getArg() {
  char *argAsChars = strtok(NULL, " ");
  if (argAsChars == NULL) {
    return -1;
  }

  // Convert the argument based on type
  if (sizeof(T) == sizeof(int)) {
    return atoi(argAsChars);  // Convert to int
  } else if (sizeof(T) == sizeof(float)) {
    return atof(argAsChars);  // Convert to float
  }

  return -1;
}

// ====================================
// Define Command Handlers
// ====================================
void getPumpState() {
  // Get pump number from args
  int pumpNumber = getArg<int>();
  CHECK_ARG(pumpNumber);

  // Check that pump number is valid
  if (pumpNumber < 0 || pumpNumber >= sizeof(pumps)) {
    THROW_BAD_ARGS();
  }

  // Send state over serial
  Serial.println(pumps[pumpNumber].getState());

  FUNCTION_COMPLETE();
}

void enablePump() {
  // Get pump number from args
  int pumpNumber = getArg<int>();
  CHECK_ARG(pumpNumber);

  // Check that pump number is valid
  if (pumpNumber < 0 || pumpNumber >= sizeof(pumps)) {
    THROW_BAD_ARGS();
  }

  // Turn on the pump
  pumps[pumpNumber].enable();

  FUNCTION_COMPLETE();
}

void enablePumpForTime() {
  // Get pump number and time to be on from args
  int pumpNumber = getArg<int>();
  CHECK_ARG(pumpNumber);
  int timeOn = getArg<int>();  // time in ms
  CHECK_ARG(timeOn);

  // Check that pump number is valid
  if (pumpNumber < 0 || pumpNumber >= sizeof(pumps)) {
    THROW_BAD_ARGS();
  }

  // Turn on the pump
  pumps[pumpNumber].enable();

  // Create RelayOnTimer struct for the loop
  RelayOnTimer *relayOnTimer = new RelayOnTimer();
  relayOnTimer->relay = &pumps[pumpNumber];

  noInterrupts();  // Temporarily disable interrupts to safely copy the variable
  relayOnTimer->endTime = longTimer + timeOn;
  interrupts();  // Re-enable interrupts

  // Add relayOnTimer to the list
  relaysOnTimers.push_back(relayOnTimer);

  // FUNCTION_COMPLETE(); // Handled in void loop when timer has expired
}

void disablePump() {
  // Get pump number from args
  int pumpNumber = getArg<int>();
  CHECK_ARG(pumpNumber);

  // Check that pump number is valid
  if (pumpNumber < 0 || pumpNumber >= sizeof(pumps)) {
    THROW_BAD_ARGS();
  }

  // Turn on the pump
  pumps[pumpNumber].disable();

  FUNCTION_COMPLETE();
}

void setBaseTemp() {
  // Get base number and set temperature from args
  int baseNumber = getArg<int>();
  CHECK_ARG(baseNumber);
  int setpoint = getArg<float>();
  CHECK_ARG(setpoint);

  // Check that base number is valid
  if (baseNumber < 0 || baseNumber >= sizeof(bases)) {
    THROW_BAD_ARGS();
  }

  // Set the base's temperature
  bases[baseNumber]->heater->setTemperature(setpoint);

  FUNCTION_COMPLETE();
}

void getBaseTemp() {
  // Get base number from args
  int baseNumber = getArg<int>();
  CHECK_ARG(baseNumber);

  // Check that base number is valid
  if (baseNumber < 0 || baseNumber >= sizeof(bases)) {
    THROW_BAD_ARGS();
  }

  // Set the base's temperature
  Serial.println(bases[baseNumber]->heater->getTemperature());

  FUNCTION_COMPLETE();
}

void enableUltrasonic() {
  // Get base number from args
  int baseNumber = getArg<int>();
  CHECK_ARG(baseNumber);

  // Check that base number is valid
  if (baseNumber < 0 || baseNumber >= sizeof(bases)) {
    THROW_BAD_ARGS();
  }

  // Enable the ultrasonic transducer for the base
  bases[baseNumber]->ultrasonic->enable();

  FUNCTION_COMPLETE();
}

void enableUltrasonicForTime() {
  // Get base number and time to be on from args
  int baseNumber = getArg<int>();
  CHECK_ARG(baseNumber);
  int timeOn = getArg<int>();  // time in ms
  CHECK_ARG(timeOn);

  // Check that pump number is valid
  if (baseNumber < 0 || baseNumber >= sizeof(bases)) {
    THROW_BAD_ARGS();
  }

  // Turn on the pump
  bases[baseNumber]->ultrasonic->enable();

  // Create RelayOnTimer struct for the loop
  RelayOnTimer *relayOnTimer = new RelayOnTimer();
  relayOnTimer->relay = bases[baseNumber]->ultrasonic;

  noInterrupts();  // Temporarily disable interrupts to safely copy the variable
  relayOnTimer->endTime = longTimer + timeOn;
  interrupts();  // Re-enable interrupts

  // Add relayOnTimer to the list
  relaysOnTimers.push_back(relayOnTimer);

  // FUNCTION_COMPLETE(); // Handled in void loop when timer has expired
}

void disableUltrasonic() {
  // Get base number from args
  int baseNumber = getArg<int>();
  CHECK_ARG(baseNumber);

  // Check that base number is valid
  if (baseNumber < 0 || baseNumber >= sizeof(bases)) {
    THROW_BAD_ARGS();
  }

  // Disable the ultrasonic transducer for the base
  bases[baseNumber]->ultrasonic->disable();

  FUNCTION_COMPLETE();
}

void setup() {
  // Init serial and I2C communication
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(250000);  // us

  // Initialize the LCD
  lcd.begin(Wire);                  // Set up the LCD for I2C communication
  lcd.setBacklight(255, 255, 255);  // Set backlight to bright white
  lcd.setContrast(5);               // Set contrast. Lower to 0 for higher contrast.
  lcd.disableSplash();              // Disables the splash screen
  lcd.clear();                      // Clear the display - this moves the cursor to home position as well
  lcd.print("Booting!");

  // Setup vector to store relays on timers
  relaysOnTimers.setStorage(relaysOnTimersStorage);

  // Init board components
  quadRelay = new RelayBoard(new Qwiic_Relay(RELAY_ADDR), 4, 0);
  quadRelay2 = new RelayBoard(new Qwiic_Relay(RELAY_ADDR2), 4, 4);
  solidStateRelay = new RelayBoard(new Qwiic_Relay(RELAY_SOLIDSTATE), 2, 8);
  singleRelay3 = new RelayBoard(new Qwiic_Relay(RELAY_ADDR3), 1, 9);
  bases[0] = new CellBase(0, new Relay(solidStateRelay, 1), 0.12, 0.5, 1, new UltrasonicDriver(quadRelay2, 3));
  bases[1] = new CellBase(1, new Relay(solidStateRelay, 2), 0.12, 0.5, 1, new UltrasonicDriver(quadRelay2, 4));
  pumps[0] = Pump(quadRelay, 1);
  pumps[1] = Pump(quadRelay, 2);
  pumps[2] = Pump(quadRelay, 3);
  pumps[3] = Pump(quadRelay, 4);
  pumps[4] = Pump(quadRelay2, 1);
  pumps[5] = Pump(quadRelay2, 2);

  // Check ADS1015 is working (Board for thermistor ADC over I2C)
  if (!ads.begin()) {
    Serial.println("Error: ADS1015 not found.");
    lcd.print("Error: ADS1015 not found.");
    while (1)
      ;
  }

  // Populate hashtable for handlers
  commandHandlers.put("get_pump_state", getPumpState);
  commandHandlers.put("set_pump_on", enablePump);
  commandHandlers.put("set_pump_off", disablePump);
  commandHandlers.put("set_pump_on_time", enablePumpForTime);

  commandHandlers.put("set_ultrasonic_on", enableUltrasonic);
  commandHandlers.put("set_ultrasonic_on_time", enableUltrasonicForTime);
  commandHandlers.put("set_ultrasonic_off", disableUltrasonic);

  commandHandlers.put("get_base_temp", getBaseTemp);
  commandHandlers.put("set_base_temp", setBaseTemp);

  // Set the targetTemperatures based on the EEPROM
  // read_from_eeprom_setPoints();

  lcd.clear();

  noInterrupts();  // Disable all interrupts while configuring

  TCCR1A = 0;  // Set entire TCCR1A register to 0
  TCCR1B = 0;  // Set entire TCCR1B register to 0
  TCNT1 = 0;   // Reset counter

  OCR1A = 1562;                         // Compare match register (16MHz clock / 1024 prescaler / 10Hz)
  TCCR1B |= (1 << WGM12);               // CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Prescaler 1024
  TIMSK1 |= (1 << OCIE1A);              // Enable timer compare interrupt

  interrupts();  // Enable interrupts

  // Send startup message
  // Used by python client to know a reset happened even if the python did not trigger this
  Serial.println("START");
}

// Interrupt handler for longTimer
ISR(TIMER1_COMPA_vect) {
  // This will not overflow for half a billion years since uint64_t
  longTimer = longTimer + 100;  // 100ms have passed each time the interrupt triggers
}

void loop() {
  // Set up watchdog timer, if loop does not happen within 1s it will reset
  wdt_reset();
  wdt_enable(WDTO_1S);
  delay(50);

  // Make a local copy to avoid reading while being updated by ISR
  uint64_t currentCount;
  noInterrupts();  // Temporarily disable interrupts to safely copy the variable
  currentCount = longTimer;
  interrupts();  // Re-enable interrupts

  // Run the heater pid loop for all bases
  for (CellBase *base : bases) {
    base->heater->updateHeaterPID();
  }

  // Update the LCD display
  write_to_lcd();

  // Handle relays on timer if timer is complete
  // Interrupt isn't really necessary for this but could be added
  for (int i = 0; i < relaysOnTimers.size(); i++) {
    RelayOnTimer *relayOnTimer = (relaysOnTimers[i]);
    // Handle millis int overflow to not have to reset every 49 days
    if (relayOnTimer->endTime <= longTimer) {
      // Turn relay off
      relayOnTimer->relay->disable();

      // Pop RelayOnTimer off list
      relaysOnTimers.remove(i);

      FUNCTION_COMPLETE();  // Provide response code for end of relay on timer handler
    }
  }

  // If there are relays on timers ignore all other instructions
  // This maintains the sync for Python scripting because wrapper will wait until response code is recieved
  if (relaysOnTimers.size() == 0) {
    // Get next command from pc
    while (Serial.available()) {
      char inputChar = Serial.read();
      if (inputChar != '\n')  // End of line
      {
        message_from_pc += inputChar;  // Add character to message
      } else {
        // Get char* so strtok can be used to tokenize the command
        message_from_pc.toCharArray(charArray, sizeof(charArray));
        // Get the first token (word)
        char *token = strtok(charArray, " ");

        // Call the associated handler with the token name
        if (commandHandlers.containsKey((String)token)) {
          (*commandHandlers.get((String)token))();
        }

        // Clear the command
        message_from_pc = "";
      }
    }
  }
}