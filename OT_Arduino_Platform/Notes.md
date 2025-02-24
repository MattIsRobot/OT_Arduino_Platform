# Arduino code initial version analysis

## Definitions
AutoPID.h
AutoPID PID0/1 PID controllers for thermocouple + heater system

## Setup

1. Begin Serial connection at 115200 baud
2. Begin I2C communication setip with clock at 100000 (Wire.h) (Speed for standard mode)
3. LCD config
4. Define relays
    1. quadRelay
    2. quadRelay2
    3. solidStateRelay
    4. singleRelay3
5. Initialize temperature sensors
    SparkFun_MCP9600.h
    1. Define addresses
    2. Check connected status
    3. Check device ID
6. Initialize PID controls (PID0, PID1)
    1. setBangBang(2)
    2. setTimeStep(1200)

## Loop

Once every second
1. Manage heaters
    1. Read the temperature
    2. Update PID Controllers
        DC is directly settable for Qwiic_Relay obj?
        setSlowPWM
    3. Update LCDs with temperature values
2. Get commands from Serial connection
    1. Read char from serial
        1. Proceed when char is the end_marker
        2. If not end_marker add char to buffer as the cmd
    2. Parse data from pc
        1. Get the first token of the cmd buffer
        2. Switch-case based on command used, enter command-specific code
            1. Parse command-specific tokens from cmd
            2. Handle command
            * *Temperature setpoint0/1
                Written to EEPROM


# Component needs
## Thermocouple
* Read temperature at thermocouple
* Read ambient temperature

## 


# Serial commands OLD
* set_relay_on_time INT_RELAYID INT_DELAYMS
* set_relay_on INT_RELAYID
* set_relay_off INT_RELAYID

* read_temp0
* read_temp1
* read_temp0_ambient
* read_temp1_ambient
* setpoint0 INT_TEMP
* setpoint1 INT_TEMP

* get_relay_0_state
* get_relay_1_state
* get_relay_2_state
* get_relay_3_state
* get_relay_4_state
<!-- * get_relay_5_state
* get_relay_6_state
* get_relay_7_state
* get_relay_8_state -->
* get_relay_9_state