// **Defines config for OT2 side**
#ifndef BOARDCONFIG
#define BOARDCONFIG

// Relays
#define RELAY_ADDR 0x6D  // Relays 0-3
#define RELAY_ADDR2 0x69  // Relays 4-7
#define RELAY_SOLIDSTATE 0x0A // Solid state relays for 110VAC
#define RELAY_ADDR3 0x18 // Relay 8 NOT USED

// Min and max temperatures of the heating elements
#define HEATER_MAX 100
#define HEATER_MIN 0

// Constants for thermistor decoding
#define RC 10000.0 // Reference resistance in thermistor circuit
#define VCC 3.3 // ADS1015 reference voltage
#define R25 10000.0 // From thermistor datasheet
#define B2585 3977.0 // From thermistor datasheet

// Actuated reactor base
#define HAS_ACUATED_REACTOR false
#define ACTUATED_REACTOR_PIN 13

#endif