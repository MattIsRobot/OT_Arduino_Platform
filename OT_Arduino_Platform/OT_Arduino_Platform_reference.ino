// /*
//   Software written by
//   Nis Fisker-Bødker
//   nis.fisker@gmail.com
//   2024 at University of Toronto

//   Components used:
//   Sparkfun Arduino Redboard with Qwiic
//   2 x SparFun MCP9600 (for temperature measurements in liquids)
//   1 x SparkFun Dual Solid State Relay (for heat control)
//   1 x Sparkfun Quad Relay (for control of peristaltic pumps)
//   1 x Sparkfun SerLCD
//   1 x Sparkfun single relay (for control of electrode connection)

// */
// #include <Wire.h>
// #include <EEPROM.h>
// const byte buff_size = 40;
// char input_buffer[buff_size];
// const char start_marker = '<';
// const char end_marker = '>';
// byte bytes_rcvd = 0;
// boolean read_in_progress = false;
// boolean new_data_pc = false;
// String message_from_pc;
// int direction_in = 0;
// int steps_in = 0;
// int step_time_in = 0;
// int m_num = 0;
// float ultrasound_time_in;


// /*
//    Load temperature sensor
// */
// #include <SparkFun_MCP9600.h>
// MCP9600 tempSensor0;
// float sample_temperature0;
// float ambient_temperature0;
// MCP9600 tempSensor1;
// float sample_temperature1;
// float ambient_temperature1;

// /*
//    Load LCD
// */
// #include <SerLCD.h>
// SerLCD lcd;  // Initialize the library with default I2C address 0x72


// /*
//    Load relays
// */
// // Relay 0-3
// #include <SparkFun_Qwiic_Relay.h>
// #define RELAY_ADDR 0x6D  // Alternate address 0x6C
// Qwiic_Relay quadRelay(RELAY_ADDR);

// // Solid state relays for 110VAC
// #define RELAY_SOLIDSTATE 0x0A
// Qwiic_Relay solidStateRelay(RELAY_SOLIDSTATE);

// // Relay 4-7
// // Remember this is NOT a standard I2C add - it was coded to the quad relay.
// #define RELAY_ADDR2 0x6C  // 0x09
// Qwiic_Relay quadRelay2(RELAY_ADDR2);

// // Relay 8
// #define RELAY_ADDR3 0x18
// Qwiic_Relay singleRelay3(RELAY_ADDR3);

// /*
//    Load PID control
// */
// #include <AutoPID.h>
// #define OUTPUT_MIN 0
// #define OUTPUT_MAX 255
// #define KP0 .12
// #define KI0 0.5
// #define KD0 1
// #define KP1 .12
// #define KI1 0.5
// #define KD1 1
// double temperature0, outputVal0;
// double temperature1, outputVal1;
// double setPoint0;
// double setPoint1;
// AutoPID PID0(&temperature0, &setPoint0, &outputVal0, OUTPUT_MIN, OUTPUT_MAX, KP0, KI0, KD0);
// AutoPID PID1(&temperature1, &setPoint1, &outputVal1, OUTPUT_MIN, OUTPUT_MAX, KP1, KI1, KD1);
// unsigned long lastTempUpdate;  //tracks clock time of last temp update
// int dutyCycle0 = 0;            //Tracks how much the relay is on at 1Hz. 120 = 100%.
// int dutyCycle1 = 0;            //Tracks how much the relay is on at 1Hz. 120 = 100%.

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();
//   Wire.setClock(100000);


//   /*
//      Setup LCD
//   */
//   // Serial.println("Booting LCD");
//   lcd.begin(Wire);                  //Set up the LCD for I2C communication
//   lcd.setBacklight(255, 255, 255);  //Set backlight to bright white
//   lcd.setContrast(5);               //Set contrast. Lower to 0 for higher contrast.
//   lcd.clear();                      //Clear the display - this moves the cursor to home position as well
//   lcd.print("Booting!");


//   /*
//      Setup relays
//   */
//   if (!quadRelay.begin()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Quad relay not connected");
//     Serial.println("Check connections to Qwiic Relay 0-3.");
//     delay(2000);
//   }
//   if (!solidStateRelay.begin()) {
//     lcd.setCursor(0, 0);
//     lcd.print("SolStateRelay not connected");
//     Serial.println("Check connections to solid state relay.");
//     delay(2000);
//   }
//   if (!quadRelay2.begin()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Quad relay not connected");
//     Serial.println("Check connections to Qwiic Relay 4-7.");
//     delay(2000);
//   }
//   if (!singleRelay3.begin()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Single relay not connected");
//     Serial.println("Check connections to Qwiic Relay 8.");
//     delay(2000);
//   }

//   /*
//      Setup temperature sensor 0
//   */
//   // Serial.println("Booting temperature sensors");
//   tempSensor0.begin(0x60);  // Uses the default address (0x60) for SparkFun Thermocouple Amplifier
//   tempSensor1.begin(0x67);

//   // Check if temperature sensor is connected
//   // Serial.println("Check if temperature sensor 0 is online");
//   if (!tempSensor0.isConnected()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Temp 0 not connected");
//     Serial.println("Temperature sensor 0 not connected");
//     delay(10000);
//   }

//   // Check if the temperature sensor address is correct
//   // Serial.println("Check temperature sensor 0 address");
//   if (!tempSensor0.checkDeviceID()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Temp 0 addr. is wrong");
//     Serial.println("Temperature sensor 0 address is wrong");
//     delay(10000);
//   }


//   /*
//      Setup temperature sensor 1
//   */
//   // Check if temperature sensor 1 is connected
//   // Serial.println("Check if temperature sensor 1 is online");
//   if (!tempSensor1.isConnected()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Temp 1 not connected");
//     Serial.println("Temperature sensor 1 not connected");
//     delay(10000);
//   }

//   // Check if the temperature sensor address is correct
//   // Serial.println("Check temperature sensor 1 address");
//   if (!tempSensor1.checkDeviceID()) {
//     lcd.setCursor(0, 0);
//     lcd.print("Temp 1 addr. is wrong");
//     Serial.println("Temperature sensor 1 address is wrong");
//     delay(10000);
//   }



//   /*
//      Setup PID control
//   */
//   read_from_eeprom_setPoints();

//   // if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
//   PID0.setBangBang(2);
//   PID1.setBangBang(2);
//   //set PID update interval to 1200ms
//   PID0.setTimeStep(1200);
//   PID1.setTimeStep(1200);

//   lcd.clear();
// }

// void loop() {
//   // Updating the sensor values
//   delay(1000);
//   sample_temperature0 = read_sample_temperature(0);
//   sample_temperature1 = read_sample_temperature(1);



//   // Update PID controller 0
//   temperature0 = sample_temperature0;
//   PID0.run();
//   if (outputVal0 > 0) {
//     dutyCycle0 = (outputVal0 / 255) * 120;
//   } else {
//     dutyCycle0 = 0;
//   }
//   solidStateRelay.setSlowPWM(1, dutyCycle0);

//   // Update PID controller 1
//   temperature1 = sample_temperature1;
//   PID1.run();
//   if (outputVal1 > 0) {
//     dutyCycle1 = (outputVal1 / 255) * 120;
//   } else {
//     dutyCycle1 = 0;
//   }
//   solidStateRelay.setSlowPWM(2, dutyCycle1);

//   // Write the sensor values to the LCD
//   write_to_lcd(sample_temperature0, sample_temperature1, dutyCycle0, dutyCycle1);


//   // Check if there is instructions from the PC
//   get_data_from_pc();
// }

// void get_data_from_pc() {

//   // receive data from PC and save it into input_buffer
//   while (Serial.available()) {
//     char x = Serial.read();

//     // the order of these IF clauses is significant

//     if (x == end_marker) {
//       read_in_progress = false;
//       new_data_pc = true;
//       input_buffer[bytes_rcvd] = 0;
//       parse_data();
//     }

//     if (read_in_progress) {
//       input_buffer[bytes_rcvd] = x;
//       bytes_rcvd++;
//       if (bytes_rcvd == buff_size) {
//         bytes_rcvd = buff_size - 1;
//       }
//     }

//     if (x == start_marker) {
//       bytes_rcvd = 0;
//       read_in_progress = true;
//     }
//   }
// }

// void parse_data() {
//   // split the data into its parts
//   char *str_to_ind;  // this is used by strtok() as an index

//   str_to_ind = strtok(input_buffer, ",");  // get the first part - the string
//   message_from_pc = String(str_to_ind);

//   if (message_from_pc == "set_relay_on_time") {
//     str_to_ind = strtok(NULL, ",");    // this continues where the previous call left off
//     int relay_num = atoi(str_to_ind);  // convert this part to an integer

//     str_to_ind = strtok(NULL, ",");
//     int time_ms = atoi(str_to_ind);

//     if (0 <= relay_num && relay_num <= 3)  // First quad relay
//     {
//       int relay = relay_num + 1;
//       quadRelay.turnRelayOn(relay);
//       delay(time_ms);
//       quadRelay.turnRelayOff(relay);
//     }
//     if (4 <= relay_num && relay_num <= 7)  // Second quad relay
//     {
//       int relay = relay_num - 3;
//       quadRelay2.turnRelayOn(relay);
//       delay(time_ms);
//       quadRelay2.turnRelayOff(relay);
//     }
//     if (8 == relay_num)  // Single relay
//     {
//       singleRelay3.turnRelayOn();
//       delay(time_ms);
//       singleRelay3.turnRelayOff();
//     }
//     Serial.println("#");  // '#' indicates process is done
//   }

//   if (message_from_pc == "set_relay_on") {
//     str_to_ind = strtok(NULL, ",");    // this continues where the previous call left off
//     int relay_num = atoi(str_to_ind);  // convert this part to an integer

//     if (0 <= relay_num && relay_num <= 3)  // First quad relay
//     {
//       int relay = relay_num + 1;
//       quadRelay.turnRelayOn(relay);
//     }
//     if (4 <= relay_num && relay_num <= 7)  // Second quad relay
//     {
//       int relay = relay_num - 3;
//       quadRelay2.turnRelayOn(relay);
//     }
//     if (8 == relay_num)  // Single relay
//     {
//       singleRelay3.turnRelayOn();
//     }
//     Serial.println("#");  // '#' indicates process is done
//   }

//   if (message_from_pc == "set_relay_off") {
//     str_to_ind = strtok(NULL, ",");    // this continues where the previous call left off
//     int relay_num = atoi(str_to_ind);  // convert this part to an integer

//     if (0 <= relay_num && relay_num <= 3)  // First quad relay
//     {
//       int relay = relay_num + 1;
//       quadRelay.turnRelayOff(relay);
//     }
//     if (4 <= relay_num && relay_num <= 7)  // Second quad relay
//     {
//       int relay = relay_num - 3;
//       quadRelay2.turnRelayOff(relay);
//     }
//     if (8 == relay_num)  // Single relay
//     {
//       singleRelay3.turnRelayOff();
//     }
//     Serial.println("#");  // '#' indicates process is done
//   }


//   if (message_from_pc == "read_temp0") {
//     Serial.println(sample_temperature0);
//   }
//   if (message_from_pc == "read_temp1") {
//     Serial.println(sample_temperature1);
//   }
//   if (message_from_pc == "read_temp0_ambient") {
//     Serial.println(ambient_temperature0);
//   }
//   if (message_from_pc == "read_temp1_ambient") {
//     Serial.println(ambient_temperature1);
//   }
//   if (message_from_pc == "setpoint0") {
//     str_to_ind = strtok(NULL, ",");  // this continues where the previous call left off
//     setPoint0 = atoi(str_to_ind);
//     setPoint0 = setPoint0 / 10;
//     write_to_eeprom_setPoint0(setPoint0);
//   }
//   if (message_from_pc == "setpoint1") {
//     str_to_ind = strtok(NULL, ",");  // this continues where the previous call left off
//     setPoint1 = atoi(str_to_ind);
//     setPoint1 = setPoint1 / 10;
//     write_to_eeprom_setPoint1(setPoint1);
//   }
//   // ... in the future

//   if (message_from_pc == "get_relay_0_state") {
//     int state = quadRelay.getState(1);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_1_state") {
//     int state = quadRelay.getState(2);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_2_state") {
//     int state = quadRelay.getState(3);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_3_state") {
//     int state = quadRelay.getState(4);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_4_state") {
//     int state = quadRelay2.getState(1);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_5_state") {
//     int state = quadRelay2.getState(2);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_6_state") {
//     int state = quadRelay2.getState(3);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_7_state") {
//     int state = quadRelay2.getState(4);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_8_state") {
//     int state = solidStateRelay.getState(1);
//     Serial.println(state);  // 0 = off, 1 = on
//   }
//   if (message_from_pc == "get_relay_9_state") {
//     int state = solidStateRelay.getState(2);
//     Serial.println(state);  // 0 = off, 1 = on
//   }

//   else {
//     Serial.println("Unknown command: " + message_from_pc);
//   }
// }

// void ultrasound(float ultrasound_time) {
//   // flip relay on
//   delay(ultrasound_time);
//   // Flip relay off
//   Serial.println("#");  // '#' indicates process is done
// }

// void drain(float drain_time) {
//   // Do something
//   Serial.println("#");  // '#' indicates process is done
// }

// void transfer_liquid(float drain_time) {
//   // Do something
//   Serial.println("#");  // '#' indicates process is done
// }

// float read_sample_temperature(int sensor) {
//   // There are two sensors on this system.
//   // Read each of them dependen on the input
//   float sample_temp = 999.99;
//   if (sensor == 0) {
//     if (tempSensor0.available()) {
//       sample_temp = tempSensor0.getThermocoupleTemp();
//     }
//   }
//   if (sensor == 1) {
//     if (tempSensor1.available()) {
//       sample_temp = tempSensor1.getThermocoupleTemp();
//     }
//   }

//   return sample_temp;
// }

// float read_ambient_temperature(int sensor) {
//   // There are two sensors on this system.
//   // Read each of them dependen on the input
//   float ambient_temp = 999.99;
//   if (sensor == 0) {
//     if (tempSensor0.available()) {
//       ambient_temp = tempSensor0.getAmbientTemp();
//     }
//   }
//   if (sensor == 1) {
//     if (tempSensor1.available()) {
//       ambient_temp = tempSensor1.getAmbientTemp();
//     }
//   }

//   return ambient_temp;
// }

// void write_to_lcd(float sample_temp0, float sample_temp1, double PIDoutput0, double PIDoutput1) {
//   lcd.setCursor(0, 0);
//   lcd.print("0:");
//   lcd.print(sample_temp0);
//   lcd.print(" ");
//   lcd.setCursor(9, 0);
//   lcd.print("1:");
//   lcd.print(sample_temp1);
//   lcd.print(" ");
//   lcd.setCursor(0, 1);
//   lcd.print("P0:");
//   lcd.print(round(PIDoutput0));
//   lcd.print(" ");
//   lcd.setCursor(8, 1);
//   lcd.print("P1:");
//   lcd.print(round(PIDoutput1));
//   lcd.print(" ");
// }

// void read_from_eeprom_setPoints() {
//   // Read setpoints from EEPROM
//   int setPoint0_int;
//   int setPoint1_int;
//   EEPROM.get(0, setPoint0_int);
//   EEPROM.get(4, setPoint1_int);

//   // Divide setPoint0 and setPoint1 by 10 to get the decimal point back
//   setPoint0 = setPoint0_int / 10;
//   setPoint1 = setPoint1_int / 10;
// }

// void write_to_eeprom_setPoint0(float setPoint0_tmp) {
//   // Multiply setPoint0 by 10 to get rid of the decimal point
//   int setPoint0_int = int(setPoint0_tmp * 10);

//   // Check that number is between 0-255 to fill only one byte
//   if (setPoint0_int > 255) {
//     setPoint0_int = 255;
//   }
//   if (setPoint0_int < 0) {
//     setPoint0_int = 0;
//   }

//   // Write setpoints to EEPROM
//   EEPROM.put(0, setPoint0_int);
// }

// void write_to_eeprom_setPoint1(float setPoint1_tmp) {
//   // Multiply setPoint1 by 10 to get rid of the decimal point
//   int setPoint1_int = int(setPoint1_tmp * 10);

//   // Check that number is between 0-255 to fill only one byte
//   if (setPoint1_int > 255) {
//     setPoint1_int = 255;
//   }
//   if (setPoint1_int < 0) {
//     setPoint1_int = 0;
//   }

//   // Write setpoints to EEPROM
//   EEPROM.put(4, setPoint1_int);
// }
