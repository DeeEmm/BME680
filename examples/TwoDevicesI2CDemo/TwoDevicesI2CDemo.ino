/*! @file TwoDevicesI2CDemo.ino

@section TwoDevicesI2CDemo_intro_section Description

Example program for using I2C to set and read data from 2 Bosch BME680 sensors. The sensors measure temperature, 
pressure, humidity and air quality and is described at https://www.bosch-sensortec.com/bst/products/all_products/BME680. 
The datasheet is available from Bosch at https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680_DS001-11.pdf \n\n

The most recent version of the BME680 library is available at https://github.com/SV-Zanshin/BME680 and the 
documentation of the library as well as example programs are described in the project's wiki pages located at 
https://github.com/SV-Zanshin/BME680/wiki. \n\n

The BME680 is an extremely small physical package that is so tiny as to be impossible to solder at home, hence it 
will be used as part of a third-party breakout board. There are several such boards available at this time, for 
example \n
Company  | Link
-------  | ----------
Sparkfun | https://www.sparkfun.com/products/14570
BlueDot  | https://www.bluedot.space/sensor-boards/bme680/
Adafruit | https://learn.adafruit.com/adafruit-BME680-humidity-barometric-pressure-temperature-sensor-breakout \n\n

Bosch supplies sample software that runs on various platforms, including the Arduino family; this can be downloaed
at https://github.com/BoschSensortec/BSEC-Arduino-library . This software is part of the Bosch "BSEC" (Bosch 
Sensortec Environmental Cluster) framework and somewhat bulky and unwieldy for typical Arduino applications, hence
the choice to make a more compact and rather less abstract library. 
 
This example program explicitly initializes each BME680 - one at address 0x76 and the other at 0x77. The BME680s are
initialized in separate library instances. The library does not using floating point numbers to save on memory space
and computation time. The values for Temperature, Pressure and Humidity are returned in deci-units, e.g. a 
Temperature reading of "2731" means "27.31" degrees Celsius.\n
The program measures only the temperature and humidity on the two devices and outputs the values.
\n\n

@section TwoDevicesI2CDemolicense License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details. You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@section TwoDevicesI2CDemoauthor Author

Written by Arnd\@SV-Zanshin

@section TwoDevicesI2CDemoversions Changelog

Version | Date       | Developer                     | Comments
------- | ---------- | ----------------------------- | ----------------------------------------------
1.0.0   | 2020-05-16 | https://github.com/SV-Zanshin | Issue #11. Cloned and adapted from I2CDemo.ino
*/
#include "Zanshin_BME680.h" // Include the BME680 Sensor library
/*******************************************************************************************************************
** Declare all program constants                                                                                  **
*******************************************************************************************************************/
const uint32_t SERIAL_SPEED = 115200; ///< Set the baud rate for Serial I/O

/*******************************************************************************************************************
** Declare global variables and instantiate classes                                                               **
*******************************************************************************************************************/
BME680_Class BME680_1; ///< Create an instance of the BME680 class for the first device
BME680_Class BME680_2; ///< Create an instance of the BME680 class for the second device

void setup()
{
  /*!
  @brief    Arduino method called once at startup to initialize the system
  @details  This is an Arduino IDE method which is called first upon boot or restart. It is only called one time
            and then control goes to the main "loop()" method, from which control never returns
  @return   void
  */
  Serial.begin(SERIAL_SPEED); // Start serial port at Baud rate
  #ifdef  __AVR_ATmega32U4__  // If this is a 32U4 processor, then wait 3 seconds to initialize USB port
    delay(3000);
  #endif
  Serial.print(F("Starting TwoDevicesI2CDemo example program for BME680\n"));
  Serial.print(F("- Initializing BME680 sensor 1 at address 0x76\n"));
  while (!BME680_1.begin(I2C_STANDARD_MODE,0x76)) // Start BME680 using I2C, use device at address 0x76
  {
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  } // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680_1.setOversampling(TemperatureSensor,Oversample16);    // Use enumerated type values
  BME680_1.setOversampling(HumiditySensor,   Oversample16);    // Use enumerated type values
  BME680_1.setOversampling(PressureSensor,   SensorOff);       // Use enumerated type value to turn off
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680_1.setIIRFilter(IIR4);                                 // Use enumerated type values
  Serial.print(F("- Turning off pressure and gas sensors\n")); // "�C" symbols
  BME680_1.setGas(0,0);                                        // If either Temp/Time is zero then turn off
  Serial.print(F("\n\n- Initializing BME680 sensor 2 at address 0x77\n"));
  while (!BME680_2.begin(I2C_STANDARD_MODE,0x77)) // Start BME680 using I2C, use device at address 0x76
  {
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  } // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680_2.setOversampling(TemperatureSensor,Oversample16);    // Use enumerated type values
  BME680_2.setOversampling(HumiditySensor,   Oversample16);    // Use enumerated type values
  BME680_2.setOversampling(PressureSensor,   SensorOff);       // Use enumerated type value to turn off
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680_2.setIIRFilter(IIR4);                                 // Use enumerated type values
  Serial.print(F("- Turning off pressure and gas sensors\n")); // "�C" symbols
  BME680_2.setGas(0,0);                                        // If either Temp/Time is zero then turn off
} // of method setup()
void loop() 
{
  /*!
  @brief    Arduino method for the main program loop
  @details  This is the main program for the Arduino IDE, it is an infinite loop and keeps on repeating. 
            The "sprintf()" function is to pretty-print the values, since floating point is not supported on the 
            Arduino, split the values into those before and those after the decimal point.
  @return   void
  */
  static int32_t  temp1, humidity1, pressure1, gas1;                                 // Variables for BME680 1
  static int32_t  temp2, humidity2, pressure2, gas2;                                 // Variables for BME680 1
  static char     buf[16];                                                           // Text buffer for sprintf
  static uint16_t loopCounter = 0;                                                   // Display iterations
  if (loopCounter % 25 == 0)                                                         // Display header every 25 loops
  {                                                                                  //
    Serial.print(F("\nLoop Temp1\xC2\xB0\x43 Humid1% | Temp2\xC2\xB0\x43 Humid2%")); // Show header plus unicode "�C"
    Serial.print(F("\n==== ======= ======= | ======= =======\n"));                   // symbol
  } // if-then time to show headers                                                  //
  BME680_1.getSensorData(temp1,humidity1,pressure1,gas1);                            // Get the most recent readings
  BME680_2.getSensorData(temp2,humidity2,pressure2,gas2);                            // Get the most recent readings
  sprintf(buf, "%4d %4d.%02d", ++loopCounter%9999,                                   // Clamp iterations to 9999,
          (int8_t)(temp1/100),(uint8_t)(temp1%100));                                 // Temperature in decidegrees
  Serial.print(buf);                                                                 //
  sprintf(buf, "%4d.%03d", (int8_t)(humidity1/1000),(uint16_t)(humidity1%1000));     // Humidity in milli-percent
  Serial.print(buf);                                                                 //
  sprintf(buf, " | %4d.%02d", (int8_t)(temp2/100),(uint8_t)(temp2%100));             // Temperature in decidegrees
  Serial.print(buf);                                                                 //
  sprintf(buf, "%4d.%03d\n", (int8_t)(humidity1/1000),(uint16_t)(humidity1%1000));   // Humidity in milli-percent
  Serial.print(buf);                                                                 //
  delay(10000);                                                                      // Wait 10s before repeating
} // of method loop()