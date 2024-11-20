/*! @file SPIDemo.ino

@section SPIDemo_intro_section Description

Example program for using hardware SPI to set and read the Bosch BME680 sensor. The sensor measures
temperature, pressure and humidity and is described at
https://www.bosch-sensortec.com/bst/products/all_products/BME680. The datasheet is available from
Bosch at https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680_DS001-11.pdf
\n\n

This library is forked from Zanduino and ported to work with the ESP32 as part of the DIY-Flow-Bench 
project, but should also work on other ESP32 Based controllers.

The most recent version of the BME680 library is available at https://github.com/Zanduino/BME680
and the documentation of the library as well as example programs are described in the project's wiki
pages located at https://github.com/Zanduino/BME680/wiki. \n\n

@section SPIDemolicense License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU
General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version. This program is distributed in the hope that it will
be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should have
received a copy of the GNU General Public License along with this program.  If not, see
<http://www.gnu.org/licenses/>.

@section SPIDemoauthor Author

WWritten by Arnd <Arnd@Zanduino.Com> at https://www.github.com/SV-Zanshin
Ported for ESP32 by DeeEmm deeemm@deeemm.com> at https://www.github.com/DeeEmm


Version Info
---------------------------------------------------------------------------------------------------
Version numbers follow format:  Maj . Minor . Build 
Build numbers follow format: YY MM DD VV Where VV is the incremental daily version

!! Most recent entry at top !!

Version #               - Description of Change
---------------------------------------------------------------------------------------------------


V 1.0.24111801          - Forked from https://github.com/Zanduino/BME680 [Version 1.0.3]

*/

#include "DeeEmm_BME680.h"  // Include the BME680 Sensor library
/**************************************************************************************************
** Declare all program constants                                                                 **
**************************************************************************************************/
const uint32_t SERIAL_SPEED = 115200;  ///< Set the baud rate for Serial I/O

/*! The pin used for slave-select can be freely chosen from the digital pins available. This is
    the default pin used on an Arduino Micro */
const uint8_t SPI_CS_PIN = SS;
/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/
BME680_Class BME680;  ///< Create an instance of the BME680

float altitude(const int32_t press, const float seaLevel = 1013.25);  ///< Forward declaration
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used
             (see https://en.wikipedia.org/wiki/Atmospheric_pressure for details)
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()
void setup() {
  /*!
  @brief    Arduino method called once at startup to initialize the system
  @details  This is an Arduino IDE method which is called first upon boot or restart. It is only
            called one time and then control goes to the main "loop()" method, from which control
            never returns
  @return   void
  */
  Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
#ifdef __AVR_ATmega32U4__      // If this is a 32U4 processor, then wait 3 seconds to initialize USB
  delay(3000);
#endif
  Serial.println(F("Starting Hardware SPIDemo example program for BME680"));
  Serial.print(F("- Initializing BME680 sensor\n"));

  while (!BME680.begin(SPI_CS_PIN))  // Start using hardware SPI protocol
  {
    Serial.println(F("-  Unable to find BME680. Waiting 3 seconds."));
    delay(3000);
  }  // of loop until device is located
  Serial.println(F("- Setting 16x oversampling for all sensors"));
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  Serial.println(F("- Setting IIR filter to maximum value of 16 samples"));
  BME680.setIIRFilter(IIR16);
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbol
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds
}  // of method setup()
void loop() {
  /*!
  @brief    Arduino method for the main program loop
  @details  This is the main program for the Arduino IDE, it is an infinite loop
            The "sprintf()" function is to pretty-print the values, since floating point is not
            supported on the Arduino, split the values into those before and those after the
            decimal point.
  @return   void
  */
  static int32_t  temp, humidity, pressure, gas;  // Temp variables
  static char     buf[16];                        // springtf text buffer
  static float    alt;                            // temp altitude
  static uint16_t loopCounter = 0;                // Display iterations
  if (loopCounter % 25 == 0) {                    // Display header
    Serial.print(F("\nLoop Temp\xC2\xB0\x43 Humid% Press hPa   Alt m  Air m"));  // Show header "�C"
    Serial.print(F("\xE2\x84\xA6\n==== ====== ====== ========= ======= =======\n"));
  }                                                      // if-then time to show headers
  BME680.getSensorData(temp, humidity, pressure, gas);   // Get readings
  sprintf(buf, "%4d %3d.%02d", ++loopCounter % 9999,     // Clamp to 9999,
          (int8_t)(temp / 100), (uint8_t)(temp % 100));  // Temp decidegrees
  Serial.print(buf);
  sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000),
          (uint16_t)(humidity % 1000));  // Humidity milli-%
  Serial.print(buf);
  sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100),
          (uint8_t)(pressure % 100));  // Pressure Pascals
  Serial.print(buf);
  alt = altitude(pressure);                                                // temp altitude
  sprintf(buf, "%5d.%02d", (int16_t)(alt), ((uint8_t)(alt * 100) % 100));  // Altitude in meters
  Serial.print(buf);
  sprintf(buf, "%5d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  // Resistance milliohms
  Serial.print(buf);
  delay(10000);  // Wait 10s before repeating
}  // of method loop()
