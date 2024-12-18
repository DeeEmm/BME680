/* I2CDemo.ino

Description
---------------------------------------------------------------------------------------------------


Example program for using I2C to set and read the Bosch BME680 sensor. The sensor measures
temperature, pressure and humidity and is described at
https://www.bosch-sensortec.com/bst/products/all_products/BME680. The datasheet is available from
Bosch at https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680_DS001-11.pdf \n\n


This library is forked from Zanduino and ported to work with the ESP32 as part of the DIY-Flow-Bench 
project, but should also work on other ESP32 Based controllers.

The original version of the BME680 library is available at https://github.com/Zanduino/BME680
and the documentation of the library as well as example programs are described in the project's wiki
pages located at https://github.com/Zanduino/BME680/wiki. \n\n

This example program initializes the BME680 to use I2C for communications. The library does not
using floating point numbers to save on memory space and computation time. The values for
Temperature, Pressure and Humidity are returned in deci-units, e.g. a Temperature reading of "2731"
means "27.31" degrees Celsius. The display in the example program uses floating point for
demonstration purposes only.  Note that the temperature reading is generally higher than the ambient
temperature due to die and PCB temperature and self-heating of the element.\n\n

The pressure reading needs to be adjusted for altitude to get the adjusted pressure reading. There
are numerous sources on the internet for formulae converting from standard sea-level pressure to
altitude, see the data sheet for the BME180 on page 16 of
http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf. Rather than put a floating-point
function in the library which may not be used but which would use space, an example altitude
computation function has been added to this example program to show how it might be done.

License
---------------------------------------------------------------------------------------------------


This program is free software: you can redistribute it and/or modify it under the terms of the GNU
General Public License as published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version. This program is distributed in the hope that it will
be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should have
received a copy of the GNU General Public License along with this program.  If not, see
<http://www.gnu.org/licenses/>.


Author
---------------------------------------------------------------------------------------------------


Written by Arnd <Arnd@Zanduino.Com> at https://www.github.com/SV-Zanshin
Ported for ESP32 by DeeEmm deeemm@deeemm.com> at https://www.github.com/DeeEmm


Version Info
---------------------------------------------------------------------------------------------------
See changelog


*/

#include <Arduino.h>
#include "DeeEmm_BME680.h"  // Include the BME680 Sensor library


// Declare constants
const uint32_t SERIAL_SPEED{115200};  ///< Set the baud rate for Serial I/O
#define I2C_ADDRESS 0x77


// Create BME680 class instance
BME680_Class BME680;

//Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  



void setup(void) {
  Serial.begin(SERIAL_SPEED);  

  Serial.print(F("Starting I2CDemo example program for BME680\n"));
  Serial.print(F("- Initializing BME680 sensor\n"));

  while (!BME680.begin(I2C_STANDARD_MODE, I2C_ADDRESS)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  

  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values

  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values

  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  // BME680.setGas(320, 150);  // 320�c for 150 milliseconds
  BME680.setGas(0, 0); // Turns off gas measurements
}  




void loop() {
  static int32_t  temp, humidity, pressure, gas;  // BME readings
  static char     buf[16];                        // sprintf text buffer
  static float    alt;                            // Temporary variable
  static uint16_t loopCounter = 0;                // Display iterations

  if (loopCounter % 25 == 0) {                    // Show header @25 loops
    Serial.print(F("\nLoop Temp\xC2\xB0\x43 Humid% Press hPa   Alt m Air m"));
    Serial.print(F("\xE2\x84\xA6\n==== ====== ====== ========= ======= ======\n"));  // "�C" symbol
  }                                                     

  BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings

  if (loopCounter++ != 0) { 
    
    // Temp in decidegrees
    sprintf(buf, "%4d %3d.%02d", (loopCounter - 1) % 9999, (int8_t)(temp / 100), (uint8_t)(temp % 100));   
    Serial.print(buf);

    // Humidity milli-pct
    sprintf(buf, "%3d.%03d", (int8_t)(humidity / 1000), (uint16_t)(humidity % 1000)); 
    Serial.print(buf);

    // Pressure Pascals
    sprintf(buf, "%7d.%02d", (int16_t)(pressure / 100), (uint8_t)(pressure % 100));  
    Serial.print(buf);

    // temp altitude
    alt = altitude(pressure);                                                

    // Altitude meters
    sprintf(buf, "%5d.%02d", (int16_t)(alt), ((uint8_t)(alt * 100) % 100));  
    Serial.print(buf);

    // Resistance milliohms
    sprintf(buf, "%4d.%02d\n", (int16_t)(gas / 100), (uint8_t)(gas % 100));  
    Serial.print(buf);

    delay(10000); 
  }               
}
