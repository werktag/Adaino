/**
 * resolution.ino
 * 
 * This is a more advance example for setting the resolution of the converted
 * result.
 * 
 * When converting an analog input to the digital domain, the digital result 
 * can only be represented by a limited number of discrete values. This number 
 * is called the resolution and usually expressed in the number of bits 
 * required to store the value.
 *
 * Analog-to-Digital Converters (ADCs) can often represent the result as a 8-,
 * 10- or 12-bit value. In case of a 8-bit value, this means that an analog 
 * input of 0 V is converted to integer value 0 while the maximum analog input 
 * voltage (the analog reference voltage) is converted to 255 (2^8-1). If the 
 * analog reference voltage is set 3.3 V, this means that the voltage 
 * resolution (also called the least significant bit / LSB voltage) of the ADC 
 * is 0.013 V (3.3 V / 255).
 * 
 * The below table summarizes the resolution numbers for 8-, 10- and 12-bit 
 * resolutions.
 * 
 *  +------------+-------------------+-------------+
 *  | resolution | max input voltage | LSB voltage |
 *  +------------+-------------------+-------------+
 *  |      8-bit |               255 |    0.0129 V |
 *  |     10-bit |              1023 | 0.0032 V    |
 *  |     12-bit |              4095 | 0.0008 V    |
 *  +------------+-------------------+-------------+
 * 
 * 
 * The circuit:
 * 
 *   * An Arduino device with an SAMD21 microcontroller (e.g. Arduino MKR * or 
 *     Adafruit Feather M0).
 *   * Connect an analog signal to Arduino's analog input pin A1. The signals 
 *     voltage is expected to be between 0 and 3.3 V.
 * 
 * 
 * Created by Andre Meyer <andmeyer@werktag.io>
 * 
 */

#include <Arduino.h>
#include <Adaino.h>

void setup()
{
  // Setup serial port and wait for connection
  Serial.begin(115200);
  while (!Serial) ;
  
  // Print example information to the serial output
  Serial.println("[ADAINO EXAMPLE]");
  Serial.println("resolution.ino");
  Serial.println("[ADAINO DATA]");

  // In this example, you sample the analog inputs with the same settings as 
  // used in the 'ada.ino' example.
  AnalogIn.setAnalogInput(A1);
  AnalogIn.useFreerunMode();
  AnalogIn.enableAcqBuffer(); 

  // Set the resolution. Adaino supports resolutions of 8-, 10- and 12-bits.
  AnalogIn.setResolution(12);

  // Start the analog data acquisition
  AnalogIn.begin();
}

void loop()
{
  // Read the analog input and send it to the serial output
  int analogResult = AnalogIn.readInput();
  Serial.println(analogResult); 
}

