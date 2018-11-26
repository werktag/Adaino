/**
 * single_fullspeed.ino
 * 
 * Adaino example for sampling the analog input using the single conversion 
 * mode at the maximum sampling rate possible. In this mode, the acquisition 
 * timing is fully controlled by software.
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
  Serial.println("single_fullspeed.ino");
  Serial.println("[ADAINO DATA]");

  // Set the analog input pin to be sampled
  AnalogIn.setAnalogInput(A1);
  
  // Use single conversion mode
  AnalogIn.useSingleMode();

  // Set the timeout to zero. This makes the readInput() method non-blocking 
  // and you can transfer the result to the serial output while the next 
  // conversion is ongoing.
  AnalogIn.setTimeout(0);
  
  // Disable auto-trigger
  AnalogIn.disableAutoTrigger();

  // Enable the acquisition buffer to have a more relaxed timing behavior for 
  // processing the data, i.e. sending it over the serial port.
  AnalogIn.enableAcqBuffer();
  
  // Start analog data acquisition
  AnalogIn.begin();
  
  // Start analog conversion of first sample
  AnalogIn.startConversion();
}


void loop()
{
  // Read the analog input.
  int analogResult = AnalogIn.readInput();
  // As a the timeout is set to 0 and the readInput() method is non-blocking, 
  // you have to check if the result is valid (greater of equal to zero).
  if (analogResult >= 0) {
    // You have read a valid result and can start the next analog conversion.
    AnalogIn.startConversion();
    // While the next sample is converted, you can send the acquired result to 
    // the serial port.
    Serial.println(analogResult);
  } 
}

