/**
 * single.ino
 * 
 * Adaino example for infrequent sampling of the analog input where the exact 
 * sampling time is not of outmost importance. A suitable use case might be the
 * sampling of a temperature sensor (thermistor).
 * 
 * Adaino's functionality used in this example is comparable to Arduino's 
 * built-in analogRead() function as used in the 'arduino.ino' example.
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
  Serial.println("single.ino");
  Serial.println("[ADAINO DATA]");
  
  // set the analog input pin to be sampled
  AnalogIn.setAnalogInput(A1);
  
  // Use the signle conversion mode for infrequenct sampling of an analog 
  // input. In this mode, only a single sample is converted at a time. As we 
  // have enough time to process all the data, we do not have to enable the 
  // acquisition buffer and can keep the corresponding memory resources free 
  // for other needs.

  // By default, AnalogIn is configured to use single conversion mode and no 
  // acquisition buffer. Therefore, you do not have to change any settings here 
  // and can start the analog acquisition immediately.
  AnalogIn.begin();
}


void loop()
{
  // Read the analog input and send it to the serial output
  int analogResult = AnalogIn.readInput();
  Serial.println(analogResult);

  // Simulate the infrequent read of the analog input using a random delay 
  // between 5 and 15 ms.
  delay(random(5, 15));
}

