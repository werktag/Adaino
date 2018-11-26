/**
 * single_lowspeed.ino
 * 
 * Adaino example for sampling the analog input using the single conversion 
 * mode at a software controlled rate. In this example, we use a sampling period of 800 us.
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
  if (time_for_next_trigger()) {
    // The sampling period expired and it is time for starting the next 
    // conversion
    AnalogIn.startConversion();
  }

  // Read the analog input.
  int analogResult = AnalogIn.readInput();
  
  if (analogResult >= 0) {
    // You have read a valid result and can sent it to the serial output.
    Serial.println(analogResult);
  } 
}


unsigned long lastTime = 0;
unsigned long currentTime;
unsigned long samplingPeriod = 800; // sampling period in us

// time_for_next_trigger() returns true if the sampling period has expired and 
// a new conversion can be started.
bool time_for_next_trigger() {
    currentTime = micros();
    if (currentTime - lastTime >= samplingPeriod) {
        lastTime = currentTime;
        return true;
    }
    return false;
}