/**
 * ada.ino
 * 
 * Basic example showing how to use Adaino, an Analog Data Acquisition library 
 * for Arduino. In this example, we want to sample the analog input at the 
 * highest rate possible without losing any sample. This is required to 
 * reconstruct and process signals with high frequency spectrums. 
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

// Include the Adaino header file to make the libary's functionality available 
// in your sketch
#include <Adaino.h>

void setup()
{
  // Setup serial port and wait for connection
  Serial.begin(115200);
  while (!Serial) ;
  
  // Print example information to the serial output
  Serial.println("[ADAINO EXAMPLE]");
  Serial.println("ada.ino");
  Serial.println("[ADAINO DATA]");

  // The AnalogIn instance is used to sample analog inputs. This instance 
  // controls Arduino's on-chip Analog-to-Digital Converter (ADC) peripheral 
  // and gives you access to the converted results.
  // The below methods configure AnalogIn to sample the analog inputs as 
  // required by this example. In general, AnalogIn is configured in the 
  // setup() function of your sketch.

  // Set the analog input pin to be sampled
  AnalogIn.setAnalogInput(A1);

  // Use the free running acquisition mode if you want to sample the analog 
  // input continously and at the highest sampling rate possible.
  AnalogIn.useFreerunMode();

  // If you want to sample the analog input at a higher rate, it is recommended 
  // to enable the acquisition buffer as samples can get lost otherwise.
  AnalogIn.enableAcqBuffer(); 

  // Start the analog data acquisition. This is in general performed in the 
  // setup() function of your sketch.
  AnalogIn.begin();
}

void loop()
{
  // Read the analog input and send it to the serial output
  int analogResult = AnalogIn.readInput();
  Serial.println(analogResult); 
}

