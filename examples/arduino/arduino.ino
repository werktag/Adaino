/**
 * arduino.ino
 * 
 * This is a reference example using Arduino's built-in function analogRead() 
 * for reading the analog inputs. 
 * 
 * While analogRead() provides the basic access for reading values from the 
 * analog inputs at infrequent rates, it fails sampling the analog inputs at 
 * higher rates. As you can see in the other examples, Adaino offers you the 
 * missing functionality and versatility to do exactly that.
 * 
 * Refer to the 'single.ino' example, if you want to replace Arduino's built-in
 * functionality with Adaino.
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

void setup()
{
  // Setup serial port and wait for connection
  Serial.begin(115200);
  while (!Serial) ;
  
  // Print example information to the serial output
  Serial.println("[ADAINO EXAMPLE]");
  Serial.println("arduino.ino");
  Serial.println("[ADAINO DATA]");

  // Set the analog reference to the default value (3.3 V) 
  analogReference(AR_DEFAULT);

  // The first conversion after setting the analog reference is invalid. 
  // Therefore, read an analog conversion result which is not used before 
  // acquiring the real data.
  analogRead(A1);
}

void loop()
{
  // Read the analog input and send it to the serial output. As the built-in 
  // analogRead() function is blocking and the conversion of a new sample is 
  // not started in the background, the maximum sampling rate is expected to be
  // slower than in the 'ada.ino' example.
  int analogResult = analogRead(A1);
  Serial.println(analogResult); 
}

