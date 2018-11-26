/**
 * analog_reference.ino
 * 
 * This is a more advance example for setting the analog reference.
 * 
 * When converting an analog input to a digital value, the analog input is 
 * compared to an analog reference voltage. This reference voltage represents 
 * the maximum voltage which can be converted to a meaningful digitial value 
 * and therefore, the analog input is expected to have a voltage between 0 Volt
 * and this reference voltage.
 * 
 * If you set the analog reference voltage to a smaller value than your maximum 
 * input voltage, input values above the reference voltage will be saturated 
 * and cannot be distinguished from the reference voltage.
 * 
 * If you set the analog reference voltage to a (much) higher value than your 
 * maximum expected input voltage, you will lose signal accuracy because a 
 * digital representation is limited by a predefined number of discrete values.
 * Refere to example 'resolution.ino' for more information about digital 
 * resolution.
 * 
 * Adaino offers you to set this reference value to predefined internal 
 * references (generated in the Arduino chip itself) or to an external 
 * reference voltage connected to the VREFA pin.
 * 
 * 
 * The circuit:
 * 
 *   * An Arduino device with an SAMD21 microcontroller (e.g. Arduino MKR * or 
 *     Adafruit Feather M0).
 *   * Connect an analog signal to Arduino's analog input pin A1. The signals 
 *     voltage is expected to be between 0 and 2.23 V.
 *   * ATTENTION: The voltage range of the analog input signal is different 
 *                than in the other examples.
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
  Serial.println("analog_reference.ino");
  Serial.println("[ADAINO DATA]");

  // In this example, you sample the analog inputs with the same settings as 
  // used in the 'ada.ino' example.
  AnalogIn.setAnalogInput(A1);
  AnalogIn.useFreerunMode();
  AnalogIn.enableAcqBuffer(); 

  // Set the analog reference voltag. The following settings are supported by
  // Adaino:
  // 
  //  - ADA_AR_DEFAULT:      3.3 V (the default analog reference voltage)
  //  - ADA_AR_INTERNAL:     same as ADA_AR_INTERNAL2V23 (2.23 V)
  //  - ADA_AR_EXTERNAL:     analog reference connected to external reference 
  //                         pin VREFA
  //  - ADA_AR_INTERNAL1V0:  1.0 V
  //  - ADA_AR_INTERNAL1V65: 1.65 V
  //  - ADA_AR_INTERNAL2V23: 2.23 V
  //
  // In this example, the reference voltage is set to 2.23 V.
  AnalogIn.setAnalogReference(ADA_AR_INTERNAL2V23);

  // Start the analog data acquisition
  AnalogIn.begin();
}

void loop()
{
  // Read the analog input and send it to the serial output
  int analogResult = AnalogIn.readInput();
  Serial.println(analogResult); 
}

