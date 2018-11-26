/**
 * buffered_arduino.ino
 * 
 * This is a reference example using Adruino's built-in analogRead() function.
 * 
 * In a normal Arduino sketch, you are not only sampling analog inputs. It is 
 * likely that you are also processing the acquired data or communicate with 
 * other devices or the worldwide web. In this example, we are simulating those
 * kinds of activities by buffering the acquired samples and sending them to 
 * the serial output in chunks of 100 samples. 
 * 
 * While we are tranfering this data to the serial output, the sketch is 
 * blocked and you are not able to read the next sample from the analog input 
 * while the tranfer is active. If you are sampling a fast changig signal, you
 * will simply lose important signal information.
 * 
 * By connecting a periodic analog signal to the analog input of your Arduino 
 * device, it is easy to see that you are going to miss some samples. See the 
 * circuit desciption for more recommendations.
 * 
 * 
 * The circuit:
 * 
 *   * An Arduino device with an SAMD21 microcontroller (e.g. Arduino MKR * or 
 *     Adafruit Feather M0).
 *   * Connect an analog signal to Arduino's analog input pin A1. The signals 
 *     voltage is expected to be between 0 and 3.3 V.
 *   * As an example, you can use a signal generator to generate a sine wave 
 *     with a frequency of 50 Hz, an offset of 1 V and an amplitude of 1 V and
 *     connect it the analog input pin A1.
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

  // Read the analog input and add the result to the buffer.
  int analogResult = analogRead(A1);
  add_to_buffer(analogResult);

  if (buffer_is_full()) {
    // The buffer is full and its data has to be sent to the serial output.
    transfer_buffer();
  }
}


// Create a result buffer for 100 samples.
const int BUFFER_SIZE = 100;
int buffer[BUFFER_SIZE];

// Define a counter variable. It is used as the buffer's index later on.
int counter = 0;

// add_to_buffer() adds a result to the buffer and increments the buffer index (counter) by one.
void add_to_buffer(int result) {
  buffer[counter] = result;
  counter = counter + 1;
}

bool buffer_is_full() {
  return counter == BUFFER_SIZE;
}

// transfer_buffer() sends the full buffer to the serial output and blocks your
// sketch while active. It is likely that you have similar functions in your 
// sketch if you process the acquired data or communicate with other devices or 
// the worldwide web.
void transfer_buffer() {
  for (int i=0; i < BUFFER_SIZE; i++) {
    Serial.println(buffer[i]);
    // Reset the counter to zero.
    counter = 0;
  }
}

