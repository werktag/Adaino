/**
 * buffered_freerun.ino
 * 
 * This example shows that Adaino is capable of sampling the analog input while
 * your sketch is running other tasks. This is a likely scenario in every 
 * Arduino sketch, because you do not only sample analog inputs, but also need 
 * to process the acquired data or communicate with other devices or the 
 * worldwide web.
 * 
 * In this example, we are simulating thosekinds of activities by buffering the
 * acquired samples and sending them to the serial output in chunks of 100 
 * samples.
 * 
 * By connecting a periodic analog signal to the analog input of your Arduino 
 * device, it is easy to see that this examples samples the analog input 
 * reliably and you do not lose important singal information. See the circuit 
 * desciption for more recommendations.
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
#include <Adaino.h>


void setup()
{
  // Setup serial port and wait for connection
  Serial.begin(115200);
  while (!Serial) ;
  
  // Print example information to the serial output
  Serial.println("[ADAINO EXAMPLE]");
  Serial.println("buffered_freerun.ino");
  Serial.println("[ADAINO DATA]");

  // Set the analog input pin to be sampled
  AnalogIn.setAnalogInput(A1);

  // Use the free running acquisition mode if you want to sample the analog 
  // input continously and at the highest sampling rate possible with the 
  // current settings.
  AnalogIn.useFreerunMode();

  // If you want to sample the analog input at a higher rate, it is recommended 
  // to enable the acquisition buffer as samples can get lost otherwise.
  AnalogIn.enableAcqBuffer(); 

  // Start the analog data acquisition
  AnalogIn.begin();

}


void loop()
{
  // Read the analog input and add it to the buffer.
  int analogResult = AnalogIn.readInput();
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

