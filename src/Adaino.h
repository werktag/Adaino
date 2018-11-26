/******************************************************************************
 * Copyright (c) 2018, Werktag GmbH
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *    * Neither the name of {{ project }} nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/**
 * Adaino.h
 * 
 * An Analog Data Acquisition Library for Arduino and IoT.
 * 
 * Created by Andre Meyer <andmeyer@werktag.io>
 * 
 */

#ifndef _ADAINO_H_
#define _ADAINO_H_

#include "Arduino.h"


// ANALOG REFERENCE PRESETS
//
// Arduino defines presets for analog references which are essentially sets  of
// a gain and reference value. The same presets are redefined below and 
// available for the setAnalogReference() method found in the AnalogObject class below.

typedef enum _ada_ar_t {
    ADA_AR_DEFAULT,      // the default analog reference is set to 3.3 V
    ADA_AR_INTERNAL,     // same as ADA_AR_INTERNAL2V23 (2.23 V)
    ADA_AR_EXTERNAL,     // analog reference connected to external reference pin VREFA
    ADA_AR_INTERNAL1V0,  // analog reference set to 1.0 V
    ADA_AR_INTERNAL1V65, // analog reference set to 1.65 V
    ADA_AR_INTERNAL2V23  // analog reference set to 2.23 V
} ada_ar_t;


// AnalogIn: An object for reading analog inputs. It uses the Analog 
// Digital Converter (ADC) peripheral of the Arduino device to do this.
//
// IMPORTANT NOTE: As there is only one ADC peripheral on an Arduino device,
// you can instantiate only one AnalogObject instance in a sketch (Arduino 
// program)

class AnalogDevice {
    public:
        
        AnalogDevice();
        virtual ~AnalogDevice();

        // setAnalogInput(...): set the analog input pin
        // At the moment, only a single pin can be defined (no multiplexing 
        // support)
        void setAnalogInput(unsigned long pin);

        // setResolution(...): set the bit resolution of the analog conversion
        //  results. Supported values are 8, 10 (default) and 12.
        void setResolution(int res);

        // setAnalogReference(...): set the analog reference voltage according 
        // to the presets defined above
        void setAnalogReference(ada_ar_t ref);


        // ACQUISITION BUFFER

        // enableAcqBuffer(...): analog conversion results are buffered. By 
        // default, no acquisition buffer is enabled.
        void enableAcqBuffer(int bufferSize = -1);

        // disableAcqBuffer(): disables the acquisition buffer and analog 
        // conversion results are lost if not read before the next result is 
        // ready (volatile mode)
        void disableAcqBuffer();
        

        // OPERATION MODE

        // useSingleMode(): by using the single conversion mode, only a single 
        // analog conversion is triggered at a time
        void useSingleMode();

        // useFreerunMode(): by using the freerun mode, an initial trigger 
        // starts a continuous (freerunning) acquisition
        void useFreerunMode();
        

        // AUTO TRIGGER
        // By default, auto trigger is enabled and an ADC conversion is started 
        // if not already pending

        void disableAutoTrigger();
        
        void enableAutoTrigger();

        // START/STOP OF ANALOG DATA ACQUISITION

        // begin(): start the analog data acquisition.
        int begin();
        
        // end(): stop the analog data acquisition.
        int end();
        
        // startConversion(): start an analog conversion (software trigger)
        void startConversion();

        
        // RESULT ACCESS
        // The following methods provide a user-friendly access to the analog 
        // result buffer, i.e. they wait for a valid result until a predefined 
        // timeout has expired (default is 1 second). Set the timeout to zero 
        // (0) if you want the methods to be non-blocking.

        // setTimeout(...): Set the timeout in milliseconds (default is 
        // 1000 ms).
        void setTimeout(unsigned long timeout);

        // getTimeout(): returns the set timeout (in milliseconds)
        unsigned long getTimeout();

        // readInput(): read an analog result. The method waits for a valid 
        // result or until the specified timeout has expired. In case the 
        // timeout expires befor a result is valid, -1 is returned.
        int readInput();

        // peekInput(): peek an analog result. The method waits for a valid 
        // result or until the specified timeout has expired. In case the 
        // timeout expires, -1 is returned.
        int peekInput();

        // readChunk(...): read one or more analog conversion results into a 
        // buffer. The method returns if no more results are available or the 
        // specified number of results are read. Returns the number of results 
        // stored in the buffer.
        int readChunk(unsigned short* buffer, int len);

        // peekChunk(...): peek one or more analog conversion results into a 
        // buffer. Returns the number of results stored in the buffer.
        int peekChunk(unsigned short* buffer, int len);

        // available(): returns the number of analog conversion results 
        // available
        int available(); 

        // flush(): flushes the acquisition buffer
        void flush();

        
        // OVERRUN HANDLING
        // Depending on the acquisition mode, the ADC device and/or the acquisition buffer
        // can overrun (overflow) if the results are not read in time.

        // getOverrun(): returns 1 if an overrun condition occured, 0 otherwise
        int getOverrun();

        // clearOverrun(): clears the overrun flag
        void clearOverrun();
        

    private:

        unsigned long _timeout;
        unsigned long _startMillis;


};

extern AnalogDevice AnalogIn; // On-chip analog input (ADC)

#endif // _ADAINO_H_