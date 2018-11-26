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
 * SAMD21_ADC.h
 * 
 * ADC peripheral driver for SAMD21 microcontrollers.
 * 
 * Andre Meyer <andmeyer@werktag.io>
 * 
 */

#ifndef _SAMD21_ADC_H_
#define _SAMD21_ADC_H_


#include "Arduino.h"
#include "wiring_private.h"
#include "ADCBuffer.h"

//#define SAMD21_ADC_DEBUG 1

// ADC Acquisition Mode
typedef enum {
    ADC_ACQMODE_VOLATILE, // samples are not buffered and read directly from the ADC's result register
    ADC_ACQMODE_BUFFERED  // samples are buffered using a software ring buffer and an interrupt service routine (ISR). This is the preffered mode for continuous operation modes
} adc_acqmode_t;

// ADC Operation Mode
typedef enum {
    ADC_OPMODE_SINGLE, // ADC is operated in single conversion mode
    ADC_OPMODE_FREERUN // ADC is operated in freerun mode. Note that first conversion in freerun mode has to be triggered
} adc_opmode_t;

// ADC Trigger Mode
typedef enum {
    ADC_TRIGMODE_SW,    // ADC conversion is started by a software trigger
    ADC_TRIGMODE_EVENT  // ADC conversion is started by an external event, not yet supported
} adc_trigmode_t;

// ADC State
typedef enum {
    ADC_STATE_OFF,     // ADC is off
    ADC_STATE_STANDBY, // ADC is on but no analog conversion is running
    ADC_STATE_RUNNING  // ADC is on and analog conversion is running
} adc_state_t;

// Analog Reference
typedef enum {
    ADC_AR_DEFAULT,
    ADC_AR_INTERNAL,
    ADC_AR_EXTERNAL,
    ADC_AR_INTERNAL1V0,
    ADC_AR_INTERNAL1V65,
    ADC_AR_INTERNAL2V23
} adc_ar_t;

// Interrupt flags
typedef enum {
    ADC_IRQ_RESRDY,
    ADC_IRQ_OVERRUN,
    ADC_IRQ_WINMON,
    ADC_IRQ_SYNCRDY
} adc_irq_t;


class SAMD21_ADC {
    public:

        // Constructor
        SAMD21_ADC();
        virtual ~SAMD21_ADC();
        
        int setAcquisitionMode(adc_acqmode_t mode);
        adc_acqmode_t getAcquisitionMode();

        int setTriggerMode(adc_trigmode_t mode);
        adc_trigmode_t getTriggerMode();

        int setOperationMode(adc_opmode_t mode);
        adc_opmode_t getOperationMode();

        int setBufferSize(int size); // size = -1 useses device defaults

        int enableAutoTrigger();
        int disableAutoTrigger();

        int start(); // start ADC acquisition
        int stop(); // stop ADC acquisition

        int available(); // number of conversion results available
        int read(); // read conversion result, returns -1 if no result is available
        int peek(); // peek conversion result, returns -1 if no result is available
        void flushBuffer(); // flush acquisition buffer

        int readChunk(uint16_t* buffer, int len); // read multiple ADC conversion results
        int peekChunk(uint16_t* buffer, int len); // peek multiple ADC conversion results
        
        int getBufferOverrun(); // returns 1 if acquisition buffer has overrun, 0 otherwise
        void clearBufferOverrun(); // clears buffer overrun flag
        
        int getDeviceOverrun(); // returns 1 if device has overrun, 0 otherwise
        void clearDeviceOverrun(); // clears device overrun flag

        int init();
        
        // basic functions for ADC peripheral access

        int enable();   // enable ADC device
        int disable(); // disable ADC device
        
        int startConversion(); // start ADC conversion
        int flush(); // flush ADC conversion

        uint16_t readResult(); // read conversion result
                
        int setInput(uint32_t pin); // set analog input to be converted
        
        void setPrescaler(); // set prescaler
        void getPrescaler();

        void setSamplingTime(); // set sampling length
        void getSamplingTime();

        void setResolution(int res=12); // set resolution in bits
        
        int setReference(adc_ar_t analogReference);

        int setConversionMode();
        
        // Interrupt handling
        
        typedef void (*adc_callback_t)(void);

        int registerInterrupt(adc_irq_t irqType, adc_callback_t);
        int unregisterInterrupt(adc_irq_t irqType);

        int enableInterrupts();
        int disableInterrupts();
        
        static void onResultReady(); // callback for result ready interrupt has to call static member variable (or member variable of a dedicated instance)

        
    private:

        adc_state_t _state;

        adc_acqmode_t _acqMode;
        adc_opmode_t _opMode;
        adc_trigmode_t _trigMode;

        bool _autoTriggerEnabled;
        void executeAutoTrigger();
        
        ADCBuffer* _buffer;
        int _bufferSize;

        uint8_t _intEnMask;
        adc_callback_t _resultReadyCallback;

};

extern SAMD21_ADC ADCInst; // SAMD21 provides only one ADC instance

#endif // _SAMD21_ADC_H_