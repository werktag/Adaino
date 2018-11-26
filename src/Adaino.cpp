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
 * Adaino.cpp
 * 
 * Andre Meyer <andmeyer@werktag.io>
 * 
 */

#include "Adaino.h"
#include "ada/SAMD21_ADC.h"


AnalogDevice::AnalogDevice() : _timeout(1000) {
    ADCInst.init();
    ADCInst.enableAutoTrigger();
}


AnalogDevice::~AnalogDevice() {
    ADCInst.init();
}


void AnalogDevice::setAnalogInput(unsigned long pin) {
    ADCInst.setInput(pin);
}


void AnalogDevice::setResolution(int res) {
    ADCInst.setResolution(res);
}


void AnalogDevice::setAnalogReference(ada_ar_t ref) {
    adc_ar_t adcRef;
    switch (ref) {
        case ADA_AR_INTERNAL:
            adcRef = ADC_AR_INTERNAL;
            break;
        case ADA_AR_EXTERNAL:
            adcRef = ADC_AR_EXTERNAL;
            break;
        case ADA_AR_INTERNAL1V0:
            adcRef = ADC_AR_INTERNAL1V0;
            break;
        case ADA_AR_INTERNAL1V65:
            adcRef = ADC_AR_INTERNAL1V65;
            break;
        case ADA_AR_INTERNAL2V23:
            adcRef = ADC_AR_INTERNAL2V23;
            break;
        case ADA_AR_DEFAULT: 
        default:
            adcRef = ADC_AR_DEFAULT;        
    }
    ADCInst.setReference(adcRef);
}


void AnalogDevice::enableAcqBuffer(int bufferSize) {
    ADCInst.setAcquisitionMode(ADC_ACQMODE_BUFFERED);
    ADCInst.setBufferSize(bufferSize);
}


void AnalogDevice::disableAcqBuffer() {
    ADCInst.setAcquisitionMode(ADC_ACQMODE_VOLATILE);
}


void AnalogDevice::useSingleMode() {
    ADCInst.setOperationMode(ADC_OPMODE_SINGLE);
    ADCInst.setTriggerMode(ADC_TRIGMODE_SW);
} 


void AnalogDevice::useFreerunMode() {
    ADCInst.setOperationMode(ADC_OPMODE_FREERUN);
    ADCInst.setTriggerMode(ADC_TRIGMODE_SW);
}


void AnalogDevice::disableAutoTrigger() {
    ADCInst.disableAutoTrigger();
}


void AnalogDevice::enableAutoTrigger() {
    ADCInst.enableAutoTrigger();
}


int AnalogDevice::begin() {
    return ADCInst.start();
}


int AnalogDevice::end() {
    return ADCInst.stop();
}


void AnalogDevice::startConversion() {
    ADCInst.startConversion();
}


void AnalogDevice::setTimeout(unsigned long timeout) {
    _timeout = timeout;
}


unsigned long AnalogDevice::getTimeout() {
    return _timeout;
}


int AnalogDevice::readInput() {
    if (_timeout == 0) {
        return ADCInst.read();
    }
    else {
        int res;
        _startMillis = millis();
        do {
            res = ADCInst.read();
            if (res >= 0) {
                return res;
            }
        } while (millis() - _startMillis < _timeout);
        return -1; // -1 indicates timout expiration (no result ready)
    }
    return -1; 
}


int AnalogDevice::peekInput() {
    if (_timeout == 0) {
        return ADCInst.peek();
    }
    else {
        int res;
        _startMillis = millis();
        do {
            res = ADCInst.peek();
            if (res >= 0) {
                return res;
            }
        } while (millis() - _startMillis < _timeout);
        return -1; // -1 indicates timout expiration (no result ready)
    }
    return -1;
}


int AnalogDevice::readChunk(unsigned short* buffer, int len) {
    int count = ADCInst.readChunk(buffer, len);
    while (count < len) {
        int res = readInput();
        if (res < 0) {
            break;
        }
        buffer[count++] = (unsigned short)(res);
    }
    return count;
}


int AnalogDevice::peekChunk(unsigned short* buffer, int len) {
    int count = ADCInst.readChunk(buffer, len);
    while (count < len) {
        int res = peekInput();
        if (res < 0) {
            break;
        }
        buffer[count++] = res;
    }
    return count;
}


int AnalogDevice::available() {
    return ADCInst.available();
}


void AnalogDevice::flush() {
    ADCInst.flushBuffer();
}


int AnalogDevice::getOverrun() {
    return (ADCInst.getBufferOverrun() + ADCInst.getDeviceOverrun() == 0) ? 0 : 1; 
}


void AnalogDevice::clearOverrun() {
    ADCInst.clearBufferOverrun();
    ADCInst.clearDeviceOverrun();
}


AnalogDevice AnalogIn;