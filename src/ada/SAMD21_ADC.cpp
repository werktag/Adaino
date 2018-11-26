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
 * SAMD21_ADC.cpp
 * 
 * Andre Meyer <andmeyer@werktag.io>
 * 
 */

#include "SAMD21_ADC.h"

#if defined(ARDUINO_ARCH_SAMD) // SAMD specific code

// Debug functionality

#ifdef SAMD21_ADC_DEBUG
#define PIN 10
#ifdef _VARIANT_ARDUINO_ZERO_
volatile uint32_t *setPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg;
volatile uint32_t *clrPin = &PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg;
const uint32_t  PinMASK = (1ul << g_APinDescription[PIN].ulPin);
#endif

typedef struct {
    int startCnt;
    uint16_t isrCnt;
} samd21_adc_debug_t;

samd21_adc_debug_t debugStruct;

void printSamd21AdcDebugInfo() {
    Serial.println("## SAMD21_ADC Debug Info ##");
    Serial.print("startCnt = ");
    Serial.println(debugStruct.startCnt);
    Serial.print("isrCnt = ");
    Serial.println(debugStruct.isrCnt);
}

void resetSamd21AdcDebugStruct() {
    debugStruct.startCnt = 0;
    debugStruct.isrCnt = 0;
}

#endif




// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}


SAMD21_ADC::SAMD21_ADC(void) : _state(ADC_STATE_OFF), _acqMode(ADC_ACQMODE_VOLATILE), 
_opMode(ADC_OPMODE_SINGLE), _trigMode(ADC_TRIGMODE_SW), _autoTriggerEnabled(true), 
_intEnMask(0), _resultReadyCallback(NULL), _buffer(NULL), _bufferSize(-1) {

    // As a single instance of this class is instantiated at the end of this file, 
    // no ADC settings should be set in the constructor. Ohterwise, Arduino's default 
    // analogRead() function might be influenced.
    
}


SAMD21_ADC::~SAMD21_ADC() {
    // empty
}


int SAMD21_ADC::setAcquisitionMode(adc_acqmode_t mode) {
    if (_state != ADC_STATE_OFF) {
        return 1;
    }
    if (_acqMode == ADC_ACQMODE_BUFFERED) {
        unregisterInterrupt(ADC_IRQ_RESRDY);
    }
    switch (mode) {
        case ADC_ACQMODE_BUFFERED:
            registerInterrupt(ADC_IRQ_RESRDY, SAMD21_ADC::onResultReady);
            break;
        case ADC_ACQMODE_VOLATILE:
        default:
            ;
    }
    _acqMode = mode;
    return 0;
}


adc_acqmode_t SAMD21_ADC::getAcquisitionMode() {
    return _acqMode;
}


int SAMD21_ADC::setOperationMode(adc_opmode_t mode) {
    if (_state != ADC_STATE_OFF) {
        return 1;
    }
    switch (mode) {
        case ADC_OPMODE_FREERUN:
            syncADC();
            ADC->CTRLB.bit.FREERUN = 1;
            break;
        case ADC_OPMODE_SINGLE:
        default:
            syncADC();
            ADC->CTRLB.bit.FREERUN = 0;
    }
    _opMode = mode;
    return 0;
}


adc_opmode_t SAMD21_ADC::getOperationMode() {
    return _opMode;
}


int SAMD21_ADC::setTriggerMode(adc_trigmode_t mode) {
    if (_state != ADC_STATE_OFF) {
        return 1;
    }
    _trigMode = mode;
    return 0;
}


adc_trigmode_t SAMD21_ADC::getTriggerMode() {
    return _trigMode;
}


int SAMD21_ADC::setBufferSize(int size) {
    _bufferSize = size;
}


int SAMD21_ADC::enableAutoTrigger() {
    _autoTriggerEnabled = true;
}


int SAMD21_ADC::disableAutoTrigger() {
    _autoTriggerEnabled = false;
}


int SAMD21_ADC::start() {
    
    #ifdef SAMD21_ADC_DEBUG
    resetSamd21AdcDebugStruct();
    #endif

    if (_state != ADC_STATE_OFF) {
        return 1;
    }
    if (_acqMode == ADC_ACQMODE_BUFFERED) {
        _buffer = new ADCBuffer(_bufferSize);
        if (_buffer == NULL) {
            return 1;
        }
        _buffer->reset();
    }
    enableInterrupts();
    enable();
    _state = ADC_STATE_STANDBY;
    if (_opMode == ADC_OPMODE_FREERUN) {
        _state = ADC_STATE_RUNNING;
        // in freerun mode, no sw trigger is required to start the ADC
    }
    
    return 0;
}


int SAMD21_ADC::stop() {
    if (_state != ADC_STATE_OFF) {
        disable();
        disableInterrupts();
        if (_buffer) { // release buffer memory (for buffered acquisition mode)
            delete _buffer;
            _buffer = NULL;
        }
        _state = ADC_STATE_OFF;
    }
    
    #ifdef SAMD21_ADC_DEBUG
    printSamd21AdcDebugInfo();
    #endif

    return 0;
}


int SAMD21_ADC::available() {
    if (_state == ADC_STATE_OFF) {
        return 0;
    }
    if (_acqMode == ADC_ACQMODE_VOLATILE) {
        // no results are buffered and therefore at max 1 result can be read
        return ADC->INTFLAG.bit.RESRDY;
    }
    else { // _acqMode == ADC_ACQMODE_BUFFERED
        size_t bytesAvailable = _buffer->available();
        int resultsAvailable = bytesAvailable / sizeof(uint16_t);
        return resultsAvailable;
    }
}


int SAMD21_ADC::read() {
    uint16_t result;
    if (readChunk(&result, 1)) {
        return result;
    }
    else {
        return -1;
    }
}

int SAMD21_ADC::peek() {
    uint16_t result;
    if (peekChunk(&result, 1)) {
        return result;
    }
    else {
        return -1;
    }
}


void SAMD21_ADC::flushBuffer() {
    if (_buffer) {
        _buffer->flush();
    }
}

void SAMD21_ADC::executeAutoTrigger() {
    if ((_autoTriggerEnabled == true) && (_state == ADC_STATE_STANDBY)) {
        startConversion();
        _state = ADC_STATE_RUNNING;
    }
}

int SAMD21_ADC::readChunk(uint16_t* buffer, int len) {
    
    if (_state == ADC_STATE_OFF) {
        return 0;
    }

    int resultsRead = 0;
    if (_acqMode == ADC_ACQMODE_BUFFERED) {
        // read available items from buffer
        size_t bytesRead = _buffer->read(buffer, len*sizeof(uint16_t));
        resultsRead = bytesRead / sizeof(uint16_t);
        if (_opMode == ADC_OPMODE_SINGLE && resultsRead == 0) {
            executeAutoTrigger();
        }
    }
    else { // ADC_ACQMODE_VOLATILE
        // check status flag and read result if ready
        if (ADC->INTFLAG.bit.RESRDY) {
            buffer[0] = readResult();               // ... read result ...
            ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;  // ... and clear the Result Ready flag
            resultsRead = 1;
            _state = ADC_STATE_STANDBY;
        }
        else {
            if (_opMode == ADC_OPMODE_SINGLE) {
                executeAutoTrigger();
            }
        }
    }

    return resultsRead;
}


int SAMD21_ADC::peekChunk(uint16_t* buffer, int len) {

    if (_state == ADC_STATE_OFF) {
        return 0;
    }

    int resultsRead = 0;
    if (_acqMode == ADC_ACQMODE_BUFFERED) {
        // read available items from buffer
        size_t bytesRead = _buffer->peek(buffer, len*sizeof(uint16_t));
        resultsRead = bytesRead / sizeof(uint16_t);
        if (resultsRead == 0) {
            executeAutoTrigger();
        }
    }
    else { // ADC_ACQMODE_VOLATILE
        // check status flag and read result if ready
        if (ADC->INTFLAG.bit.RESRDY) {
            buffer[0] = readResult(); // ... read result (and do not clear status flag because we peek)
            _state = ADC_STATE_RUNNING; // as we just peeked, the ADC state should beset back to running
            resultsRead = 1;
        }
        else {
            executeAutoTrigger();
        }
    }

    return resultsRead;
}


int SAMD21_ADC::getBufferOverrun() {
    int overrun = 0;
    if (_buffer) {
        overrun = _buffer->getOverrun();
    }
    return overrun;
}


void SAMD21_ADC::clearBufferOverrun() {
    if (_buffer) {
        _buffer->clearOverrun();
    }
}


int SAMD21_ADC::getDeviceOverrun() {
    return ADC->INTFLAG.bit.OVERRUN == 0 ? 0 : 1;

}


void SAMD21_ADC::clearDeviceOverrun() {
    ADC->INTFLAG.bit.OVERRUN = 1;
}


// General board initialization is performed in wiring.c/init()

int SAMD21_ADC::init() {

    if (_state != ADC_STATE_OFF) {
        return 1;
    }

    // Set sample length and averaging
    syncADC();
    ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
    syncADC();
    ADC->SAMPCTRL.reg = 0x3F;  ; //sample length in 1/2 CLK_ADC cycles Default is 3F

    //Control B register
    setResolution(10);
    syncADC();
    ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV512_Val;
    syncADC();
    ADC->CTRLB.bit.CORREN = 0;
    syncADC();
    ADC->CTRLB.bit.FREERUN = 0;
    syncADC();
    ADC->CTRLB.bit.LEFTADJ = 0;
    syncADC();
    ADC->CTRLB.bit.DIFFMODE = 0;
    syncADC();
    
    setTriggerMode(ADC_TRIGMODE_SW);
    setAcquisitionMode(ADC_ACQMODE_VOLATILE);
    setOperationMode(ADC_OPMODE_SINGLE);

    setReference(ADC_AR_DEFAULT);
    
    return 0;
}


int SAMD21_ADC::setReference(adc_ar_t analogReference) {

    if (_state != ADC_STATE_OFF) {
        return 1;
    }

    syncADC();
    switch (analogReference) {
        case ADC_AR_INTERNAL:
        case ADC_AR_INTERNAL2V23:
            ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
            ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
            break;
        
        case ADC_AR_EXTERNAL:
            ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
            ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
            break;
        
        case ADC_AR_INTERNAL1V0:
            ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
            ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;   // 1.0V voltage reference
            break;
        
        case ADC_AR_INTERNAL1V65:
            ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
            ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
            break;
        
        case ADC_AR_DEFAULT:
        default:
            ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
            ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
            break;
    }

    // The first conversion after changing the reference has to be discarded
    adc_opmode_t prevOpMode = _opMode;
    adc_trigmode_t prevTrigMode = _trigMode;
    adc_acqmode_t prevAcqMode = _acqMode;
    setOperationMode(ADC_OPMODE_SINGLE);
    setTriggerMode(ADC_TRIGMODE_SW);
    setAcquisitionMode(ADC_ACQMODE_VOLATILE);
    start();
    startConversion();
    while(read()<0);
    stop();
    setOperationMode(prevOpMode);
    setTriggerMode(prevTrigMode);
    setAcquisitionMode(prevAcqMode);
    
    return 0;
}


void SAMD21_ADC::setPrescaler() {
    
}


void SAMD21_ADC::setResolution(int res) {

    if (res > 10) {
		ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
	} else if (res > 8) {
		ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
	} else {
		ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
	}

    syncADC();
}


int SAMD21_ADC::setInput(uint32_t pin) {
    syncADC();
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
    return 0;
}


int SAMD21_ADC::enable() {
    /*
    * Bit 1 ENABLE: Enable
    *   0: The ADC is disabled.
    *   1: The ADC is enabled.
    * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
    * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
    * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
    *
    * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
    * configured. The first conversion after the reference is changed must not be used.
    */
    syncADC();
    ADC->CTRLA.bit.ENABLE = 0x01;
    syncADC();
    return 0;
}


int SAMD21_ADC::disable() {
    syncADC();
    ADC->CTRLA.bit.ENABLE = 0x00;   // Disable ADC
    syncADC();
    ADC->SWTRIG.reg = 0x01;         //  and flush for good measure
    syncADC();
    return 0;    
}


int SAMD21_ADC::startConversion() {
    // Start conversion by sw trigger
    syncADC();
    ADC->SWTRIG.bit.START = 1;
    _state = ADC_STATE_RUNNING;
    #ifdef SAMD21_ADC_DEBUG
    debugStruct.startCnt++;
    #endif
    return 0;
}


uint16_t SAMD21_ADC::readResult() {
    uint16_t result = ADC->RESULT.reg;
    if (_opMode != ADC_OPMODE_FREERUN) {
        _state = ADC_STATE_STANDBY;
    }
    return result;
}


int SAMD21_ADC::registerInterrupt(adc_irq_t irqType, adc_callback_t callback) {
    if (irqType == ADC_IRQ_RESRDY) {
        _resultReadyCallback = SAMD21_ADC::onResultReady;
        _intEnMask |= ADC_INTFLAG_RESRDY;
    }
    return 0;
}


int SAMD21_ADC::unregisterInterrupt(adc_irq_t irqType) {
    if (irqType == ADC_IRQ_RESRDY) {
        _resultReadyCallback = NULL;
        _intEnMask &= ~ADC_INTFLAG_RESRDY;
    }
    return 0;
}


int SAMD21_ADC::enableInterrupts() {
    //syncADC();
    //ADC->INTENCLR.reg = 0xFF; // disable and clear all interrupts
    //syncADC();
    
    // enable the ADC interrupt at lowest priority
    if (_intEnMask) {
        NVIC_DisableIRQ(ADC_IRQn);
        NVIC_ClearPendingIRQ(ADC_IRQn);
        NVIC_SetPriority(ADC_IRQn, 0);
        NVIC_EnableIRQ(ADC_IRQn);    
    }
    
    if (_intEnMask & ADC_INTFLAG_RESRDY) {
        ADC->INTENSET.bit.RESRDY = 1; // enable RESRDY interrupt
        syncADC();
    }
    
    return 0; 
}


int SAMD21_ADC::disableInterrupts() {
    ADC->INTENCLR.bit.RESRDY = 1; // disable all internal ADC interrupts
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_ClearPendingIRQ(ADC_IRQn);
    return 0;
}


// static callback function can only use static variables or variables of a dedicated instance. we use the latter.
void SAMD21_ADC::onResultReady() {
    // read result and add it to buffer
    uint16_t res = ADCInst.readResult();
    #ifdef SAMD21_ADC_DEBUG
    ADCInst._buffer->write(&(debugStruct.isrCnt), sizeof(uint16_t));
    #else
    ADCInst._buffer->write(&res, sizeof(uint16_t));
    #endif
}


#ifdef __cplusplus
extern "C" {
#endif
    // ADC Interrupt Service Routine
    void ADC_Handler() {
        
        uint8_t mask = ADC_INTFLAG_RESRDY;
        uint8_t status = ADC->INTFLAG.reg & mask; // read interrupt status

        ADC->INTFLAG.reg = status; // clear enabled interrupts

        if (status & ADC_INTFLAG_RESRDY) {
            ADCInst.onResultReady();
        }


        #ifdef SAMD21_ADC_DEBUG
        debugStruct.isrCnt++;
        #endif
        
    }

#ifdef __cplusplus
}
#endif


SAMD21_ADC ADCInst;

#else
#error “This library only supports boards with an SAMD processor.”
#endif