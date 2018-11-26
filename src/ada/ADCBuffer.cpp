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
 * ADCBuffer.cpp
 * 
 * Andre Meyer <andmeyer@werktag.io>
 * 
 */

#include <string.h>
#include <stdlib.h>
#include "ADCBuffer.h"


ADCBuffer::ADCBuffer(int size) {
    _bufferSize = size;
    if (_bufferSize < 0) { // use device defaults
        _bufferSize = ADC_BUFFER_SIZE;
    }
    _buffer = (uint8_t*)malloc(_bufferSize*sizeof(uint8_t)); // check if memory has been allocated...
    reset();
}


ADCBuffer::~ADCBuffer() {
    free(_buffer);
}


void ADCBuffer::reset() {
    _rdIndex = 0;
    _wrIndex = 0;
    memset(_buffer, 0x00, _bufferSize);
    _overrun = 0;
}


size_t ADCBuffer::available() {
    size_t space;
    if (_rdIndex <= _wrIndex) {
        space = _wrIndex - _rdIndex;
    }
    else {
        space = _bufferSize - _rdIndex + _wrIndex;
    }
    return space;
}


size_t ADCBuffer::availableForWrite() {
    size_t space;
    if (_rdIndex <= _wrIndex) {
        space = _bufferSize - _wrIndex + _rdIndex - 1;
    }
    else {
        space = _rdIndex - _wrIndex - 1;
    }
    return space;
}


bool ADCBuffer::full() {
    return availableForWrite() == 0;
}


bool ADCBuffer::empty() {
    return available() == 0;
}


void ADCBuffer::write(const void *buffer, size_t size) {
    size_t space = availableForWrite();
    if (size > space) { // buffer is overrunning >> throw away old data by overwritting it with new data
        int missingSpace = size - space;
        _overrun += missingSpace;
        _rdIndex += missingSpace;
    }
    if (_wrIndex + size < _bufferSize) {
        memcpy(_buffer+_wrIndex, buffer, size);
        _wrIndex = (_wrIndex + size) % _bufferSize;
    }
    else {
        size_t firstSize = _bufferSize - _wrIndex;
        memcpy(_buffer+_wrIndex, buffer, firstSize);
        _wrIndex = size - firstSize;
        memcpy(_buffer, buffer + firstSize, _wrIndex);
    }
}


size_t ADCBuffer::read(void *buffer, size_t size) {
    size_t avail = available();
    if (size > avail) {
        size = avail;
    }
    if (size > 0) {
        if (_rdIndex + size < _bufferSize) {
            memcpy(buffer, _buffer+_rdIndex, size);
            _rdIndex = (_rdIndex + size) % _bufferSize;
        }
        else {
            size_t firstSize = _bufferSize - _rdIndex;
            memcpy(buffer, _buffer+_rdIndex, firstSize);
            _rdIndex = size - firstSize;
            memcpy(buffer + firstSize, _buffer, _rdIndex);
        }
    }
    return size;
}


size_t ADCBuffer::peek(void *buffer, size_t size) {
    size_t avail = available();
    if (size > avail) {
        size = avail;
    }
    if (size > 0) {
        memcpy(buffer, &_buffer[_rdIndex], size);
    }
    return size;
}


void ADCBuffer::flush() {
    _wrIndex = _rdIndex;
}


void* ADCBuffer::data() {
    return (void*)_buffer;
}


int ADCBuffer::getOverrun() {
    return _overrun;
}


void ADCBuffer::clearOverrun() {
    _overrun = 0;
}