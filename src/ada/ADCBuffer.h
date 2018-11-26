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
 * ADCBuffer.h
 * 
 * Buffer for ADC data acquisition. It uses a ring buffer architecture.
 * 
 * Andre Meyer <andmeyer@werktag.io>
 * 
 */

#ifndef _ADCBUFFER_H_
#define _ADCBUFFER_H_

#include <stddef.h>
#include <stdint.h>

#define ADC_BUFFER_SIZE 1024

class ADCBuffer {
    public:
        ADCBuffer(int size = -1);
        virtual ~ADCBuffer();

        void reset();

        // buffer status
        size_t availableForWrite();
        size_t available(); // available for read
        bool full();
        bool empty();

        // read/write access
        void write(const void *buffer, size_t size);
        size_t read(void *buffer, size_t size);
        size_t peek(void *buffer, size_t size);
        void flush();

        void* data();

        int getOverrun();
        void clearOverrun();

        int getWrIdx() { return _wrIndex; }
        int getRdIdx() { return _rdIndex; }

        
    private:
        uint8_t* _buffer;
        int _bufferSize;
        volatile int _overrun;
        volatile int _rdIndex;
        volatile int _wrIndex;
        
};

#endif // _ADCBUFFER_H_