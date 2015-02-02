/**************************************************************************
 *
 * Copyright (c) 2013, Qromodyn Corporation
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 **************************************************************************/

#ifndef _QDN_ADC_H_
#define _QDN_ADC_H_

#include <vector>
#include "qdn_gpio.h"

class QDN_ADC;
class QDN_ADC_Pin;
class QDN_SPI;
class QDN_DMA;

/// Class to wrap the ADC hardware.
///
class QDN_ADC
{
public:
    /// Constructor.
    /// Unit is 1, 2, 3 for the ADC unit.
    QDN_ADC(int unit);

    /// configure the ADC for non-DMA readout.
    ///
    void Configure();

    /// configure a range of ADC pins to this ADC and configure to use DMA to copy the data directly to memory
    ///
    void DMA_Configure(QDN_DMA& dma, volatile uint16_t* dstArray, const std::vector<const QDN_ADC_Pin*>& adcList);


    void Enable();

    void EnableAndCalibrate();

    /// Enable the temperature sensor.
    ///
    void EnableTempSensor();
private:
    friend QDN_ADC_Pin;
    friend QDN_DMA;
    int sampleTime;
    ADC_TypeDef* adc;
};

/// Class to wrap a particular pin to a particular ADC unit.
/// To use, first create the QDN_ADC instance, then create
/// this object to connect the pin to the ADC.
class QDN_ADC_Pin : public QDN_Pin {
public:

    /// Constructor for ADC PIN.
    QDN_ADC_Pin(GPIO_TypeDef* gpio0, int pin0, QDN_ADC& adc0);
    uint16_t ReadOnce();
    uint16_t Channel;
protected:
    QDN_ADC& adc;
};

#endif
