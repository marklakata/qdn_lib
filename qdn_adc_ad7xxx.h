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

// Support for Analog Devices ADC AD7xxx family,
// including AD7685


#ifndef _QDN_ADC_AD7XXX_H_
#define _QDN_ADC_AD7XXX_H_

#include <stdint.h>


class QDN_SPI;
class QDN_GPIO_Output;
class QDN_GPIO_InputN;
class QDN_InputPin;

class QDN_ADC_AD7xxx
{
public:
	QDN_ADC_AD7xxx(QDN_SPI& spi, QDN_GPIO_Output& convst);
	void Init(void);

	void Convert(void);
	uint16_t Read(void);
	uint16_t ConvertAndRead(void);

protected:
	QDN_SPI&          spi;
	QDN_GPIO_Output&  convst;
};

template<int ChainLen>
class QDN_ChainedADC_AD7xxx : public QDN_ADC_AD7xxx
{
public:
    QDN_ChainedADC_AD7xxx(QDN_SPI& spi, QDN_GPIO_Output& convst, QDN_GPIO_InputN& busy0)
        : QDN_ADC_AD7xxx(spi,convst)
        , busy(busy0)
    {
    }

    bool  ChainedConvertAndRead(uint16_t* data);
    bool  Read(uint16_t* data);
    QDN_GPIO_InputN&  busy;
};

template<int ChainLen>
class QDN_AsyncChainedADC_AD7xxx : public QDN_ADC_AD7xxx
{
public:
    QDN_AsyncChainedADC_AD7xxx(QDN_SPI& spi, QDN_GPIO_Output& convst)
		: QDN_ADC_AD7xxx(spi,convst)
	{
	}

    void Read(uint16_t* data);
};
#endif
