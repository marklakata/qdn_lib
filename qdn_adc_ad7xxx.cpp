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
#include "qdn_adc_ad7xxx.h"
#include "qdn_gpio.h"
#include "qdn_spi.h"
#include "qdn_xos.h"

QDN_ADC_AD7xxx::QDN_ADC_AD7xxx(QDN_SPI& spi0, QDN_GPIO_Output& convst0, QDN_GPIO_Input& busy0)
    : spi(spi0)
    , convst(convst0)
    , busy(busy0)
{
}

void QDN_ADC_AD7xxx::Init(void)
{
	convst.Deassert();
}

void QDN_ADC_AD7xxx::Convert(void)
{
	convst.Assert();
}

uint16_t QDN_ADC_AD7xxx::Read(void)
{
	//
	convst.Deassert();
	uint16_t value;

	value = spi.WriteReadU16_BE(0); // this might be Little Endian..
	return value;
}


uint16_t QDN_ADC_AD7xxx::ConvertAndRead(void)
{
	Convert();
	XOS_Delay100Ns(22); // from AD7685 datasheet. Tconv >= 2.2 microseconds
	return Read();
}
