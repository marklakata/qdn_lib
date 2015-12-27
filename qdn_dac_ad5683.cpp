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


#include "qdn_dac_ad5683.h"
#include "qdn_gpio.h"
#include "qdn_spi.h"

static const uint8_t NOP = 0x0;
static const uint8_t WriteInputRegister = 0x10;
static const uint8_t UpdateDacRegister = 0x20;
static const uint8_t WriteDacAndInputRegister = 0x30;
static const uint8_t WriteControlRegister = 0x40;
static const uint8_t ReadbackInputRegister = 0x50;

static const uint16_t ControlReset = 0x8000;
static const uint16_t ControlPD1   = 0x4000;
static const uint16_t ControlPD0   = 0x2000;
static const uint16_t ControlRef   = 0x1000;
static const uint16_t ControlGain  = 0x0800;
static const uint16_t ControlDCEN  = 0x0400;



static QDN_OutputPin noConnect(false);

QDN_DAC_AD5683::QDN_DAC_AD5683(QDN_SPI& spi0, QDN_GPIO_OutputN& cs0)
	: spi(spi0)
	, cs(cs0)
	, ldac(noConnect)
//	, gainCommand(GAIN_1X)

{

}

#if 0
QDN_DAC_AD5683::QDN_DAC_AD5683(QDN_SPI& spi0, QDN_GPIO_OutputN& cs0, QDN_OutputPin& ldac0)
	: spi(spi0)
	, cs(cs0)
	, ldac(ldac0)
//	, gainCommand(GAIN_1X)

{

}
#endif

void QDN_DAC_AD5683::Init(void)
{
	cs.Init();
	ldac.Init();
	cs.Deassert();

	spi
		.SetClockRateShift(3)
		.SetClockPhase(spi.ClockPhase::FirstEdge)
//		.SetClockPhase(spi.ClockPhase::SecondEdge)
		.Init();

	Write(WriteControlRegister,ControlReset);
}



#if 0
QDN_DAC_AD5683& QDN_DAC_AD5683::SetGain(const QDN_DAC_AD5683::Gain gain)
{
	if (gain == Gain::Gain1X)
	{
		gainCommand = GAIN_1X;
	} else if (gain == Gain::Gain2X) {
		gainCommand = GAIN_2X;
	}
	return *this;
}
#endif


void QDN_DAC_AD5683::Write(const uint8_t command, const uint16_t db)
{
	cs.Assert();
	spi.WriteReadU8(command | (db >> 12));
	spi.WriteReadU8((db >> 4) & 0xFF);
        spi.WriteReadU8((db << 4) & 0xFF);
	cs.Deassert();
	ldac.Assert();
	ldac.Deassert();
}

#if 0
int16_t QDN_DAC_AD5683::HighZ(void)
{
	WriteCommand(SHUTDOWN);
	return 0;
}
#endif

int16_t QDN_DAC_AD5683::SetOutputImmediately(const uint16_t count )
{
//    Write(WriteDacAndInputRegister,count);
    Write(WriteInputRegister,count);
    counts_ = count;

    return 0;
}

uint16_t QDN_DAC_AD5683::GetOutput()
{
    return counts_;
}

