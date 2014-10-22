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


#include "qdn_dac_mcp4822.h"
#include "qdn_gpio.h"
#include "qdn_spi.h"

#define DAC_CHAN_A  0x0000
#define DAC_CHAN_B  0x8000

// mask 0x4000 is don't care

#define GAIN_1X     0x2000
#define GAIN_2X     0x0000

#define OPERATING   0x1000
#define SHUTDOWN    0x0000



QDN_OutputPin noConnect(false);

QDN_DAC_MCP4822::QDN_DAC_MCP4822(QDN_SPI& spi0, QDN_GPIO_OutputN& cs0)
	: spi(spi0)
	, cs(cs0)
	, ldac(noConnect)
	, gainCommand(GAIN_1X)

{

}

QDN_DAC_MCP4822::QDN_DAC_MCP4822(QDN_SPI& spi0, QDN_GPIO_OutputN& cs0, QDN_OutputPin& ldac0)
	: spi(spi0)
	, cs(cs0)
	, ldac(ldac0)
	, gainCommand(GAIN_1X)

{

}

void QDN_DAC_MCP4822::Init(void)
{
	cs.Init();
	ldac.Init();
	cs.Deassert();

	spi
		.SetClockRateShift(3)
		.SetClockPhase(spi.ClockPhase::FirstEdge)
//		.SetClockPhase(spi.ClockPhase::SecondEdge)
		.Init();

	SetGain(Gain::Gain1X);
	SetOutput(Channel::ChannelA,0);
	SetOutput(Channel::ChannelB,0);
}




QDN_DAC_MCP4822& QDN_DAC_MCP4822::SetGain(const QDN_DAC_MCP4822::Gain gain)
{
	if (gain == Gain::Gain1X)
	{
		gainCommand = GAIN_1X;
	} else if (gain == Gain::Gain2X) {
		gainCommand = GAIN_2X;
	}
	return *this;
}


void QDN_DAC_MCP4822::WriteCommand(const uint16_t command)
{
	cs.Assert();
	spi.WriteReadU8((command >> 8) & 0xFF);
	spi.WriteReadU8(command & 0xFF);
	cs.Deassert();
	ldac.Assert();
	ldac.Deassert();
}

int16_t QDN_DAC_MCP4822::HighZ(void)
{
	WriteCommand(SHUTDOWN);
	return 0;
}

int16_t QDN_DAC_MCP4822::SetOutput(const QDN_DAC_MCP4822::Channel channel, const uint16_t count )
{
    uint16_t command = 0;

	if (channel == Channel::ChannelA)
	{
		command |= DAC_CHAN_A;
	} else if (channel == Channel::ChannelB)
	{
		command |= DAC_CHAN_B;
	} else {
		return -1;
	}

	if (count > 0x0FFF)
	{
		return -2;
	}

	command |= gainCommand;
	command |= OPERATING;
	command |= count;

	WriteCommand(command);
	counts[static_cast<int>(channel)] = count;

	return 0;
}

uint16_t QDN_DAC_MCP4822::GetOutput(const QDN_DAC_MCP4822::Channel channel)
{
    return counts[static_cast<int>(channel)];
}
