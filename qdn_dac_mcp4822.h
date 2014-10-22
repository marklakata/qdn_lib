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

#ifndef _QDN_DAC_MCP4822_H_
#define _QDN_DAC_MCP4822_H_

#include <stdint.h>

class QDN_SPI;
class QDN_GPIO_OutputN;
class QDN_OutputPin;


class QDN_DAC_MCP4822
{
public:
	QDN_DAC_MCP4822(QDN_SPI& spi, QDN_GPIO_OutputN& cs);
	QDN_DAC_MCP4822(QDN_SPI& spi, QDN_GPIO_OutputN& cs, QDN_OutputPin& ldac);

	void Init(void);

	enum class Gain { Gain1X, Gain2X };
	QDN_DAC_MCP4822& SetGain(const Gain gain);

	/// Count is 0 to 0xFFF (12 bit)
	enum class Channel { ChannelA, ChannelB };
	int16_t  SetOutput(const Channel channel, const uint16_t count);
	uint16_t GetOutput(const Channel channel);
	int16_t HighZ(void);

private:
	void WriteCommand(const uint16_t command);

	QDN_SPI& spi;
	QDN_GPIO_OutputN& cs;
	QDN_OutputPin& ldac;

	uint16_t gainCommand;
	uint16_t counts[2];
};




#endif
