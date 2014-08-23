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

#ifndef _QDN_SPI_H_
#define _QDN_SPI_H_

#include <stdint.h>

#include <stm32f10x_spi.h>
class QDN_GPIO_Output;
class QDN_GPIO_Input;
class QDN_DMA;

struct SpiThing;

class QDN_SPI
{
public:
	QDN_SPI(int unit, QDN_GPIO_Output& Clk, QDN_GPIO_Output& MOSI, QDN_GPIO_Input& MISO);

	enum class ClockPolarity : uint8_t { IdleHi, IdleLo };
	enum class ClockPhase    : uint8_t { FirstEdge, SecondEdge };
	QDN_SPI& SetClockPolarity(ClockPolarity state0);
	QDN_SPI& SetClockPhase(ClockPhase phase)       ;
	QDN_SPI& SetClockRateShift(uint32_t prescaler0);
	QDN_SPI& SetBitMode(uint8_t bits);
	void Init(void);

	uint8_t  WriteReadU8(uint8_t byte);
	uint16_t WriteReadU16_LE(uint16_t word);
	uint16_t WriteReadU16_BE(uint16_t word);

	void EnableDMA(void);

private:
    friend QDN_DMA;

	QDN_GPIO_Output& Clk;
	QDN_GPIO_Output& MOSI;
	QDN_GPIO_Input& MISO;
	SpiThing* spi;
    SPI_InitTypeDef spiInitStruct;

};


#endif
