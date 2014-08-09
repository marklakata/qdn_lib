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

#include "qdn_spi.h"
#include "qdn_gpio.h"
#include "qdn_util.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"

#if 1
struct SpiThing : public SPI_TypeDef
{

};
#else
struct SpiThing {
	SPI_TypeDef spi;
};
#endif


QDN_SPI::QDN_SPI(int unit, QDN_GPIO_Output& Clk0, QDN_GPIO_Output& MOSI0, QDN_GPIO_Input& MISO0)
	: Clk(Clk0)
	, MOSI(MOSI0)
	, MISO(MISO0)
	, clockPolarity(ClockPolarity::IdleLo)
    , clockPhase(ClockPhase::FirstEdge)
	, rightShift(1)
{
	switch(unit)
	{
	case 1:spi = static_cast<SpiThing*>(SPI1); break;
	case 2:spi = static_cast<SpiThing*>(SPI2); break;
	case 3:spi = static_cast<SpiThing*>(SPI3); break;
	default: QDN_Exception(); break;
	}

	Clk.SetMode(GPIO_Mode_AF_PP);
	MOSI.SetMode( GPIO_Mode_AF_PP);
	MISO.SetMode( GPIO_Mode_AF_PP);
}

void QDN_SPI::Init(void)
{
	Clk.HighSpeedInit();
	MOSI.HighSpeedInit();
	MISO.HighSpeedInit();

	RCC->APB2ENR |= RCC_APB2Periph_AFIO ;

	if (spi == SPI1)  RCC->APB2ENR |= RCC_APB2Periph_SPI1;
	else if (spi == SPI2) RCC->APB1ENR |= RCC_APB1Periph_SPI2;
	else if (spi == SPI3) RCC->APB1ENR |= RCC_APB1Periph_SPI3;


    SPI_InitTypeDef spiInitStruct;

    SPI_I2S_DeInit(spi);
    SPI_StructInit(&spiInitStruct);
    spiInitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInitStruct.SPI_Mode      = SPI_Mode_Master;
    spiInitStruct.SPI_DataSize  = SPI_DataSize_8b;
    spiInitStruct.SPI_CPOL      = ( clockPolarity == ClockPolarity::IdleHi) ? SPI_CPOL_High  : SPI_CPOL_Low;
    spiInitStruct.SPI_CPHA      = ( clockPhase == ClockPhase::FirstEdge   ) ? SPI_CPHA_1Edge : SPI_CPHA_2Edge;
    spiInitStruct.SPI_NSS       = SPI_NSS_Soft;
    uint16_t prescalarMask = (uint16_t)(rightShift - 1) << 3;
    if (!IS_SPI_BAUDRATE_PRESCALER(prescalarMask)) QDN_Exception();
    spiInitStruct.SPI_BaudRatePrescaler = prescalarMask;
    spiInitStruct.SPI_FirstBit  = SPI_FirstBit_MSB;
   // spiInitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(spi, &spiInitStruct);

    SPI_Cmd(spi, ENABLE);
}


uint8_t QDN_SPI::WriteReadU8(uint8_t byte)
{
	spi->DR = byte;
	while( (spi->SR & SPI_I2S_FLAG_TXE)  ==0 ) ; // wait until TX is empty
	while( (spi->SR & SPI_I2S_FLAG_RXNE) ==0 ) ; // wait until RX is full

	return (uint8_t) spi->DR; //lint !e529
}

uint16_t QDN_SPI::WriteReadU16_LE(uint16_t word)
{
	uint16_t result = 0;
	result =  WriteReadU8(word & 0xFF);
	result |= (static_cast<uint16_t>(WriteReadU8(word >>8))<<8);
	return result;
}


uint16_t QDN_SPI::WriteReadU16_BE(uint16_t word)
{
	uint16_t result = 0;
	result =  (static_cast<uint16_t>(WriteReadU8(word >> 8)) << 8);
	result |= WriteReadU8(word & 0xFF);
	return result;
}
