#ifndef _QDN_SPI_H_
#define _QDN_SPI_H_

#include <stdint.h>

class QDN_GPIO_Output;
class QDN_GPIO_Input;

class QDN_SPI
{
public:
	QDN_SPI(QDN_GPIO_Output& Clk, QDN_GPIO_Output& MOSI, QDN_GPIO_Input& MISO);
	void SetRate(uint32_t rateHz);
	void Write(uint8_t byte);
	void Write(uint16_t word);
};


#endif
