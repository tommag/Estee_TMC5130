/*
MIT License

Copyright (c) 2016 Mike Estee
Copyright (c) 2017 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef ESTEE_TMC5130_H
#define ESTEE_TMC5130_H

#include <Arduino.h>
#include <SPI.h>
#include <Estee_TMC5130_registers.h>

class Estee_TMC5130 {
public:
	static constexpr uint8_t IC_VERSION = 0x11;

	enum MotorDirection { NORMAL_MOTOR_DIRECTION =	0x00, INVERSE_MOTOR_DIRECTION = 0x1 };

	Estee_TMC5130(uint32_t fclk=F_CPU);
	~Estee_TMC5130();

	// start/stop this module
	bool begin(uint8_t ihold, uint8_t irun, MotorDirection stepper_direction/*=NORMAL_MOTOR_DIRECTION*/);
	void end();

	virtual uint32_t readRegister(uint8_t address) = 0;	// addresses are from TMC5130.h
	virtual uint8_t  writeRegister(uint8_t address, uint32_t data) = 0;

	// internal clock measuring
	// NOTE: Driver MUST BE DISABLED DURING THIS CALL
	float updateFrequencyScaling();

	// high level interface
	void setRampCurves() {}

protected:
	static constexpr uint8_t WRITE_ACCESS = 0x80;	//Register write access for spi / uart communication

private:
	uint32_t _fclk;
};


/* SPI interface : 
 * the TMC5130 SWSEL input has to be low (default state).
 */
class Estee_TMC5130_SPI : public Estee_TMC5130 {
public:
	Estee_TMC5130_SPI( uint8_t chipSelectPin,	// pin to use for the SPI bus SS line
		uint32_t fclk=F_CPU,
		const SPISettings &spiSettings=SPISettings(1000000, MSBFIRST, SPI_MODE0), // spi bus settings to use
		SPIClass& spi=SPI ); // spi class to use

	uint32_t readRegister(uint8_t address);	// addresses are from TMC5130.h
	uint8_t  writeRegister(uint8_t address, uint32_t data);
	uint8_t  readStatus();

private:
	uint8_t _CS;
	SPISettings _spiSettings;
	SPIClass &_spi;

	void _beginTransaction();
	void _endTransaction();
};


/* Base UART interface :
 * the TMC5130 SWSEL input must be tied high.
 *
 * This class does not handle TX/RX switch on the half-duplex bus.
 * It should be used only if there is another mechanism to switch between
 * transmission and reception (e.g. on Teensy the Serial class can be configured
 * to control an external transceiver).
 *
 * Serial must be initialized externally. Serial.setTimeout() must be set to a
 * decent value to avoid blocking for too long if there is a RX error.
 */
class Estee_TMC5130_UART : public Estee_TMC5130 {
public:
	/* Read register return codes */
	enum ReadStatus {SUCCESS, NO_REPLY, BAD_CRC};

	Estee_TMC5130_UART(Stream& serial=Serial, // Serial port to use
		uint8_t slaveAddress = 0, // TMC5130 slave address (default 0 if NAI is low, 1 if NAI is high)
		uint32_t fclk=F_CPU);

	uint32_t readRegister(uint8_t address, ReadStatus *status);	// addresses are from TMC5130.h. Pass an optional status pointer to detect failures.
	uint32_t readRegister(uint8_t address) { return readRegister(address, nullptr); }
	uint8_t  writeRegister(uint8_t address, uint32_t data);

	void resetCommunication(); // Reset communication with TMC5130 : pause activity on the serial bus.

	void setSlaveAddress(uint8_t slaveAddress);

	//TODO add optional internal checks (reg write counter, retry in case of bad CRC)


protected:
	Stream &_serial;
	uint8_t _slaveAddress;

	virtual void beginTransmission() {}
	virtual void endTransmission() {}

private:
	const uint8_t SYNC_BYTE = 0x05;

	void computeCrc(uint8_t *datagram, uint8_t datagramLength);
};


/* UART interface with external transceiver support :
 * the TMC5130 SWSEL input must be tied high.
 * See TMC5130 datasheet §5.4 figure 5.2 for wiring details
 *
 * This interface switches a digital pin to control an external transceiver to
 * free the bus when not transmitting.
 *
 * This is not optimized : the interface has to wait for the end of the
 * transmission.
 *
 * Serial must be initialized externally. Serial.setTimeout() must be set to a
 * decent value to avoid blocking for too long if there is a RX error.
 */
class Estee_TMC5130_UART_Transceiver : public Estee_TMC5130_UART {
public:
	Estee_TMC5130_UART_Transceiver(uint8_t txEnablePin, // pin to enable transmission on the external transceiver
		Stream& serial=Serial, // Serial port to use
		uint8_t slaveAddress = 0, // TMC5130 slave address (default 0 if NAI is low, 1 if NAI is high)
		uint32_t fclk=F_CPU)
	: Estee_TMC5130_UART(serial, slaveAddress, fclk), _txEn(txEnablePin)
	{
		pinMode(_txEn, OUTPUT);
	}

protected:
	void beginTransmission()
	{
		digitalWrite(_txEn, HIGH);
	}

	void endTransmission()
	{
		_serial.flush();
		digitalWrite(_txEn, LOW);
	}

private:
	uint8_t _txEn;
};


#endif // ESTEE_TMC5130_H
