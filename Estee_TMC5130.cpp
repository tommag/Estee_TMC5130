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

#include "Estee_TMC5130.h"

Estee_TMC5130::Estee_TMC5130(uint32_t fclk)
: _fclk(fclk)
{

}

Estee_TMC5130::~Estee_TMC5130()
{
	;
}


bool Estee_TMC5130::begin( uint8_t ihold, uint8_t irun, MotorDirection stepper_direction )
{
	// set initial currents and delay
	TMC5130_Reg::IHOLD_IRUN_Register iholdrun = { 0 };
	iholdrun.ihold = ihold;
	iholdrun.irun = irun;
	iholdrun.iholddelay = 7;
	writeRegister(TMC5130_Reg::IHOLD_IRUN, iholdrun.value);

	// use position mode
	writeRegister(TMC5130_Reg::RAMPMODE, TMC5130_Reg::POSITIONING_MODE);

	// set ramp curves
	//writeRegister(TMC5130_Reg::A_1, 0xffff );
	writeRegister(TMC5130_Reg::V_1, 0x0 ); // disable A1 & D1 in position mode, amax, vmax only
	writeRegister(TMC5130_Reg::AMAX, 0xffff );
	writeRegister(TMC5130_Reg::VMAX, 0xffff );
	//writeRegister(TMC5130_Reg::DMAX, 0xffff );

	// set chopper config
	TMC5130_Reg::CHOPCONF_Register chopconf = { 0 };
	chopconf.toff = 5;
	chopconf.hend_offset = 1;
	chopconf.tbl = 0b10;
	writeRegister(TMC5130_Reg::CHOPCONF, chopconf.value);

	TMC5130_Reg::GCONF_Register gconf = { 0 };
	gconf.en_pwm_mode = true;
	gconf.shaft = stepper_direction;
	writeRegister(TMC5130_Reg::GCONF, gconf.value);

	return false;
}

void Estee_TMC5130::end()
{
	// no-op, just stop talking....
	; // FIXME: try and shutdown motor/chips?
}


// From "28.1 Using the Internal Clock", how to figure out the step/sec scaling value
float Estee_TMC5130::updateFrequencyScaling()
{
	int32_t vmax = 10000;
	int32_t dt_ms = 100;

	writeRegister(TMC5130_Reg::VMAX, 0);
	writeRegister(TMC5130_Reg::RAMPMODE, TMC5130_Reg::VELOCITY_MODE_POS);
	writeRegister(TMC5130_Reg::AMAX, 60000);
	writeRegister(TMC5130_Reg::VMAX, vmax);
	int32_t xactual1 = readRegister(TMC5130_Reg::XACTUAL);
	delay(dt_ms);
	int32_t xactual2 = readRegister(TMC5130_Reg::XACTUAL);
	writeRegister(TMC5130_Reg::VMAX,0);	// halt

	// scaling factor
	return (vmax * (dt_ms/1000.0)) / (xactual2 - xactual1);
}
