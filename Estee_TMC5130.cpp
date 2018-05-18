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
	iholdrun.ihold = constrain(ihold, 0, 31);
	iholdrun.irun = constrain(irun, 0, 31);
	iholdrun.iholddelay = 7;
	writeRegister(TMC5130_Reg::IHOLD_IRUN, iholdrun.value);

	// use position mode
	setRampMode(POSITIONING_MODE);

	TMC5130_Reg::GCONF_Register gconf = { 0 };
	gconf.en_pwm_mode = true; //Enable stealthChop PWM mode
	gconf.shaft = stepper_direction;
	writeRegister(TMC5130_Reg::GCONF, gconf.value);

	// Recommended settings in quick config guide
	TMC5130_Reg::PWMCONF_Register pwmconf = { 0 };
	pwmconf.pwm_autoscale = true;
	pwmconf.pwm_grad = 1;
	pwmconf.pwm_ampl = 255;
	pwmconf.pwm_freq = 0b01; // recommended : 35kHz with internal typ. 13.2MHZ clock. 0b01 => 2/683 * f_clk = 38653Hz
	writeRegister(TMC5130_Reg::PWMCONF, pwmconf.value);

	// Recommended settings in quick config guide
	TMC5130_Reg::CHOPCONF_Register chopconf = { 0 };
	chopconf.toff = 4;
	chopconf.tbl = 0b10;
	chopconf.hstrt_tfd = 4;
	chopconf.hend_offset = 0;
	writeRegister(TMC5130_Reg::CHOPCONF, chopconf.value);

	//Set default start, stop, threshold speeds.
	setRampSpeeds(0, 0.1, 0); //Start, stop, threshold speeds

	return false;
}

void Estee_TMC5130::end()
{
	// no-op, just stop talking....
	; // FIXME: try and shutdown motor/chips?
}

bool Estee_TMC5130::isLastReadSuccessful()
{
	return _lastRegisterReadSuccess;
}

// From "28.1 Using the Internal Clock", how to figure out the step/sec scaling value
float Estee_TMC5130::updateFrequencyScaling()
{
	int32_t vmax = 10000;
	int32_t dt_ms = 100;

	//TODO disable driver

	setRampMode(VELOCITY_MODE);
	writeRegister(TMC5130_Reg::AMAX, 60000);
	writeRegister(TMC5130_Reg::VMAX, vmax);
	int32_t xactual1 = readRegister(TMC5130_Reg::XACTUAL);
	delay(dt_ms);
	int32_t xactual2 = readRegister(TMC5130_Reg::XACTUAL);
	writeRegister(TMC5130_Reg::VMAX,0);	// halt

	//TODO re-enable driver

	// scaling factor
	return (vmax * (dt_ms/1000.0)) / (xactual2 - xactual1);
}

void Estee_TMC5130::setRampMode(Estee_TMC5130::RampMode mode)
{
	switch (mode)
	{
		case POSITIONING_MODE:
		writeRegister(TMC5130_Reg::RAMPMODE, TMC5130_Reg::POSITIONING_MODE);
		break;

		case VELOCITY_MODE:
		setMaxSpeed(0); // There is no way to know if we should move in the positive or negative direction => set speed to 0.
		writeRegister(TMC5130_Reg::RAMPMODE, TMC5130_Reg::VELOCITY_MODE_POS);
		break;

		case HOLD_MODE:
		writeRegister(TMC5130_Reg::RAMPMODE, TMC5130_Reg::HOLD_MODE);
		break;
	}

	_currentRampMode = mode;
}

long Estee_TMC5130::getCurrentPosition()
{
	uint32_t uStepPos = readRegister(TMC5130_Reg::XACTUAL);

	if (uStepPos == 0xFFFFFFFF)
		return NAN;
	else
		return (int)uStepPos / _uStepCount;
}

long Estee_TMC5130::getTargetPosition()
{
	uint32_t uStepPos = readRegister(TMC5130_Reg::XTARGET);

	if (uStepPos == 0xFFFFFFFF)
		return NAN;
	else
		return (int)uStepPos / _uStepCount;
}

float Estee_TMC5130::getCurrentSpeed()
{
	uint32_t data = readRegister(TMC5130_Reg::VACTUAL);

	if (data == 0xFFFFFFFF)
		return NAN;

	// Returned data is 24-bits, signed => convert to 32-bits, signed.
	if (bitRead(data, 23)) // highest bit set => negative value
		data |= 0xFF000000;

	return speedToHz(data);
}

void Estee_TMC5130::setCurrentPosition(long position)
{
	writeRegister(TMC5130_Reg::XACTUAL, position * _uStepCount);
}

void Estee_TMC5130::setTargetPosition(long position)
{
	writeRegister(TMC5130_Reg::XTARGET, position * _uStepCount);
}

void Estee_TMC5130::setMaxSpeed(float speed)
{
	writeRegister(TMC5130_Reg::VMAX, speedFromHz(abs(speed)));

	if (_currentRampMode == VELOCITY_MODE)
	{
		writeRegister(TMC5130_Reg::RAMPMODE, speed < 0.0f ? TMC5130_Reg::VELOCITY_MODE_NEG : TMC5130_Reg::VELOCITY_MODE_POS);
	}
}

void Estee_TMC5130::setRampSpeeds(float startSpeed, float stopSpeed, float transitionSpeed)
{
	writeRegister(TMC5130_Reg::VSTART, speedFromHz(abs(startSpeed)));
	writeRegister(TMC5130_Reg::VSTOP, speedFromHz(abs(stopSpeed)));
	writeRegister(TMC5130_Reg::V_1, speedFromHz(abs(transitionSpeed)));
}

void Estee_TMC5130::setAcceleration(float maxAccel)
{
	setAccelerations(maxAccel, maxAccel, maxAccel, maxAccel);
}

void Estee_TMC5130::setAccelerations(float maxAccel, float maxDecel, float startAccel, float finalDecel)
{
	writeRegister(TMC5130_Reg::AMAX, accelFromHz(abs(maxAccel)));
	writeRegister(TMC5130_Reg::DMAX, accelFromHz(abs(maxDecel)));
	writeRegister(TMC5130_Reg::A_1, accelFromHz(abs(startAccel)));
	writeRegister(TMC5130_Reg::D_1, accelFromHz(abs(finalDecel)));
}

void Estee_TMC5130::stop()
{
	// §14.2.4 Early Ramp Termination option b)
	writeRegister(TMC5130_Reg::VSTART, 0);
	writeRegister(TMC5130_Reg::VMAX, 0);
}

void Estee_TMC5130::setModeChangeSpeeds(float pwmThrs, float coolThrs, float highThrs)
{
	writeRegister(TMC5130_Reg::TPWMTHRS, thrsSpeedToTstep(pwmThrs));
	writeRegister(TMC5130_Reg::TCOOLTHRS, thrsSpeedToTstep(coolThrs));
	writeRegister(TMC5130_Reg::THIGH, thrsSpeedToTstep(highThrs));
}
