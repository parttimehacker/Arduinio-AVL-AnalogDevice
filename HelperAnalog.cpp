/* /////////////////////////////////////////////////////////////////////////////
Modern Art Electronics; parttimehacker@gmail.com; cyWren Systems Inc; MIT License.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
//////////////////////////////////////////////////////////////////////////  */

#include "HelperAnalog.h"
#include "Arduino.h"

//#define DEBUG

HelperAnalog::HelperAnalog()
{
    // assume invalid 
    _analogPin = -1;
    _calibrated = false;
}

void HelperAnalog::init( int analogPin, float refVoltage, int samples, int sampleDelay  )
{
    _analogPin = analogPin;
    _refVoltage = refVoltage;
    _samples = samples;
    _sampleDelay = sampleDelay;   
    _calibrated = false;
    _slope = 1.0;
    _intercept = 0.0;
    pinMode(analogPin,INPUT);
}

bool HelperAnalog::calibrate( float slope, float intercept )
{    
	if (_analogPin == -1) {
#ifdef DEBUG
	Serial.println("HelperAnalog:calibrate: invalid analog pin");
#endif
		_calibrated = false;
		return false;
	}
	_slope = slope;
	_intercept = intercept;
	_calibrated = true;
	_calibrated = read();
    return _calibrated;
}

bool HelperAnalog::read( void )
{
    if (!_calibrated) {
		return false;
	}
	analogRead(_analogPin);	// stabilize ADC, i.e.,  1 ADC for 6 analog pins
	delay(_sampleDelay);
	long measurements = 0;
	for (int i = 0; i < _samples; i++) {
		measurements += analogRead(_analogPin);
		delay(_sampleDelay);
	}
    _counts = measurements / _samples;
#ifdef DEBUG
	Serial.print("HelperAnalog:counts: "); Serial.println(_counts);
#endif
    return true;
}

bool HelperAnalog::calibrated()
{
	return _calibrated;
}

int HelperAnalog::counts()
{
	if (_calibrated) {
		return _counts;
	} else {
#ifdef DEBUG
		Serial.println("HelperAnalog:counts: not calibrated");
#endif
		return 0;
	}
}

float HelperAnalog::millivolts()
{
	if (_calibrated) {
	    _millivolts = (((float)_counts * _refVoltage) / 1024.0);
		return _millivolts;
	} else {
#ifdef DEBUG
		Serial.println("HelperAnalog:millivolts: not calibrated");
#endif
		return 0.0;
	}
}

float HelperAnalog::engineeringUnits()
{
	if (_calibrated) {
		millivolts();
	    _engineeringUnits = (_slope*_millivolts) + _intercept;
		return _engineeringUnits;
	} else {
#ifdef DEBUG
		Serial.println("HelperAnalog:engineeringUnits: not calibrated");
#endif
		return 0.0;
	}
}