/* ///////////////////////////////////////////////////////////////////////////
Modern Art Electronics; parttimehacker@gmail.com; cyWren Systems Inc; 
 
MIT License.
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

#ifndef __HelperAnalog__
#define __HelperAnalog__

#include "Arduino.h"

class HelperAnalog
{
public:

// Instantiate the class: setup a bunch of defaults
    HelperAnalog();
    
// Initialize object: Arduino pin and optional reference voltage, e.g., 3.3 volts
    void init( 
    	int analogPin, 
    	float refVoltage = 5.0, 
    	int samples = 10, 
    	int sampleDelay = 10  );

// Calibrate sensor using simple linear curve fit
    bool calibrate( 
    	float slope = 1.0, 
    	float intercept = 0.0 );
        
// Read the sensor: collect a sample of counts and then compute average
    bool read( void );
    
// Calibrated: check to see if you have a valid sample
    bool calibrated( void );

// Methods to return counts, millivolts and EU for the sensor
    int 	counts();
    float 	millivolts();
    float   engineeringUnits();
    
private:
    int		_analogPin;
    float	_refVoltage;
    int		_sampleDelay;
    int		_samples;
	bool	_calibrated;
    float	_slope;
    float	_intercept;
    int		_counts;
    float	_millivolts;
    float	_engineeringUnits;
};   

#endif /* defined(____HelperAnalog__) */
