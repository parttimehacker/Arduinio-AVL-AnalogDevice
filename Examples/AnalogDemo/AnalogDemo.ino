
/* /////////////////////////////////////////////////////////////////////////////
Modern Art Electronics Project, Analog Example Application

Copyright (c) 2014. cyWren Systems, Inc. MIT License.
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
////////////////////////////////////////////////////////////////////////// */

#include <HelperAnalog.h>

///////
// Arduino ATMega328 Pins
///////

#define D2_INT0_PIN         2	// Digital		Interrupt 0
#define D3_INT1_PWM_PIN     3	// Digital	PWM	Interrupt 1
#define D4_PIN              4	// Digital
#define D5_PWM_PIN          5	// Digital      PWM
#define D6_PWM_PIN          6	// Digital	PWM
#define D7_PIN              7	// Digital
#define D8_PIN              8	// Digital
#define D9_PWM_PIN	    9	// Digital	PWM
#define D10_PWM_PIN	   10	// Digital	PWM
#define D11_PWM_MOSI_PIN   11	// Digital	PWM	SPI	MOSI
#define D12_PWM_MISO_PIN   12	// Digital		SPI	MISO
#define D13_PWM_SCK_PIN    13	// Digital		SPI	SCK

#define	A0_PIN            A0	// Analog
#define	A1_PIN	          A1	// Analog
#define	A2_PIN            A2	// Analog
#define	A3_PIN            A3	// Analog
#define	A4_SDA_PIN	  A4	// Analog		I2C	SDA
#define	A5_SCL_PIN	  A5	// Analog		I2C	SCL

///////
// pin assignments

#define PHOTO_RESISTOR    A3_PIN
	

////////////////////////////////////////////////////////////////////////////
// Object instantiations
///////

HelperAnalog  lightSensor; 


////////////////////////////////////////////////////////////////////////////
// generic methods
///////

void selfTest() {
  Serial.println("Completed Self Test");
}


/////////////////////////////////////////////////////////////////////////////
// Interrupt Handlers
///////


////////////////////////////////////////////////////////////////////////////
// Handle IR remote codes
///////


/////////////////////////////////////////////////////////////////////////////
// NRF24L01 radio messages
///////


////////////////////////////////////////////////////////////////////////////
// I2C stuff 
///////


/////////////////////////////////////////////////////////////////////////////
// Initialization
///////

void setup() {

  // analogReference(EXTERNAL); // 5 volts

  Serial.begin(57600);
  Serial.println("\n\rAnalogSensor Demo 0.1");
  
  float volts = 5.0;    // this is a 5 volt ATMega 16MHz project
  int samples = 10;     // collect 10 samples and return the average
  int interval = 10;    // delay 10 milliseconds between samples
  
  lightSensor.init(PHOTO_RESISTOR,volts,samples,interval);       // simple photo resistor
  
  float slope = 1.0;    // no real conversion to lumens, etc.
  float intercept = 0.0;

  lightSensor.calibrate(slope,intercept);

  // simple status light show
  selfTest();
  
  // get ready, get set, go!
  delay(500);
  Serial.println("Looping ...");
  
}


////////////////////////////////////////////////////////////////////////////
// Main control logic
///////


void loop() {
  
  delay(1000);
  
  // take measurements

  if (lightSensor.read()) {
    
    int counts = lightSensor.counts();
    Serial.print("Raw Counts="); Serial.println(counts);
    
    float millivolts = lightSensor.millivolts();
    Serial.print("Computed mV="); Serial.println(millivolts);
        
    float EU = lightSensor.engineeringUnits();
    Serial.print("Computed EU="); Serial.println(EU);
     
 } else {
    Serial.println("Tilt!  can't read the photoresistor");
  }

}



