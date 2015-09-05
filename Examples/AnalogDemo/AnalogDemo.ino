#include <HelperPhotoResistor.h>


/* 
/////////////////////////////////////////////////////////////////////////////
 
Modern Art Electronics Project
  
Analog Example Application

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

////////////////////////////////////////////////////////////////////////// 
*/
#include <Wire.h>

#include <HelperLED.h>
#include <HelperPhotoResistor.h>
#include <HelperAnalog.h>
#include <HelperTMP36.h>
#include <DHT.h>

#include <FastIO.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_SR.h>
#include <LiquidCrystal_SR2W.h>
#include <LiquidCrystal_SR3W.h>

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

#define DHT_11            D5_PWM_PIN
#define GREEN_LED         D6_PWM_PIN
#define DIP1              D7_PIN
#define DIP2              D8_PIN 

#define THERMO_RESISTER   A1_PIN
#define PHOTO_RESISTER    A3_PIN
#define	LCD_SDA	          A4_SDA_PIN	
#define	LCD_SCL	          A5_SCL_PIN	

////////////////////////////////////////////////////////////////////////////
// Object instantiations
///////

HelperLED       greenLED;

HelperPhotoResistor     lightSensor;  // simple photoresister
int lightCt  = 0;
float lightMv    = 0.0;
int lightEu    = 0.0;

HelperTMP36     tmpSensor;    // tmp36 thermoresister
int tmpCt    = 0;
float tmpMv      = 0.0;
float tmpEu      = 0.0;

uint8_t  switch1  = 0;
uint8_t  switch2  = 0;

DHT dht(DHT_11, DHT11);
float temperature = 0.0;
float humidity = 0.0;

// Set the LCD I2C address
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  


////////////////////////////////////////////////////////////////////////////
// generic methods
///////

void selfTest() {
  Serial.println("Completed Self Test");
}


/////////////////////////////////////////////////////////////////////////////
// Interrupt Handlers
///////

#define BUTTON_INTERRUPT  0
volatile long buttonCounter = 0;
void buttonInterrupt() {
  buttonCounter++;
}

////////////////////////////////////////////////////////////////////////////
// Handle IR remote codes
///////


/////////////////////////////////////////////////////////////////////////////
// NRF24L01 radio messages
///////


////////////////////////////////////////////////////////////////////////////
// I2C stuff 
///////

byte tmpPict[8] = //icon for thermonitor
{
    B00100,
    B01010,
    B01010,
    B01110,
    B01110,
    B11111,
    B11111,
    B01110
};

byte lightPict[8] = //icon for light
{
    B10101,
    B00000,
    B01010,
    B00000,
    B00100,
    B00000,
    B01110,
    B01110
};

void displayInfo(  ) {
  char buf[17];
  memset(buf,0,17); // no float for sprintf ?!?
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Analog Lib Demo");
  lcd.setCursor(0, 1);
  lcd.write(1);
  lcd.setCursor(2, 1);
  lcd.print("Temp");
  lcd.setCursor(7, 1);
  lcd.write(2);
  lcd.setCursor(9, 1);
  lcd.print("Light"); 
}

void displayTmpInfo(  ) {
  char buf[17];
  memset(buf,0,17); // no float for sprintf ?!?
  int f = tmpSensor.fahrenheit();
  int c = tmpSensor.celsius();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(1);
  lcd.setCursor(2, 0);
  lcd.print(f);
  lcd.setCursor(4, 0);
  lcd.print((char)223); //degree sign
  lcd.print("F");
  lcd.setCursor(7, 0);
  lcd.print(c);
  lcd.setCursor(9, 0);
  lcd.print((char)223); //degree sign
  lcd.print("C");
  lcd.setCursor(12, 0);
  lcd.print((float)temperature,0);
  lcd.setCursor(14, 0);
  lcd.print((char)223); //degree sign
  lcd.print("F");
  
  memset(buf,0,17);
  int bug = tmpMv; // no float for sprintf ?!?
  sprintf(buf,"CT=%d mV=%d",tmpCt,bug);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void displayLightInfo(  ) {
  char buf[17];
  memset(buf,0,17);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(2);
  lcd.setCursor(2, 0);
  
  int bug = lightMv; // no float for sprintf ?!?
  sprintf(buf,"Ct=%d mV=%d",lightCt,bug);
  lcd.print(buf);

  bug = lightEu;
  memset(buf,0,17); // no float for sprintf ?!?
  sprintf(buf,"Lux=%d",bug);
  lcd.setCursor(0, 1);
  lcd.print(buf);
}


////////////////////////////////////////////////////////////////////////////
// sensor stuff 
///////

void takeMeasurements() {

  lightSensor.read();
  lightCt = lightSensor.counts();  
  lightMv = lightSensor.millivolts();
  lightEu = lightSensor.engineeringUnits();
  Serial.print("Light: Ct = "); Serial.print(lightCt);
  Serial.print("\tMv = "); Serial.print(lightMv);
  Serial.print("\tEu = "); Serial.println(lightEu);

  tmpSensor.read();
  tmpCt = tmpSensor.counts();  
  tmpMv = tmpSensor.millivolts();
  tmpEu = tmpSensor.engineeringUnits();
  Serial.print("TMP36: Ct = "); Serial.print(tmpCt);
  Serial.print("\tMv = "); Serial.print(tmpMv);
  Serial.print("\tEu = "); Serial.println(tmpEu);
  
  temperature = dht.readTemperature(true);
  Serial.print("DHT11 Tmp = "); Serial.println(temperature);
  
  humidity = dht.readHumidity();
  Serial.print("DHT11 Hum = "); Serial.println(humidity);
  
  int dip = digitalRead(DIP1);
  if (dip == LOW) {
    displayLightInfo();
  } else {
    displayTmpInfo();
  }
  
}


/////////////////////////////////////////////////////////////////////////////
// Initialization
///////

void setup() {

  analogReference(EXTERNAL); // 5 volts

  Serial.begin(57600);
  Serial.println("\n\rAnalogSensor Demo 0.1");
  
  greenLED.init(GREEN_LED);
  
  lightSensor.init(PHOTO_RESISTER,5.0,10,5);       // simple photo resistor
  lightSensor.calibrate(50,1020,0,255,true);

  tmpSensor.init(THERMO_RESISTER,5.0);       // simple photo resistor
  tmpSensor.calibrate();
  
  pinMode(DIP1,INPUT_PULLUP);
  pinMode(DIP2,INPUT_PULLUP);
  
  Wire.begin();
  lcd.begin(16,2);
  lcd.setCursor(0, 0);
  lcd.createChar(1,tmpPict);
  lcd.createChar(2,lightPict);
  displayInfo();
  
  // interrupts
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  
  attachInterrupt(BUTTON_INTERRUPT,buttonInterrupt,RISING);
  buttonCounter=0;
  
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
  
  if (buttonCounter>0) {
    buttonCounter = 0;
    takeMeasurements();
  }

}



