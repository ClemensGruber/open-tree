/* 
  Open Tree | Soil Moisture 
  Version 0.x
  -------------------------
  sensors:  | soil moisture
            | soil temperature / waterproof DS18B20
  monitor:  | battery (todo)
  transp:   | Wifi 
            | LoRa / TTN (todo)
         
  Copyright (c) 2021 by Clemens Gruber 

  2021-03 | initial version
            - user butten switchs between frames 
  2021-06 | adaption for ESP32
            - board: WiFi Kit 32 Dev-Board, https://heltec.org/project/wifi-kit-32/
              with Onboard 128 x 64 px OLED 
              tutorial board installation: https://heltec-automation-docs.readthedocs.io/en/latest/esp8266+arduino/quick_start.html#via-arduino-board-manager
            - OLED-lib: https://github.com/HelTecAutomation/Heltec_ESP32 

  
  --------------------------------------------------- 
  ClimART project
  by ZK/U – Zentrum fuer Kunst und Urbanistik, Berlin
  ---------------------------------------------------
  
  GNU GPL v3 License 
  ------------------
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  Licence can be viewed at                               
  http://www.gnu.org/licenses/gpl-3.0.txt

*/

// -------------------------+------
// variables you can modify | START
// -------------------------+------

// todo 
// number of sensors / pins
// wifi credentials 

// calibration values 
const int inAirValue   = 3300;   // change this value to the raw value you get with soil sensor in air
const int inWaterValue =  330;   // change this value to the raw value you get with soil sensor in water 

// debug via serial
#define debug

// -------------------------+----
// variables you can modify | END
// -------------------------+----

#include "Arduino.h"  // must be before any string definition!

// version 
String version = "0.7";  

// libraries 
// OLED
//#include <Wire.h>
#include "heltec.h"
#include "oled/OLEDDisplayUi.h"

// BME280
//#include <Wire.h> // already done 
#include "SparkFunBME280.h"

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// images
#include "images.h"

// WiFi
#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#include <HTTPClient.h>


// pin definitions 
const int buttonPin    =  0;     // pushbutton pin
const int ONE_WIRE_BUS = 32;     // temperature sensor DS18B20 data pin 

// constants 
const int TEMPERATURE_PRECISION = 12;  // DS18B20 resolution 

// variables 
// soil moisture
int soilMoistureRaw = 0;
int soilMoisturePercent;
int soilMoistureCategory;
int categoryNumbers = 6;
int categoryWidth   = (inAirValue - inWaterValue) / categoryNumbers;


int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

char temperatureC[6];  // DS18B20

char humidityCase[4];  // BME280 
char temperatureCCase[6];
char pressureCase[5];

// unsigned longs because time measure in milliseconds
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 30;    // the debounce time; increase if the output flickers

unsigned long lastSensorReadTime = 0;  // last sensor reading
unsigned long sensorReadInterval = 3 * 1000;  // seconds * ms 

unsigned long long lastSendDataTime = 0;  // last sensor reading
unsigned long long sendDataInterval = 30 * 1000;  // minutes * seconds * ms 

// SH1107 OLED via I2C
extern Heltec_ESP32 Heltec;
OLEDDisplayUi ui(Heltec.display);


// BME
BME280 bme280;

// DS18B20
OneWire oneWire(ONE_WIRE_BUS);        // setup a oneWire instance for communication 
DallasTemperature sensors(&oneWire);  // pass the oneWire reference to Dallas Temperature
DeviceAddress TempSensor;             // arrays to hold device addresses



// OLEDDisplayUi functions
void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(128, 0, String(millis()));
}

// OLED frames
// soil moisture 
void frameMoisture(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
//  display->setFont(ArialMT_Plain_10);
  display->setFont(ArialMT_Plain_16);
  display->drawString(0 + x, 0 + y, " Feuchte");
  display->drawString(0 + x, 18 + y, " Boden");
  display->setFont(ArialMT_Plain_24);
  display->drawString(5 + x, 38 + y, String(soilMoisturePercent) + " %");
}

// soil temperature
void frameTemperature(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
//  display->setFont(ArialMT_Plain_10);
  display->setFont(ArialMT_Plain_16);
  display->drawString(0 + x, 0 + y, " Temperatur");
  display->drawString(0 + x, 18 + y, " Boden");
  display->setFont(ArialMT_Plain_24);
  display->drawString(0 + x, 38 + y, String(temperatureC) + " °C");
}

// electronic case humidity and temperature
void frameCase(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
//  display->setFont(ArialMT_Plain_10);
  display->setFont(ArialMT_Plain_16);
  display->drawString(0 + x, 0 + y, " Gehäuse");
  
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 20 + y, " Feuchte: " + String(humidityCase) + " %");
  display->drawString(0 + x, 34 + y, " Temperatur: " + String(temperatureCCase) + " °C");
  display->drawString(0 + x, 48 + y, " Luftdruck: " + String(pressureCase) + " hPa");  
}

// debug 
void frameDebug(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x,  0 + y, " debug moisture");
  display->drawString(0 + x,  2 + y, " ______________");
  display->drawString(0 + x, 42 + y, " Prozent: " + String(soilMoisturePercent) + " %");
  display->drawString(0 + x, 30 + y, " Kategorie: " + String(soilMoistureCategory) + " (of " + String(categoryNumbers) + ")");
  display->drawString(0 + x, 18 + y, " Rohwert: " + String(soilMoistureRaw));
  display->drawString(0 + x, 54 + y, " ");
}

// credits 
void frameCredits(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x,  0 + y, "Version "+version);
  display->setFont(ArialMT_Plain_16);
  display->drawString(0 + x, 15 + y, "Open Tree");
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 38 + y, "ClimART by ZK/U");
  display->drawString(0 + x, 50 + y, "© 2021 Clemens Gruber");
}

// ZK/U logo
void frameLogo(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  // draw an xbm image.
  // Please note that everything that should be transitioned
  // needs to be drawn relative to x and y
  // positon centered 128 x 64 px OLED 
  display->drawXbm((128-logo_zku_width)/2 + x, (64-logo_zku_height)/2 +2 + y, logo_zku_width, logo_zku_height, logo_zku_bits);
}


// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {frameMoisture, frameTemperature, frameCase, frameDebug, frameCredits, frameLogo};

// how many frames are there?
int frameCount = 6;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {msOverlay};
int overlaysCount = 1;


// read sensors
void readSensors()
{
  // soil moisture 
  soilMoistureRaw = analogRead(33);  // read raw value

  soilMoistureRaw = constrain(soilMoistureRaw, inWaterValue, inAirValue);  
  soilMoistureCategory = map(soilMoistureRaw, inAirValue, inWaterValue, 0, categoryNumbers);
  soilMoisturePercent  = map(soilMoistureRaw, inAirValue, inWaterValue, 0, 100);

  #ifdef debug
    Serial.print("soil moisture raw value:");
    Serial.print(soilMoistureRaw);
    Serial.print("\t category:");
    Serial.print(soilMoistureCategory);
    Serial.print("\t percent:");
    Serial.println(soilMoisturePercent);
  #endif
  
  // DS18B20
  // temperature request to all devices on the bus, takes around 1 second at 12bit resolution! 
  float tempSensorValue = sensors.getTempC(TempSensor);
  // only one digit 
  dtostrf(tempSensorValue,5,1,temperatureC);
  // initiate next physical reading non blocking 
  sensors.requestTemperatures();

  // BME280
  bme280.setMode(MODE_FORCED); //Wake up sensor and take reading
  delay(5);

  while(bme280.isMeasuring() == false); // wait for sensor to start measurment
  while(bme280.isMeasuring() == true);  // hang out while sensor completes the reading   
  // Sensor is now back asleep but we can get the data
    
  int bmePressureValue = bme280.readFloatPressure();   // pressure
  bmePressureValue = bmePressureValue / 100;   // hPa instead Pa 
  dtostrf(bmePressureValue,4,0,pressureCase);
  
  float bmeTemperatureValue = bme280.readTempC();   // temperature
  dtostrf(bmeTemperatureValue,5,1,temperatureCCase);

  int bmeHumidityValue = bme280.readFloatHumidity();   // humidity 
  dtostrf(bmeHumidityValue,3,0,humidityCase);
}

void sendData()
{
  #ifdef debug
    Serial.println("Sending Data!");
    Serial.println("Connecting Wifi...");
  #endif
  if(wifiMulti.run() == WL_CONNECTED) {
    #ifdef debug
      Serial.println();
      Serial.print("WiFi connected, IP address: ");
      Serial.println(WiFi.localIP());
    #endif
  }
  
  String uploadUrl = String("http://your-domain.net/orchard/upload.php?gardener=your-gardener&id=your-id&node=apple-01dataset=timestamp-by-server," + String(soilMoisturePercent) + "," + String(temperatureC));

  // replace spaces with plus for proper URL 
  for (int i = 0; uploadUrl[i] != 0; i++) {
    if (uploadUrl[i] == ' ') uploadUrl[i] = '+';
  }
    
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(uploadUrl.c_str());
    
    // Send HTTP GET request
    int httpResponseCode = http.GET();
    
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}

// switched power control
void VextON(void)
{
  digitalWrite(Vext, LOW);
  delay(100);
}

void VextOFF(void)  // Vext default OFF
{
  digitalWrite(Vext, HIGH);
}


void setup() {
  #ifdef debug
    Serial.begin(115200);
    delay(100);
    
    Serial.println();
    Serial.println();
  #endif

  wifiMulti.addAP("your-ssid-1", "your-pw-1");
  wifiMulti.addAP("your-ssid-2", "your-pw-2");

  // power switch pin as output
  pinMode(Vext, OUTPUT);

  // switech power on 
  VextON();
  
  // button as input
  pinMode(buttonPin, INPUT);

  // BME280 
//  Wire.begin();
  Wire.begin (4, 15);  // sda, scl
  bme280.setI2CAddress(0x76);   // set I2C address, default for the lib (without this line of code) is 0x77
  bme280.beginI2C();
  bme280.setMode(MODE_SLEEP);   // sleep for now

  // Dallas temperature library
  sensors.begin();

  // Must be called before search()
  oneWire.reset_search();
  // assigns the first address found to "TempSensor" sensor
  if (!oneWire.search(TempSensor)) {
    #ifdef debug
      Serial.println("unable to find DS18B20 address");
    #endif
  }
  
  // temperature
  sensors.setResolution(TempSensor, TEMPERATURE_PRECISION);   // set the resolution per device
  // get an initial value, setWaitForConversion is by dafault true so it will take 1 second
  sensors.requestTemperatures(); 
  readSensors();
  // after we have a reading switch to non blocking method 
  sensors.setWaitForConversion(false);

  
  // OLED
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);

	// The ESP is capable of rendering 60fps in 80Mhz mode
	// but that won't give you much time for anything else
	// run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(30);

	// Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // enable or disable auto transition
//  ui.enableAutoTransition();
  ui.disableAutoTransition();  // use e.g. ui.nextFrame(); for manual switching

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(RIGHT);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
//  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();
  Heltec.display->flipScreenVertically();
}


void loop() {
  // read sensors every x secons / sensorReadInterval
  if ((millis() - lastSensorReadTime) > sensorReadInterval) {
    readSensors();
    lastSensorReadTime = millis();  // reset counter 
  }

  // submit sensor data every x minuts / sendDataInterval
  if ((millis() - lastSendDataTime) > sendDataInterval) {
    sendData();
    lastSendDataTime = millis();  // reset counter 
  }
  
  // OLED
  ui.update();
  
/* for ui.enableAutoTransition();
  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
    delay(remainingTimeBudget);
  }
*/

  // button 
  int reading = digitalRead(buttonPin); // read button state

  // check debounce 
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed
    if (reading != buttonState) {
      buttonState = reading;

      // only switch frame if the new button state is truely HIGH
      if (buttonState == LOW) {
        ui.nextFrame();
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
