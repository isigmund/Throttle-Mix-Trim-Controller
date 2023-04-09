#include <Arduino.h>
#include "Joystick.h"
#include <ADS1115_WE.h>
#include <Wire.h>
#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg


#define I2C_ADDRESS 0x48
//#define SERIAL_OUT 
#define troMax 8192
#define mixMax 8192
#define trmMax 8192
#define DEADZONE 2

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, // hidReportId
                   JOYSTICK_TYPE_JOYSTICK,     // joystickType
                   0,                          // buttonCount
                   0,                          // hatSwitchCount
                   false,                      // includeXAxis
                   false,                      // includeYAxis
                   false,                      // includeZAxis
                   true,                       // includeRxAxis
                   true,                       // includeRyAxis
                   true,                       // includeRzAxis
                   false,                      // includeRudder
                   false,                      // includeThrottle
                   false,                      // includeAccelerator
                   false,                      // includeBrake
                   false                       // includeSteering
);


// Initializing Axis as Integers, at a 0 default value
int32_t throttleAverage = 0;
int32_t mixtureAverage = 0;
int32_t trimAverage = 0;

int32_t prevThrottleAverage = 0;
int32_t prevMixtureAverage = 0;
int32_t prevTrimAverage = 0;

// Set Auto Send State
// Enables Auto Sending, allowing the controller to send information to the HID system, rather than waiting to be asked.
const bool initAutoSendState = true;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);


// initialize Averager objects
movingAvg troAverager(32);                // define the moving average object
movingAvg mixAverager(32);                // define the moving average object
movingAvg trmAverager(32);                // define the moving average object


int32_t readChannelRaw(ADS1115_MUX channel)
{
  int32_t value = 0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while (adc.isBusy())  {
  }
  value = adc.getRawResult();
  return value;
}

void setup()
{
  Wire.begin();
  #ifdef SERIAL_OUT
  Serial.begin(9600);
  #endif

  if (!adc.init())
  {
    #ifdef SERIAL_OUT
    Serial.println("ADS1115 not connected!");
    #endif
  }

 


  troAverager.begin();	
  trmAverager.begin();	
  mixAverager.begin();	


  // setup ADC
  adc.setConvRate(ADS1115_860_SPS);   //uncomment if you want to change the default

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   *
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); // comment line/change parameter to change range

  // put your setup code here, to run once:
  Joystick.setRzAxisRange(0, troMax); // 23767   throttle
  Joystick.setRxAxisRange(0, mixMax); // 23767   mixture
  Joystick.setRyAxisRange(0, trmMax); // 23767   trim

  // Start Joystick - Needed to start the Joystick function libary
  Joystick.begin();
}

void loop()
{
  int32_t throttle = map(readChannelRaw(ADS1115_COMP_0_GND), -40, 24300, 0, troMax);
  throttleAverage = troAverager.reading(throttle);
  if (prevThrottleAverage < throttleAverage - DEADZONE || prevThrottleAverage > throttleAverage - DEADZONE ) {
    #ifdef SERIAL_OUT  
      Serial.print(" troAverage: ");
      Serial.println(throttleAverage);  
    #endif
    Joystick.setRzAxis(throttleAverage);
    prevThrottleAverage = throttleAverage;
  }  


  int32_t mixture = map(readChannelRaw(ADS1115_COMP_1_GND), -40, 24300, 0, mixMax);
  mixtureAverage = mixAverager.reading(mixture);
  if (prevMixtureAverage < mixtureAverage - DEADZONE || prevMixtureAverage > mixtureAverage + DEADZONE) {
    #ifdef SERIAL_OUT
      Serial.print(" mixAverage: ");
      Serial.println(mixtureAverage);  
    #endif
    Joystick.setRxAxis(mixtureAverage);
    prevMixtureAverage = mixtureAverage;
  }


  int32_t trim = map(readChannelRaw(ADS1115_COMP_2_GND), 3790, 18880, 0, trmMax);
  trimAverage = trmAverager.reading(trim);
  if (prevTrimAverage < trimAverage - DEADZONE || prevTrimAverage > trimAverage + DEADZONE) {
    #ifdef SERIAL_OUT
      Serial.print(" trmAverage: ");
      Serial.println(trimAverage);
    #endif    
    Joystick.setRyAxis(trimAverage);
    prevTrimAverage = trimAverage;
  }
   
  // Pole Delay/Debounce
  // To reduce unessecary processing, the frequency of the reading loop is delayed. The value(in ms) can be changed to match requirement
  delay(10);
}