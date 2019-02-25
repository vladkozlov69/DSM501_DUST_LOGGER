#include <RTClib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include "MovingAverage.h"

#define SD_CS 4


RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27,16,2);


File logFile;

char filePath[25];
char dataBuf[20];
bool sdReady = false;

MovingAverage ma10(5);
MovingAverage ma25(5);

// begin DSM501 sensor
#define DUST_SENSOR_DIGITAL_PIN_PM10  3        // DSM501 Pin 2
#define DUST_SENSOR_DIGITAL_PIN_PM25  2        // DSM501 Pin 4

unsigned long sampletime_ms = 60000;
unsigned long sdLoggingInterval = 0;

volatile unsigned long pm10_start = -1;
volatile unsigned long pm10_data;
volatile bool pm10_ready = false;
unsigned long pm10_lowPulseOccupance = 0;

volatile unsigned long pm25_start = -1;
volatile unsigned long pm25_data;
volatile bool pm25_ready = false;
unsigned long pm25_lowPulseOccupance = 0;


void change10()
{
	if (digitalRead(DUST_SENSOR_DIGITAL_PIN_PM10) == 0)
	{
		pm10_start = micros();
		pm10_ready = false;
	}
	else
	{
		if (pm10_start > 0)
		{
			pm10_data = micros() - pm10_start;
			pm10_start = -1;
			pm10_ready = true;
		}
	}
}

void change25()
{
	if (digitalRead(DUST_SENSOR_DIGITAL_PIN_PM25) == 0)
	{
		pm25_start = micros();
		pm25_ready = false;
	}
	else
	{
		if (pm25_start > 0)
		{
			pm25_data = micros() - pm25_start;
			pm25_start = -1;
			pm25_ready = true;
		}
	}
}

void setup()
{
//  Serial.begin(9600);



  pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT_PULLUP);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  rtc.begin();

  sdReady = SD.begin(SD_CS);

  if (sdReady)
  {
    SD.mkdir("/DSM501");
  }

  lcd.setCursor(0,0);
  lcd.print(sdReady ? "SD OK" : "NO SD");

  attachInterrupt(digitalPinToInterrupt(DUST_SENSOR_DIGITAL_PIN_PM10), change10, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DUST_SENSOR_DIGITAL_PIN_PM25), change25, CHANGE);

  // wait 60s for DSM501 to warm up
  for (int i = 1; i <= 6; i++)
  {
    delay(1000); // 1s
    lcd.setCursor(0,1);
    lcd.print(i);
    lcd.print(" s warming");
//    Serial.print(i);
//    Serial.println(" s (wait 60s for DSM501 to warm up)");
  }

  lcd.setCursor(0,1);
  lcd.print("Ready!          ");
//  Serial.print("Ready!");
//  Serial.println(sdReady ? "SD OK" : "NO SD");

  pm10_data = 0;
  pm10_ready = false;
  pm25_data = 0;
  pm25_ready = false;
}

void loop()
{
  if (pm10_ready)
  {
	  pm10_lowPulseOccupance = pm10_lowPulseOccupance + pm10_data;
	  pm10_data = 0;
	  pm10_ready = false;
  }

  if (pm25_ready)
  {
	  pm25_lowPulseOccupance = pm25_lowPulseOccupance + pm25_data;
	  pm25_data = 0;
	  pm25_ready = false;
  }

  if (millis() - sdLoggingInterval > sampletime_ms)
  {
    float ratio10 = pm10_lowPulseOccupance/(sampletime_ms * 10.0);
    float ratio25 = pm25_lowPulseOccupance/(sampletime_ms * 10.0);
//    Serial.print("R10: "); Serial.print(ratio10); Serial.print(" | R25: "); Serial.println(ratio25);

    lcd.setCursor(0,1);
    lcd.print("R10/25:");
    lcd.print(ratio10, 2); lcd.print("/"); lcd.print(ratio25, 2); lcd.print("     ");

    if (sdReady && (ratio10 < 90.0) && (ratio25 < 90.0))
    {
      DateTime now = rtc.now();
      sprintf(filePath, "/DSM501/%04d%02d%02d.csv", now.year(), now.month(), now.day());
      logFile = SD.open(filePath, FILE_WRITE);

      if (logFile)
      {
//    	  // dustduino -----
//          double countP1 = 1.1*pow(ratio10,3)-3.8*pow(ratio10,2)+520*ratio10+0.62;
//          double countP2 = 1.1*pow(ratio25,3)-3.8*pow(ratio25,2)+520*ratio25+0.62;
//          float PM10count = countP2;
//          float PM25count = countP1 - countP2;
//
//          // first, PM10 count to mass concentration conversion
//          double r10 = 2.6*pow(10,-6);
//          double pi = 3.14159;
//          double vol10 = (4/3)*pi*pow(r10,3);
//          double density = 1.65*pow(10,12);
//          double mass10 = density*vol10;
//          double K = 3531.5;
//          float concLarge = (PM10count)*K*mass10;
//
//          // next, PM2.5 count to mass concentration conversion
//          double r25 = 0.44*pow(10,-6);
//          double vol25 = (4/3)*pi*pow(r25,3);
//          double mass25 = density*vol25;
//          float concSmall = (PM25count)*K*mass25;
//          // dustduino -----

        sprintf(dataBuf,"%04d-%02d-%02d", now.year(), now.month(), now.day());
        logFile.print(dataBuf);
        logFile.print(" ");

        sprintf(dataBuf,"%02d:%02d", now.hour(), now.minute());
        logFile.print(dataBuf);
        logFile.print(",");

        logFile.print(ratio10, 2);
        logFile.print(",");

        logFile.print(ratio25, 2);
        logFile.print(",");

        logFile.print(ma10.process(ratio10), 2);
        logFile.print(",");

        logFile.print(ma25.process(ratio25), 2);
        logFile.println();



//        logFile.print(concLarge, 3);
//        logFile.print(",");
//
//        logFile.println(concSmall, 3);

        logFile.close();

        lcd.setCursor(10, 0);
        lcd.print(dataBuf);
      }
      else
      {
        lcd.setCursor(0,0);
        lcd.print("NO SD");
      }
    }

    pm10_lowPulseOccupance = 0;
    pm25_lowPulseOccupance = 0;
    sdLoggingInterval = millis();
  }

}

//void hackAIR::humidityCompensation(hackAirData &data, float humidity) {
//  data.pm25 = data.pm25 / (1.0 + 0.48756 * pow((humidity / 100.0), 8.60068));
//  data.pm10 = data.pm10 / (1.0 + 0.81559 * pow((humidity / 100.0), 5.83411));
//}

