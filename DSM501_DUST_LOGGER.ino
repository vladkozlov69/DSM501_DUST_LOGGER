#include <RTClib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>

#define SD_CS 4

RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27,16,2);

File logFile;

char filePath[25];
char dataBuf[12];
bool sdReady = false;

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
  Serial.begin(9600);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM10,INPUT);
  pinMode(DUST_SENSOR_DIGITAL_PIN_PM25,INPUT);

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


  // wait 60s for DSM501 to warm up
  for (int i = 1; i <= 6; i++)
  {
    delay(1000); // 1s
    lcd.setCursor(0,1);
    lcd.print(i);
    lcd.print(" s warming");
    Serial.print(i);
    Serial.println(" s (wait 60s for DSM501 to warm up)");
  }

  lcd.setCursor(0,1);
  lcd.print("Ready!          ");
  Serial.print("Ready!");
  Serial.println(sdReady ? "SD OK" : "NO SD");

  attachInterrupt(digitalPinToInterrupt(DUST_SENSOR_DIGITAL_PIN_PM10), change10, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DUST_SENSOR_DIGITAL_PIN_PM25), change25, CHANGE);

}

void loop()
{
  if (pm10_ready)
  {
    pm10_lowPulseOccupance = pm10_lowPulseOccupance + pm10_data;
    pm10_data = 0;
    pm10_ready = false;
    //Serial.print("pm10_low"); Serial.println(pm10_lowPulseOccupance);
  }

  if (pm25_ready)
  {
    pm25_lowPulseOccupance = pm25_lowPulseOccupance + pm25_data;
    pm25_data = 0;
    pm25_ready = false;
    //Serial.print("pm25_low"); Serial.println(pm25_lowPulseOccupance);
  }

  if (millis() - sdLoggingInterval > sampletime_ms)
  {
    float ratio10 = pm10_lowPulseOccupance/(sampletime_ms * 10.0);
    float ratio25 = pm25_lowPulseOccupance/(sampletime_ms * 10.0);
    Serial.print("R10: "); Serial.print(ratio10); Serial.print(" | R25: "); Serial.println(ratio25);

    lcd.setCursor(0,1);
    lcd.print("R10/25:"); lcd.print(ratio10, 2); lcd.print("/"); lcd.print(ratio25, 2); lcd.print("     ");

    if (sdReady)
    {
      DateTime now = rtc.now();
      sprintf(filePath, "/DSM501/%04d%02d%02d.csv", now.year(), now.month(), now.day());
      logFile = SD.open(filePath, FILE_WRITE);

      if (logFile)
      {
        sprintf(dataBuf,"%04d-%02d-%02d", now.year(), now.month(), now.day());
        logFile.print(dataBuf);
        logFile.print(" ");

        sprintf(dataBuf,"%02d:%02d", now.hour(), now.minute());
        logFile.print(dataBuf);
        logFile.print(",");

        logFile.print(ratio10, 2);
        logFile.print(",");

        logFile.println(ratio25, 2);
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

