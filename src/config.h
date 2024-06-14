#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include "MQUnifiedsensor.h"
#include <HardwareSerial.h>
#include <U8g2lib.h>
//#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "BluetoothSerial.h"
#include <SD.h>
#include <SPI.h>
#include "DFRobot_PH.h"
#include <time.h>
#include <nvs_flash.h>
#include <nvs.h>
//#include "esp_attr.h"
#include "DFRobot_ESP_EC.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "freertos/ringbuf.h"
#include "soc/lldesc.h"

void BluetoothListen(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray(void *parameter);
void counting(void *parameter);



const int MaxMeasurements = 50;
struct Measurement {
  float phValue;
  float ecValue;
  float AcsValueF;
  float flowRate;
  float flowRate2;
  float Volt;
  float DS18B20_1;
  float DS18B20_2;
  float DS18B20_3;
  float DS18B20_4;
  float DS18B20_5;
  float temperature;
  float humidity;
  float ppmH;
  float ppmCO;
  uint64_t ts;
}; 
extern Measurement measurement[MaxMeasurements];
extern QueueHandle_t measurementQueue; // Define the queue handle
void printCMD();
void printCMDList(Measurement list);

/*      Configuration     */
extern bool sendhttp;
extern String payload;

/*      Display      */
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C 
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled;

/*      MQ-7 MQ-8 sensor       */
void mq7_init(MQUnifiedsensor& MQ7);
void mq8_init(MQUnifiedsensor& MQ8);

//For GSMSerial output on OLED
extern U8G2LOG u8g2log;
extern volatile int stateBigOled;

/*      GSM Functions    */
void saveTimestamp(uint64_t timestamp);
uint64_t getSavedTimestamp();
extern uint64_t savedTimestamp;
void parseDatetime();
//extern HardwareSerial gsmSerial;

extern String apn, apn_User, apn_Pass;
extern char httpapi[];

//void GA6_init();
void getTime();
void getTimeNow();  
void post_http(String j); 
time_t convertToUnixTimestamp(String date, String time);
void readGsmResponse();
void initialize_gsm();
void initialize_gsm2();


/*      Display Setup     */
void printBigOled(String x);
void init_displays();


/*      DS18B20 sensor       */
//extern struct Measurement measurement;
extern const int DS18B20_PIN;
void printDS18B20Address();
void AllDS18B20Sensors(Measurement& measurement);
//void AllDS18B20Sensors(Measurement& measurement);

/*      Flow sensor       */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num);
extern float flowRate, flowRate2;

/*      Bluetooth          */
extern BluetoothSerial SerialBT;
extern String message;
extern char incomingChar;
void sendFileOverBluetooth(const char* path);
void readFileAndSendOverBluetooth(fs::FS &fs, const char *path);
void sendFileOverBluetoothInOneGo2(const char* path);

/*        SD Card         */
extern int CS_PIN;
void SD_init();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
//void createDir(fs::FS &fs, const char * path);
//void removeDir(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
//void renameFile(fs::FS &fs, const char * path1, const char * path2);
//void deleteFile(fs::FS &fs, const char * path);
//void testFileIO(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void logMeasurement(String measurement);
void sendFileOverBluetoothInOneGo(const char* path);

/*      Configuration     */
extern int buttonbigOled, buttonsmallOled;
void buttonInterrupt_bigOled();
void buttonInterrupt_smallOled();

/*      Ph Sensor         */
extern DFRobot_PH ph;
extern int PH_PIN;
float pH();
float readTemperature();

/*      Conductivity Sensor   */
extern DFRobot_ESP_EC ec;
//extern float voltage_cond, temperature_cond;
extern int EC_PIN; // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
float Cond();

/*      Current Sensor   */
#define ACS712_30A_SENSITIVITY    66      // mV/A

enum ACS712XXType{
  ACS712_30A = 2,
  ACS712XX_N = 3
};

class ACS712XX{
  public:
    ACS712XX(ACS712XXType _type, uint8_t _pin, uint8_t _adc_ref = 5 ,int _adc_fs = 4096);
    float autoCalibrate();
    float getAC(float _freq = 50.0,float _n_total_period = 1.0);
    float getDC(int _count = 10);
    float getOffset()                 { return offset;                  }
    float getSensitivity()            { return 1000.0/inv_sensitivity;  }
    void setOffset(float _offset)     { offset = _offset;               }
    void setSensitivity(float _sen);
    void reset();

  private:
    uint8_t sensor_sen = 66;
    float inv_sensitivity;
    float results_adjuster;
    float offset;
    int adc_fs;
    uint8_t ACS712XX_pin;
    ACS712XXType type;
};

extern int CurrentPin;
float CurrentSensor_quick();

#endif // CONFIG_H
