#include "config.h"
/*      DS18B20 sensor            */
OneWire oneWire(DS18B20_PIN);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
int numberOfDevices;                  // Number of temperature devices found
DeviceAddress tempDeviceAddress;      // We'll use this variable to store a found device address
volatile float DS18B20_1, DS18B20_2,DS18B20_3, DS18B20_4, DS18B20_5 = 0.0; // Initialize to a default value

//DS18B20 Find and print Address
void printDS18B20Address() {
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();  
  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++) {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");      
      // Print the address
      for (uint8_t j = 0; j < 8; j++) {
          if (tempDeviceAddress[j] < 16) Serial.print("0");
          Serial.print(tempDeviceAddress[j], HEX);
        }
        Serial.println();
      } else {
        Serial.print("Found ghost device at ");
        Serial.print(i, DEC);
        Serial.print(" but could not detect address. Check power and cabling");
        Serial.println();
      }
    }
}
// Loop through each device, print out DS18B20 temperature data
void AllDS18B20Sensors(Measurement& measurement) {
  sensors.requestTemperatures(); // Send the command to get temperatures
  int numberOfDevices = sensors.getDeviceCount();
  for(int i = 0; i < numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {
        // Print the data
        float tempC = sensors.getTempC(tempDeviceAddress);
        // Assign readings to variables Tempc1 and Tempc2
        if(i == 0) {
            measurement.DS18B20_1 = tempC;
        } else if(i == 1) {
            //DS18B20_2 = tempC;
            measurement.DS18B20_2 = tempC;
        }
        else if(i == 2) {
            //DS18B20_3 = tempC; 
            measurement.DS18B20_3 = tempC; 
        }
        else if(i == 3) {
            //DS18B20_4 = tempC;
            measurement.DS18B20_4 = tempC;
        }
        else if(i == 4) {
            //DS18B20_5 = tempC;
            measurement.DS18B20_5 = tempC;
        }
    }
  }  
  sensors.setResolution(9);
}


/*              Setup Flowsensor    */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal GPIO
        .pulse_gpio_num = pulse_gpio_num,
        // No control GPIO needed
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Ignore negative edge
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 0,
        .counter_l_lim = 0,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}


/*      MQ-7 MQ-8 sensor            */
float RatioMQ7CleanAir = 27.5;
float RatioMQ8CleanAir = 70.0;
void mq7_init(MQUnifiedsensor& MQ7)
{
     vTaskDelay(500 / portTICK_PERIOD_MS);
    // CO
    MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
    // MQ7.setA(521853); MQ7.setB(-3.821); // Configurate the ecuation values to get Benzene concentration
    MQ7.setA(99.042);
    MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value
    MQ7.init();
    vTaskDelay(5 / portTICK_PERIOD_MS);

    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ7.update(); // Update data, the arduino will be read the voltage on the analog pin
        calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
        Serial.print(".");
    }
    MQ7.setR0(calcR0 / 10);
    Serial.println("Done calculating R0 for MQ7!.");
    /*
      //If the RL value is different from 10K please assign your RL value with the following method:
      MQ7.setRL(9.87);
    */
    if (isinf(calcR0))
    {
        Serial.println("MQ7 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    } // while(1);
    if (calcR0 == 0)
    {
        Serial.println("MQ7 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    } // while(1);
    /*****************************  MQ CAlibration ********************************************/
    Serial.println("MQ7 initialized!");    
    MQ7.serialDebug(true);
}

void mq8_init(MQUnifiedsensor& MQ8)
{
     vTaskDelay(500 / portTICK_PERIOD_MS);
    // Hydrogen
    MQ8.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ8.setA(976.97);
    MQ8.setB(-0.688); // Configure the equation to to calculate H2 concentration
    MQ8.init();
    Serial.print("Calibrating MQ8 please wait.");
    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ8.update(); // Update data, the arduino will read the voltage from the analog pin
        calcR0 += MQ8.calibrate(RatioMQ8CleanAir);
        Serial.print(".");
    }
    MQ8.setR0(calcR0 / 10);
    Serial.println("R0 for MQ8 calculation done!.");

    if (isinf(calcR0))
    {
        Serial.println("MQ8 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    }
    if (calcR0 == 0)
    {
        Serial.println("MQ8 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    }
    /*****************************  MQ CAlibration ********************************************/
    Serial.println("MQ8 initialized!");    
    MQ8.serialDebug(true);
}

/*      Display Setup               */
/*      Switching screens           */
volatile bool buttonBigPressed, buttonSmallPressed = false;

U8G2_SSD1306_128X64_NONAME_1_HW_I2C bigOled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);
//U8G2_SH1106_128X64_NONAME_1_HW_I2C bigOled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

//U8G2_SH1106_128X64_NONAME_1_HW_I2C
//For GSMSerial output on OLED
#define U8LOG_WIDTH 12 //25
#define U8LOG_HEIGHT 6 //8+
uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
U8G2LOG u8g2log;

void init_displays(){  
  bigOled.setI2CAddress(0x3C * 2);  bigOled.setBusClock(400000); 

  //Wire.setClock(100000); 

  bigOled.begin();  
  bigOled.clearBuffer();
  bigOled.setFont(u8g2_font_6x12_mf);	// set the font for the terminal window
  bigOled.setDisplayRotation(U8G2_R1);
  u8g2log.begin(bigOled, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(2);	// set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(1);		// 0: Update screen with newline, 1: Update screen for every char 
  Serial.println("Displays initialized!");      
  //Serial.println("Big Display height: " + bigOled.getDisplayHeight() + " Big Display Width: "  + bigOled.getDisplayHeight());
}

void printBigOled(String x){
  bigOled.firstPage();
  do {
    bigOled.setFont(u8g2_font_4x6_mr); 
    bigOled.drawStr(0, 20, "Flow: ");
    bigOled.drawStr(0, 40, x.c_str());
  } while (bigOled.nextPage()); 
}

/*              Setup Currentsensor    */
ACS712XX :: ACS712XX(ACS712XXType _type, uint8_t _pin, uint8_t _adc_ref, int _adc_fs) {
  type = _type;
  adc_fs = _adc_fs;
  
  // Define sensor pin
  ACS712XX_pin = _pin;

  // Define sensor offset
  offset = _adc_fs/2.0;
  
  // Define sensor sensitivity according to the sensor type.
  inv_sensitivity = 1000.0 / float(sensor_sen);  // V/A -> A/V

  results_adjuster = float(_adc_ref) / float(_adc_fs);  // V/ADC
  results_adjuster *= inv_sensitivity;                  // V/ADC x A/V -> A/ADC
}

float ACS712XX :: autoCalibrate() {  // required time, around 1 ms
  int _adc = 0,_sample = 0;
  while (_sample < 10) {
    _adc += analogRead(ACS712XX_pin);
    _sample ++;
  }
  
  offset = float(_adc) * 0.1; // average of 10 samples
  return offset;
}

/* Description  : This function measures the AC current.
 * Argument (s) : Frequency, Number of measurement period 
 * Default  (s) : Frequency -> 50, Number of measurement period -> 1.0
 * Arg.(s) Range: Frequency -> Around 1 to 200 Hz, Number of measurement period -> 0.5 to 20(for 50Hz)
 * Return       : AC current in Ampere
 */
float ACS712XX :: getAC(float _freq, float _n_total_period) {  // It can measure minimum 0.5 maximum
  float _signal_period = 1000000.0 / _freq;                    // Hz -> us
  _signal_period *= _n_total_period;
  
//  Here, +8us confirms the last iteration. According to Arduino documentation,
//  the micros() function has a resolution of 4us.
  unsigned long _total_time = (unsigned long)_signal_period + 8;
  unsigned long _start_time = micros();

  unsigned long _adc_sqr_sum = 0;
  unsigned int _adc_count = 0;
  long _adc;

  while (micros() - _start_time < _total_time) { // it capable for maximum 4096 samples.
    _adc_count++;
    _adc = analogRead(ACS712XX_pin) - offset;
    _adc_sqr_sum += _adc*_adc;
  }
  
  float _avg_adc_sqr_sum = _adc_sqr_sum / (float)_adc_count;
  float _rms_current = sqrt(_avg_adc_sqr_sum) * results_adjuster;   // ADC x A/ADC -> A
  return _rms_current;
}

float ACS712XX :: getDC(int _count) {  // required time, around 1 ms of default 10 samples.
  int _adc = 0, _sample = 0;
  while (_sample < _count) {
    _adc += analogRead(ACS712XX_pin);
    _sample ++;
  }
  
  float _adc_avg = (float(_adc) / float(_count)) - offset;  // average of 10 samples and remove offset
  float _current = _adc_avg * results_adjuster;   // ADC x A/ADC -> A
  return _current;
}

void ACS712XX :: setSensitivity(float _sen) {
  results_adjuster /= inv_sensitivity;                  // A/ADC / A/V -> V/ADC
  inv_sensitivity = 1000.0 / _sen;                      // V/A -> A/V
  results_adjuster *= inv_sensitivity;                  // V/ADC x A/V -> A/ADC
}

void ACS712XX :: reset() {
  float _sen = float(sensor_sen);
  setSensitivity(_sen);
  offset = adc_fs/2.0;
}

float CurrentSensor_quick() {
    float current_voltage, current = 0.0;
    
    float R1 = 1000.0;
    float R2 = 2000.0;

    const int numSamples = 100;
    float adc_voltage_sum = 0.0;

    // Read ADC value multiple times to average
    for (int i = 0; i < numSamples; i++) {
        int adc = analogRead(CurrentPin);
        adc_voltage_sum += adc * (3.3 / 4095.0);
        delay(1);  // Small delay to allow for better averaging
    }

    // Average the ADC voltage
    float adc_voltage = adc_voltage_sum / numSamples;
    Serial.println("ADC voltage: " + String(adc_voltage));

    // Calculate the sensor voltage
    current_voltage = (adc_voltage *  R2) / (R1 + R2) ;
    Serial.println("Current voltage: " + String(current_voltage));

    // Measure this value when no current is flowing to calibrate zeroCurrentVoltage
    float zeroCurrentVoltage = 0.48;  // Use the previously measured value or measure again

    // ACS712 sensitivity (e.g., 185mV/A for ACS712-05B)
    float sensitivity = 0.066;  // Change this value based on your specific ACS712 model

    // Calculate the current
    current = (current_voltage - zeroCurrentVoltage) / sensitivity;
    Serial.println("Current: " + String(current));

    return current;
}

/*      Conductivity Sensor   */
DFRobot_ESP_EC ec;
volatile float voltage_cond, temperature_cond = 25;  // variable for storing the potentiometer value
extern int EC_PIN;
float ecValueFloat = 0;
float Cond(){
  static unsigned long timepoint = millis();
	if (millis() - timepoint > 1000U) //time interval: 1s
	{
		timepoint = millis();
		voltage_cond = analogRead(EC_PIN)/4095.0*3300;
		//Serial.println("voltage: " + String(voltage_cond, 4));
		//temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
		//Serial.println("voltage_cond: " String(temperature_cond, 1));
		//Serial.println("^C");

		ecValueFloat = ec.readEC(voltage_cond, temperature_cond); // convert voltage to EC with temperature compensation
		//Serial.println("EC: " + String(measurement.ecValue, 4) + " ms/cm");
	}
	ec.calibration(voltage_cond, temperature_cond); // calibration process by Serail CMD

  //Serial.print(ecValue,2);  Serial.println("ms/cm");
  return ecValueFloat;
}

/*      pH Sensor             */
float voltage_pH, phValue, temperature_pH = 25;
float pH(){
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime >= 1000) {  // tijdsinterval: 1s
        lastTime = currentTime;
        
        temperature_pH = readTemperature();  // Lees de temperatuur om temperatuurcompensatie uit te voeren
        voltage_pH = analogRead(PH_PIN) / 1024.0 * 5000;  // Lees de spanning
        phValue = ph.readPH(voltage_pH, temperature_pH);  // Converteer spanning naar pH met temperatuurcompensatie
        
        //Serial.print("Temperature: ");
        //Serial.println(temperature_pH, 1);
        //Serial.print("°C  pH: ");
        //Serial.println(phValue, 2);
        
        ph.calibration(voltage_pH, temperature_pH);  // Kalibratieproces via Seriële CMD
    }
  return phValue;
}
 
float readTemperature() {
    // Voeg hier je code toe om de temperatuur van je temperatuursensor te krijgen
    // Bijvoorbeeld, als je een LM35 gebruikt:
    // int rawValue = analogRead(TEMP_SENSOR_PIN);
    // float millivolts = (rawValue / 1024.0) * 5000;
    // return millivolts / 10;
    // kan nog standaard waarde zoals 25 graden
    
    //Serial.println("Received 1st temperature: " + String(a));
    //Serial.println("Received 2nd temperature: " + String(b));
    // Als placeholder, retourneer een constante waarde:
    return 25.0;
}
/*      SD card       */
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void SD_init(){
  if (!SD.begin(CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        //Light up RED LED
    }
    Serial.println("SD Card initialized.");
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    //Light up RED LED
    //return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  File file = SD.open("/log.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/log.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

/*


void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}


void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}
*/
/*      Bluetooth Setup   */
void sendFileOverBluetooth(const char* path) {    
        File file = SD.open(path, FILE_READ);
        if (!file) {
            Serial.println("Failed to open file for reading");
            return;
        }
        while (file.available()) {
            SerialBT.write(file.read());
        }
        file.close();
        Serial.println("File sent over Bluetooth");    
}

void logMeasurement(String s) {   
        File dataFile = SD.open("/log.txt", FILE_APPEND);
        if (dataFile) {
            dataFile.println(s);
            dataFile.close();
            Serial.println("Data written to file.");
        } else {
            Serial.println("Error opening file for writing.");
        }
}

void readFileAndSendOverBluetooth(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  while (file.available()) {
    SerialBT.write(file.read());
  }

  file.close();
}
void sendFileOverBluetoothInOneGo(const char* path) {
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading: " + String(path));
        return;
    }

    size_t fileSize = file.size();
    uint8_t* buffer = new uint8_t[fileSize];

    if (buffer == nullptr) {
        Serial.println("Failed to allocate memory for buffer");
        file.close();
        return;
    }

    size_t bytesRead = file.read(buffer, fileSize);
    if (bytesRead != fileSize) {
        Serial.println("Failed to read file: " + String(path) + ", bytesRead: " + String(bytesRead) + ", fileSize: " + String(fileSize));
        delete[] buffer;
        file.close();
        return;
    }

    file.close();

    SerialBT.write(buffer, fileSize);
    delete[] buffer;

    Serial.println("File sent over Bluetooth: " + String(path));
}
void sendFileOverBluetoothInOneGo2(const char* path) {
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    size_t fileSize = file.size();
    uint8_t* buffer = new uint8_t[fileSize];

    if (file.read(buffer, fileSize) != fileSize) {
        Serial.println("Failed to read file");
        delete[] buffer;
        file.close();
        return;
    }

    file.close();

    SerialBT.write(buffer, fileSize);
    delete[] buffer;

    Serial.println("File sent over Bluetooth");
}

unsigned long button_time = 0;  
unsigned long last_button_time = 0; 
void buttonInterrupt_bigOled() {
  button_time = millis();
  if(button_time - last_button_time > 500){
    last_button_time = button_time;
    buttonBigPressed = true; // Set button press flag        
  }
}

void buttonInterrupt_smallOled() {
  buttonSmallPressed = true; // Set button press flag
}
