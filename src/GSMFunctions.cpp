#include "config.h"
extern HardwareSerial gsmSerial;
volatile int stateBigOled = 1;
String response, longitude, latitude, date, time_gsm, jsonPayload = "";

void readGsmResponse() {
    char c;
    response = "";
    unsigned long startTime = millis();
    unsigned long lastReadTime = millis();
    const unsigned long readTimeout = 500; // Time to wait for new data in milliseconds

    while (millis() - startTime < 10000) { // Overall timeout after 10 seconds (was 5 but httpdata didn't work correctly)
        while (gsmSerial.available() > 0) {
            uint8_t byteFromSerial = gsmSerial.read();
            c = char(byteFromSerial);
            response += c;
            Serial.write(byteFromSerial);
            if(stateBigOled==1){
            u8g2log.print(c); 
            }
            lastReadTime = millis(); // Update last read time
            //vTaskDelay(1/portTICK_PERIOD_MS);//delay(5);
        }

        // Check if there's been no data read for the readTimeout duration
        if (millis() - lastReadTime > readTimeout) {
            break;
        }
    }
}

void parseDatetime() {
  bool validResponse = false;
  while (!validResponse) {
    Serial.println("Requesting datetime...");
    gsmSerial.println("AT+CIPGSMLOC=1,1");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    readGsmResponse();
    String resp = response;
    vTaskDelay(100 / portTICK_PERIOD_MS);

    int startIndex = resp.indexOf("+CIPGSMLOC: ");
    if (startIndex == -1) {
      Serial.println("Error: Invalid time, trying again...");
      Serial.println("Response: " + resp);
      continue;  // Retry
    }

    int endIndex = resp.indexOf("\r\n", startIndex);
    String data = resp.substring(startIndex + 12, endIndex);

    // Split the response string by commas
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    int thirdComma = data.indexOf(',', secondComma + 1);
    int fourthComma = data.indexOf(',', thirdComma + 1);

    if (firstComma == -1 || secondComma == -1 || thirdComma == -1 || fourthComma == -1) {
      Serial.println("Error: Malformed response, trying again...");
      continue;  // Retry
    }

    latitude = data.substring(firstComma + 1, secondComma);
    longitude = data.substring(secondComma + 1, thirdComma);
    date = data.substring(thirdComma + 1, fourthComma);
    time_gsm = data.substring(fourthComma + 1);

    // Check if the date and time are valid
    if (date.toDouble() == 0.000000 || time_gsm.toDouble() == 0.000000) {
      Serial.println("Error: Invalid date/time, trying again...");
      Serial.println("Parsed Date: " + date);
      Serial.println("Parsed Time: " + time_gsm);

      gsmSerial.println("AT+CIPGSMLOC=2,1");
      vTaskDelay(100 / portTICK_PERIOD_MS);
      readGsmResponse();
      resp = response;
      vTaskDelay(100 / portTICK_PERIOD_MS);

      startIndex = resp.indexOf("+CIPGSMLOC: ");
      if (startIndex == -1) {
        Serial.println("Error: Invalid time, trying again...");
        Serial.println("Response: " + resp);
        continue;  // Retry
      }

      endIndex = resp.indexOf("\r\n", startIndex);
      data = resp.substring(startIndex + 12, endIndex);

      // Split the response string by commas
      firstComma = data.indexOf(',');
      secondComma = data.indexOf(',', firstComma + 1);

      if (firstComma == -1 || secondComma == -1) {
        Serial.println("Error: Malformed response, trying again...");
        continue;  // Retry
      }

      date = data.substring(firstComma + 1, secondComma);
      time_gsm = data.substring(secondComma + 1);
    }

    // Check if the date and time are reasonable values
    //long long ts = convertToUnixTimestamp(date, time_gsm);
    //if (ts < 0) {
    //  Serial.println("Error: Invalid date/time, trying again...");
    //  Serial.println("Parsed ts: " + ts);
    //  continue;  // Retry
    //}

    // If all validations pass, set validResponse to true
    validResponse = true;
    Serial.println("Valid datetime received.");
    Serial.println("Parsed Date: " + date);
    Serial.println("Parsed Time: " + time_gsm);
    Serial.println("Parsed Latitude: " + latitude);
    Serial.println("Parsed Longitude: " + longitude);
    Serial.println("time_gsm created in parseDatetime: " + time_gsm);
  }
}

void getTime(){
    gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\", \"IP\""); // Sets the mode to GPRS
    vTaskDelay(500 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn); // Set APN parameters
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User); // Set APN username
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=3,1,\"PWD\","+ apn_Pass); // Set APN password
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=1,1"); // Open the carrier with previously defined parameters "start command"
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=2,1"); // Query the status of previously opened GPRS carrier 
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    readGsmResponse();
    vTaskDelay(100 / portTICK_PERIOD_MS);
   
    gsmSerial.println("AT+CIPGSMLOC=1,1");
    readGsmResponse();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("convertToUnixTimestamp with 1,1: ");
    parseDatetime();
    convertToUnixTimestamp(date, time_gsm);
    gsmSerial.println("AT+CIPGSMLOC=2,1");
    readGsmResponse();
    vTaskDelay(100 / portTICK_PERIOD_MS);    
}

void getTimeNow(){
    Serial.println("Get time now().");
    vTaskDelay(100/portTICK_PERIOD_MS);
    gsmSerial.println("AT+CIPGSMLOC=1,1");
    readGsmResponse();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    parseDatetime();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("Lat: " + latitude + " Long: " + longitude + " Date: " + date + " Time: " + time_gsm);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("convertToUnixTimestamp with 1,1: ");
    convertToUnixTimestamp(date, time_gsm);
    Serial.println("In case that didn't work: ");
    gsmSerial.println("AT+CIPGSMLOC=2,1");
    readGsmResponse();
    vTaskDelay(100 / portTICK_PERIOD_MS);   
    Serial.println("Lat: " + latitude + " Long: " + longitude + " Date: " + date + " Time: " + time_gsm);
    convertToUnixTimestamp(date, time_gsm);
    vTaskDelay(100 / portTICK_PERIOD_MS); 
}

void saveTimestamp(uint64_t timestamp_ms) {
    nvs_flash_init();
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_set_u64(my_handle, "timestamp_ms", timestamp_ms);
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

uint64_t getSavedTimestamp() {
    uint64_t timestamp_ms = 0;
    nvs_flash_init();
    nvs_handle_t my_handle;
    nvs_open("storage", NVS_READWRITE, &my_handle);
    nvs_get_u64(my_handle, "timestamp_ms", &timestamp_ms);
    nvs_close(my_handle);
    return timestamp_ms;
}

time_t convertToUnixTimestamp(String date, String time) {
    uint64_t timestamp_ms = 0;
  // Extract year, month, day from date
  int year = date.substring(0, 4).toInt();
  int month = date.substring(5, 7).toInt();
  int day = date.substring(8, 10).toInt();
  
  // Extract hour, minute, second, millisecond from time
  int hour = time.substring(0, 2).toInt();
  int minute = time.substring(3, 5).toInt();
  int second = time.substring(6, 8).toInt();
  int millisecond = time.substring(9, 12).toInt(); // Assuming time is formatted as HH:MM:SS.mmm

  // Create a tm struct
  struct tm t;
  memset(&t, 0, sizeof(t));  // Initialize to zero
  t.tm_year = year - 1900;   // tm_year is years since 1900
  t.tm_mon = month - 1;      // tm_mon is 0-11
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  t.tm_isdst = -1;           // Not set by default

  // Convert to time_t (UNIX timestamp)
  time_t timestamp = mktime(&t);
  if (timestamp == -1) {
    Serial.println("Failed to convert time using mktime.");
    return -1;
  }
  
  // Print the intermediate timestamp
  Serial.println("Intermediate timestamp: " + String(timestamp));

  // Calculate the timestamp with milliseconds
  timestamp_ms = timestamp * 1000LL; // + millisecond;

  // Print the final timestamp with milliseconds
  Serial.println("Final timestamp with milliseconds: " + String(timestamp_ms));
  saveTimestamp(timestamp_ms);
  return timestamp_ms; 
}

void post_http(String jsonPayload){
    /*          Post HTTP data                */
    Serial.println("Post http data...");
    gsmSerial.println("AT+HTTPINIT=?");       //Initialize HTTP service
    readGsmResponse();
    gsmSerial.println("AT+HTTPINIT");       //Initialize HTTP service
    readGsmResponse();
    vTaskDelay(10 / portTICK_PERIOD_MS);    
    gsmSerial.println("AT+HTTPPARA=\"CID\",1"); //Define carrier profile zeker
    readGsmResponse();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+HTTPPARA=\"URL\", \"http://jrbubuntu.ddns.net:5000/api/telemetry\""); //Pass URL to be called
    readGsmResponse();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\""); //Define content type
    readGsmResponse();
    vTaskDelay(10 / portTICK_PERIOD_MS);        
    
    int dataSize = jsonPayload.length();
    String dataSizeStr = String(dataSize);
    String httpDataCommand = "AT+HTTPDATA=" + dataSizeStr + ",20000";
    //Serial.print("Received payload: ");
    //Serial.println(jsonPayload);
    Serial.print("Data size: ");
    Serial.println(dataSizeStr);
    vTaskDelay(50 / portTICK_PERIOD_MS);

    gsmSerial.println(httpDataCommand);
    readGsmResponse(); 
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    gsmSerial.println(String(jsonPayload));
    readGsmResponse();
    vTaskDelay(500 / portTICK_PERIOD_MS); //5000

    gsmSerial.println("AT+HTTPACTION=1");       //Post HTTP data
    readGsmResponse();
    vTaskDelay(50 / portTICK_PERIOD_MS); //50
    gsmSerial.println("AT+HTTPREAD");          //Read HTTP response
    readGsmResponse();
    vTaskDelay(50 / portTICK_PERIOD_MS); //50
    //Serial.println(String(jsonPayload));
}

enum GsmState {
    GSM_INIT,
    GSM_SET_CONTYPE,
    GSM_SET_APN,
    GSM_SET_USER,
    GSM_SET_PASS,
    GSM_ACTIVATE_GPRS,
    GSM_QUERY_STATUS,
    GSM_CONFIG_DONE
};

GsmState gsmState = GSM_INIT;
unsigned long previousMillis = 0;
const long interval1 = 5000;
const long interval2 = 2000;
const long interval3 = 3000;

void initialize_gsm() {
    unsigned long currentMillis = millis();
    switch (gsmState) {
        case GSM_INIT:
            Serial.println("Configure APN settings.");
            gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\", \"IP\"");
             
            previousMillis = currentMillis;
            gsmState = GSM_SET_CONTYPE;
            break;

        case GSM_SET_CONTYPE:
            if (currentMillis - previousMillis >= interval1) {
                gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn);
                 
                previousMillis = currentMillis;
                gsmState = GSM_SET_APN;
            }
            break;

        case GSM_SET_APN:
            if (currentMillis - previousMillis >= interval2) {
                gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User);
                 
                previousMillis = currentMillis;
                gsmState = GSM_SET_USER;
            }
            break;

        case GSM_SET_USER:
            if (currentMillis - previousMillis >= interval2) {
                gsmSerial.println("AT+SAPBR=3,1,\"PWD\","+ apn_Pass);
                 
                previousMillis = currentMillis;
                gsmState = GSM_SET_PASS;
            }
            break;

        case GSM_SET_PASS:
            if (currentMillis - previousMillis >= interval2) {
                Serial.println("APN settings configured.");
                Serial.println("Configure GPRS settings.");
                gsmSerial.println("AT+SAPBR=1,1");
                 
                previousMillis = currentMillis;
                gsmState = GSM_ACTIVATE_GPRS;
            }
            break;

        case GSM_ACTIVATE_GPRS:
            if (currentMillis - previousMillis >= interval3) {
                gsmSerial.println("AT+SAPBR=2,1");
                 
                previousMillis = currentMillis;
                gsmState = GSM_QUERY_STATUS;
            }
            break;

        case GSM_QUERY_STATUS:
            if (currentMillis - previousMillis >= interval3) {
                Serial.println("GPRS settings configured.");
                gsmState = GSM_CONFIG_DONE;
            }
            break;

        case GSM_CONFIG_DONE:
            // Initialization done
            break;
    }
}

void initialize_gsm2() {
    Serial.println("Configure APN settings.");

    gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\", \"IP\""); // Sets the mode to GPRS
    vTaskDelay(500 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=3,1,\"APN\"," + apn); // Set APN parameters
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=3,1,\"USER\"," + apn_User); // Set APN username
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=3,1,\"PWD\","+ apn_Pass); // Set APN password
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    Serial.println("APN settings configured.");
    Serial.println("Configure GPRS settings.");
    
    gsmSerial.println("AT+SAPBR=1,1"); // Open the carrier with previously defined parameters "start command"
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    gsmSerial.println("AT+SAPBR=2,1"); // Query the status of previously opened GPRS carrier 
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    readGsmResponse();
    
    Serial.println("GPRS settings configured.");
}


/*
void GA6_init()
{
    Serial.println("GA6_init.");
    Serial.println("Initializing IoT-GA6.");
    // Set APN (Access Point Name) details
    //To set TCP function
    //gsmSerial.println("AT+CGDCONT=1,\"IP\",\"data.lycamobile.nl\"");
    //To set http or FTP
    gsmSerial.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\", IP"); 
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+SAPBR=3,1,\"APN\",\"data.lycamobile.nl\"");  
     
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+SAPBR=3,1,\"USER\",\"lmnl\""); 
     
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gsmSerial.println("AT+SAPBR=3,1,\"PWD\",\"plus\"");
     
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    Serial.println("APN settings configured.");

    Serial.println("Activating GPRS (PDP) context.");
    gsmSerial.println("AT+CGACT=1,1");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Return current stat of PDD context.");
    gsmSerial.println("AT+CGDCONT?");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Bringing up wireless connection.");
    gsmSerial.println("AT+CIICR");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Check if there's an IP.");
    gsmSerial.println("AT+CIFSR");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    Serial.println("Check the registration status.");
    gsmSerial.println("AT+CREG?");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Check attach status.");
    gsmSerial.println("AT+CGACT?");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Attach to network.");
    gsmSerial.println(" b");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Wait for attach.");
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    Serial.println("Start task and set the APN.");
    //gsmSerial.println("AT+CSTT=\"data.lycamobile.nl\", \"lmnl\", \"plus\"");
    // 
    //vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Send AT+CIPSTATUS?. Return STATE: IP START."); // Check if IP stack is intitalized.
    gsmSerial.println("AT+CIPSTATUS");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Bring up the wireless connection.");
    gsmSerial.println("AT+CIICR");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Wait for bringup.");
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    Serial.println("Get the local IP address.");
    gsmSerial.println("AT+CIFSR");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("Get the status of the IP connection.");
    gsmSerial.println("AT+CIPSTATUS");
     
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    Serial.println("APN settings configured.");
}
*/
