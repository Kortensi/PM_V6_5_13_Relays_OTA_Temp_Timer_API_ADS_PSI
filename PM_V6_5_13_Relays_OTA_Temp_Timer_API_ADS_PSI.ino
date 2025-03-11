#define BLYNK_TEMPLATE_ID "TMPL2Ew4QXhyB"
#define BLYNK_TEMPLATE_NAME "Pool Monitor"
#define BLYNK_FIRMWARE_VERSION "6.5.13"

#define BLYNK_PRINT Serial  // Comment this out to disable prints and save space
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <time.h>
#include <Arduino_JSON.h>
#include <HTTPClient.h>
#include "BlynkEdgent.h"
#include "ADS1X15.h"
#include "DHT.h"

ADS1115 ADS(0x48);

#define APP_DEBUG

#define DHTPIN 10          // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11      // DHT 11
DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor.

String json_array;

const char *ssid = "KOR 2G";      // CHANGE IT
const char *pass = "FC9BE2541C";  // CHANGE IT

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -21600;
const int daylightOffset_sec = 3600;

//=========================================================================
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;

unsigned long pmstartMillis;  //some global variables available anywhere in the program
unsigned long pmMillis;       // Pump morning count
unsigned long pnstartMillis;  //some global variables available anywhere in the program
unsigned long pnMillis;       // Pump night count

// Setup variables for time tracking
unsigned long SHcurrentMillis;
unsigned long SHpreviousMillis = 0;  // Stores the last time the relay was turned on
unsigned long SHinterval = 3600000;  // Interval of 1 hour in milliseconds
unsigned long SHduration = 600000;   // Duration of 10 minutes in milliseconds

unsigned long PSIcurrentMillis;  //PSI Current Mills
unsigned long PSIstartMillis;    //PSI Start Mills
unsigned long PSIMillis;         // PSI count

//=========================================================================
int httpResponseCode;
double apitemp;
double apihumidity;
double apiwindspd;
double apiwinddir;
unsigned long last_time = 0;
unsigned long timer_delay = 60000;
String apiwinddir2;

//=========================================================================
#define relayPin1 1   // relay signal pin1
#define relayPin2 2   // relay signal pin2
#define relayPin3 3   // relay signal pin3
#define relayPin4 4   // relay signal pin4
#define relayPin5 45  // relay signal pin45
#define relayPin6 46  // relay signal pin46

int relay1State = 0;  // Define variable to store relay state
int relay2State = 0;  // Define variable to store relay state
int relay3State = 0;  // Define variable to store relay state
int relay4State = 0;  // Define variable to store relay state
int relay5State = 0;  // Define variable to store relay state
int relay6State = 0;  // Define variable to store relay state


//=========================================================================  // This is for the 3 Temp Probes
#define ONE_WIRE_BUS 6  // This is the pin for 3 Temp Probes
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress tempSensor1 = { 0x28, 0x2E, 0x4A, 0x80, 0xE3, 0xE1, 0x3C, 0xB9 };  // Temperature probe address #1
DeviceAddress tempSensor2 = { 0x28, 0x71, 0x09, 0x80, 0xE3, 0xE1, 0x3C, 0x14 };  // Temperature probe address #2
DeviceAddress tempSensor3 = { 0x28, 0xF0, 0x8A, 0x80, 0xE3, 0xE1, 0x3C, 0x4C };  // Temperature probe address #3

// Variables for storing temperatures
double temperature1;  // Pool Temp
double temperature2;  // Air Temp
double temperature3;  // Solar Heater Temp

//=========================================================================  // PSI Pressure Sensor G1/4 Pressure Transducer Sensor Pressure Gauge Transduce Input 5V Output 0.5-4.5V / 0-5V Pressure Transmitter for Water(0-80PSI)
const int ADS_gain = 6.144;
const int ADS_max = 32767;
double ADS_0;
double ADS_CV;
double ADS_CP;

//=========================================================================
BLYNK_WRITE(V30) {  //Relay Low Setting
  int value = param.asInt();
  Serial.println(value);
  if (value == 0) {
    digitalWrite(45, LOW);
    relay5State = 0;
    Serial.println("Relay 5 OFF");
  }
  if (value == 1) {
    digitalWrite(45, LOW);
    relay5State = 0;
    Serial.println("Relay 5 OFF");
    digitalWrite(46, LOW);
    relay6State = 0;
    Serial.println("Relay 6 OFF");
    Blynk.virtualWrite(V30, 0);
    Blynk.virtualWrite(V31, 0);
    delay(5000);
    digitalWrite(45, HIGH);
    relay5State = 1;
    Serial.println("Relay 5 ON");
    Blynk.virtualWrite(V30, 1);
  }
}

BLYNK_WRITE(V31) {  //Relay High Setting
  int value = param.asInt();
  Serial.println(value);
  if (value == 0) {
    digitalWrite(46, LOW);
    relay6State = 0;
    Serial.println("Relay 6 OFF");
  }
  if (value == 1) {
    digitalWrite(46, LOW);
    relay6State = 0;
    Serial.println("Relay 6 OFF");
    digitalWrite(45, LOW);
    relay5State = 0;
    Serial.println("Relay 5 OFF");
    Blynk.virtualWrite(V30, 0);
    Blynk.virtualWrite(V31, 0);
    delay(5000);
    digitalWrite(46, HIGH);
    relay6State = 1;
    Serial.println("Relay 6 ON");
    Blynk.virtualWrite(V31, 1);
  }
}


BLYNK_WRITE(V33) {  //Relay #1
  int value = param.asInt();
  Serial.println(value);
  if (value == 1) {
    digitalWrite(1, HIGH);
    relay1State = 1;
    Serial.println("Relay 1 ON");
  }
  if (value == 0) {
    digitalWrite(1, LOW);
    relay1State = 0;
    Serial.println("Relay 1 OFF");
  }
}


BLYNK_WRITE(V34) {  //Relay #2
  int value = param.asInt();
  Serial.println(value);
  if (value == 1) {
    digitalWrite(2, HIGH);
    relay2State = 1;
    Serial.println("Relay 2 ON");
  }
  if (value == 0) {
    digitalWrite(2, LOW);
    relay2State = 0;
    Serial.println("Relay 2 OFF");
  }
}

BLYNK_WRITE(V35) {  //Relay #3
  int value = param.asInt();
  Serial.println(value);
  if (value == 1) {
    digitalWrite(3, HIGH);
    relay1State = 1;
    Serial.println("Relay 3 ON");
  }
  if (value == 0) {
    digitalWrite(3, LOW);
    relay1State = 0;
    Serial.println("Relay 3 OFF");
  }
}


BLYNK_WRITE(V36) {  //Relay #4
  int value = param.asInt();
  Serial.println(value);
  if (value == 1) {
    digitalWrite(4, HIGH);
    relay4State = 1;
    Serial.println("Relay 4 ON");
  }
  if (value == 0) {
    digitalWrite(4, LOW);
    relay4State = 0;
    Serial.println("Relay 4 OFF");
  }
}



//=========================================================================
void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}


//=========================================================================
void setup() {
  Serial.begin(115200);
  BlynkEdgent.begin();

  // ADS1115 start
  Serial.println(__FILE__);
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
  Wire.begin();
  ADS.begin();
  ADS.setGain(0);  //  6.144 volt

  dht.begin();  // Initialize DHT sensor.

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  //=========================================================================
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  pinMode(relayPin4, OUTPUT);
  pinMode(relayPin5, OUTPUT);
  pinMode(relayPin6, OUTPUT);

  //=========================================================================
  startMillis = millis();  //initial start time

  //=========================================================================
  Blynk.virtualWrite(V30, LOW);
  Blynk.virtualWrite(V31, LOW);
  Blynk.virtualWrite(V33, LOW);
  Blynk.virtualWrite(V34, LOW);
  Blynk.virtualWrite(V35, LOW);
  Blynk.virtualWrite(V36, LOW);
}


//=========================================================================
void loop() {
  BlynkEdgent.run();
  Blynk.virtualWrite(V39, millis() / 3600000);
  sendsensor();
  PSI();
  shpump();
  pumpmorning();
  pumpnight();




  //==================================================================================   API call to home weather station
  if ((millis() - last_time) > timer_delay) {

    if (WiFi.status() == WL_CONNECTED) {
      String server = "https://api.weather.com/v2/pws/observations/current?stationId=KTXSANAN3299&format=json&units=e&apiKey=07f9936af6a64d0cb9936af6a61d0c21";
      //      Serial.println(server);

      json_array = GET_Request(server.c_str());
      Serial.println(json_array);
      JSONVar my_obj = JSON.parse(json_array);


      //if (JSON.typeof(my_obj) == "undefined") {
      //if (JSON.typeof(my_obj) == "{}") {
      if (httpResponseCode < 0) {
        Serial.println("Parsing input failed!");
        // add delay
        return;
      }

      //Serial.print("JSON.typeof(my_obj) = ");
      //Serial.println(JSON.typeof(my_obj));  // prints: object

      Serial.print("Temperature: ");
      Serial.println((double)my_obj["observations"][0]["imperial"]["temp"]);
      apitemp = ((double)my_obj["observations"][0]["imperial"]["temp"]);  // Stores temp in F. Change getTempF to getTempC for celcius.
      Blynk.virtualWrite(V41, apitemp);                                   // Send temp to Blynk virtual pin 41
      Serial.print("Humidity: ");
      Serial.println((double)my_obj["observations"][0]["imperial"]["dewpt"]);
      apihumidity = ((double)my_obj["observations"][0]["imperial"]["dewpt"]);  // Stores temp in F. Change getTempF to getTempC for celcius.
      Blynk.virtualWrite(V42, apihumidity);                                    // Send temp to Blynk virtual pin 42
      Serial.print("Wind Speed: ");
      Serial.println((double)my_obj["observations"][0]["imperial"]["windSpeed"]);
      apiwindspd = ((double)my_obj["observations"][0]["imperial"]["windSpeed"]);  // Stores temp in F. Change getTempF to getTempC for celcius.
      Blynk.virtualWrite(V43, apiwindspd);                                        // Send temp to Blynk virtual pin 43
      Serial.print("Wind Direction: ");
      Serial.println((double)my_obj["observations"][0]["winddir"]);
      apiwinddir = ((double)my_obj["observations"][0]["winddir"]);  // Stores temp in F. Change getTempF to getTempC for celcius.
      // Blynk.virtualWrite(V44, apiwinddir);                          // Send temp to Blynk virtual pin 44

      String apiwinddir2;  // Variable to store the resulting wind direction
      {
        if ((346 < apiwinddir && apiwinddir <= 360) || (0 <= apiwinddir && apiwinddir <= 15)) {
          apiwinddir2 = "N";
        } else if (16 < apiwinddir && apiwinddir <= 35) {
          apiwinddir2 = "N/NE";
        } else if (36 < apiwinddir && apiwinddir <= 55) {
          apiwinddir2 = "NE";
        } else if (56 < apiwinddir && apiwinddir <= 75) {
          apiwinddir2 = "E/NE";
        } else if (76 < apiwinddir && apiwinddir <= 105) {
          apiwinddir2 = "E";
        } else if (106 < apiwinddir && apiwinddir <= 125) {
          apiwinddir2 = "E/SE";
        } else if (126 < apiwinddir && apiwinddir <= 145) {
          apiwinddir2 = "SE";
        } else if (146 < apiwinddir && apiwinddir <= 165) {
          apiwinddir2 = "S/SE";
        } else if (166 < apiwinddir && apiwinddir <= 195) {
          apiwinddir2 = "S";
        } else if (196 < apiwinddir && apiwinddir <= 215) {
          apiwinddir2 = "S/SW";
        } else if (216 < apiwinddir && apiwinddir <= 235) {
          apiwinddir2 = "SW";
        } else if (236 < apiwinddir && apiwinddir <= 255) {
          apiwinddir2 = "W/SW";
        } else if (256 < apiwinddir && apiwinddir <= 285) {
          apiwinddir2 = "W";
        } else if (286 < apiwinddir && apiwinddir <= 305) {
          apiwinddir2 = "W/NW";
        } else if (306 < apiwinddir && apiwinddir <= 325) {
          apiwinddir2 = "NW";
        } else if (326 < apiwinddir && apiwinddir <= 345) {
          apiwinddir2 = "N/NW";
        }
      }
      Serial.print("Apiwinddir2: ");         // Print the resulting wind direction
      Serial.println(apiwinddir2);           // Print the resulting wind direction
      Blynk.virtualWrite(V32, apiwinddir2);  // Send wind Dir to Blynk virtual pin 32

    } else {
      Serial.println("WiFi Disconnected");
    }
    last_time = millis();
  }
}

//==================================================================================   API call to home weather station checking for errors
String GET_Request(const char *server) {
  HTTPClient http;
  http.begin("https://api.weather.com/v2/pws/observations/current?stationId=KTXSANAN3299&format=json&units=e&apiKey=07f9936af6a64d0cb9936af6a61d0c21");
  httpResponseCode = http.GET();
  //  int httpResponseCode = http.GET();
  String payload = "{}";

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();

  return payload;
}


//============================================================================================   Sent data from Temperature probes
void sendsensor() {
  currentMillis = millis();                  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= 60000)  //test whether the period has elapsed
  {
    sensors.requestTemperatures();                 // Polls the sensors
    temperature1 = sensors.getTempF(tempSensor1);  // Stores temp in F. Change getTempF to getTempC for celcius.
    Blynk.virtualWrite(V37, temperature1);         // Send temp to Blynk virtual pin 37

    // sensors.requestTemperatures();  //Remove??
    temperature2 = sensors.getTempF(tempSensor2);
    Blynk.virtualWrite(V38, temperature2);

    // sensors.requestTemperatures();  //Remove??
    temperature3 = sensors.getTempF(tempSensor3);
    Blynk.virtualWrite(V40, temperature3);

    // Loop through each device, print out temperature data

    // Output the device ID 1
    Serial.print("Temperature for Sensor 1 F: ");
    Serial.println(temperature1);
    // Print the data

    // Output the device ID 2
    Serial.print("Temperature for Sensor 2 F: ");
    Serial.println(temperature2);
    // Print the data

    // Output the device ID 3
    Serial.print("Temperature for Sensor 3 F: ");
    Serial.println(temperature3);
    // Print the data


    //============================================================================================ DHT Sensor
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    Serial.print(f);
    Serial.println(F("°F"));

    Blynk.virtualWrite(V52, h);
    Blynk.virtualWrite(V53, f);

    startMillis = currentMillis;  //IMPORTANT to save the start time of the current state.
    printLocalTime();
  }
}



//============================================================================================ PSI Sensor on ADS1115 - 0
void PSI() {
  PSIcurrentMillis = millis();
  if (PSIcurrentMillis - PSIstartMillis >= 15000) {
    // Serial.println("Voltage: ");
    // int16_t raw = ADS.readADC(0);
    // int16_t PSIv = (ADS.toVoltage(raw), 3);
    // Serial.println(raw);
    // Serial.println(ADS.toVoltage(raw), 3);
    // Serial.println(PSIv);

    ADS_0 = ADS.readADC(0);
    ADS_CV = (((ADS_0 / ADS_max) * ADS_gain));
    ADS_CP = ((25.426 * ADS_CV) - 14.416);

    Blynk.virtualWrite(V50, ADS_0);
    Blynk.virtualWrite(V49, ADS_CV);
    Blynk.virtualWrite(V51, ADS_CP);

    Serial.print("ADS_0: ");
    Serial.println(ADS_0);
    Serial.print("ADS_CV: ");
    Serial.println(ADS_CV);
    Serial.print("ADS_CP: ");
    Serial.println(ADS_CP);

    PSIstartMillis = PSIcurrentMillis;
    // printLocalTime();
  }
}



//============================================================================================  Turn the solar heater pump on for 10 minutes every hour
void shpump() {
  // Check if tempSensor3 is greater than tempSensor1

  if (temperature3 > temperature1) {
    // Get current time in milliseconds
    //    unsigned long SHcurrentMillis = millis();
    SHcurrentMillis = millis();

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }
    // Check if it has been an hour since the last time the relay was turned on
    if (SHcurrentMillis - SHpreviousMillis >= SHinterval) {
      // Turn on relayPin1
      digitalWrite(relayPin1, HIGH);
      Serial.println("Relay 1 ON");
      Blynk.virtualWrite(V33, 1);
      Serial.println(&timeinfo, "%A, %H:%M:%S");

      // Update previousMillis to currentMillis
      SHpreviousMillis = SHcurrentMillis;
    }

    // Check if 10 minutes has passed since the relay was turned on
    if (SHcurrentMillis - SHpreviousMillis >= SHduration) {
      // Turn off relayPin1
      digitalWrite(relayPin1, LOW);
      Serial.println("Relay 1 OFF");
      Blynk.virtualWrite(V33, 0);
      Serial.println(&timeinfo, "%A, %H:%M:%S");
    }
  }
}

//============================================================================================  Set the pump speed to low at night
void pumpnight() {
  pnMillis = millis();                    //get the current "time" (actually the number of milliseconds since the program started)
  if (pnMillis - pnstartMillis >= 60000)  //test whether the period has elapsed
  {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }
    if ((timeinfo.tm_hour == 18) && (timeinfo.tm_min == 59)) {  // Check if it is 8:59 PM - Set Pool pump to low setting
      digitalWrite(45, LOW);
      Serial.println("Relay 5 OFF");
      digitalWrite(46, LOW);
      Serial.println("Relay 6 OFF");
      Blynk.virtualWrite(V30, 0);
      Blynk.virtualWrite(V31, 0);
      delay(5000);
      digitalWrite(45, HIGH);
      Serial.println("Relay 5 ON");
      Blynk.virtualWrite(V31, 1);
      Serial.println(&timeinfo, "%A, %H:%M:%S");
    }
    pnstartMillis = pnMillis;  //IMPORTANT to save the start time of the current state.
  }
}

//============================================================================================  Set the pump speed to high in the morning
void pumpmorning() {
  pmMillis = millis();                    //get the current "time" (actually the number of milliseconds since the program started)
  if (pmMillis - pmstartMillis >= 60000)  //test whether the period has elapsed
  {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }
    if ((timeinfo.tm_hour == 7) && (timeinfo.tm_min == 59)) {  // Check if it is 8:59 am - Set Pool pump to high setting
      digitalWrite(5, LOW);
      Serial.println("Relay 5 OFF");
      digitalWrite(6, LOW);
      Serial.println("Relay 6 OFF");
      Blynk.virtualWrite(V31, 0);
      Blynk.virtualWrite(V32, 0);
      delay(5000);
      digitalWrite(45, HIGH);
      Serial.println("Relay 6 ON");
      Blynk.virtualWrite(V32, 1);
      Serial.println(&timeinfo, "%A, %H:%M:%S");
    }
    pmstartMillis = pmMillis;  //IMPORTANT to save the start time of the current state.
  }
}
