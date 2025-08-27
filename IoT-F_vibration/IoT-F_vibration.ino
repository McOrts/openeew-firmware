// --------------------------------------------------------------------------------------------
// BOARD: ESP32-WROOM-DA
// --------------------------------------------------------------------------------------------

#include <Arduino.h>
//#if defined(ARDUINO_ARCH_ESP32)
  #include <esp32-hal-ledc.h>
//#endif

#define ARDUINOJSON_USE_DOUBLE 1

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <ETH.h>
#include <time.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adxl355.h> // forked from https://github.com/markrad/esp32-ADXL355
#include <math.h>
#include "config.h"
#include "secrets.h" //  MQTT and Sensor information
#include <cppQueue.h>
#include "soc/soc.h" // Enable/Disable BrownOut detection
#include "soc/rtc_cntl_reg.h"

// --------------------------------------------------------------------------------------------
// DEFINITION OF VARIABLES

String getHeaderValue(String header, String headerName)
{
  return header.substring(strlen(headerName.c_str()));
}

// Timezone info
#define TZ_OFFSET 0 // (EST) Hours timezone offset to GMT (without daylight saving time)
#define TZ_DST 0    // Minutes timezone offset for Daylight saving

// Objects
WiFiClientSecure net;
WiFiClient espClient;
PubSubClient clientMqtt(espClient);

void NTPConnect();
void SendLiveData2Cloud();
void Send10Seconds2Cloud();

// Ethernet variables
#ifdef ETH_CLK_MODE
#undef ETH_CLK_MODE
#endif
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN 2 // Ethernet on production board
#define ETH_TYPE ETH_PHY_LAN8720
//#define ETH_ADDR 0
static constexpr uint8_t ETH_PHY_ADDR = 0;
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18

// Network variables
static bool bEthConnected = false;
static bool bEthConnecting = false;
static bool bWiFiConnected = false;
static bool bNetworkInterfaceChanged = false;

// --------------------------------------------------------------------------------------------
// NTP time
void getNTPtimestamp();
void SyncNTPtime();
long NTP_timestamp = 0;
long NTP_timestamp_new = 0;
double time_since_NTP;
double device_t;

// --------------------------------------------------------------------------------------------
// ADXL Accelerometer
void IRAM_ATTR isr_adxl();
int32_t Adxl355SampleRate = 31; // Reporting Sample Rate [31,125]
int8_t CHIP_SELECT_PIN_ADXL = 15;
int8_t ADXL_INT_PIN = 35; // ADXL is on interrupt 35 on production board
Adxl355::RANGE_VALUES range = Adxl355::RANGE_VALUES::RANGE_2G;
Adxl355::ODR_LPF odr_lpf;
Adxl355::STATUS_VALUES adxstatus;
Adxl355 adxl355(CHIP_SELECT_PIN_ADXL);
SPIClass *spi1 = NULL;
long fifoOut[32][3];
bool fifoFull = false;
int fifoCount = 0;
int STA_len = 32;  // can change to 125
int LTA_len = 320; // can change to 1250
int QUE_len = LTA_len + STA_len;
bool STALTAMODE = true;
double TrueSampleRate;

// --------------------------------------------------------------------------------------------
// Variables to hold accelerometer data
// 10 second FIFO queue for STA / LTA algorithm
typedef struct AccelXYZ
{
  double x;
  double y;
  double z;
} AccelReading;
cppQueue StaLtaQue(sizeof(AccelReading), 352, FIFO); // 11 seconds of Accelerometer data
uint32_t numSecsOfAccelReadings = 0;

// --------------------------------------------------------------------------------------------
// NeoPixel LEDs
#include <Adafruit_NeoPixel.h>
#define LED_PIN 16
#define LED_COUNT 3
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
void NeoPixelStatus(int);
void NeoPixelBreathe(int);
bool breathedirection = true;
int breatheintensity = 1;

// LED colors
#define LED_OFF 0
#define LED_CONNECTED 1     // Cyan breath
#define LED_FIRMWARE_OTA 2  // Magenta
#define LED_CONNECT_WIFI 3  // Green
#define LED_CONNECT_CLOUD 4 // Cyan fast
#define LED_LISTEN_WIFI 5   // Blue
#define LED_WIFI_OFF 6      // White
#define LED_SAFE_MODE 7     // Magenta breath
#define LED_FIRMWARE_DFU 8  // Yellow
#define LED_ERROR 9         // Red
#define LED_ORANGE 10       // Orange

// --------------------------------------------------------------------------------------------
// Buzzer Alarm
bool EarthquakeAlarmBool = false;
void EarthquakeAlarm(int);
void AlarmBuzzer();
int freq = 4000;
int channel = 0;
int resolution = 8;
int io = 5;

// --------------------------------------------------------------------------------------------
// STA/LTA Algorithm globals
bool bPossibleEarthQuake = false;
double thresh = 4.0;
double stalta[3] = {0, 0, 0};
double sample[3] = {0, 0, 0};
double sampleSUM[3] = {0, 0, 0};
double ltSUM[3] = {0, 0, 0};
double sample1[3] = {0, 0, 0};
double LTAsample1[3] = {0, 0, 0};
double offset[3] = {0, 0, 0};
double sampleABS[3] = {0, 0, 0};
double sample1ABS = 0;
double LTAsample1ABS = 0;
double stav[3] = {0, 0, 0};
double ltav[3] = {0, 0, 0};

// --------------------------------------------------------------------------------------------
// ADXL
void IRAM_ATTR isr_adxl()
{
  fifoFull = true;
  // fifoCount++;
}
unsigned long previousMillis = 0;
const long interval = 5000;
unsigned long lastMillis = 0;
time_t now;
time_t nowish = 1510592825;
time_t periodic_timesync;

// --------------------------------------------------------------------------------------------
// FUNCTION DEFINITION

// Start Accelerometer
void StartADXL355()
{
  // odr_lpf is a global
  adxl355.start();
  delay(1000);

  // Calibrating the ADXL355 can cause brownouts
  NeoPixelStatus(LED_OFF);                   // turn off the LED to reduce power consumption
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  if (adxl355.isDeviceRecognized())
  {
    Serial.println("Initializing sensor");
    adxl355.initializeSensor(range, odr_lpf, debug);
    Serial.println("Calibrating sensor");
    double rec_start = micros();
    adxl355.calibrateSensor(20, debug); // This has been increased to make traces start closer to zero
    double rec_stop = micros();
    // Calculate true sample rate
    // ADXL355 is slightly off the correct time and we need to use the true rate in messages
    TrueSampleRate = (32 * 20 * 1000000) / (rec_stop - rec_start) - 0.04;
    Serial.print("ADXL355 Accelerometer true sample rate: ");
    Serial.println(TrueSampleRate, 5);

    Serial.println("ADXL355 Accelerometer activated");

    bool bDiscardInitialADXLreadings = true;
    while (bDiscardInitialADXLreadings)
    {
      adxstatus = adxl355.getStatus();
      if (adxstatus & Adxl355::STATUS_VALUES::FIFO_FULL)
      {
        adxl355.readFifoEntries((long *)fifoOut);
        bDiscardInitialADXLreadings = false;
      }
    }
    Serial.println("ADXL355 Accelerometer first samples discarded");
  }
  else
  {
    Serial.println("Unable to get accelerometer");
  }
  Serial.println("Finished accelerometer configuration");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1); // enable brownout detector
}

// Connect to MQTT
void connectToMqtt()
{
  Serial.print("MQTT connecting to broker ");
  while (!clientMqtt.connected()) {
    Serial.print(".");
    if (clientMqtt.connect(mqtt_id, mqttUser, mqttPassword )) { 
      Serial.println("");
      Serial.println("MQTT connected");
      // Once connected, publish an announcement...
      clientMqtt.publish(mqtt_sub_topic_healthcheck, "starting");
      //clientMqtt.publish(mqtt_sub_topic_ip, IP.c_str());
      // ... and subscribe
      clientMqtt.subscribe(mqtt_sub_topic_operation);
    } else {
      Serial.print("[MQTT]Error, rc=");
      Serial.print(clientMqtt.state());
      Serial.println("[MQTT] trying in 5 seconds...");
      delay(5000);
    }
  }
}

void connectToWiFi(String init_str)
{
  if (bEthConnected)
  {
    return;
  }
  if (init_str != emptyString)
    Serial.print(init_str);

  if (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print("Restarting");
    ESP.restart();
  }
  if (init_str != emptyString)
    Serial.println("ok!");
}

void checkWiFiThenMQTT(void)
{
  connectToWiFi("Checking WiFi");
  connectToMqtt();
}

void checkWiFiThenMQTTNonBlocking(void)
{
  connectToWiFi(emptyString);
  if (millis() - previousMillis >= interval && !clientMqtt.connected())
  {
    previousMillis = millis();
    connectToMqtt();
  }
}

void checkWiFiThenReboot(void)
{
  if (bEthConnected)
  {
    delay(1000);
    Serial.println("Will try to reconnect to MQTT");
    connectToMqtt();
  }
  else
  {
    delay(1000);
    Serial.print("Restarting");
    ESP.restart();
  }
}

// void NetworkEvent(WiFiEvent_t event)
void NetworkEvent(arduino_event_id_t event) {
  switch (event) {

    // -------- Wi-Fi (STA) ----------
    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("ESP32 WiFi interface ready");
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("ESP32 WiFi started");
      WiFi.setHostname("openeew-sensor-wifi");
      break;

    case ARDUINO_EVENT_SC_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("ESP32 WiFi connected to AP");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      bWiFiConnected = false;
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("ESP32 station got IP from connected AP");
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      bWiFiConnected = true;
      if (bEthConnected) {
        Serial.println("Ethernet is already connected");
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_STOP:
      Serial.println("WiFi Stopped");
      NeoPixelStatus(LED_WIFI_OFF); // White
      break;

    // (opcional, si usas soft-AP):
    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println("ESP32 soft-AP stop");
      break;

    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("A station connected to ESP32 soft-AP");
      break;

    // -------- Ethernet (LAN8720, etc.) ----------
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("openeew-sensor-eth");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      bEthConnecting = true;
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) { Serial.print(", FULL_DUPLEX"); }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      bEthConnected = true;

      // Preferir ETH si está disponible: conmuta MQTT si procede
      if (clientMqtt.connected()) {
        Serial.println("Previously connected to WiFi, try to switch the MQTT connection to Ethernet");
        bNetworkInterfaceChanged = true;
        // El reconnect lo gestiona tu loop
      }
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      bEthConnected = false;
      if (clientMqtt.connected()) {
        Serial.println("Previously connected to Ethernet, try to switch the MQTT connection to WiFi");
        bNetworkInterfaceChanged = true;
      }
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      bEthConnected = false;
      break;

    // -------- Otros ----------
    default:
      Serial.print("Unhandled Network Interface event : ");
      Serial.println((int)event);
      break;
  }
}

void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void SetTimeESP32()
{
  time_t now = time(nullptr);
  Serial.print("Before time sync: ");
  Serial.print(ctime(&now));

  // Set time from NTP servers
  configTime(TZ_OFFSET * 3600, TZ_DST * 60, "time.nist.gov", "pool.ntp.org", "cz.pool.ntp.org");
  Serial.print("Waiting for time");
  while (time(nullptr) <= 100000)
  {
    NeoPixelStatus(LED_FIRMWARE_DFU); // blink yellow
    Serial.print(".");
    delay(100);
  }
  unsigned timeout = 5000;
  unsigned start = millis();
  while (millis() - start < timeout)
  {
    now = time(nullptr);
    if (now > (2019 - 1970) * 365 * 24 * 3600)
    {
      break;
    }
    delay(100);
  }
  delay(1000); // Wait for time to fully sync

  Serial.print("\nAfter time sync : ");
  now = time(nullptr);
  Serial.print(ctime(&now));
  periodic_timesync = now;
  // periodically resync the time to prevent drift
}

void SyncNTPtime()
{

  // Set time from NTP servers
  // configTime(TZ_OFFSET * 3600, TZ_DST * 60, "time.nist.gov", "pool.ntp.org");

  struct tm NTP_time_new;
  if (!getLocalTime(&NTP_time_new))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  NTP_timestamp_new = mktime(&NTP_time_new);
}

//================================= Message timestamps ================================

void getNTPtimestamp()
{

  // Time now
  double timeNow = micros();

  // Get device time
  device_t = NTP_timestamp + (timeNow - time_since_NTP) / 1000000 + 0.015;

  // Serial.print("Device time: ");
  // Serial.println(device_t, 4);
}

//========================== Sensor commands handling ============================
// Send 10 seconds in 1 data message
void Send10Seconds2Cloud()
{
  // DynamicJsonDocument is stored on the heap
  // Allocate a ArduinoJson buffer large enough to 10 seconds of Accelerometer trace data
  DynamicJsonDocument historydoc(16384);
  JsonObject payload = historydoc.to<JsonObject>();
  // JsonArray  alltraces    = payload.createNestedArray("traces");
  JsonObject acceleration = payload.createNestedObject("traces");

  // Load the key/value pairs into the serialized ArduinoJSON format
  payload["device_id"] = deviceID;
  payload["sr"] = TrueSampleRate;
  payload["user"] = user;
  payload["network"] = network;
  payload["station"] = station;

  getNTPtimestamp();
  payload["device_t"] = serialized(String(device_t, 6));

  // Generate an array of json objects that contain x,y,z arrays of 32 floats.
  // [{"x":[],"y":[],"z":[]},{"x":[],"y":[],"z":[]}]
  AccelReading AccelRecord;
  for (uint16_t idx = 0; idx < StaLtaQue.getCount(); idx++)
  {
    if (StaLtaQue.peekIdx(&AccelRecord, idx))
    {
      // char reading[75];
      // snprintf( reading, 74, "[ x=%3.3f , y=%3.3f , z=%3.3f ]", AccelRecord.x, AccelRecord.y, AccelRecord.z);
      // Serial.println(reading);

      acceleration["x"].add(AccelRecord.x);
      acceleration["y"].add(AccelRecord.y);
      acceleration["z"].add(AccelRecord.z);
    }
  }

  // Serialize the History Json object into a string to be transmitted
  // serializeJson(historydoc,Serial);  // print to console
  static char historymsg[16384];
  serializeJson(historydoc, historymsg, 16383);

  int jsonSize = measureJson(historydoc);
  Serial.print("Sending 10 seconds of accelerometer readings in a MQTT packet of size: ");
  Serial.println(jsonSize);
  clientMqtt.setBufferSize((jsonSize + 50)); // increase the MQTT buffer size

  // Publish the message to MQTT Broker
  if (!clientMqtt.publish(mqtt_pub_topic_data, historymsg)) {
    Serial.println("MQTT Publish failed");
  } else {
    Serial.println("MQTT Send10Seconds2Cloud");
  }

  clientMqtt.setBufferSize(2000); // reset the MQTT buffer size
  historydoc.clear();
}

// Send a simple data message
void SendLiveData2Cloud()
{
  // variables to hold accelerometer data
  // DynamicJsonDocument is stored on the heap
  DynamicJsonDocument jsonDoc(3500);
  JsonObject payload = jsonDoc.to<JsonObject>();
  // JsonArray  traces       = payload.createNestedArray("traces");
  JsonObject acceleration = payload.createNestedObject("traces");

  // Load the key/value pairs into the serialized ArduinoJSON format
  payload["device_id"] = deviceID;
  payload["sr"] = TrueSampleRate; // Adxl355SampleRate;
  payload["user"] = user;
  payload["network"] = network;
  payload["station"] = station;

  getNTPtimestamp();
  payload["device_t"] = serialized(String(device_t, 6));


  // Generate an array of json objects that contain x,y,z arrays of 32 floats.
  // [{"x":[],"y":[],"z":[]},{"x":[],"y":[],"z":[]}]
  AccelReading AccelRecord;
  // Send the last 32 records (or less) from the queue
  uint16_t idx = StaLtaQue.getCount();
  if (idx >= 32)
  {
    idx = idx - 32;
  }
  for (; idx < StaLtaQue.getCount(); idx++)
  {
    if (StaLtaQue.peekIdx(&AccelRecord, idx))
    {
      // char reading[75];
      // snprintf( reading, 74, "[ x=%3.3f , y=%3.3f , z=%3.3f ]", AccelRecord.x, AccelRecord.y, AccelRecord.z);
      // Serial.println(reading);

      acceleration["x"].add(AccelRecord.x);
      acceleration["y"].add(AccelRecord.y);
      acceleration["z"].add(AccelRecord.z);
    }
  }

  // Serialize the current second Json object into a string to be transmitted
  static char msg[2000];
  serializeJson(jsonDoc, msg, 2000);
  // Serial.println(msg);

  int jsonSize = measureJson(jsonDoc);
  Serial.print("Sending 1 second packet: User: ");
  Serial.print(user);
  Serial.print(", Net: ");
  Serial.print(network);
  Serial.print(", Sta: ");
  Serial.print(station);
  Serial.print(", Size: ");
  Serial.print(jsonSize);
  Serial.print(", NTP time: ");
  Serial.println(device_t, 4);

  
  clientMqtt.setBufferSize((jsonSize + 50)); // increase the MQTT buffer size

  if (!clientMqtt.publish(mqtt_pub_topic_data, msg)){
    Serial.println("MQTT Publish failed");
  } else {
    Serial.println("MQTT SendLiveData2Cloud");
  }

  clientMqtt.setBufferSize(2000); // reset the MQTT buffer size
  jsonDoc.clear();
}

// Handle subscribed MQTT topics - Alerts and Sample Rate changes
void messageReceived(char *topic, byte *payload, unsigned int length)
{
  // Receive and decode message
  StaticJsonDocument<2000> jsonMQTTReceiveDoc;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");

  std::string action = topic;

  payload[length] = 0; // ensure valid content is zero terminated so can treat as c-string
  // Serial.println((char *)payload);
  DeserializationError err = deserializeJson(jsonMQTTReceiveDoc, (char *)payload);
  if (err)
  {
    Serial.print(F("deserializeJson() failed with code : "));
    Serial.println(err.c_str());
  }
  else
  {
    JsonObject cmdData = jsonMQTTReceiveDoc.as<JsonObject>();

    auto alarm = cmdData["state"]["desired"]["alarm"];
    auto samplerate = cmdData["state"]["desired"]["samplerate"];
    auto staltamode = cmdData["state"]["desired"]["acqmode"];
    //auto senddata = cmdData["state"]["desired"]["senddata"];
    auto staltathresh = cmdData["state"]["desired"]["staltathresh"];
    //auto forcerestart = cmdData["state"]["desired"]["forcerestart"];
    // auto factoryreset = cmdData["state"]["desired"]["factoryreset"];
    //auto soh = cmdData["state"]["desired"]["soh"];
    //auto timentp = cmdData["state"]["desired"]["timentp"];
    auto pub = false;

    // ======= Test device commands =======
    // Test alarm
    if (alarm.as<String>().equalsIgnoreCase("true"))
    {
      Serial.println("Alarm received: " + alarm.as<String>());
      // Sound the Buzzer & Blink the LED RED
      EarthquakeAlarmBool = true;
      EarthquakeAlarm(LED_ERROR);
      EarthquakeAlarmBool = false;
    }  

    // ======= Configuration commands =======

    // STA/LTA mode
    if (!staltamode.as<String>().equalsIgnoreCase("null") && STALTAMODE != staltamode.as<bool>())
    {
      Serial.println("Switching the acquisition mode...");
      STALTAMODE = staltamode.as<bool>();
      pub = true;
    }

    // STA/LTA Threshold
    if (!staltathresh.as<String>().equalsIgnoreCase("null") && thresh != staltathresh.as<double>())
    {
      // Override the `thresh` global
      char newthreshmsg[50];
      snprintf(newthreshmsg, 49, "Previous STA/LTA Shake Threshold : %5.2f", thresh);
      Serial.println(newthreshmsg);
      thresh = staltathresh.as<double>();
      snprintf(newthreshmsg, 49, "Override STA/LTA Shake Threshold : %5.2f", thresh);
      Serial.println(newthreshmsg);
      pub = true;

      // char char_array[staltathresh.as<String>().length() + 1];
      // strcpy(char_array, staltathresh.as<String>().c_str());
      // PublishMessage("update", "reported", "staltamode", char_array);
    }

    // Samplerate
    if (!samplerate.as<String>().equalsIgnoreCase("null") && Adxl355SampleRate != samplerate.as<int32_t>())
    {
      // Set the ADXL355 Sample Rate
      int32_t NewSampleRate = 0;
      bool SampleRateChanged = false;
      pub = true;

      NewSampleRate = samplerate.as<int32_t>(); // this form allows you specify the type of the data you want from the JSON object
      if (NewSampleRate == 31)
      {
        // Requested sample rate of 31 is valid
        Adxl355SampleRate = 31;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
      }
      else if (NewSampleRate == 125)
      {
        // Requested sample rate of 125 is valid
        Adxl355SampleRate = 125;
        SampleRateChanged = true;
        odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
      }
      else if (NewSampleRate == 0)
      {
        // Turn off the sensor ADXL
        Adxl355SampleRate = 0;
        SampleRateChanged = false; // false so the code below doesn't restart it
        Serial.println("Stopping the ADXL355");
        adxl355.stop();
        StaLtaQue.flush(); // flush the Queue
        strip.clear();     // Off
        strip.show();
      }
      else
      {
        // invalid - leave the Sample Rate unchanged
      }

      Serial.print("ADXL355 Sample Rate has been changed:");
      Serial.println(Adxl355SampleRate);
      // SampleRateChanged = false;
      Serial.println(SampleRateChanged);
      if (SampleRateChanged)
      {
        Serial.println("Changing the ADXL355 Sample Rate");
        adxl355.stop();
        delay(1000);
        Serial.println("Restarting");
        StartADXL355();
        breatheintensity = 1;
        breathedirection = true;
      }
      jsonMQTTReceiveDoc.clear();
    }

  }
}

//================================= LED and Buzzer ================================
// Setting LED colors
void NeoPixelStatus(int status)
{
  // Turn leds off to cause a blink effect
  strip.clear(); // Off
  strip.show();  // This sends the updated pixel color to the hardware.
  delay(400);    // Delay for a period of time (in milliseconds).

  switch (status)
  {
  case LED_OFF:
    strip.clear(); // Off
    break;
  case LED_CONNECTED:
    strip.setBrightness(10);
    strip.fill(strip.Color(0, 255, 255), 0, 3); // Cyan breath - dim
    Serial.println("LED_CONNECTED - Cyan");
    break;
  case LED_FIRMWARE_OTA:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 0, 255), 0, 3); // Magenta
    Serial.println("LED_FIRMWARE_OTA - Magenta");
    break;
  case LED_CONNECT_WIFI:
    strip.setBrightness(10);
    strip.fill(strip.Color(0, 255, 0), 0, 3); // Green
    Serial.println("LED_CONNECT_WIFI - Green");
    break;
  case LED_CONNECT_CLOUD:
    strip.setBrightness(10);
    strip.fill(strip.Color(0, 255, 255), 0, 3); // Cyan fast
    Serial.println("LED_CONNECT_CLOUD - Cyan");
    break;
  case LED_LISTEN_WIFI:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 165, 0), 0, 3); // Orange
    Serial.println("LED_LISTEN_WIFI - Orange");
    break;
  case LED_WIFI_OFF:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 255, 255), 0, 3); // White
    Serial.println("LED_WIFI_OFF - White");
    break;
  case LED_SAFE_MODE:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 0, 255), 0, 3); // Magenta breath
    Serial.println("LED_SAFE_MODE - Magenta");
    break;
  case LED_FIRMWARE_DFU:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 255, 0), 0, 3); // Yellow
    Serial.println("LED_FIRMWARE_DFU - Yellow");
    break;
  case LED_ORANGE:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 165, 0), 0, 3); // Red
    Serial.println("LED_ORANGE - Orange");
    break;
  case LED_ERROR:
    strip.setBrightness(10);
    strip.fill(strip.Color(255, 0, 0), 0, 3); // Red
    Serial.println("LED_ERROR - Red");
    break;
  default:
    strip.clear(); // Off
    break;
  }
  strip.show(); // Send the updated pixel color to the hardware
}

// Slow breathe of the sensor LEDs
void NeoPixelBreathe(int status)
{
  if (breatheintensity < 0)
    breatheintensity = 0;
  strip.setBrightness(breatheintensity); // slow breathe the LED
  // Serial.printf("Brightness is %d\n",breatheintensity);
  switch (status)
  {
  case 0:
    strip.fill(strip.Color(0, 255, 255), 0, 3);
    break;
  case 1:
    strip.fill(strip.Color(0, 255, 0), 0, 3);
    break;
  }

  strip.show();

  // Increase or decrease the LED intensity
  breathedirection ? breatheintensity++ : breatheintensity--;
}

// Sound the Buzzer & Blink the LED
void EarthquakeAlarm(int AlarmLEDColor)
{
  Serial.println("Earthquake Alarm!");
  strip.setBrightness(255); // The breathe intensity might have the brightness low
  for (int i = 0; i < 10; i++)
  {
    if (EarthquakeAlarmBool)
    {
      delay(500);
      NeoPixelStatus(AlarmLEDColor); // Alarm - blink red or orange
      AlarmBuzzer();
    }
    clientMqtt.loop(); // Process any incoming MQTT topics (which might stop the alarm)
  }
  strip.setBrightness(breatheintensity); // reset the brightness to the prior intensity
  //digitalWrite(io, LOW);                 // turn off buzzer
}

// Generate Buzzer sounds
void AlarmBuzzer()
{
  ledcWrite(channel, 50);
  delay(100);
  ledcWrite(channel, 500);
  delay(100);
  ledcWrite(channel, 2000);
  delay(100);
  ledcWrite(channel, 4000);
  delay(100);
}



//================================= Main setup and loop ================================
void setup() {

  // Start serial console
  Serial.begin(115200);

  NeoPixelStatus(LED_OFF); // turn off the LED to reduce power consumption
  strip.setBrightness(50); // Dim the LED to 20% - 0 off, 255 full bright

  // Start Network connections
  WiFi.onEvent(NetworkEvent);

  // Start the ETH interface, if it is available, before WiFi
  ETH.begin(ETH_TYPE, ETH_PHY_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_POWER_PIN, ETH_CLK_MODE);
  delay(5000);
  if (bEthConnecting)
  {
    while (!bEthConnected)
    {
      Serial.println("Waiting for Ethernet to start...");
      delay(500);
    }
  }

  // If Eth is not aviailable, try Wifi
  if (!bWiFiConnected && !bEthConnected)
  {
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

  byte mac[6]; // the MAC address of your Wifi shield
  WiFi.macAddress(mac);
  // Output this ESP32 Unique WiFi MAC Address
  Serial.print("WiFi MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Device ID: ");
  Serial.println(deviceID);
  WiFi.setHostname(deviceID);

  // Set the time on the ESP32
  SetTimeESP32();

  // MQTT connection 
  clientMqtt.setServer(mqtt_server, mqtt_port);
  clientMqtt.setCallback(messageReceived);

#if OPENEEW_SAMPLE_RATE_125
  odr_lpf = Adxl355::ODR_LPF::ODR_125_AND_31_25;
#endif

#if OPENEEW_SAMPLE_RATE_31_25
  odr_lpf = Adxl355::ODR_LPF::ODR_31_25_AND_7_813;
#endif

  pinMode(ADXL_INT_PIN, INPUT);
  pinMode(CHIP_SELECT_PIN_ADXL, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ADXL_INT_PIN), isr_adxl, FALLING);

  spi1 = new SPIClass(HSPI);
  adxl355.initSPI(*spi1);
  StartADXL355();

  //ledcSetup(channel, freq, resolution);
  //ledcAttachPin(io, channel);
  //pinMode(io, OUTPUT);
  //digitalWrite(io, LOW); // turn off buzzer

  // Connect to MQTT
  connectToMqtt();
  delay(3000);
}

void loop()
{
  if (!clientMqtt.connected())
  {
    // checkWiFiThenMQTT();
    checkWiFiThenReboot();
  }
  else
  {
    clientMqtt.loop();

    //====================== ADXL Accelerometer =====================

    if (fifoFull)
    {
      fifoFull = false;
      adxstatus = adxl355.getStatus();

      if (adxstatus & Adxl355::STATUS_VALUES::FIFO_FULL)
      {

        // Keep track of the heap in case heap fragmentation returns
        // Serial.println( xPortGetFreeHeapSize() );
        int numEntriesFifo = adxl355.readFifoEntries((long *)fifoOut);
        // numEntriesFifo = numEntriesFifo-1;

        if (numEntriesFifo != -1)
        {
          // Declare one AccelReading structure for this iteration of loop()
          // so it doesn't need to go in and out of scope in various for() loops below
          //   typedef struct AccelXYZ {
          //     double x; double y; double z;
          //   } AccelReading ;

          AccelReading AccelRecord;

          // [{"x":[9.479,0],"y":[0.128,-1.113],"z":[-0.185,123.321]},{"x":[9.479,0],"y":[0.128,-1.113],"z":[-0.185,123.321]}]
          double gal;
          double x, y, z;
          for (int i = 0; i < numEntriesFifo; i++)
          {
            gal = adxl355.valueToGals(fifoOut[i][0]);
            x = round(gal * 1000) / 1000;
            AccelRecord.x = x;

            gal = adxl355.valueToGals(fifoOut[i][1]);
            y = round(gal * 1000) / 1000;
            AccelRecord.y = y;

            gal = adxl355.valueToGals(fifoOut[i][2]);
            z = round(gal * 1000) / 1000;
            AccelRecord.z = z;

            StaLtaQue.push(&AccelRecord);
          }

          if (STALTAMODE) {
            // Do some STA / LTA math here...
            char mathmsg[65];
            snprintf(mathmsg, 64, "Calculating STA/LTA from %d accelerometer readings", StaLtaQue.getCount());
            // Serial.println(mathmsg);
            if (StaLtaQue.isFull()) {
              /////////////////// find offset ////////////////
              int queCount = StaLtaQue.getCount();

              for (int idx = 0; idx < queCount; idx++)
              {
                if (StaLtaQue.peekIdx(&AccelRecord, idx))
                {
                  sample[0] = AccelRecord.x;
                  sample[1] = AccelRecord.y;
                  sample[2] = AccelRecord.z;
                  for (int j = 0; j < 3; j++)
                  {
                    sampleSUM[j] += sample[j];
                  }
                }
              }
              for (int j = 0; j < 3; j++)
              {
                offset[j] = sampleSUM[j] / (QUE_len);
              }

              /////////////////// find lta /////////////////
              sampleSUM[0] = 0;
              sampleSUM[1] = 0;
              sampleSUM[2] = 0;
              for (int idx = 0; idx < LTA_len; idx++)
              {
                if (StaLtaQue.peekIdx(&AccelRecord, idx))
                {
                  sampleABS[0] = abs(AccelRecord.x - offset[0]);
                  sampleABS[1] = abs(AccelRecord.y - offset[1]);
                  sampleABS[2] = abs(AccelRecord.z - offset[2]);
                  for (int j = 0; j < 3; j++)
                  {
                    sampleSUM[j] += sampleABS[j];
                  }
                }
              }
              for (int j = 0; j < 3; j++)
              {
                ltav[j] = sampleSUM[j] / (LTA_len);
              }

              //////////////////// find sta ///////////////////////
              sampleSUM[0] = 0;
              sampleSUM[1] = 0;
              sampleSUM[2] = 0;
              for (int idx = LTA_len - STA_len; idx < LTA_len; idx++)
              {
                if (StaLtaQue.peekIdx(&AccelRecord, idx))
                {
                  sampleABS[0] = abs(AccelRecord.x - offset[0]);
                  sampleABS[1] = abs(AccelRecord.y - offset[1]);
                  sampleABS[2] = abs(AccelRecord.z - offset[2]);
                  for (int j = 0; j < 3; j++)
                  {
                    sampleSUM[j] += sampleABS[j];
                  }
                }
              }
              for (int j = 0; j < 3; j++)
              {
                stav[j] = sampleSUM[j] / STA_len;
                stalta[j] = stav[j] / ltav[j];
                if (bPossibleEarthQuake == false)
                {
                  if (stalta[j] >= thresh)
                  {
                    // Whoa - STA/LTA algorithm detected some anomalous shaking
                    Serial.printf("STA/LTA = %f = %f / %f (%i)\n", stalta[j], stav[j], ltav[j], j);
                    bPossibleEarthQuake = true;
                  }
                }
              }

              //// find STA/LTA for the other 31 samples but without doing the summing again

              for (int idx = LTA_len + 1; idx < QUE_len; idx++)
              {
                if (StaLtaQue.peekIdx(&AccelRecord, idx))
                {
                  sample[0] = AccelRecord.x;
                  sample[1] = AccelRecord.y;
                  sample[2] = AccelRecord.z;
                }
                if (StaLtaQue.peekIdx(&AccelRecord, idx - STA_len))
                {
                  sample1[0] = AccelRecord.x;
                  sample1[1] = AccelRecord.y;
                  sample1[2] = AccelRecord.z;
                }
                if (StaLtaQue.peekIdx(&AccelRecord, idx - LTA_len))
                {
                  LTAsample1[0] = AccelRecord.x;
                  LTAsample1[1] = AccelRecord.y;
                  LTAsample1[2] = AccelRecord.z;
                }
                for (int j = 0; j < 3; j++)
                {
                  sampleABS[j] = abs(sample[j] - offset[j]);
                  sample1ABS = abs(sample1[j] - offset[j]);
                  LTAsample1ABS = abs(LTAsample1[j] - offset[j]);
                  stav[j] += (sampleABS[j] - sample1ABS) / STA_len;
                  ltav[j] += (sampleABS[j] - LTAsample1ABS) / LTA_len;
                  stalta[j] = stav[j] / ltav[j];
                  if (bPossibleEarthQuake == false)
                  {
                    if (stalta[j] >= thresh)
                    {
                      // Whoa - STA/LTA algorithm detected some anomalous shaking
                      Serial.printf("STA/LTA = %f = %f / %f (%i)\n", stalta[j], stav[j], ltav[j], j);
                      bPossibleEarthQuake = true;
                    }
                  }
                }
              }
            }

            if (numSecsOfAccelReadings > 0)
            {
              SendLiveData2Cloud();
              numSecsOfAccelReadings--;
              bPossibleEarthQuake = false;
            }
            else if (bPossibleEarthQuake)
            {
              // The STA/LTA algorithm detected some anomalous shaking
              // If this is continued shaking, the above SendLiveData2Cloud()
              // function has already sent current accelerometer data
              // so don't send it again.
              bPossibleEarthQuake = false;

              // Start sending 5 minutes of live accelerometer data
              Serial.println("Start sending 5 minutes of live accelerometer data");
              numSecsOfAccelReadings = 300;

              // Send the previous 10 seconds of history to the cloud
              Send10Seconds2Cloud();
            }

            // Switch the direction of the LEDs
            breathedirection = breathedirection ? false : true;
          }
          else
          {
            // If the STALTAMODE is set to false, the device sends data continuously
            SendLiveData2Cloud();
            // Switch the direction of the LEDs
            breathedirection = breathedirection ? false : true;
          }

          // When this loop is done, drop 32 records off the queue
          if (StaLtaQue.isFull())
          {

            for (int i = 0; i < 32; i++)
              StaLtaQue.drop();
          }
        }
      }
    }

    // Set the LED light
    if (numSecsOfAccelReadings > 0 || STALTAMODE == false)
    {
      if ((millis() / 100) % 12 > 6)
      {
        strip.setBrightness(10);
        strip.fill(strip.Color(0, 255, 0), 0, 3);
      }
      else
      {
        strip.clear();
      }
    }
    else
    {
      if ((millis() / 1000) % 4 > 0)
      {
        strip.setBrightness(1);
        strip.fill(strip.Color(106, 160, 241), 0, 3);
      }
      else
      {
        strip.clear();
      }
    }
    strip.show();

    // Get NTP timestamp
    SyncNTPtime();
    if (NTP_timestamp_new - NTP_timestamp > 0)
    {
      NTP_timestamp = NTP_timestamp_new;
      time_since_NTP = micros();
    }
  }

  // Delay the main loop
  delay(1);
}
