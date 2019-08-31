// Default Arduino includes
#include <Arduino.h>
#include <WiFi.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <ArduinoJson.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include "DHT.h"

#define DHTPIN 12
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
SSD1306Wire display(0x3c, 5, 4);

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;

/** Unique device name */
char apName[] = "ESP32-xxxxxxxxxxxx";
/** Selected network
    true = use primary network
    false = use secondary network
*/
bool usePrimAP = true;
/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
/** Connection change status */
bool connStatusChanged = false;

const char* mqttServer = "farmer.cloudmqtt.com";
const int mqttPort = 18602;
const char* mqttUser = "gdcqpytd";
const char* mqttPassword = "d2ADEyedfhP7";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
float temperature = 0;

/**
   Create unique device name from MAC address
 **/
void createName() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Write unique name into apName
  sprintf(apName, "ESP32-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
}

// List of Service and Characteristic UUIDs
#define SERVICE_UUID  "0000aaaa-ead2-11e7-80c1-9a214cf093ae"
#define WIFI_UUID     "00005555-ead2-11e7-80c1-9a214cf093ae"

/** SSIDs of local WiFi networks */
String ssidPrim;
String ssidSec;
/** Password for local WiFi network */
String pwPrim;
String pwSec;

/** Characteristic for digital output */
BLECharacteristic *pCharacteristicWiFi;
/** BLE Advertiser */
BLEAdvertising* pAdvertising;
/** BLE Service */
BLEService *pService;
/** BLE Server */
BLEServer *pServer;

StaticJsonBuffer<200> jsonBuffer;

/**
   MyServerCallbacks
   Callbacks for client connection and disconnection
*/
class MyServerCallbacks: public BLEServerCallbacks {
    // TODO this doesn't take into account several clients being connected
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE client connected");
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("BLE client disconnected");
      pAdvertising->start();
    }
};

/**
   MyCallbackHandler
   Callbacks for BLE client read/write requests
*/
class MyCallbackHandler: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() == 0) {
        return;
      }
      Serial.println("Received over BLE: " + String((char *)&value[0]));

      // Decode data
      int keyIndex = 0;
      for (int index = 0; index < value.length(); index ++) {
        value[index] = (char) value[index] ^ (char) apName[keyIndex];
        keyIndex++;
        if (keyIndex >= strlen(apName)) keyIndex = 0;
      }

      /** Json object for incoming data */
      JsonObject& jsonIn = jsonBuffer.parseObject((char *)&value[0]);
      if (jsonIn.success()) {
        if (jsonIn.containsKey("ssidPrim") &&
            jsonIn.containsKey("pwPrim") &&
            jsonIn.containsKey("ssidSec") &&
            jsonIn.containsKey("pwSec")) {
          ssidPrim = jsonIn["ssidPrim"].as<String>();
          pwPrim = jsonIn["pwPrim"].as<String>();
          ssidSec = jsonIn["ssidSec"].as<String>();
          pwSec = jsonIn["pwSec"].as<String>();

          Preferences preferences;
          preferences.begin("WiFiCred", false);
          preferences.putString("ssidPrim", ssidPrim);
          preferences.putString("ssidSec", ssidSec);
          preferences.putString("pwPrim", pwPrim);
          preferences.putString("pwSec", pwSec);
          preferences.putBool("valid", true);
          preferences.end();

          Serial.println("Received over bluetooth:");
          Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
          Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
          connStatusChanged = true;
          hasCredentials = true;
        } else if (jsonIn.containsKey("erase")) {
          Serial.println("Received erase command");
          Preferences preferences;
          preferences.begin("WiFiCred", false);
          preferences.clear();
          preferences.end();
          connStatusChanged = true;
          hasCredentials = false;
          ssidPrim = "";
          pwPrim = "";
          ssidSec = "";
          pwSec = "";

          int err;
          err = nvs_flash_init();
          Serial.println("nvs_flash_init: " + err);
          err = nvs_flash_erase();
          Serial.println("nvs_flash_erase: " + err);
        } else if (jsonIn.containsKey("reset")) {
          WiFi.disconnect();
          esp_restart();
        }
      } else {
        Serial.println("Received invalid JSON");
      }
      jsonBuffer.clear();
    };

    void onRead(BLECharacteristic *pCharacteristic) {
      Serial.println("BLE onRead request");
      String wifiCredentials;

      /** Json object for outgoing data */
      JsonObject& jsonOut = jsonBuffer.createObject();
      jsonOut["ssidPrim"] = ssidPrim;
      jsonOut["pwPrim"] = pwPrim;
      jsonOut["ssidSec"] = ssidSec;
      jsonOut["pwSec"] = pwSec;
      // Convert JSON object into a string
      jsonOut.printTo(wifiCredentials);

      // encode the data
      int keyIndex = 0;
      Serial.println("Stored settings: " + wifiCredentials);
      for (int index = 0; index < wifiCredentials.length(); index ++) {
        wifiCredentials[index] = (char) wifiCredentials[index] ^ (char) apName[keyIndex];
        keyIndex++;
        if (keyIndex >= strlen(apName)) keyIndex = 0;
      }
      pCharacteristicWiFi->setValue((uint8_t*)&wifiCredentials[0], wifiCredentials.length());
      jsonBuffer.clear();
    }
};

/**
   initBLE
   Initialize BLE service and characteristic
   Start BLE server and service advertising
*/
void initBLE() {
  // Initialize BLE and set output power
  BLEDevice::init(apName);
  BLEDevice::setPower(ESP_PWR_LVL_P7);

  // Create BLE Server
  pServer = BLEDevice::createServer();

  // Set server callbacks
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  pService = pServer->createService(BLEUUID(SERVICE_UUID), 20);

  // Create BLE Characteristic for WiFi settings
  pCharacteristicWiFi = pService->createCharacteristic(
                          BLEUUID(WIFI_UUID),
                          // WIFI_UUID,
                          BLECharacteristic::PROPERTY_READ |
                          BLECharacteristic::PROPERTY_WRITE
                        );
  pCharacteristicWiFi->setCallbacks(new MyCallbackHandler());

  // Start the service
  pService->start();

  // Start advertising
  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  if (hasCredentials) {
    // Check for available AP's
    if (!scanWiFi) {
      Serial.println("Could not find any AP");
    } else {
      // If AP was found, start connection
      connectWiFi();
    }
  }
}

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event) {
  isConnected = true;
  connStatusChanged = true;
}

/** Callback for connection loss */
void lostCon(system_event_id_t event) {
  isConnected = false;
  connStatusChanged = true;
}

/**
   scanWiFi
   Scans for available networks
   and decides if a switch between
   allowed networks makes sense

   @return <code>bool</code>
          True if at least one allowed network was found
*/
bool scanWiFi() {
  /** RSSI for primary network */
  int8_t rssiPrim;
  /** RSSI for secondary network */
  int8_t rssiSec;
  /** Result of this function */
  bool result = false;

  Serial.println("Start scanning for networks");

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  // Scan for AP
  int apNum = WiFi.scanNetworks(false, true, false, 1000);
  if (apNum == 0) {
    Serial.println("Found no networks");
    return false;
  }

  byte foundAP = 0;
  bool foundPrim = false;

  for (int index = 0; index < apNum; index++) {
    String ssid = WiFi.SSID(index);
    Serial.println("Found AP: " + ssid + " RSSI: " + WiFi.RSSI(index));
    if (!strcmp((const char*) &ssid[0], (const char*) &ssidPrim[0])) {
      Serial.println("Found primary AP");
      foundAP++;
      foundPrim = true;
      rssiPrim = WiFi.RSSI(index);
    }
    if (!strcmp((const char*) &ssid[0], (const char*) &ssidSec[0])) {
      Serial.println("Found secondary AP");
      foundAP++;
      rssiSec = WiFi.RSSI(index);
    }
  }

  switch (foundAP) {
    case 0:
      result = false;
      break;
    case 1:
      if (foundPrim) {
        usePrimAP = true;
      } else {
        usePrimAP = false;
      }
      result = true;
      break;
    default:
      Serial.printf("RSSI Prim: %d Sec: %d\n", rssiPrim, rssiSec);
      if (rssiPrim > rssiSec) {
        usePrimAP = true; // RSSI of primary network is better
      } else {
        usePrimAP = false; // RSSI of secondary network is better
      }
      result = true;
      break;
  }
  return result;
}

void checkStatusChanges() {
  if (connStatusChanged) {
    if (isConnected) {
      Serial.print("Connected to AP: ");
      Serial.print(WiFi.SSID());
      Serial.print(" with IP: ");
      Serial.print(WiFi.localIP());
      Serial.print(" RSSI: ");
      Serial.println(WiFi.RSSI());
      if (!client.connected()) {
        reconnect();
      }
        client.loop();
    } else {
      if (hasCredentials) {
        Serial.println("Lost WiFi connection");
        // Received WiFi credentials
        if (!scanWiFi) { // Check for available AP's
          Serial.println("Could not find any AP");
        } else { // If AP was found, start connection
          connectWiFi();
        }
      }
    }
    connStatusChanged = false;
  }
}

/**
   Start connection to AP
*/
void connectWiFi() {
  // Setup callback function for successful connection
  WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
  // Setup callback function for lost connection
  WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("Start connection to ");
  if (usePrimAP) {
    Serial.println(ssidPrim);
    WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    randomSeed(micros());
  } else {
    Serial.println(ssidSec);
    WiFi.begin(ssidSec.c_str(), pwSec.c_str());
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");
      //Once connected, publish an announcement...
      client.publish("/icircuit/presence/ESP32/", "hello world");
      // ... and resubscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte *payload, unsigned int length) {
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();
}

void setup() {
  pinMode(14, INPUT_PULLUP);
  dht.begin();
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  // Create unique device name
  createName();

  // Initialize Serial port
  Serial.begin(115200);
  // Send some device info
  Serial.print("Build: ");
  Serial.println(compileDate);

  Preferences preferences;
  preferences.begin("WiFiCred", false);
  bool hasPref = preferences.getBool("valid", false);
  if (hasPref) {
    ssidPrim = preferences.getString("ssidPrim", "");
    ssidSec = preferences.getString("ssidSec", "");
    pwPrim = preferences.getString("pwPrim", "");
    pwSec = preferences.getString("pwSec", "");

    if (ssidPrim.equals("")
        || pwPrim.equals("")
        || ssidSec.equals("")
        || pwPrim.equals("")) {
      Serial.println("Found preferences but credentials are invalid");
    } else {
      Serial.println("Read from preferences:");
      Serial.println("primary SSID: " + ssidPrim + " password: " + pwPrim);
      Serial.println("secondary SSID: " + ssidSec + " password: " + pwSec);
      hasCredentials = true;
    }
  } else {
    Serial.println("Could not find preferences, need send data over BLE");
  }
  preferences.end();

  // Start BLE server
  initBLE();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  randomSeed(analogRead(0));
}

void loop() {
  checkStatusChanges();

  long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    int motion = digitalRead(14);
    temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(humidity) || isnan(temperature) || isnan(motion)) {
      Serial.println("Failed to read from sensor!"); return;
    }
    if (motion == 1)
      display.drawStringMaxWidth(0, 30, 128, "Motion Detected" );
    else
      display.drawStringMaxWidth(0, 30, 128, "No Motion" );
    display.drawStringMaxWidth(0, 0, 128, "Humidity:" + String(humidity));
    display.drawStringMaxWidth(0, 15, 128, "Temperature:" + String(temperature));
    float hic = dht.computeHeatIndex(temperature, humidity, false);
    long randWeight = random(10);
    if(randWeight%2==0 && randWeight < 7)
      randWeight = random(1,9);
    else randWeight = 0;
    
    display.drawStringMaxWidth(0, 45, 128, "Weight:" + String(randWeight));
    display.display();

    Serial.println("Weight: " + String(randWeight));
    Serial.println("Motion: " + String(motion));
    Serial.println("Humidity: " + String(humidity) + " %\t");
    Serial.println("Temperature: " + String(temperature) + " *C\t");
    Serial.println("Heat index: " + String(hic) + " *C\t");

    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    client.publish("sensors/heat", tempString);

    dtostrf(motion, 1, 2, tempString);
    client.publish("sensors/movement", tempString);

    dtostrf(humidity, 1, 2, tempString);
    client.publish("sensors/humidity", tempString);

    dtostrf(randWeight, 1, 2, tempString);
    client.publish("sensors/weight", tempString);
  }
}
