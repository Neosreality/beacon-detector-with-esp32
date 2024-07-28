#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEAdvertisedDevice.h>
#include <WebServer.h>
#include <vector>
#include <string>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>
#include <HTTPClient.h>

#define ALARM_PIN 4 // Define the GPIO pin for the alarm

WebServer server(80);

// Global variables and semaphore declaration
SemaphoreHandle_t xSemaphore = NULL;

struct DeviceInfo {
  std::string mac;
  int rssi;
  std::string name;
  std::string advData;
  int8_t transmitPower;

  double distance() const {
    // Path loss exponent (n) typically ranges from 2 to 4 depending on the environment
    double pathLossExponent = 2.0;

    // Convert RSSI and transmit power to dBm
    double txPowerdBm = static_cast<double>(transmitPower);
    double rssidBm = static_cast<double>(rssi);

    // Calculate the distance in meters
    // Formula: distance = 10 ^ ((txPower - rssi) - A) / (10 * n)
    double A = 27.55; // Constant representing the environmental factor
    double distance = pow(10.0, ((txPowerdBm - rssidBm) - A) / (10 * pathLossExponent));

    return distance;
  }

  // Define equality operator
  bool operator==(const DeviceInfo& other) const {
    return mac == other.mac;
  }
};


std::vector<DeviceInfo> scannedDevices;
std::vector<DeviceInfo> filteredDevices;
std::vector<DeviceInfo> LastScannedDevices;
std::vector<DeviceInfo> LastFilteredDevices;

// configurations
// vector of mac addresses of devices to be scanned
std::vector<std::string> macAddresses = {
  "5f:09:f2:a2:b5:0c",
  "fe:49:0a:ef:58:6b"
};

int rssiThreshold = -70;
int rssiAlarmThreshold = -40;
bool thresholdEnabled = true;
bool isWhiteList = true;
int scanInterval = 5000;
int scanDuration = 5000;

struct WifiConfig {
  String ssid;
  String password;
};

inline void saveWifiConfiguration(const WifiConfig& config) {
  DynamicJsonDocument doc(1024);
  doc["ssid"] = config.ssid;
  doc["password"] = config.password;
  // Add other config fields as needed

  // Serialize JSON to file
  File configFile = SPIFFS.open("/wifi.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }
  serializeJson(doc, configFile);
  configFile.close();
}

WifiConfig loadWifiConfiguration() {
  WifiConfig wificonfig;
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount file system");
    return wificonfig;
  }

  File configFile = SPIFFS.open("/wifi.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return wificonfig;
  }

  size_t size = configFile.size();
  std::unique_ptr<char[]> buf(new char[size]);
  configFile.readBytes(buf.get(), size);

  DynamicJsonDocument doc(1024);
  auto error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return wificonfig;
  }

  wificonfig.ssid = doc["ssid"].as<String>();
  wificonfig.password = doc["password"].as<String>();

  //print the loaded wifi configuration to the serial monitor
  Serial.println("Loaded wifi configuration:");
  Serial.print("SSID: ");

  Serial.println(wificonfig.ssid);
  Serial.print("Password: ");
  Serial.println(wificonfig.password);
  
  configFile.close();
  return wificonfig;
}



std::vector<DeviceInfo> filterDevices(const std::vector<DeviceInfo>& devices, int rssi, const std::vector<std::string>& macAddresses) {
  std::vector<DeviceInfo> filteredDevices = devices;

  // Filter devices based on RSSI threshold if enabled
  if (thresholdEnabled) {
    for (auto& device : filteredDevices) {
      if (device.rssi < rssi) {
        device = {};
      }
    }
    filteredDevices.erase(std::remove(filteredDevices.begin(), filteredDevices.end(), DeviceInfo{}), filteredDevices.end());
  }

  if (isWhiteList) {
    for (auto& device : filteredDevices) {
      bool found = false;
      for (const auto& mac : macAddresses) {
        if (device.mac == mac) {
          found = true;
          break;
        }
      }
      if (!found) {
        device = {};
      }
    }
    filteredDevices.erase(std::remove(filteredDevices.begin(), filteredDevices.end(), DeviceInfo{}), filteredDevices.end());
  } else {
    for (auto& device : filteredDevices) {
      for (const auto& mac : macAddresses) {
        if (device.mac == mac) {
          device = {};
          break;
        }
      }
    }
    filteredDevices.erase(std::remove(filteredDevices.begin(), filteredDevices.end(), DeviceInfo{}), filteredDevices.end());
  }

  return filteredDevices;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    DeviceInfo device;
    device.mac = advertisedDevice.getAddress().toString();
    device.rssi = advertisedDevice.getRSSI();
    device.name = advertisedDevice.getName().c_str();
    device.advData = advertisedDevice.getManufacturerData().c_str();
    device.transmitPower = advertisedDevice.getTXPower();
    if (device.transmitPower == 0) {
      device.transmitPower = -50;
    }

    scannedDevices.push_back(std::move(device));
  }
};

// Load configuration from config.json file
inline void loadConfiguration() {
  // check if config file exists in directory data
  if (!SPIFFS.exists("/config.json")) {
    Serial.println("Config file does not exist");
    Serial.println("using hardcoded values");
    return;
  }
  File configFile = SPIFFS.open("/config.json", "r"); // Open the config file
  if (!configFile) {
    Serial.println("Failed to open config file");
    Serial.println("using hardcoded values");
    return;
  }

  size_t size = configFile.size(); // Get the size of the config file
  std::unique_ptr<char[]> buf(new char[size]); // Create a buffer to store the file contents
  configFile.readBytes(buf.get(), size); // Read the file into the buffer

  StaticJsonDocument<1024> doc; // Create a static JSON document
  auto error = deserializeJson(doc, buf.get()); // Parse the JSON buffer
  if (error) {
    Serial.println("Failed to parse config file");
    Serial.println("using hardcoded values");
    configFile.close(); // Close the file
    return;
  }

  // Update the global variables with the values from the config file
  JsonArray macs = doc["macAddresses"];
  macAddresses.clear(); // Clear the existing vector
  for (const auto& v : macs) {
    macAddresses.push_back(v.as<std::string>()); // Add the MAC addresses from the config file
  }
  rssiThreshold = doc["rssiThreshold"]; // Update the RSSI threshold
  rssiAlarmThreshold = doc["rssiAlarmThreshold"];
  // Update the thresholdEnabled, isWhiteList, scanInterval, and scanDuration variables from the config file
  thresholdEnabled = doc["thresholdEnabled"];
  isWhiteList = doc["isWhiteList"];
  scanInterval = doc["scanInterval"];
  scanDuration = doc["scanDuration"];

  Serial.println("Configuration loaded from config file");
  // print the loaded configuration
  Serial.println("Loaded configuration:");
  Serial.print("RSSI Threshold: ");
  Serial.println(rssiThreshold);
  Serial.print("RSSI Alarm Threshold: ");
  Serial.println(rssiAlarmThreshold);
  Serial.println("MAC Addresses:");
  for (const auto& mac : macAddresses) {
    Serial.println(mac.c_str());
  }
  Serial.print("Threshold Enabled: ");
  Serial.println(thresholdEnabled);
  Serial.print("Is White List: ");
  Serial.println(isWhiteList);
  Serial.print("Scan Interval: ");
  Serial.println(scanInterval);
  Serial.print("Scan Duration: ");
  Serial.println(scanDuration);

  configFile.close(); // Close the file
}

// save configuration to config.json file
inline void saveConfiguration() {
  StaticJsonDocument<1024> doc; // Create a static JSON document
  JsonArray macs = doc.createNestedArray("macAddresses"); // Create a nested array for MAC addresses
  for (const auto& mac : macAddresses) {
    macs.add(mac); // Add each MAC address to the array
  }
  doc["rssiThreshold"] = rssiThreshold; // Add the RSSI threshold to the document
  doc["rssiAlarmThreshold"] = rssiAlarmThreshold;
  // Add the thresholdEnabled, isWhiteList, scanInterval, and scanDuration variables to the document
  doc["thresholdEnabled"] = thresholdEnabled;
  doc["isWhiteList"] = isWhiteList;
  doc["scanInterval"] = scanInterval;
  doc["scanDuration"] = scanDuration;

  File configFile = SPIFFS.open("/config.json", "w"); // Open the config file for writing
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    configFile.close(); // Close the file
    return;
  }

  serializeJson(doc, configFile); // Serialize the JSON document to the file
  configFile.close(); // Close the file
  Serial.println("Configuration saved to config file");
  // print the saved configuration
  Serial.println("Saved configuration:");
  Serial.print("RSSI Threshold: ");
  Serial.println(rssiThreshold);
   Serial.print("RSSI Alarm Threshold: ");
  Serial.println(rssiAlarmThreshold);
  Serial.println("MAC Addresses:");
  for (const auto& mac : macAddresses) {
    Serial.println(mac.c_str());
  }
  Serial.print("Threshold Enabled: ");
  Serial.println(thresholdEnabled);
  Serial.print("Is White List: ");
  Serial.println(isWhiteList);
  Serial.print("Scan Interval: ");
  Serial.println(scanInterval);
  Serial.print("Scan Duration: ");
  Serial.println(scanDuration);
}


void handleDefault() {
    String html = R"rawliteral(
        <html>
        <body>
            <h1>Network Configuration</h1>
            <form action='/connect' method='POST'>
                <label for='ssid'>SSID:</label><br>
                <input type='text' id='ssid' name='ssid'><br>
                <label for='password'>Password:</label><br>
                <input type='password' id='password' name='password'><br><br>
                <input type='submit' value='Submit'>
            </form>
            
            <h2>Available endpoints:</h2>
            <ul>
                <li>/addMac - Add MAC address to the list</li>
                <li>/getMacs - Get the list of MAC addresses</li>
                <li>/removeMac - Remove MAC address from the list</li>
                <li>/updateRssi - Update the RSSI threshold</li>
                <li>/getConfig - Get the current configuration</li>
                <li>/sendConfig - Send the new configuration</li>
                <li>/getDevices - Get the last scanned devices</li>
                <li>/getDeviceInfo - Get the MAC address of the device</li>
            </ul>
        </body>
        </html>
    )rawliteral";

    server.send(200, "text/html", html);
}

void handleConnect() {
  int max_attempts = 30;
  String new_ssid = server.arg("ssid");
  String new_password = server.arg("password");
  //save new wifi configuration to the file system
  saveWifiConfiguration({new_ssid, new_password});
  

  WiFi.begin(new_ssid.c_str(), new_password.c_str());
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    server.send(200, "text/html", "<html><body><h1>Connected! device IP address: " + WiFi.localIP().toString() + "</h1></body></html>");
    //turn on the led to indicate successful connection
    digitalWrite(2, HIGH);
  } else {
    server.send(200, "text/html", "<html><body><h1>Failed to connect. Try again.</h1></body></html>");
  }
}

inline void connectToWiFi(WifiConfig config) {
  WiFi.begin(config.ssid.c_str(), config.password.c_str());
  int max_attempts = 30;
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.scanNetworks(); // Scan for available networks
    WiFi.mode(WIFI_AP); // Set as access point
    WiFi.softAP("ESP32_AP");
    server.on("/", handleDefault);
    server.on("/connect", HTTP_POST, handleConnect);
    server.begin();
    Serial.println("HTTP server started");
    //while loop to keep the server running
    while (WiFi.status() != WL_CONNECTED) {
      //turn off the led to indicate no connection
      digitalWrite(2, LOW);
      server.handleClient();
      delay(1);
    }
  


    // Print IP
    Serial.print("Connect to: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: "); 
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
}

inline void startAccesPoint(const char* ssid, const char* password) {
  // Start the access point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started.");
  
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
    Serial.println(apIP);
}

inline void createFileSystem() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
  }

  Serial.println("File system mounted with success");
}

inline void scanBLEDevices() {
  BLEScan* pBLEScan = BLEDevice::getScan(); // Get BLE scan object to start scanning
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Set callback to get scan results
  pBLEScan->start(scanDuration/1000, false); // Start scanning for BLE devices

  Serial.println("Scanning for BLE devices...");
  // Wait for the scan to finish and blink the LED while scanning
  for (int i = 0; i < 6; i++) {
    digitalWrite(2, LOW); // Turn off LED
    delay(100);
    digitalWrite(2, HIGH); // Turn on LED
    delay(100);
  }
}



void handleAddMac() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    auto mac = doc["macAddresses"].as<std::string>();
    // remove [" and "] from the mac address
    mac.erase(0, 2);
    mac.erase(mac.length() - 2, 2);
    macAddresses.push_back(mac.c_str());
    saveConfiguration(); // Implement this function to save the updated list to the config file
    server.send(200, "application/json", "{\"result\":\"MAC added\"}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Bad Request\"}");
  }
}

void handleGetMacs() {
  DynamicJsonDocument doc(1024);
  JsonArray macs = doc.createNestedArray("macAddresses");
  for (const auto& mac : macAddresses) {
    macs.add(mac);
  }
  std::string response;
  serializeJson(doc, response);
  server.send(200, "application/json", response.c_str());
}

void handleRemoveMac() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    auto macToRemove = doc["macAddresses"].as<std::string>();
    macToRemove.erase(0, 2);
    macToRemove.erase(macToRemove.length() - 2, 2);
    macAddresses.erase(std::remove(macAddresses.begin(), macAddresses.end(), macToRemove.c_str()), macAddresses.end());
    saveConfiguration(); // Implement this function to save the updated list to the config file
    server.send(200, "application/json", "{\"result\":\"MAC removed\"}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Bad Request\"}");
  }
}

void handleUpdateRssi() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    rssiThreshold = doc["rssiThreshold"];
    saveConfiguration(); // Implement this function to save the updated RSSI threshold to the config file
    server.send(200, "application/json", "{\"result\":\"RSSI updated to " + String(rssiThreshold) + "\"}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Bad Request\"}");
  }
}

void handleGetconfig() {
  // send the current config.json file as response
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    server.send(500, "application/json", "{\"error\":\"Failed to open config file\"}");
    return;
  }
  std::string response = configFile.readString().c_str();
  server.send(200, "application/json", response.c_str());
}

void handleSendConfig() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    JsonArray macs = doc["macAddresses"];
    macAddresses.clear();
    for (const auto& v : macs) {
      macAddresses.push_back(v.as<std::string>());
    }
    rssiThreshold = doc["rssiThreshold"];
    rssiAlarmThreshold = doc["rssiAlarmThreshold"];
    thresholdEnabled = doc["thresholdEnabled"];
    isWhiteList = doc["isWhiteList"];
    scanInterval = doc["scanInterval"];
    scanDuration = doc["scanDuration"];

    saveConfiguration(); // Implement this function to save the updated configuration to the config file
    server.send(200, "application/json", "{\"result\":\"Configuration updated\"}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Bad Request\"}");
  }
}

void handleGetDevices() {
  DynamicJsonDocument doc(1024);
  JsonArray devices = doc.createNestedArray("devices");
  for (const auto& device : LastFilteredDevices) {
    JsonObject obj = devices.createNestedObject();
    obj["mac"] = device.mac;
    obj["rssi"] = device.rssi;
    // send also device name adv data and distance
    obj["name"] = device.name;
    obj["advData"] = device.advData;
    obj["distance"] = device.distance();
  }
  std::string response;
  serializeJson(doc, response);
  server.send_P(200, "application/json", response.c_str());
}

void handleGetDeviceInfo(){
  //send the mac address and device type of the device this code is running on
  DynamicJsonDocument doc(1024);
  doc["mac"] = WiFi.macAddress();
  doc["deviceType"] = "BeaconDetector";
  std::string response;
  serializeJson(doc, response);
  server.send(200, "application/json", response.c_str());
}

// inline function to define the endpoints
inline void defineEndpoints() {
 server.on("/", HTTP_GET, handleDefault);
    server.on("/connect", HTTP_POST, handleConnect);
    server.on("/addMac", HTTP_POST, handleAddMac);
    server.on("/getMacs", HTTP_GET, handleGetMacs);
    server.on("/removeMac", HTTP_POST, handleRemoveMac);
    server.on("/updateRssi", HTTP_POST, handleUpdateRssi);
    server.on("/getConfig", HTTP_GET, handleGetconfig);
    server.on("/sendConfig", HTTP_POST, handleSendConfig);
    server.on("/getDevices", HTTP_GET, handleGetDevices);
    server.on("/getDeviceInfo", HTTP_GET, handleGetDeviceInfo);
    

    server.onNotFound([]() {
        server.send(404, "text/plain", "Not Found");
    });
}



void checkRssiAlarmThreshold(const std::vector<DeviceInfo>& devices) {
  bool alarmTriggered = false;


  for (const auto& device : devices) {
    if (device.rssi >= rssiAlarmThreshold) {
      //print the device that triggered the alarm
      Serial.print("Device that triggered the alarm: ");
      Serial.print(device.mac.c_str());
      alarmTriggered = true;
      break;
    }
  }

  if (alarmTriggered) {
    // Trigger alarm
    digitalWrite(ALARM_PIN, HIGH);
    Serial.println("Alarm triggered!");
  } else {
    // Turn off alarm
    digitalWrite(ALARM_PIN, LOW);
    Serial.println("Alarm turned off");
  }
}

void MainTask(void* pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) { // Check if WiFi is connected
      Serial.println("WiFi connection lost. Reconnecting...");
      connectToWiFi(loadWifiConfiguration()); // Reconnect to WiFi
      return;
    }
    
    scanBLEDevices(); // Scan for BLE devices

    if (scannedDevices.empty()) {
      Serial.println("No devices");
    }

    filteredDevices = filterDevices(scannedDevices, rssiThreshold, macAddresses);

    // Print if there are scanned devices
    if (!filteredDevices.empty()) {
      for (const auto& device : filteredDevices) { // write the filtered devices to the serial monitor in a single line
        Serial.print("MAC: ");
        Serial.print(device.mac.c_str());
        Serial.print(" RSSI: ");
        Serial.print(device.rssi);
        Serial.print(" Distance: ");
        Serial.print(device.distance());
        Serial.print(" Name: ");
        Serial.print(String(device.name.c_str()));
        Serial.print(" Adv Data: ");
        Serial.println(String(device.advData.c_str()));
      }
    } else {
      Serial.println("No devices found unfiltered devices are: ");
      for (const auto& device : scannedDevices) { // write the scanned devices to the serial monitor in a single line
        Serial.print("MAC: ");
        Serial.print(device.mac.c_str());
        Serial.print(" RSSI: ");
        Serial.print(device.rssi);
        Serial.print(" Distance: ");
        Serial.print(device.distance());
        Serial.print(" Name: ");
        Serial.print(String(device.name.c_str()));
        Serial.print(" Adv Data: ");
        Serial.println(String(device.advData.c_str()));
      }
    }
    
    checkRssiAlarmThreshold(filteredDevices); 

    LastScannedDevices = scannedDevices; // Save the last scanned devices
    LastFilteredDevices = filteredDevices; // Save the last filtered devices

    scannedDevices.clear(); // Clear the list for the next scan
    filteredDevices.clear(); // Clear the list for the next scan
    delay(scanInterval); // Delay before the next scan
  }
}




void HTTPServerTask(void* pvParameters) {
  server.begin(80); // Start listening for HTTP requests
  for (;;) {
    if (xSemaphoreTake(xSemaphore, (TickType_t)10) == pdTRUE) {
      // Handle HTTP requests here
      server.handleClient();
      xSemaphoreGive(xSemaphore);
    }
    delay(10); // Delay to prevent watchdog timer reset and allow other tasks to run
  }
}

 


void setup() {
  Serial.begin(115200); // Initialize serial communication

//set led pin as output
  pinMode(2, OUTPUT);

  pinMode(ALARM_PIN, OUTPUT); // Set the GPIO pin as output for the alarm
  //set the alarm pin to low
  digitalWrite(ALARM_PIN, LOW);
  //initilalize alarm parameters
  

  BLEDevice::init(""); // Initialize BLE
  createFileSystem(); // Mount the file system
  loadConfiguration(); // Load the configuration from the config file
  WifiConfig config = loadWifiConfiguration();
  defineEndpoints(); // Define HTTP server endpoints

  // Initialize semaphore
  xSemaphore = xSemaphoreCreateMutex();
  if (xSemaphore == NULL) {
    Serial.println("Failed to create semaphore");
    return;
  }

  connectToWiFi(config); // Connect to WiFi
  //startAccesPoint("ESP32_AP", "password"); // Start the access point

  // Create tasks
  xTaskCreate(MainTask, "MainTask", 4096, NULL, 1, NULL);
  xTaskCreate(HTTPServerTask, "HTTPServerTask", 4096, NULL, 1, NULL);

}

void loop() {
  // The loop function is intentionally left empty as we are using FreeRTOS tasks
}

