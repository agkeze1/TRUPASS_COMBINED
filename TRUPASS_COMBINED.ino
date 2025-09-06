#include <DFPlayerMini_Fast.h>
#include "Adafruit_MCP23X17.h"
#include <Adafruit_SSD1306.h>
#include <WiFiClientSecure.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h> 
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <driver/i2s.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <TimeLib.h>
#include <Keypad.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing Arduino reset pin)

#define SD_CS_PIN 5  // SD card CS pin

// ===== I2S pins (ESP32 → PCM5102) =====
#define I2S_BCK   26   // Bit clock
#define I2S_LRCK  27   // Left/Right clock (WS)
#define I2S_DATA  25   // Data out (DIN on PCM5102)
#define I2S_PORT  I2S_NUM_0

HardwareSerial QRcodeSerial(1);  // For QRCode scanner communication
// Define the serial pins for the QRCode scanner
#define QRCode_RX_PIN 14
#define QRCode_TX_PIN 13

HardwareSerial RFIDSerial(2);  // For RFID reader serial communication
#define RFID_RX_PIN 32 
#define RFID_TX_PIN 33 

#define BUZZER_PIN 16

#define ENTER_KEY_PIN 34

Adafruit_MCP23X17 mcp;

#define OPEN_BOOM_BARRIER_RELAY_PIN 15

String scannedQRCodeData = "";  // Global variable to store the scanned QRCode data

// Configuration commands for RFID 
uint8_t singlePollingCommand[] = {0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E}; 

WiFiManager wm; 

// ====== CONFIG ======
const char *networksFile = "/config/networks.json";
const unsigned long WIFI_CONNECT_TIMEOUT = 8000;   // Time to wait for each network
const unsigned long WIFI_CYCLE_TIMEOUT = 30000;    // Time to try all networks before starting portal
const unsigned long WIFI_CHECK_INTERVAL = 2000;    // Check WiFi every 2s
const unsigned long CONFIG_PORTAL_TIMEOUT = 120;   // Config portal timeout (seconds)

// ====== STATE ======
struct NetworkCredential {
  String ssid;
  String password;
};

std::vector<NetworkCredential> networkList;

unsigned long lastWiFiCheck = 0;
unsigned long wifiLostStart = 0;
unsigned long networkAttemptStart = 0;
int currentNetworkIndex = 0;
bool portalActive = false;
bool tryingNetworks = false;

WiFiClient wc;// Instance of wiFiClient for MQTT connection
bool wiFiConnected;  // For indicating if wiFi is connected or not
PubSubClient mqttClient(wc);

// ----- DEVICE CONFIG ---------
String estateID = ""; 
String deviceName = ""; 
String deviceType = ""; 

// Ably MQTT
const char* mqtt_server = "mqtt.ably.io";
const int mqtt_port = 1883;
const char* ably_key = "ah19qA.9AzaFw:BshXkm0AZF1BYFgIEZ71CUXIP4gkUU4WZ6dcWv3AHAQ";

// Declare MQTT Topic variables
String accessGained = "";
String newResidentAdded = "";
String residentRemoved = "";
String newVisit = "";
String visitUpdated = "";
String visitCancelled = "";
String visitEnded = "";
String feeDriveUpdated = "";
String feePaymentUpdated = "";

const char* apolloEndpoint = "https://trupass-api.myregionalhub.com/";
const char* authToken = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZCI6IjY0YWZhNWE2MWM3NjdiZTk5YTBiMGRmOCIsIm5hbWUiOiJFamlrZSBFemUiLCJhZG1pbiI6dHJ1ZSwic3VwZXJBZG1pbiI6dHJ1ZSwicHJpbWFyeUVzdGF0ZSI6eyJhZG1pbiI6dHJ1ZSwiZXN0YXRlIjoiNjRhZmI1YjExYzc2N2JlOTlhMGIwZTBhIn0sImlhdCI6MTczODU3NTgxNX0.D0qH1Jo5xqwpj1VH51EB0cq7B_bgOUEq5OYhvBrvr-k";

HTTPClient http;

bool feeDriveIsActive;

struct Resident {
  String id;
  String epc;
  String accessCode;
  std::vector<String> plots; // Store plots as a vector of comma-separated string or a dynamic array
};

struct Visitor {
  String id;
  String accessCode;
  String plot;
  String denyExit;
};

// Lookup table for visitor key-value handlers
struct VisitorKeyHandler {
  const char* key;
  void (*handler)(String, Visitor&);
};

// Lookup table for resident key-value handlers
struct ResidentKeyHandler {
  const char* key;
  void (*handler)(String, Resident&);
};

// GitHub page Raw URL for access_granted.wav file
const char* fileURL = "https://agkeze1.github.io/trupass_access_audio_files/access_granted.wav";
// const char* fileURL = "https://raw.githubusercontent.com/agkeze1/trupass_access_audio_files/main/access_granted.wav";

// SD Card access audio File Path
const char* accessGranted = "/audio/access_granted.wav"; 
const char* accessDenied = "/audio/access_denied.wav"; 
const char* accessDeniedFee = "/audio/access_denied_fee.wav"; 
const char* deviceIsReadyAudio = "/audio/device_is_ready.wav"; 
const char* networkFailedAudio = "/audio/network_failed.wav"; 

unsigned long previousQRCodeScanMillis = 0; // Holds the last time in milliseconds a qrCode scan was done  
unsigned long lastIdentificationTimeMillis = 0; // Holds the last time in milliseconds a scan was done  
int identificationDelayTime = 2000;
bool boomBarrierIsOpen = false;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Instantiate OLED 

// Keypad configuration for ESP32
const uint8_t ROWS = 4;   // Four rows
const uint8_t COLS = 3;   // Three columns

// Map keypad rows/cols to MCP pin numbers
uint8_t rowPins[ROWS] = {0, 1, 2, 3};       // GPA0–GPA3
uint8_t colPins[COLS] = {8, 9, 10};     // GPB0–GPB2

char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

// Subclass Keypad to override pin access
class MCPKeypad : public Keypad {
public:
  MCPKeypad(char *userKeymap, uint8_t *row, uint8_t *col,
            uint8_t numRows, uint8_t numCols, Adafruit_MCP23X17 *mcp)
    : Keypad(userKeymap, row, col, numRows, numCols), _mcp(mcp) {}

  void pin_write(uint8_t pin, boolean level) override {
    _mcp->digitalWrite(pin, level);
  }

  int pin_read(uint8_t pin) override {
    return _mcp->digitalRead(pin);
  }

  void pin_mode(uint8_t pin, uint8_t mode) override {
    // Translate Arduino pinMode into MCP version
    if (mode == INPUT) {
      _mcp->pinMode(pin, INPUT);
    } else if (mode == INPUT_PULLUP) {
      _mcp->pinMode(pin, INPUT_PULLUP);
    } else {
      _mcp->pinMode(pin, OUTPUT);
    }
  }

private:
  Adafruit_MCP23X17* _mcp;
};

// Instantiate keypad with MCP backend
MCPKeypad keypad = MCPKeypad(makeKeymap(keys), rowPins, colPins, 4, 3, &mcp);

String passkey = ""; // Entered Passkey

// ------- ENTER-KEY PRESS VARIABLE --------

#define DEBOUNCE_DELAY 50  

unsigned long lastDebounceTime = 0;
bool lastButtonState = HIGH;

enum Screen {HOME, PASSKEY}; // Enum for device screens
Screen currentScreen = HOME; // Active Device Screen (HOME is default)

// Github page RootCACertificate for downloading access audio files
const char* rootCACertificate = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


// For "Enter Key" long press
static unsigned long EnterkeyPressStartTime = 0;
static bool EnterkeyStillPressed = false;
unsigned long lastUpdateTime = 0;
unsigned long lastMillis = 0;
const unsigned long timeInterval = 1000; // Interval to simulate each second


// ===== WAV header struct =====
struct WavHeader {
  char riff[4];
  uint32_t overall_size;
  char wave[4];
  char fmt_chunk_marker[4];
  uint32_t length_of_fmt;
  uint16_t format_type;
  uint16_t channels;
  uint32_t sample_rate;
  uint32_t byterate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data_chunk_header[4];
  uint32_t data_size;
};

// ===== Setup I2S =====
bool i2s_initialized = false;
uint32_t current_sample_rate = 0;
uint16_t current_channels = 0;

void setupI2S(uint32_t sampleRate, int channels) {
  // If already initialized with same config, just clear DMA and return
  if (i2s_initialized && 
      sampleRate == current_sample_rate && 
      channels == current_channels) {
    i2s_zero_dma_buffer(I2S_PORT);
    return;
  }

  // If already initialized but config changed → uninstall old driver
  if (i2s_initialized) {
    i2s_driver_uninstall(I2S_PORT);
    i2s_initialized = false;
  }

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = sampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // always stereo output
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Install new driver
  if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) == ESP_OK) {
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
    i2s_initialized = true;
    current_sample_rate = sampleRate;
    current_channels = channels;
  } else {
    Serial.println("I2S install failed!");
  }
}


// --------- START FUNCTION PROTOTYPES -----------

void displayTextScreen(String text);
void handleHomeScreen(char key);
void displayEnterPasskeyScreen();
bool connectedToInternet();
void connectToWiFi();
void beepBuzzer();
void displaySplashScreen();
void displayTextScreen();
void displayHomeScreen();
void handlePasskeyScreen(); 
void openBoomBarrier();
void closeBoomBarrier();
void identificationFailed();
void handleNewVisit(JsonObject visitor);
Resident parseResidentLine(String line);
void loadNetworksFromSD();
bool startNextNetworkAttempt();
void handleWiFi();
bool isWiFiConnected();
void playAudio(const char filename);


void setup() {
  Serial.begin(115200); // Initialize Serial Monitor
  Wire.begin(21, 22);  // example: SDA=21, SCL=22
  RFIDSerial.begin(115200, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);  // Initialize RFID reader
  QRcodeSerial.begin(115200, SERIAL_8N1, QRCode_RX_PIN, QRCode_TX_PIN);  // Initialize QRCode scanner
  SPI.begin(); // Init SPI bus  

  while (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD Card initialization failed!"));
    delay(1000);
  }

  Serial.println(F("SD Card initialized."));

  // Initialize MCP23017 with default I2C address (0x20)
  if (!mcp.begin_I2C(0x20, &Wire)) {
    Serial.println("MCP23017 init failed!");
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {  // Address 0x3C or 0x3D for most 128x64 OLED displays
    Serial.println(F("SSD1306 OLED allocation failed"));
    for (;;);
  }

  mcp.pinMode(8, OUTPUT);

  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(OPEN_BOOM_BARRIER_RELAY_PIN, OUTPUT);

  digitalWrite(OPEN_BOOM_BARRIER_RELAY_PIN, LOW);

  display.setTextColor(WHITE);
  displayTextScreen("Booting..."); 

  loadNetworksFromSD();

  // Initial connection attempt
  if (!tryConnectToKnownNetworks()) {
    Serial.println("No known WiFi connected, starting portal...");
    wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
    wm.autoConnect("TRUPASS-DEVICE","Password@1");

    // Save new credentials to SD if WiFi connects
    if (WiFi.status() == WL_CONNECTED) {
      saveNewCredentialToSD(WiFi.SSID(), WiFi.psk());
    }
  } else {
    Serial.println("Connected to WiFi.");
  }
  
  // For QRCode
  while (QRcodeSerial.available()) {
    QRcodeSerial.read();  // Clear out the buffer
  }  

  File testFile = SD.open("/audio/access_granted.wav");
  if (!testFile) {
    Serial.println(F("Audio file not found or SD read error!"));
  } else {
    Serial.println(F("WAV file found. Size:"));
    Serial.println(testFile.size());
    testFile.close();
  }

  configureDeviceAndMQTTTopics(); // Configure this Identification device
  
  if (connectedToInternet()) {    
    Serial.println(F("Internet is available. Proceeding with API calls..."));
    
    configTime(0, 0, "pool.ntp.org", "time.nist.gov"); // Connect to Network time server. Needed for "trupass:access-gained" MQTT payload

    while (time(nullptr) < 100000) {
      delay(100);
      Serial.println("Waiting for time sync...");
    }
    
    Serial.println("Time synced");

    displayTextScreen("Updating Rec..."); 

    fetchEstateFeeDriveStatus(); // Fetch fee drive status and save to SD Card
    fetchAndSaveResidents(); // Fetch Residents and save to SD Card
    fetchAndSaveVisitors(); // Fetch Visitors and save to SD Card

    feeDriveIsActive = estateFeeDriveIsActive(); // Check and set if fee drive is active

    // Create audio directory if it doesn't exist
    if (!SD.exists("/audio")) {
      SD.mkdir("/audio");
    }

    // Download access granted audio file
    if (downloadAudioAndSaveToSD(fileURL, accessGranted)) {
      Serial.println(F("Access_granted audio file downloaded successfully!"));    
      
      // playAudio(accessGranted); // Play the downloaded file
        
    } else {
      Serial.println(F("Failed to download audio file."));
    }
  } else {
    Serial.println(F("No internet access!"));
  }    
  
  // Configure MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  Serial.println(F("MQTT configured and Callback set successfully"));

  displaySplashScreen(); // Display splash screen
  delay(2000);
  displayHomeScreen();

  playAudio(deviceIsReadyAudio);

  beepBuzzer();
  
  // delay(100);
  Serial.println(F("Setup complete."));
}

void loop() {
  handleWiFi();

  char key = keypad.getKey();  // Read keypad input 

  if(key){
    switch(currentScreen) {
      case HOME:
        handleHomeScreen(key);
        break;
      case PASSKEY:
        handlePasskeyScreen(key);
        break;
    }
  }

  handleEnterKeyPress();

  if(boomBarrierIsOpen && (millis() - lastIdentificationTimeMillis >= 7000)){
    closeBoomBarrier();
    Serial.println(F("Closing barrier..."));
  }

  ScanQRCode();
  ReadUHFRFID();

  // For MQTT
  if (!mqttClient.connected()) {
    connectMqtt();
  }
  mqttClient.loop();
}

void validateResidentAccess(String accessCode = "", String epc = "");

void handleResidentID(String value, Resident& resident) { 
  value.trim(); 
  resident.id = value; 
}

void handleResidentEPC(String value, Resident& resident) { 
  value.trim(); 
  resident.epc = value; 
}

void handleResidentAccessCode(String value, Resident& resident) { 
  value.trim(); 
  resident.accessCode = value; 
}

void handleResidentPlots(String value, Resident& resident) {
  value.trim();

  // Ensure it starts and ends with brackets
  if (value.startsWith("[") && value.endsWith("]")) {
    value.remove(0, 1); // Remove '['
    value.remove(value.length() - 1); // Remove ']'
  }

  value.replace(" ", ""); // Remove all spaces

  // Clear existing entries
  resident.plots.clear();

  if (value.length() == 0) return; // No plots to add

  int plotStartIndex = 0;
  int plotEndIndex = value.indexOf(',');

  while (plotEndIndex != -1) {
    String plotId = value.substring(plotStartIndex, plotEndIndex);
    if (plotId.length()) {
      plotId.trim();
      resident.plots.push_back(plotId);
    }
    plotStartIndex = plotEndIndex + 1;
    plotEndIndex = value.indexOf(',', plotStartIndex);
  }

  // Last item
  if (plotStartIndex < value.length()) {
    String plotId = value.substring(plotStartIndex);
    if (plotId.length()) {
      plotId.trim();
      resident.plots.push_back(plotId);
    }
  }
}

ResidentKeyHandler residentHandlers[] = {
  { "ID:", handleResidentID },
  { "EPC:", handleResidentEPC },
  { "AccessCode:", handleResidentAccessCode },
  { "Plots:", handleResidentPlots }
};

// Handler functions for visitor validation
void handleID(String value, Visitor& visitor) { 
  value.trim(); 
  visitor.id = value; 
}
void handleAccessCode(String value, Visitor& visitor) { 
  value.trim(); 
  visitor.accessCode = value; 
}

void handlePlot(String value, Visitor& visitor) { 
  value.trim(); 
  visitor.plot = value; 
}

void handleDenyExit(String value, Visitor& visitor) { 
  value.trim();
  visitor.denyExit = value; 
}

// Visitor validation Lookup table
const VisitorKeyHandler visitorKeyHandlers[] = {
  {"ID:", handleID},
  {"AccessCode:", handleAccessCode},
  {"Plot:", handlePlot},
  {"DenyExit:", handleDenyExit}
};

// Helper functions

void ScanQRCode() {
  if (QRcodeSerial.available()) {
    while (QRcodeSerial.available()) {
      int incomingByte = QRcodeSerial.read();
      
      // Accumulate only printable characters, excluding newline or carriage return
      if (isPrintable(incomingByte) && incomingByte != '\r' && incomingByte != '\n') {
        scannedQRCodeData += (char)incomingByte;
      } else if (incomingByte == '\r' || incomingByte == '\n') { // When end of the line is reached (newline or carriage return)  

        if(millis() - previousQRCodeScanMillis < identificationDelayTime) { // Do not process qrcode data if less than 3secs after previous scan
          scannedQRCodeData = ""; // Clear already saved qrcode record
          return;
        }

        String trimmedQRCodeData = trimSpaces(scannedQRCodeData);
        trimmedQRCodeData.toLowerCase();
        String qrCodeValue = trimmedQRCodeData.substring(3);

        int residentIndex = trimmedQRCodeData.indexOf("tr:"); // Check if the scanned data contains "tr" prefix for resident scan"
        int visitorIndex = trimmedQRCodeData.indexOf("tv:"); // Check if the scanned data contains "tv" prefix for visitor scan"

        if(residentIndex != -1) {
          Serial.println("Access Code is : " + qrCodeValue);
          beepBuzzer();
          validateResidentAccess(qrCodeValue, "");

        } else if (visitorIndex != -1) {
          Serial.println("Access Code is : " + qrCodeValue);
          beepBuzzer();
          validateVisitorAccess(qrCodeValue);
        }
        
        // Clear the buffer for the next QR code scan
        scannedQRCodeData = "";
        previousQRCodeScanMillis = millis();
      }
    }
  }
}

// Function to send command to the RFID reader
void sendRFIDCommand(const uint8_t* command, size_t length) {
  RFIDSerial.write(command, length);
}

void ReadUHFRFID(){
  sendRFIDCommand(singlePollingCommand, sizeof(singlePollingCommand));  // Send the single polling command
  delay(50); // Wait a moment for the reader to process the command    
  readRFIDResponse(); // Read and process the response
}

// Function to read the response from the RFID reader
void readRFIDResponse() {
  if (RFIDSerial.available()) {
    // Read the response frame
    uint8_t response[64];
    size_t index = 0;

    while (RFIDSerial.available() && index < sizeof(response)) {
        response[index++] = RFIDSerial.read();
    }

    // Process the response
    if (index > 0) {
      // Check the Type field
      if (response) { // Type 0x02 indicates a valid tag read   
        // The EPC starts after RSSI and PC (which are at indices 5 and 6 respectively)
        int epcLength = index - 11; // Length of EPC is (Total Length - 11)
        String epc = "";

        // Print EPC
        for (int i = 8; i < 8 + epcLength; i++) { // EPC starts at index 8
          epc += String(response[i], HEX);
          epc += " ";
        }
        
        if(epc.length() >= 20 && epc.length() < 40) {
          beepBuzzer();

          String trimmedEPC = trimSpaces(epc); // remove all spaces
          trimmedEPC.toLowerCase(); // Covert to lower case

          Serial.print(F("Valid tag detected. EPC: "));
          Serial.println(trimmedEPC);
          
          validateResidentAccess("", trimmedEPC); // Call a function to validate resident's identification
        }
      } else if (response[1] == 0x01) { // Type 0x01 indicates an error or no tag found
        Serial.println(F("Error: No tag response or invalid CRC."));
      } else {
        Serial.println(F("Unexpected response type."));
      }
    } else {
      Serial.println(F("No response received."));
    }
  }
}

// Function to trim spaces from a string
String trimSpaces(String input) {
  input.replace(" ", "");  // Remove all spaces
  input.trim();  // Remove leading and trailing whitespace
  return input;
}

void beepBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(300);
  digitalWrite(BUZZER_PIN, LOW);
}

// Fetch estate residents from api and save to SD Card    
void fetchAndSaveResidents() {
  int page = 1;          // Start with page 1
  int limit = 50;       // Fetch 50 records per request
  int totalDocs = 0;     // Total number of records
  int fetchedDocs = 0;   // Number of records fetched so far

  while (true) {
    http.begin(apolloEndpoint);
    http.setTimeout(15000);  // Set http timeout to 15 seconds
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + String(authToken));

    // Make POST request with pagination
    String query = fetchResidentsQuery(estateID, page, limit);
    int httpResponseCode = http.POST(query);

    if (httpResponseCode == HTTP_CODE_OK) {
      String response = http.getString();
      Serial.println("HTTP Response code: " + String(httpResponseCode));
      Serial.println("Response length: " + String(response.length()));

      // Parse JSON response
      DynamicJsonDocument doc(4096); // Adjust size based on chunk size
      DeserializationError error = deserializeJson(doc, response);

      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
      }

      // Extract data from JSON
      JsonObject data = doc["data"]["GetPaginatedEstateMembers"]["data"];
      JsonArray docs = data["docs"];
      totalDocs = data["totalDocs"]; // Total number of records

      if (!docs.isNull() && docs.size() > 0) {
        // Create directories if they don't exist

        if (!SD.exists("/residents")) {
          SD.mkdir("/residents");
        }

        if (!SD.exists("/residents/epc")) {
          SD.mkdir("/residents/epc");
        }

        if (!SD.exists("/residents/access_code")) {
          SD.mkdir("/residents/access_code");
        }

        // Iterate over the array and write each data to SD Card
        for (JsonObject resident : docs) {
          String id = resident["id"].as<String>();
          String epc = resident["epc"].isNull() ? "" : resident["epc"].as<String>();
          String accessCode = resident["accessCode"].isNull() ? "" : resident["accessCode"].as<String>();
           
          String plotIds = ""; // Current resident list of plots      

          JsonArray plots = resident["plots"];

          for (JsonObject plot : plots) {
            if (!plotIds.isEmpty()) {
              plotIds += ","; // Add a comma separator between plot IDs
            }
            plotIds += plot["plot"]["id"].as<String>();
          }

          String trimmedEPC = trimSpaces(epc);
          trimmedEPC.toLowerCase();

          String trimmedAccessCode = trimSpaces(accessCode);
          trimmedAccessCode.toLowerCase();

          // Write the record to the file
          String record = "ID: " + id + ", EPC: " + trimmedEPC + ", AccessCode: " + trimmedAccessCode + ", Plots: [" + plotIds + "]\n";
          
          // For saving resident record with EPC as file name
          if(!trimmedEPC.isEmpty()) {
            String eFilePath = "/residents/epc/" + trimmedEPC + ".txt";

            // Write file if it doesn't already exist
            if(!SD.exists(eFilePath)){
              // Open file on SD card for writing
              File epcFilePath = SD.open(eFilePath, FILE_WRITE); 

              if (!epcFilePath) {
                Serial.println(F("Failed to open EPC file for writing"));
                Serial.print(F("EPC : "));
                Serial.println(trimmedEPC);
                continue;
              }

              Serial.print(F("EPC : "));
              Serial.println(trimmedEPC);
              
              epcFilePath.print(record);          
              epcFilePath.close(); 
            }
          }   

          // For saving resident record with accessCode as file name
          if(!accessCode.isEmpty()) {
            String aFilePath = "/residents/access_code/" + trimmedAccessCode + ".txt";

            // Write file if it doesn't already exist
            if(!SD.exists(aFilePath)){
              // Open file on SD card
              File accessCodeFilePath = SD.open(aFilePath, FILE_WRITE); 

              if (!accessCodeFilePath) {
                Serial.println(F("Failed to open Access Code file for writing"));
                Serial.print(F("AccessCode : "));
                Serial.println(trimmedAccessCode);
                continue;
              }

              Serial.print(F("AccessCode : "));
              Serial.println(trimmedAccessCode);
              
              accessCodeFilePath.print(record);          
              accessCodeFilePath.close(); 
            }
          }         
        }

        fetchedDocs++;

        Serial.println("Page " + String(page) + " written to SD card.");
      } else {
        Serial.println(F("No more data found."));
        break;
      }

      // Check if all records have been fetched
      if (fetchedDocs >= totalDocs) {
        Serial.println(F("All Residents records fetched and written to SD card."));
        break;
      }

      // Move to the next page
      page++;
    } else {
      Serial.print(F("Error on HTTP request: "));
      Serial.println(http.errorToString(httpResponseCode));
      break;
    }

    http.end();
  }
}

void fetchAndSaveVisitors() {
  // Create directories if they don't exist

  if (!SD.exists("/visitation")) {
    SD.mkdir("/visitation");
  }

  if (!SD.exists("/visitation/access_code")) {
    SD.mkdir("/visitation/access_code");
  }   

  http.begin(apolloEndpoint);
  http.setTimeout(15000);  // Set http timeout to 15 seconds
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(authToken));

  // Define the GraphQL query schema
  String schema = R"(
    query GetDailyVisits($estate: ID) {
      GetDailyVisits(estate: $estate) {
        data {
          id
          accessCode
          plot {
            id
          }
          option {
            denyExit {
              status
            }
          }
        }
      }
    }
  )";

  // Escape newlines and quotes in the schema
  schema.replace("\n", "\\n");
  schema.replace("\"", "\\\"");

  // Construct the JSON payload
  String query = "{\"query\":\"" + schema + "\",\"variables\":{\"estate\":\"" + estateID + "\"}}";

  // Send the GraphQL query
  int httpResponseCode = http.POST(query);

  if (httpResponseCode == HTTP_CODE_OK) {
    String response = http.getString();
    Serial.println("API Response: " + response);

    // Parse the JSON response
    DynamicJsonDocument doc(6144); // Increase size if necessary
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
      Serial.print(F("JSON parsing failed: "));
      Serial.println(error.c_str());
      return;
    } 

    // Extract visitors data from JSON
    JsonArray visitors = doc["data"]["GetDailyVisits"]["data"];

    if (!visitors.isNull() && visitors.size() > 0) {
      // Create JsonObject to modify incoming JsonArray 
      StaticJsonDocument<256> newVisitorDoc;
      JsonObject modifiedDoc = newVisitorDoc.to<JsonObject>();

      // Iterate over the array and write each data to SD Card
      for (JsonObject visitor : visitors) {
        modifiedDoc["id"] = visitor["id"];
        modifiedDoc["accessCode"] = visitor["accessCode"];
        modifiedDoc["denyExit"] = visitor["option"]["denyExit"]["status"];
        modifiedDoc["plotId"] = visitor["plot"]["id"];

        handleNewVisit(modifiedDoc);        
      }

      Serial.println(F("Successfully written all visitor files!"));
        
    }
    
  } else {
    Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

void fetchEstateFeeDriveStatus() {
  http.begin(apolloEndpoint);
  http.setTimeout(15000);  // Set http timeout to 15 seconds
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(authToken));

  // Define the GraphQL query schema
  String schema = R"(
    query CheckEstateFeeDriveStatus($estate: ID) {
      CheckEstateFeeDriveStatus(estate: $estate) {
        status
      }
    }
  )";

  // Escape newlines and quotes in the schema
  schema.replace("\n", "\\n");
  schema.replace("\"", "\\\"");

  // Construct the JSON payload
  String query = "{\"query\":\"" + schema + "\",\"variables\":{\"estate\":\"" + estateID + "\"}}";

  // Send the GraphQL query
  int httpResponseCode = http.POST(query);

  if (httpResponseCode == HTTP_CODE_OK) {
    String response = http.getString();
    Serial.println("API Response: " + response);

    // Parse the JSON response
    DynamicJsonDocument doc(1024); // Increase size if necessary
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
      Serial.print(F("JSON parsing failed: "));
      Serial.println(error.c_str());
      return;
    }
    
    // Extract the required data
    bool status = doc["data"]["CheckEstateFeeDriveStatus"]["status"];

    feeDriveIsActive = status;

    // Create directory if it doesn't exist
    if (!SD.exists("/fee")) {
      SD.mkdir("/fee");
    }

    String fFilePath = "/fee/fee_drive.txt";
    String owingPlotPath = "/fee/owing_plots.txt";

    if (status){
      GetAllOwingPlotIds();
    } else {
      if(SD.exists(owingPlotPath)){
        SD.remove(owingPlotPath);
      }
    }

    // Write the data to the SD card
    File file = SD.open(fFilePath, FILE_WRITE);

    String record = String("Status: ") + (status ? "true" : "false") + "\n";

    if (file) {
      file.seek(0);
      file.print(record);

      Serial.println(F("Fee drive status written to SD card."));

      file.close();
    } else {
      Serial.println(F("Failed to open Fee drive status file for writing."));
    }

  } else {
    Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

bool estateFeeDriveIsActive() {
  // Open the file on SD card
  File file = SD.open("/fee/fee_drive.txt");

  if (!file) {
    Serial.println(F("Failed to open FeeDriveActive file for reading"));
    return false;
  }

  // Variable to store parsed data
  bool status = false;

  // Read the file line by line
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim(); // Remove leading/trailing whitespace

    // Parse the "status" line
    if (line.startsWith("Status:")) {
      String statusStr = line.substring(7); // Extract the value after "status:"
      statusStr.trim(); // Remove any extra spaces
      status = (statusStr == "true"); // Convert to boolean
    }
  }

  // Close the file
  file.close();

  // Check if status is true
  if (status) {
    Serial.println(F("Fee Drive status is true."));
    return true;
  } else {
    Serial.println(F("Fee Drive status is false."));
    return false;
  }
}

void GetAllOwingPlotIds() {
  while (true) {
    bool success = false;
    int attempts = 0;

    while (attempts < 3 && !success) {
      http.begin(apolloEndpoint);
      http.setTimeout(15000);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("Authorization", "Bearer " + String(authToken));

      String schema = R"(
        query GetAllOwingPlotIds($estate: ID) {
          GetAllOwingPlotIds(estate: $estate)
        }
      )";
        
      // Escape newlines and quotes in the schema
      schema.replace("\n", "\\n");
      schema.replace("\"", "\\\"");

      // Construct the JSON payload
      String query = "{\"query\":\"" + schema + "\",\"variables\":{\"estate\":\"" + estateID + "\"}}";

      // Send the GraphQL query
      int httpResponseCode = http.POST(query);

      if (httpResponseCode == HTTP_CODE_OK) {
        String response = http.getString();
        Serial.println("HTTP Response code: " + String(httpResponseCode));
        Serial.println("API Response: " + response);

        DynamicJsonDocument doc(8192);
        DeserializationError error = deserializeJson(doc, response);

        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
          http.end();
          return;
        }

        JsonArray data = doc["data"]["GetAllOwingPlotIds"];

        if (!data.isNull() && data.size() > 0) {
          if (!SD.exists("/fee")) {
            SD.mkdir("/fee");
          }

          if (SD.exists("/fee/owing_plots.txt")) {
            SD.remove("/fee/owing_plots.txt");
            Serial.println("Deleted existing owing_plots.txt file.");
          }

          File file = SD.open("/fee/owing_plots.txt", FILE_APPEND);
          if (!file) {
            Serial.println(F("Failed to open Owing Plots file for writing"));
            http.end();
            return;
          }

          for (String plotId : data) {
            file.print("ID: " + plotId + "\n");
          }

          file.close();
          Serial.println(F("All Owing Plots records fetched and written to SD card."));
          return;
          
        } else {
          Serial.println("No more data found.");
          http.end();
          break;
        }

        success = true;  // Mark success to break retry loop
       
      } else {
        Serial.print(F("Error on HTTP request: "));
        Serial.println(httpResponseCode);
        Serial.println(http.errorToString(httpResponseCode));

        attempts++;

        if (attempts < 3) {
          Serial.println("Retrying in 1 second...");
          delay(1000);
        } else {
          Serial.println("Failed after 3 attempts. Giving up.");
        }
      }

      http.end(); // Always call end()
    }

    // If we exhausted all attempts and still failed, stop further paging
    if (!success) {
      break;
    }
  }
}

// GetResidents apollo graphql Query string
String fetchResidentsQuery(String estateID, int page, int limit) {
  return "{"
         "\"query\": \"query {\\n GetPaginatedEstateMembers(estate: \\\"" + estateID + "\\\", page: " + String(page) + ", limit: " + String(limit) + ") { \\n data {\\n docs {\\n id\\n epc\\n accessCode\\n plots {\\n plot {\\n id }\\n }\\n }\\n totalDocs\\n }\\n }\\n}\","
         "\"variables\": {}"
         "}";
}

// Function to check if supplied resident plot is owing
bool residentPlotIsOwing(String plotId){
  Serial.println("Plot ID : ");
  Serial.println(plotId);
  // Open the file on SD card
  File owingPlotFiles = SD.open("/fee/owing_plots.txt");

  if (!owingPlotFiles) {
    Serial.println(F("Failed to open Owing Plots file for reading"));
    return false;
  }
  
  while (owingPlotFiles.available()) {
    String owingPlotId = owingPlotFiles.readStringUntil('\n');
    if (!owingPlotId.isEmpty()) {
      int idStartIndex = owingPlotId.indexOf("ID: ");

      String idValue = owingPlotId.substring(4); 

      if(idValue == plotId) {
        return true;
      }
    }
  }
  
  // Close the file
  owingPlotFiles.close();
  Serial.println(F("Finished reading Owing Plots records."));

  return false;
}


void configureDeviceAndMQTTTopics () {
  File configFile = SD.open("/config/estate.txt");
  
  if(!configFile){
    Serial.println(F("Estate ID does not exist!"));
    return;
  }
  
  String estateIDValue = "";
  String deviceNameValue = "";
  String deviceTypeValue = "";

  while(configFile.available()){
    String configObject = configFile.readStringUntil('\n');
    configObject.trim(); // Remove extra whitespace
    if (configObject.startsWith("estateID:")) {
      estateIDValue = configObject.substring(configObject.indexOf(':') + 1);
      estateIDValue.trim(); // Remove any extra spaces
      estateIDValue.toLowerCase(); // Convert to lowercase
    } else if (configObject.startsWith("deviceName:")){
      deviceNameValue = configObject.substring(configObject.indexOf(':') + 1);
      deviceNameValue.trim(); // Remove any extra spaces
      deviceNameValue.toLowerCase(); // Convert to lowercase
    } else if (configObject.startsWith("accessType:")){
      deviceTypeValue = configObject.substring(configObject.indexOf(':') + 1);
      deviceTypeValue.trim(); // Remove any extra spaces
      deviceTypeValue.toLowerCase(); // Convert to lowercase
    }
  }

  configFile.close();

  // Set global variables

  estateID = estateIDValue; 
  deviceName = deviceNameValue; 
  deviceType = deviceTypeValue; 

  //........... SET MQTT TOPICS .................

  accessGained = "trupass:access-gained";
  newResidentAdded = "trupass:" + estateID + ":resident-added";
  residentRemoved = "trupass:" + estateID + ":plot-member-removed";
  newVisit = "trupass:" + estateID + ":new-visit";
  visitUpdated = "trupass:" + estateID + ":visit-updated";
  visitCancelled = "trupass:" + estateID + ":visit-cancelled";
  visitEnded = "trupass:" + estateID + ":visit-ended";
  feeDriveUpdated = "trupass:" + estateID + ":fee-drive-updated";
  feePaymentUpdated = "trupass:" + estateID + ":fee-payment-updated";
}

void validateResidentAccess(String accessCode, String epc) {
  if(epc.isEmpty() && accessCode.isEmpty()) {
    Serial.print(F("EPC / accessCode wasn't correctly provided!"));
    delay(identificationDelayTime);
    return;    
  }

  File residentFile;

  if(!epc.isEmpty()) {
    residentFile = SD.open("/residents/epc/" + epc + ".txt", FILE_READ);
    Serial.println(F("EPC Supplied for validation!"));
  } else if (!accessCode.isEmpty()) {
    residentFile = SD.open("/residents/access_code/" + accessCode + ".txt", FILE_READ);
    Serial.println(F("Access Code Supplied for validation!"));
  }

  if (!residentFile) {
    Serial.println(F("Failed to open Resident's file for reading!"));

    // No resident record was found. Deny access
    identificationFailed();
    return;
  } 

  Serial.println(F("Yeahhh!!! Found Resident record!!."));

  String resident = residentFile.readStringUntil('\n'); // Read resident string
  Resident residentObj = parseResidentLine(resident); // Convert the resident string into an object

  // Close the file
  residentFile.close();
  Serial.println(F("Finished reading records."));

  // Check if fee drive is active and if resident's plot is owing
  if (feeDriveIsActive) {
    Serial.println("Fee Drive is active, checking plots!");
    for (String plotId : residentObj.plots) { 
      Serial.print("Plot Id is : ");
      Serial.println(plotId);
      if(residentPlotIsOwing(plotId)) { 
        // Play "Access denied, plot is owing" audio
        playAudio(accessDeniedFee);

        Serial.println(F("Access Denied, Plot is Owing!!!"));

        delay(identificationDelayTime);
        return;
      }
    }
  }  

  publishAccessGained(residentObj.id, true); // publish MQTT new-access topic

  //Identfication is successful, allow access
  identificationSuccessful();
}

Resident parseResidentLine(String line) {
  Resident resident;

  int startIndex = 0;
  int bracketDepth = 0;

  for (int i = 0; i < line.length(); i++) {
    char c = line.charAt(i);

    if (c == '[') bracketDepth++;
    else if (c == ']') bracketDepth--;
    else if (c == ',' && bracketDepth == 0) {
      String pair = line.substring(startIndex, i);
      extractResidentKeyValue(pair, resident);
      startIndex = i + 1;
    }
  }

  // Handle last pair
  if (startIndex < line.length()) {
    String pair = line.substring(startIndex);
    extractResidentKeyValue(pair, resident);
  }

  return resident;
}

const int residentHandlerCount = sizeof(residentHandlers) / sizeof(residentHandlers[0]);

// Extract resident key-value pairs using lookup table
void extractResidentKeyValue(String pair, Resident& resident) {
  pair.trim(); // Remove leading/trailing whitespace

  for (int i = 0; i < residentHandlerCount; i++) {
    if (pair.startsWith(residentHandlers[i].key)) {
      String value = pair.substring(strlen(residentHandlers[i].key));
      residentHandlers[i].handler(value, resident);
      break; // Stop after finding the first match
    }
  }
}

void validateVisitorAccess(String accessCode) {
  if(accessCode.isEmpty()) {
    Serial.print(F("Visitor's AccessCode wasn't provided!"));
    return;    
  }

  File visitorFile = SD.open("/visitation/access_code/" + accessCode + ".txt", FILE_READ);

  if (!visitorFile) {
    Serial.println(F("Failed to open Visitor's file for reading!"));

    // No visitor record was fount. Deny Access
    identificationFailed();

    return;
  } 

  Serial.println(F("Yeahhh!!! Found Visitor's record!!."));

  String visitor = visitorFile.readStringUntil('\n'); // Read visitor string
  Visitor visitorObj = parseVisitorLine(visitor); // Convert the visitor string into an object

  // Close the file
  visitorFile.close();

  Serial.println(F("Finished reading records."));

  if (feeDriveIsActive) {
     // At this point, fee drive is active, check if Plot which invited visitor is owing

    if(residentPlotIsOwing(visitorObj.plot)) { 
      Serial.println(F("Access Denied, Plot is Owing!!!"));

      // TODO: Play "Access denied plot is owing" audio
      playAudio(accessDeniedFee);
      delay(identificationDelayTime);

      return;
    }
  }

  //Remove visitor file from SD Card if Identification Device type is "Entrance"
  SD.remove("/visitation/access_code/" + accessCode + ".txt");

  if(visitorObj.denyExit == "true"){
    Serial.println("Deny exit is true");
    identificationFailed();
    return;
  }
  
  publishAccessGained(visitorObj.id, false); // publish MQTT new-access topic

  // identification is successful
  identificationSuccessful();
}

void validateAccess(String accessCode) {
  if(accessCode.isEmpty()) {
    passkey = "";

    displayHomeScreen();

    Serial.print(F("AccessCode wasn't correctly provided!"));
    delay(identificationDelayTime);
    return;    
  }

  File record;

  record = SD.open("/residents/access_code/" + accessCode + ".txt", FILE_READ);
  Serial.println(F("Access Code Supplied for validation!"));

  if (record) { // record here is resident's record
    validateResidentAccess(accessCode, "");
  } else {
    validateVisitorAccess(accessCode);
  }

}

void identificationSuccessful(){
  playAudio(accessGranted);

  Serial.println(F("Access Granted!"));     
  
  openBoomBarrier();

  delay(identificationDelayTime);
}

void identificationFailed(){
  playAudio(accessDenied);
  delay(identificationDelayTime);
}

Visitor parseVisitorLine(String line) {
  Visitor visitor;

  // Split the line by commas
  int startIndex = 0;
  int endIndex = line.indexOf(',');

  while (endIndex != -1) {
    String pair = line.substring(startIndex, endIndex);
    extractVisitorKeyValue(pair, visitor); // Process the key-value pair

    // Move to the next pair
    startIndex = endIndex + 1;
    endIndex = line.indexOf(',', startIndex);
  }

  // Handle the last pair (if any)
  if (startIndex < line.length()) {
    String pair = line.substring(startIndex);
    extractVisitorKeyValue(pair, visitor); // Process the last key-value pair
  }

  return visitor;
}

const int visitorKeyHandlerCount = sizeof(visitorKeyHandlers) / sizeof(visitorKeyHandlers[0]);

// Helper function to extract key-value pairs from a resident single line record
void extractVisitorKeyValue(String pair, Visitor& visitor) {
  pair.trim(); // Remove leading/trailing whitespace

  for (int i = 0; i < visitorKeyHandlerCount; i++) {
    if (pair.startsWith(visitorKeyHandlers[i].key)) {
      visitorKeyHandlers[i].handler(pair.substring(strlen(visitorKeyHandlers[i].key)), visitor);
      break;  // Exit loop once a match is found
    }
  }
}


// ------------ START OF WIFI CONFIGURATION AND CONNECTION -------------------

void loadNetworksFromSD() {
  File file = SD.open(networksFile);
  if (!file) {
    Serial.println("No networks.json file found.");
    return;
  }

  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, file);

  file.close();

  if (err) {
    Serial.println("Failed to parse networks.json");
    return;
  }

  for (JsonObject obj : doc.as<JsonArray>()) {
    NetworkCredential net;
    net.ssid = obj["ssid"].as<String>();
    net.password = obj["password"].as<String>();
    networkList.push_back(net);
  }

  Serial.print("Loaded ");
  Serial.print(networkList.size());
  Serial.println(" networks from SD.");
}

bool tryConnectToKnownNetworks() {
  for (auto &net : networkList) {
    WiFi.begin(net.ssid.c_str(), net.password.c_str());
    Serial.print("Connecting to: ");
    Serial.println(net.ssid);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < WIFI_CONNECT_TIMEOUT) {
      delay(100);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected to ");
      Serial.println(net.ssid);
      return true;
    }
  }
  return false;
}

bool startNextNetworkAttempt() {
  if (networkList.empty()) return false;

  NetworkCredential &net = networkList[currentNetworkIndex];
  Serial.print("Trying: ");
  Serial.println(net.ssid);

  WiFi.disconnect();
  WiFi.begin(net.ssid.c_str(), net.password.c_str());
  networkAttemptStart = millis();

  currentNetworkIndex++;
  if (currentNetworkIndex >= networkList.size()) {
    currentNetworkIndex = 0;
  }
  return true;
}

void handleWiFi() {
  if (portalActive) {
    wm.process(); // Keep portal responsive
    return;
  }

  if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();

    if (!isWiFiConnected()) {
      if (wifiLostStart == 0) {
        wifiLostStart = millis();
        tryingNetworks = true;
        currentNetworkIndex = 0;
        startNextNetworkAttempt();
      }

      if (tryingNetworks) {
        // Wait for current attempt to finish
        if (millis() - networkAttemptStart >= WIFI_CONNECT_TIMEOUT) {
          startNextNetworkAttempt();
        }
      }

      // If taking too long, start portal
      if ((millis() - wifiLostStart) > WIFI_CYCLE_TIMEOUT) {
        Serial.println("Failed to connect to known networks. Starting portal.");
        wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);
        wm.startConfigPortal("TRUPASS_DEVICE");
        delay(100);

        // After portal mode exits, check if WiFi connected and save credentials
        if (WiFi.status() == WL_CONNECTED) {
          saveNewCredentialToSD(WiFi.SSID(), WiFi.psk());
          Serial.println("Saved new WiFi credential to SD Card.");
        }

        portalActive = true;
        tryingNetworks = false;
        wifiLostStart = 0;
      }
    } else {
      if (tryingNetworks) {
        Serial.println("WiFi reconnected.");
        tryingNetworks = false;
        wifiLostStart = 0;
      }
    }
  }
}

bool isWiFiConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

void saveNewCredentialToSD(const String &newSSID, const String &newPass) {
  // Load existing JSON
  DynamicJsonDocument doc(2048);
  File file = SD.open("/networks.json", FILE_READ);
  if (file) {
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) {
      Serial.println("Error reading existing networks.json, starting fresh.");
      doc.clear();
      doc.to<JsonArray>(); // Create empty array
    }
  } else {
    Serial.println("networks.json not found, creating new one.");
    doc.to<JsonArray>(); // Create new JSON array
  }
  
  JsonArray arr = doc.as<JsonArray>(); // Ensure doc is an array

  // Check if SSID already exists
  bool exists = false;
  for (JsonObject obj : arr) {
    if (obj["ssid"].as<String>() == newSSID) {
      exists = true;
      break;
    }
  }

  // Append new credential if not already in the list
  if (!exists) {
    JsonObject newNet = arr.createNestedObject();
    newNet["ssid"] = newSSID;
    newNet["password"] = newPass;
    Serial.print(F("Added new network: "));
    Serial.println(newSSID);

    // Save back to SD
    File saveFile = SD.open(networksFile, FILE_WRITE);
    if (saveFile) {
      serializeJsonPretty(doc, saveFile);
      saveFile.close();
      Serial.println(F("Updated networks.json saved to SD."));
    } else {
      Serial.println(F("Error opening networks.json for writing."));
    }
  } else {
    Serial.println(F("Network already exists in networks.json."));
  }
}

// ------------ END OF WIFI CONFIGURATION AND CONNECTION ------------------

// Check if internet access is available
bool connectedToInternet() {
  WiFiClient client;
  return client.connect("8.8.8.8", 53);  // Google's DNS server
}

void openBoomBarrier(){
  digitalWrite(OPEN_BOOM_BARRIER_RELAY_PIN, HIGH);  
  delay(300);

  lastIdentificationTimeMillis = millis();
  boomBarrierIsOpen = true;
}

void closeBoomBarrier(){  
  digitalWrite(OPEN_BOOM_BARRIER_RELAY_PIN, LOW);  
  delay(300);

  boomBarrierIsOpen = false;
}

// Function to download access audio file from GitHub and save to SD Card
bool downloadAudioAndSaveToSD(const char* url, const char* path) {
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    // client->setCACert(rootCACertificate);
    client->setInsecure(); // Remove setCACert() and use this instead
    HTTPClient http;
    http.begin(*client, url);

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("HTTP request failed: %d\n", httpCode);
      http.end();
      delete client;
      return false;
    }

    File file = SD.open(path, FILE_WRITE);
    if (file) {
      http.writeToStream(&file);
      file.close();
      http.end();
      delete client;
      Serial.println(F("Audio Download complete."));
      return true;
    } else {
      Serial.println(F("Failed to open audio file for writing."));
    }
    delete client;
  }
  return false;
}

// Function to play audio from SD card

// ===== Play WAV =====
void playAudio(const char *filename) {
  File file = SD.open(filename);
  if (!file) {
    Serial.printf("Failed to open %s\n", filename);
    return;
  }

  // Read header
  WavHeader header;
  file.read((uint8_t *)&header, sizeof(WavHeader));

  // Debug info
  Serial.printf("Sample Rate: %lu Hz\n", header.sample_rate);
  Serial.printf("Channels: %u\n", header.channels);
  Serial.printf("Bits: %u\n", header.bits_per_sample);
  Serial.printf("Data size: %lu bytes\n", header.data_size);

  // Setup I2S
  setupI2S(header.sample_rate, header.channels);
  i2s_zero_dma_buffer(I2S_PORT);

  // Buffers
  const size_t bufferSize = 512;
  uint8_t buffer[bufferSize];
  size_t bytesRead = 0, bytesWritten = 0;

  Serial.println("Starting playback...");
  while (file.available()) {
    bytesRead = file.read(buffer, bufferSize);

    if (bytesRead > 0) {
      if (header.channels == 1) {
        // ===== Duplicate mono samples into stereo =====
        int16_t *samplesIn = (int16_t *)buffer;
        size_t sampleCount = bytesRead / 2;

        static int16_t stereoBuffer[bufferSize]; // 512 int16_t = 1024 bytes
        size_t stereoIndex = 0;

        for (size_t i = 0; i < sampleCount; i++) {
          int16_t s = samplesIn[i];
          stereoBuffer[stereoIndex++] = s;  // Left
          stereoBuffer[stereoIndex++] = s;  // Right
        }

        esp_err_t err = i2s_write(I2S_PORT, stereoBuffer, stereoIndex * sizeof(int16_t), &bytesWritten, 100 / portTICK_PERIOD_MS);
        if (err != ESP_OK) {
          Serial.println("i2s_write timeout or error");
          break; // exit playback loop to avoid infinite hang
        }

      } else {
        // Stereo file, write directly
        i2s_write(I2S_PORT, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
      }
    }
  }

  file.close();
  Serial.println("Playback finished.");
}


// MQTT CONFIGURATIONS

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  Serial.print("MQQT Message arrived");

  StaticJsonDocument<256> doc;
  String payloadStr;

  for (unsigned int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  DeserializationError error = deserializeJson(doc, payloadStr);

  if (error) {
    Serial.print(F("JSON deserialization failed: "));
    Serial.println(error.c_str());
    return;
  }

  JsonObject payloadJsonObj = doc.as<JsonObject>();

  if(strcmp(topic, newResidentAdded.c_str()) == 0){
    handleNewResidentAdded(payloadJsonObj);
  }
  else if(strcmp(topic, residentRemoved.c_str()) == 0){
    handleResidentRemoved(payloadJsonObj);
  }
  else if(strcmp(topic, newVisit.c_str()) == 0){
    handleNewVisit(payloadJsonObj);
  }
  else if(strcmp(topic, visitUpdated.c_str()) == 0){
    handleVisitUpdated(payloadJsonObj);
  }
  else if(strcmp(topic, visitCancelled.c_str()) == 0){
    handleVisitCancelledOrEnded(payloadJsonObj);
  }
  else if(strcmp(topic, visitEnded.c_str()) == 0){
    handleVisitCancelledOrEnded(payloadJsonObj);
  }
  else if(strcmp(topic, feeDriveUpdated.c_str()) == 0){
    handleFeeDriveUpdated(payloadJsonObj);
  }
  else if(strcmp(topic, feePaymentUpdated.c_str()) == 0){
    handleFeePaymentUpdated(payloadJsonObj);
  }
}

void connectMqtt() {
  Serial.print("Attempting MQTT connection...");

  String clientId = "trupass-device:mqttclient";

  // Connect using the Ably key as username, no password
  if (mqttClient.connect(clientId.c_str(), ably_key, "")) {
    Serial.println("Mqtt connected");

    // Subscribe to topic for receiving new visitors
  
    if(deviceType == "entrance"){
      mqttClient.subscribe(newVisit.c_str());
      mqttClient.subscribe(visitCancelled.c_str());
    } else if (deviceType == "exit") {
      mqttClient.subscribe(visitUpdated.c_str());
      mqttClient.subscribe(visitEnded.c_str());
    } else if (deviceType == "both") {
      mqttClient.subscribe(newVisit.c_str());
      mqttClient.subscribe(visitCancelled.c_str());
      mqttClient.subscribe(visitUpdated.c_str());
      mqttClient.subscribe(visitEnded.c_str());
    }

    mqttClient.subscribe(newResidentAdded.c_str());
    mqttClient.subscribe(residentRemoved.c_str());  
    mqttClient.subscribe(feeDriveUpdated.c_str());
    mqttClient.subscribe(feePaymentUpdated.c_str());

  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" trying again...");
  }
}


// MQTT AccessGained publish function

void publishAccessGained(const String& id, bool isResident) {

  Serial.println("Called Publish Access gained function");

  if(!mqttClient.connected()){
    Serial.println("MQTT not connected, cannot publish!");
    return;
  } 

  time_t now = time(nullptr);  // Unix timestamp in seconds
  if (now < 100000) {
    Serial.println("Time not synced yet.");
    return;
  }

  long long jsTimestamp = (long long)now * 1000;  // Convert to JavaScript-style timestamp

  StaticJsonDocument<256> doc;

  doc["id"] = id; // visitor or resident Id
  doc["t"] = jsTimestamp;
  doc["dn"] = deviceName;          
  doc["dt"] = deviceType;                      
  doc["ac"] = deviceType;                   
  doc["eId"] = estateID;                     
  doc["isR"] = isResident;

  char payload[256];
  size_t len = serializeJson(doc, payload);
  Serial.println("Payload size: " + String(len));

  if (mqttClient.publish(accessGained.c_str(), payload, len)) {
    Serial.println("New Access Gained event published");
  } else {
    Serial.println("Failed to publish access event");
  }
}


// ---------------- Start MQTT handlers --------------------

void handleNewResidentAdded(JsonObject resident) {
  serializeJson(resident, Serial);  // prints compact JSON
  Serial.println(); 

  const String id = resident["id"].as<String>();
  const String accessCode = resident["ac"].as<String>();
  const JsonArray plots = resident["plots"];

  String trimmedAccessCode = trimSpaces(accessCode);

  String aFilePath = "/residents/access_code/" + trimmedAccessCode + ".txt";
  String eFilePath = "";

  // Extract EPC if available
  String epc = "";
  if (!resident["epc"].isNull()) {
    epc = resident["epc"].as<String>();
    eFilePath = "/residents/epc/" + epc + ".txt";
  }

  // Construct plot IDs string
  String plotIds = "";
  for (JsonVariant plot : plots) {
    if (!plotIds.isEmpty()) {
      plotIds += ",";
    }
    plotIds += plot.as<String>();
  }


  if (SD.exists(aFilePath)) {
    SD.remove(aFilePath);
  }
  if (SD.exists(eFilePath)) {
    SD.remove(eFilePath);
  }

  // Write resident file by AccessCode
  File accessCodeFile = SD.open(aFilePath, FILE_WRITE);

  if (!accessCodeFile) {
    Serial.println(F("Failed to open resident file for write!"));
    return;
  }

  String record = "ID: " + id + ", EPC: " + epc + ", AccessCode: " + trimmedAccessCode + ", Plots: [" + plotIds + "]";

  accessCodeFile.println(record);  // or accessCodeFile.print(record + "\n");
  accessCodeFile.close();
  
  Serial.println("Successfully written resident with AccessCode: " + trimmedAccessCode + " to SD Card!");

  if(epc == ""){
    Serial.println(F("Resident has no EPC!"));
    return;
  }

  // Write resident file by EPC
  File epcFile = SD.open(eFilePath, FILE_WRITE);

  if (!epcFile) {
    Serial.println(F("Failed to open resident EPC file for write!"));
    return;
  }

  epcFile.println(record);  // or epcFile.print(record + "\n");
  epcFile.close();

  Serial.println("Successfully written resident with EPC: " + epc + " to SD Card!");
}

void handleResidentRemoved(JsonObject resident){  
  const String accessCode = resident["ac"].as<String>();
  const String epc = resident["epc"].as<String>();

  String trimmedAccessCode = trimSpaces(accessCode);
  String trimmedEPC = trimSpaces(epc);
  
  String aFilePath = "/residents/access_code/" + trimmedAccessCode + ".txt";
  String eFilePath = "/residents/epc/" + trimmedEPC + ".txt";

  if(SD.exists(aFilePath)){
    SD.remove(aFilePath);
  }

  if(SD.exists(eFilePath)){
    SD.remove(eFilePath);
  }

  Serial.println(F("Removed resident record successfully!"));
}

void handleNewVisit(JsonObject visitor){

  // Create directories if they don't exist

  if (!SD.exists("/visitation")) {
    SD.mkdir("/visitation");
  }

  if (!SD.exists("/visitation/access_code")) {
    SD.mkdir("/visitation/access_code");
  }   

  // Extract the required data
  const String id = visitor["id"].as<String>();
  const String accessCode = visitor["accessCode"].as<String>();
  const String plotId = visitor["plotId"].as<String>(); 
  const String denyExit = visitor["denyExit"].as<String>(); 

  String trimmedAccessCode = trimSpaces(accessCode);
  trimmedAccessCode.toLowerCase();   

  String vFilePath = "/visitation/access_code/" + trimmedAccessCode + ".txt";

  // Write file if it doesn't already exist
  if (!SD.exists(vFilePath)) {
    File visitorFile = SD.open(vFilePath, FILE_WRITE);

    if (!visitorFile) {
      Serial.println(F("Failed to open visitor file for write !"));
      return;
    }

    // Construct the record to write to SD Card 
    String record = "ID: " + id + ", AccessCode: " + trimmedAccessCode + ", Plot: " + plotId + ", DenyExit: " + denyExit + " \n";

    visitorFile.print(record);

    Serial.println("Successfully written visitor with AccessCode : " + trimmedAccessCode + " to SD Card!");

    visitorFile.close();
  } else {
    Serial.println("Failed to write visitor with AccessCode : " + trimmedAccessCode + " to SD Card! Visitor already exits!");
  }
}

void handleVisitUpdated(JsonObject visitor){  
  // Create directories if they don't exist

  if (!SD.exists("/visitation")) {
    SD.mkdir("/visitation");
  }

  if (!SD.exists("/visitation/access_code")) {
    SD.mkdir("/visitation/access_code");
  }   

  // Extract the required data
  const String id = visitor["id"].as<String>();
  const String accessCode = visitor["accessCode"].as<String>();
  const String plotId = visitor["plotId"].as<String>(); 
  const String denyExit = visitor["denyExit"].as<String>(); 

  String trimmedAccessCode = trimSpaces(accessCode);
  trimmedAccessCode.toLowerCase();   

  String vFilePath = "/visitation/access_code/" + trimmedAccessCode + ".txt";

  // Write file if it doesn't already exist
  if (SD.exists(vFilePath)) {
    SD.remove(vFilePath);
  }

  File visitorFile = SD.open(vFilePath, FILE_WRITE);

  if (!visitorFile) {
    Serial.println(F("Failed to open visitor file for write !"));
    return;
  }

  // Construct the record to write to SD Card 
  String record = "ID: " + id + ", AccessCode: " + trimmedAccessCode + ", Plot: " + plotId + ", DenyExit: " + denyExit + " \n";

  visitorFile.print(record);

  Serial.println("Updated visitor with AccessCode : " + trimmedAccessCode );

  visitorFile.close();
}

void handleVisitCancelledOrEnded(JsonObject visitor){
  const String accessCode = visitor["accessCode"].as<String>();

  String trimmedAccessCode = trimSpaces(accessCode);
  trimmedAccessCode.toLowerCase();   

  String vFilePath = "/visitation/access_code/" + trimmedAccessCode + ".txt";


  // Delete record if it exists
  if (SD.exists(vFilePath)) {
    SD.remove(vFilePath);
    Serial.println("Successfully removed visitor with AccessCode : " + trimmedAccessCode );

  } 
}

void handleFeeDriveUpdated(JsonObject feeDrive){
  // Create directory if it doesn't exist
  if (!SD.exists("/fee")) {
    SD.mkdir("/fee");
  }

  const String status = feeDrive["status"].as<String>();
  String trimmedStatus = trimSpaces(status);
  trimmedStatus.toLowerCase();

  String filePath = "/fee/fee_drive.txt";
  String owingPlotsPath = "/fee/owing_plots.txt";

  File feeDriveFile = SD.open(filePath, FILE_WRITE);

  if (!feeDriveFile) {
    Serial.println(F("Failed to open fee_drive.txt for writing"));
    return;
  }

  String statusStr = "Status: " + trimmedStatus;
   
  feeDriveFile.seek(0); // Clear old content (overwrite)  

  if(trimmedStatus == "true" || trimmedStatus == "1"){
    feeDriveIsActive = true;
    feeDriveFile.println(statusStr);
    Serial.println("Fee drive active, fetching Owing Plots!");
    GetAllOwingPlotIds();
  } else {
    feeDriveIsActive = false;
    feeDriveFile.println(statusStr);
    if(SD.exists(owingPlotsPath)){
      SD.remove(owingPlotsPath);
    }
  }

  feeDriveFile.close();

  Serial.print(F("Updated Fee Drive status to: "));
  Serial.println(trimmedStatus);


  //if status is false, setFeeDrive in SD card and delete owing plot record in sd card
}

void handleFeePaymentUpdated(JsonObject payload){
  serializeJson(payload, Serial);  // prints compact JSON
  Serial.println(); 
}

// ---------------- End MQTT handlers --------------------


// ---------- START HANDLERS FUNCTIONS -----------

void handleHomeScreen(char key) {
  if(key == '*' || key == '#' ) return;

  beepBuzzer();
  currentScreen = PASSKEY;
  handlePasskeyScreen(key);
}

void handlePasskeyScreen(char key) {
  beepBuzzer();
  if (key >= '0' && key <= '9' && passkey.length() < 6) {
    passkey += key;  // Append the entered number to passkey
    displayEnterPasskeyScreen();  // Update display
  } else if (key == '*' && passkey.length() > 0) {
    // Clear passkey
    passkey = "";
    displayEnterPasskeyScreen();  // Update display
  } else if (key == '#' && passkey.length() > 0) {
    // Remove last character from passkey
    passkey.remove(passkey.length() - 1);
    displayEnterPasskeyScreen();
  } else if (key == '#' && passkey.length() == 0) {
    // Go back to Home screen
    displayHomeScreen();
  }
}

void handleEnterKeyPress(){
  if(passkey.length() != 6) {
    return;
  }

  beepBuzzer();
  
  bool reading = digitalRead(ENTER_KEY_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("Enter key pressed!");
      performAction();
    }
  }

  lastButtonState = reading;




   bool reading = digitalRead(ENTER_BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("Enter key pressed!");
      validateAccess(passkey); 
    }
  }

  lastButtonState = reading;
}

// ---------- END HANDLERS FUNCTIONS -----------


// ---------- START DISPLAY FUNCTIONS -----------

void displayTextScreen(String text){
  display.clearDisplay();
  display.setCursor((SCREEN_WIDTH-text.length()*6)/2, (SCREEN_HEIGHT)/2-4);
  display.setTextSize(1);
  display.print(text);
  display.display();
}

void displaySplashScreen(){
  char text[] = "-TRUPASS-";
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor((SCREEN_WIDTH - strlen(text)*6*2) / 2, SCREEN_HEIGHT/2-8);
  display.print(text);
  display.display();
}

void displayHomeScreen() {
  currentScreen = HOME;

  char label1[] = "Enter";
  char label2[] = "Passkey";  

  passkey = ""; // Empty passkey

  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor((SCREEN_WIDTH-strlen(label1)*12)/2, (SCREEN_HEIGHT)/2-14);
  display.print(label1);
  display.setCursor((SCREEN_WIDTH-strlen(label2)*12)/2, (SCREEN_HEIGHT)/2+4);
  display.print(label2);
  display.display(); 
}

void displayEnterPasskeyScreen() {
  currentScreen = PASSKEY;
  String label = "Enter Passkey";

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor((SCREEN_WIDTH-label.length()*6)/2, 12);
  display.print(label);
  displayPasskey();
  display.display();
}

void displayPasskey(){
  int passkeyX = (SCREEN_WIDTH - passkey.length()*12)/2;
  int passkeyY = (SCREEN_HEIGHT)/2-4;

  String maskedPasskey = "";

  for(int i = 0; i < passkey.length(); i++){
    maskedPasskey += "*";
  }

  display.setTextSize(2);
  
  display.setCursor(passkeyX, passkeyY);
  display.print(maskedPasskey);
}