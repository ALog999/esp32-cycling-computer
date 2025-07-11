#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <TinyGPSPlus.h>

// Display Setup
#define TFT_CS     5
#define TFT_RST    22
#define TFT_DC     21
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// GPS Setup
#define GPS_RX 16  // Connect to GPS TX
#define GPS_TX 17  // Connect to GPS RX
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

double totalDistanceMeters = 0;
double lastLat = 0;
double lastLng = 0;
bool hasLastLocation = false;

const float MIN_VALID_SPEED_MPH = 0.5; // Threshold to reduce noise in speed readings

// BLE UUIDs
static BLEUUID hrServiceUUID((uint16_t)0x180D);
static BLEUUID hrMeasurementUUID((uint16_t)0x2A37);
static BLEUUID powerMeterServiceUUID("00001818-0000-1000-8000-00805f9b34fb");
static BLEUUID powerMeterMeasurementUUID((uint16_t)0x2A63);

// BLE Globals
BLEClient* pClient;
BLERemoteCharacteristic* pRemoteCharacteristic;
BLEClient* pPowerMeterClient;
BLERemoteCharacteristic* pPowerMeterCharacteristic;

bool hrmConnected = false;
bool powerMeterConnected = false;
bool connectingHRM = false;
bool connectingPowerMeter = false;

BLEAddress hrmAddress("ec:72:5d:a5:a4:64");  // Fixed HRM address
BLEAddress powerMeterAddress("da:31:1b:7c:9f:c9");  // Fixed Power Meter address

int heartRate = 0;
int power = 0;
float averagePower = 0.0;
int powerReadingsCount = 0;

BLEAdvertisedDevice* myDevice = nullptr;
BLEAdvertisedDevice* powerMeterDevice = nullptr;

// BLE Scan Callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("Found BLE device: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.getAddress().equals(hrmAddress)) {
      Serial.println("Heart Rate Monitor Found.");
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      connectingHRM = true;
    }

    if (advertisedDevice.getAddress().equals(powerMeterAddress)) {
      Serial.println("Power Meter Found.");
      powerMeterDevice = new BLEAdvertisedDevice(advertisedDevice);
      connectingPowerMeter = true;
    }
  }
};

// HRM Notify Callback
static void hrmNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 2) {
    heartRate = pData[1];
    Serial.print("Heart Rate: ");
    Serial.println(heartRate);

    tft.fillRect(0, 80, 160, 30, ST7735_BLACK);
    tft.setCursor(10, 80);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.println("Heart Rate:");
    tft.setTextSize(2);
    tft.setCursor(90, 80);
    tft.print(heartRate);
  }
}

// Power Notify Callback
static void powerMeterNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (length >= 3) {
    power = (pData[1] << 8) | pData[2];
    Serial.print("Power: ");
    Serial.println(power);

    averagePower = ((averagePower * powerReadingsCount) + power) / (++powerReadingsCount);

    tft.fillRect(0, 120, 160, 30, ST7735_BLACK);
    tft.setCursor(10, 120);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.println("Power:");
    tft.setTextSize(2);
    tft.setCursor(90, 120);
    tft.print(power);

    tft.fillRect(0, 150, 160, 30, ST7735_BLACK);
    tft.setCursor(10, 150);
    tft.setTextSize(1);
    tft.println("Avg:");
    tft.setTextSize(2);
    tft.setCursor(60, 150);
    tft.print((int)averagePower);
  }
}

// Connect to HRM
bool connectToHRM() {
  if (myDevice == nullptr) return false;

  Serial.println("Connecting to HRM...");
  pClient = BLEDevice::createClient();
  if (!pClient->connect(myDevice)) return false;

  auto service = pClient->getService(hrServiceUUID);
  if (!service) return false;

  pRemoteCharacteristic = service->getCharacteristic(hrMeasurementUUID);
  if (!pRemoteCharacteristic) return false;

  pRemoteCharacteristic->registerForNotify(hrmNotifyCallback);
  hrmConnected = true;
  return true;
}

// Connect to Power Meter
bool connectToPowerMeter() {
  if (powerMeterDevice == nullptr) return false;

  Serial.println("Connecting to Power Meter...");
  pPowerMeterClient = BLEDevice::createClient();
  if (!pPowerMeterClient->connect(powerMeterDevice)) return false;

  auto service = pPowerMeterClient->getService(powerMeterServiceUUID);
  if (!service) return false;

  pPowerMeterCharacteristic = service->getCharacteristic(powerMeterMeasurementUUID);
  if (!pPowerMeterCharacteristic) return false;

  pPowerMeterCharacteristic->registerForNotify(powerMeterNotifyCallback);
  powerMeterConnected = true;
  return true;
}

// Setup
void setup() {
  Serial.begin(115200);

  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.println("Initializing...");

  // Start GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Start BLE
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
}

// Loop
void loop() {
  // BLE reconnections
  if (!hrmConnected && connectingHRM) {
    if (connectToHRM()) Serial.println("HRM Connected.");
    else Serial.println("HRM Connection Failed.");
    connectingHRM = false;
  }

  if (!powerMeterConnected && connectingPowerMeter) {
    if (connectToPowerMeter()) Serial.println("Power Meter Connected.");
    else Serial.println("Power Meter Connection Failed.");
    connectingPowerMeter = false;
  }

  // GPS parsing
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Distance calculation
  if (gps.location.isUpdated()) {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();

    if (hasLastLocation) {
      double distanceBetween = TinyGPSPlus::distanceBetween(lastLat, lastLng, currentLat, currentLng);
      totalDistanceMeters += distanceBetween;
    } else {
      hasLastLocation = true;
    }

    lastLat = currentLat;
    lastLng = currentLng;
  }

  // GPS speed + distance display
  if (gps.speed.isUpdated()) {
    float speedKmh = gps.speed.kmph();
    float speedMph = speedKmh * 0.621371;

    // Filter out noise
    if (speedMph < MIN_VALID_SPEED_MPH) {
      speedMph = 0.0;
    }

    Serial.print("Speed (mph): ");
    Serial.println(speedMph);

    float distanceMiles = totalDistanceMeters / 1609.34;

    // Speed display
    tft.fillRect(0, 40, 160, 30, ST7735_BLACK);
    tft.setCursor(10, 40);
    tft.setTextSize(1);
    tft.println("Speed:");
    tft.setTextSize(2);
    tft.setCursor(90, 40);
    tft.print(speedMph, 1);

    // Distance display
    tft.fillRect(0, 10, 160, 30, ST7735_BLACK);
    tft.setCursor(10, 10);
    tft.setTextSize(1);
    tft.println("Distance:");
    tft.setTextSize(2);
    tft.setCursor(90, 10);
    tft.print(distanceMiles, 1);
  }

  delay(200);
}
