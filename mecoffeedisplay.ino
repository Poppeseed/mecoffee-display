#include "BLEDevice.h"
#include "Arduino.h"
#include "heltec.h"
#include "images.h"

#define MSG_TIMEOUT_MS          1000
#define SCAN_TIME_MILLIS        10000
#define SHOT_DUR_MILLIS         30000
#define DEBUG_MSGS_ON_OLED      false
#define DEBUG_MSGS_ALL          false
#define MILLIS_TO_SEC(x)        (x/1000)
#define MIN_TEMP_PROG_BAR       80
#define MIN_TEMP_DISPLAY_ON     40

static const String address("7c:ec:79:cb:67:ea");
static BLEUUID serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID("0000ffe1-0000-1000-8000-00805f9b34fb");

static boolean connectionFound = false;
static boolean connected = false;
static boolean doScan = false;
static unsigned long lastScan = -1;

static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

static boolean brewing = false;
static boolean boiling = false;
static boolean boilingSteam = false;
static unsigned long shotStarted = -1;
static unsigned long shotTime = -1;
static int timestamp;
static float reqTemp, curTemp;
static int shotTimeMs;

static boolean lastDisplayOn;


static int lastTmpMsgMs;
static int lastShtMsgMs;
static int lastPidMsgMs;

static void processMessages(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
static void handleMsgTimeouts();
static bool connectToServer();
static void displayConnectedHorizontal();
static void displayConnectedVertical();
static void scan();
static void drawVerticalProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress);

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        bool found = advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID);
        if(found)
        {
            if(DEBUG_MSGS_ALL) Serial.println(advertisedDevice.toString().c_str());
            Serial.println(advertisedDevice.getName().c_str() + String(" found")); // meCoffee found

            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            connectionFound = true;
        }
        else
        {
            // if(DEBUG_MSGS_ALL) Serial.println(String("BLE Device found: ") + advertisedDevice.toString().c_str());
            // else Serial.print(advertisedDevice.getName().c_str() + String(", "));
        }
    }
};

class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pclient)
    {
        Serial.println("onConnect");
    }

    void onDisconnect(BLEClient *pclient)
    {
        connected = false;
        Serial.println("onDisconnect");
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Display...");

    Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
    Heltec.display->screenRotate(ANGLE_270_DEGREE);
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->setLogBuffer(5, 30);
    Heltec.display->clear();

    Serial.println("Starting BLE Client app...");

    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    scan();
}

void loop() {
    
    if (connectionFound)
    {
        if (connectToServer())
        {
            connected = true;
            Serial.println("Connected to the BLE Server.");
        }
        else
        {
            Serial.println("We have failed to connect to the server; there is nothing more we will do.");
        }
        connectionFound = false;
    }
    if((millis() - lastScan) > SCAN_TIME_MILLIS)
    {
        doScan = true;
    }
    if(!connected && doScan)
    {
        Heltec.display->clear();
        scan();
        doScan = false;
    }

    if (brewing || boiling || boilingSteam || curTemp >= MIN_TEMP_DISPLAY_ON)
    {
        if(!lastDisplayOn)
        {
            Heltec.display->displayOn();
            lastDisplayOn = true;
        }
        displayConnectedVertical();
    }
    else
    {
        if(lastDisplayOn)
        {
            Heltec.display->displayOff();
            lastDisplayOn = false;
        }
    }
    
    handleMsgTimeouts();

    delay(100);
}

static void scan()
{
    Heltec.display->setFont(ArialMT_Plain_10);
    Serial.println("Scanning...");
    Heltec.display->println("Scanning..."); Heltec.display->drawLogBuffer(0, 0); Heltec.display->display();
    BLEScanResults foundDevices = BLEDevice::getScan()->start(MILLIS_TO_SEC(SCAN_TIME_MILLIS), false);

    lastScan = millis();

    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());
    Serial.println("Scan done!");
    BLEDevice::getScan()->clearResults();
}

static void processMessages(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    String sData = (char *)pData;

    if (sData.startsWith("tmp"))
    {
        int req, cur;
        sscanf((char *)pData, "tmp %d %d %d", &timestamp, &req, &cur);

        // convert requested
        reqTemp = req / 100.0f;

        if(reqTemp > 110)
            boilingSteam = true;
        else
            boilingSteam = false;

        // convert current, account for value x10 or not
        if(cur >= 100 && cur < 200)
            curTemp = (float)cur;
        else
            curTemp = cur / 10.0f;

        lastTmpMsgMs = millis();
    }
    else if (sData.startsWith("sht"))
    {
        int i, ms;
        sscanf((char *)pData, "sht %d %d", &i, &ms);

        if (ms == 0)
        {
            brewing = true;
            shotStarted = millis();
        }
        else
        {
            brewing = false;
        }

        shotTime = ms / 1000;

        lastShtMsgMs = millis();
    }
    else if (sData.startsWith("pid"))
    {
        boiling = true;

        lastPidMsgMs = millis();
    }
}

static void handleMsgTimeouts()
{
    if(millis() - lastTmpMsgMs > MSG_TIMEOUT_MS)
    {
        boilingSteam = false;
    }

    if(millis() - lastShtMsgMs > MSG_TIMEOUT_MS)
    {
        // brewing = false;
    }

    if(millis() - lastPidMsgMs > MSG_TIMEOUT_MS)
    {
        if(!boilingSteam)
        {
            boiling = false;
        }
    }
}

static bool connectToServer()
{
    Serial.println(String("Connecting to ") + myDevice->getAddress().toString().c_str());
    //Heltec.display->println(String("Connecting to ") + myDevice->getAddress().toString().c_str()); Heltec.display->drawLogBuffer(0, 0); Heltec.display->display();
    
    BLEClient *pClient = BLEDevice::createClient(); Serial.println(" - Created client");
    pClient->setClientCallbacks(new MyClientCallback());

    pClient->connect(myDevice); Serial.println(" - Connected to server");

    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr)
    {
        Serial.println(String("Failed: ") + serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr)
    {
        Serial.println(String("Failed to find Characteristic: ") + charUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our characteristic");

    if (pRemoteCharacteristic->canNotify())
        pRemoteCharacteristic->registerForNotify(processMessages);

    return true;
}

static void displayConnectedVertical()
{
    int x = 0;
    int y = 0;
    int width = 64;

    Heltec.display->clear();
    
    y = 0;
    Heltec.display->setFont(ArialMT_Plain_24);
	Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(width, y, String(curTemp, 0) + "°");
    y += 26;

    Heltec.display->setFont(ArialMT_Plain_10);
	Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(width, y, String(reqTemp, 0) + "°");
    y += 12;
    int progress = (int)(max(0.0f, curTemp - MIN_TEMP_PROG_BAR) * 100.0 / (reqTemp - MIN_TEMP_PROG_BAR));
    // Heltec.display->drawProgressBar(0, y, width-1, 15, progress);
    drawVerticalProgressBar(0, 0, 15, 100, progress);

    if(brewing)
    {
        shotTimeMs = (millis() - shotStarted);
    }

    y = 62;
    Heltec.display->setFont(ArialMT_Plain_16);
    Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(width, y, String(shotTimeMs / 1000.0, 1) + "s");
    y += 18;

    progress = (shotTimeMs * 100 / SHOT_DUR_MILLIS) % 100;
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(width, y, String(progress) + "%");
    y += 12;

    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(width, y, String(SHOT_DUR_MILLIS / 1000) + "s");
    y += 12;

    Heltec.display->drawProgressBar(0, y, width-1, 13, progress);

    Heltec.display->display();
}

static void drawProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress) {
  uint16_t radius = height >> 1;
  uint16_t xRadius = x + radius;
  uint16_t yRadius = y + radius;
  uint16_t doubleRadius = radius << 1;
  uint16_t innerRadius = radius - 2;

  Heltec.display->setColor(WHITE);
  Heltec.display->drawCircleQuads(xRadius, yRadius, radius, 0b00000110);
  Heltec.display->drawHorizontalLine(xRadius, y, width - doubleRadius + 1);
  Heltec.display->drawHorizontalLine(xRadius, y + height, width - doubleRadius + 1);
  Heltec.display->drawCircleQuads(x + width - radius, yRadius, radius, 0b00001001);

  uint16_t maxProgressWidth = (width - doubleRadius + 1) * progress / 100;

  Heltec.display->fillCircle(xRadius, yRadius, innerRadius);
  Heltec.display->fillRect(xRadius + 1, y + 2, maxProgressWidth, height - 3);
  Heltec.display->fillCircle(xRadius + maxProgressWidth, yRadius, innerRadius);
}

static void drawVerticalProgressBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress) {
  uint16_t radius = width >> 1;
  uint16_t xRadius = x + radius;
  uint16_t yRadius = y + radius;
  uint16_t doubleRadius = radius << 1;
  uint16_t innerRadius = radius - 2;

  Heltec.display->setColor(WHITE);
  Heltec.display->drawCircleQuads(xRadius, yRadius, radius, 0b00000011);
  Heltec.display->drawVerticalLine(x, yRadius, height - doubleRadius + 1);
  Heltec.display->drawVerticalLine(x + width, yRadius, height - doubleRadius + 1);
  Heltec.display->drawCircleQuads(xRadius, y + height - radius, radius, 0b00001100);

  uint16_t maxProgressHeight = (height - doubleRadius + 1) * progress / 100;

  Heltec.display->fillCircle(xRadius, height - yRadius - maxProgressHeight, innerRadius);
  Heltec.display->fillRect(x + 2, height - yRadius + 1 - maxProgressHeight, width - 3, maxProgressHeight);
  Heltec.display->fillCircle(xRadius, height - yRadius, innerRadius);
}


static void drawVerticalProgressBarDown(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t progress) {
  uint16_t radius = width >> 1;
  uint16_t xRadius = x + radius;
  uint16_t yRadius = y + radius;
  uint16_t doubleRadius = radius << 1;
  uint16_t innerRadius = radius - 2;

  Heltec.display->setColor(WHITE);
  Heltec.display->drawCircleQuads(xRadius, yRadius, radius, 0b00000011);
  Heltec.display->drawVerticalLine(x, yRadius, height - doubleRadius + 1);
  Heltec.display->drawVerticalLine(x + width, yRadius, height - doubleRadius + 1);
  Heltec.display->drawCircleQuads(xRadius, y + height - radius, radius, 0b00001100);

  uint16_t maxProgressHeight = (height - doubleRadius + 1) * progress / 100;

  Heltec.display->fillCircle(xRadius, yRadius, innerRadius);
  Heltec.display->fillRect(x + 2, yRadius + 1, width - 3, maxProgressHeight);
  Heltec.display->fillCircle(xRadius, yRadius + maxProgressHeight, innerRadius);
}
