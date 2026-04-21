#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <NimBLEDevice.h>
#include <SPI.h>
#include <SD.h>

//Pin definitions

// I2C pins for SEN54
#define I2C_SDA 8
#define I2C_SCL 9

// 02 ADC pins for ME2-02
//#define CO_ADC_PIN 4
#define O2_ADC_PIN 5

// ZE07-CO UART pins
#define ZE07_RX_PIN 18  // ESP32 receives from ZE07 TXD. listens to sensor TXD wire
#define ZE07_TX_PIN 17  // Optional only for commands 

//Alert pins
#define LED_YELLOW  6
#define LED_RED     7    
#define BUZZER_PIN  14 

// SD card module SPI pins
#define SD_SCK   12
#define SD_MISO  13
#define SD_MOSI  11
#define SD_CS    10

// BLE UUIDs
#define SERVICE_UUID "238d650a-4700-4cfb-a085-748030f31de5"
#define CHARACTERISTIC_UUID "b67f033d-ce15-4434-83db-31f8161afaea"

//Globals
// SEN54 object
SensirionI2CSen5x sen5x;
// Create UART1 for the ZE07 sensor 
HardwareSerial ze07Serial(1);
// SPI object for SD card
SPIClass sdSPI(FSPI);
// BLE objects
NimBLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;
// SD state
bool sdReady = false;

// Timing millis based
unsigned long lastO2SampleMs  = 0;
unsigned long lastZE07ReadMs  = 0;
unsigned long lastSen54ReadMs = 0;
unsigned long lastBleSendMs   = 0;
unsigned long lastPrintMs     = 0;
unsigned long lastSdLogMs     = 0;

// ADC / TIA config for O2
const int ADC_MAX_COUNTS = 4095;     // Max count for 12-bit ADC
const float ADC_REF_VOLTAGE = 3.3f;  // ADC reference voltage

const float O2_VREF = 1.65f;        //midpoint VREF from TIA
const float O2_RF_OHMS = 10000.0f;   // testing with 10k ohms feedback TIA

// Sensor #1 inspection-sheet fit:
// 20.9% -> 138 uA
// 10.0% -> 64.6 uA
// O2(%) = 0.1485 * IuA + 0.41
const float O2_SLOPE  = 0.1485f;
const float O2_OFFSET = 0.41f;

// Thresholds for alerts 
const float O2_ALARM  = 19.5f;

const float CO_WARNING_PPM = 35.0f;
const float CO_DANGER_PPM  = 50.0f;

// Latest SEN54 values
float latestPm2p5 = 0.0f;
float latestPm10 = 0.0f;
float latestTemp = 0.0f;
float latestRh = 0.0f;
bool sen54Valid = false;

// Latest ME2-O2 values
uint16_t latestO2Counts = 0;
float latestO2Voltage = 0.0f;
float latestO2CurrentUA = 0.0f;
float latestO2Percent = 0.0f;

//Latest ZE07-CO value
float latestZE07ppm = 0.0f;
bool ze07Valid = false;

// BLE callbacks
// Runs when phone connects/disconnects
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("Phone connected");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("Phone disconnected");

        NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
        pAdvertising->start();
        Serial.println("Advertising restarted");
    }
};

// Convert ADC counts to voltage
// Formula: voltage = (counts / 4095) * 3.3
float countsToVoltage(uint16_t counts) {
    return ((float)counts / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;
}

// Take N ADC samples and average them to reduce ADC noise
uint16_t readAveragedADC(uint8_t pin, uint8_t numSamples) {
    uint32_t sum = 0;

    for (uint8_t i = 0; i < numSamples; i++) {
        sum += analogRead(pin);

        // Small spacing between ADC reads
        delayMicroseconds(100);
    }
    return (uint16_t)(sum / numSamples);
}

// Convert TIA output Voltage back to sensor current (TIA equation) I = (Vref - Vout) / Rf
float voltageToCurrentUA(float vout) {
    float currentA = (O2_VREF - vout) / O2_RF_OHMS;
    return currentA * 1e6f;
}

// Convert sensor current to O2 percentage using calibration
float currentUAToO2Percent(float currentUA) {
    return (O2_SLOPE * currentUA) + O2_OFFSET;
}

// ADC setup for ME2-O2 sensor channels
void setupO2ADC() {
    // Set ADC resolution to 12-bit
    analogReadResolution(12);

    // Set 11dB attenuation for wider input voltage range
    analogSetPinAttenuation(O2_ADC_PIN, ADC_11db);

    // Configure pins as inputs
    pinMode(O2_ADC_PIN, INPUT);

    Serial.println("ME2-O2 ADC setup complete");
}

// Read ME2-O2 sensor channels
// This reads the TIA output voltage through the ESP32 ADC
void readO2Sensor(uint8_t numSamples) {
    // Read raw ADC counts
    latestO2Counts = readAveragedADC(O2_ADC_PIN, numSamples);
    // Convert ADC counts to voltage
    latestO2Voltage = countsToVoltage(latestO2Counts);
    if (latestO2Counts <= 5) {
        latestO2CurrentUA = 0.0f;
        latestO2Percent = 0.0f;
        Serial.println("O2 reading invalid - check wiring / hook grabbers");
        return;
    }
    // Converts voltage to sensor current
    latestO2CurrentUA = voltageToCurrentUA(latestO2Voltage);
    // Converts current to oxygen percentage using calibration
    latestO2Percent = currentUAToO2Percent(latestO2CurrentUA);
}

// ZE07 checksum helper
// Frame points to an array of bytes. This is 9-byte UART frame from sensor
uint8_t ze07Checksum(const uint8_t *frame) { 
    uint8_t sum = 0;
    // skips index 0 because it is the start byte
    for (int i = 1; i <= 7; i++) {
        sum += frame[i]; // sum = frame[1] +....+ frame[7]
    }
    // Functional description: Sum check [Not (Byte1+Byte2+…Byte7) +1] from datasheet
    // This creates a checksum to verify correct data from the sensor
    sum = (~sum) + 1;
    return sum;
}

// Read one ZE07 ppm value if a valid frame is available
bool readZE07ppm(float &ppm) {
    //check how many bytes are in UART buffer need at least 9bytes
    if (ze07Serial.available() < 9) {
        return false;
    }
    // Throw away bytes until start byte is found
    while (ze07Serial.available() > 0 && ze07Serial.peek() != 0xFF) {
        ze07Serial.read();
    }
    // Need 9 bytes after aligning to frame start
    if (ze07Serial.available() < 9) {
        return false;
    }
    uint8_t frame[9]; // Create an array to store 9 bytes
    ze07Serial.readBytes(frame, 9); // Read 9 bytes into array
    // Double check start byte
    if (frame[0] != 0xFF) {
        return false;
    }
    // Check checksum and compare with received checksum
    if (ze07Checksum(frame) != frame[8]) {
        return false;
    }
    // Gas concentration value 
    uint16_t raw = ((uint16_t)frame[4] * 256) + frame[5];
    ppm = raw * 0.1f;

    return true;
}

// Updates CO reading from ZE07 sensor
void updateZE07() {
    // Temporary variable to store new CO value ppm
    float ppm = 0.0f;
    // readZE07ppm() returns true only if checksum + data are valid
    if (readZE07ppm(ppm)) {
        // Save the valid CO reading to global variable
        latestZE07ppm = ppm;
        // Mark that we have received valid data at least once
        ze07Valid = true;
    }
    // If read fails, we keep the previous value   
}

// Sensor SEN54 setup
void setupSen54() {
    Wire.begin(I2C_SDA, I2C_SCL);
    sen5x.begin(Wire);

    uint16_t sensorStatus;
    char errorMessage[256];

    sensorStatus = sen5x.deviceReset();
    if (sensorStatus != 0) {
        errorToString(sensorStatus, errorMessage, 256);
        Serial.printf("Reset Error: %s\n", errorMessage);
    }

    delay(100);

    sensorStatus = sen5x.startMeasurement();
    if (sensorStatus != 0) {
        errorToString(sensorStatus, errorMessage, 256);
        Serial.printf("Start Error: %s\n", errorMessage);
    } else {
        Serial.println("SEN54 measurement started");
        Serial.println("Waiting for stable readings...");
    }
}

// Read SEN54 values
void readSen54() {
    float pm1, pm2p5, pm4, pm10, rh, temp, voc, nox;

    uint16_t sensorStatus = sen5x.readMeasuredValues(
        pm1, pm2p5, pm4, pm10, rh, temp, voc, nox
    );

    if (sensorStatus == 0) {
        latestPm2p5 = pm2p5;
        latestPm10  = pm10;
        latestTemp  = temp;
        latestRh    = rh;

        Serial.println("----------------------------");
        Serial.printf("Time: %lu ms\n", millis());

        Serial.printf("PM2.5: %-6.2f ug/m3   PM10: %-6.2f ug/m3\n", pm2p5, pm10);
        Serial.printf("Temp : %-6.2f C       RH  : %-6.2f %%\n", temp, rh);
    } else {
        char errorMessage[256];
        errorToString(sensorStatus, errorMessage, 256);
        Serial.printf("Sensor Error: %s\n", errorMessage);
    }
}

// BLE setup create BLE server, service, and characteristic
void setupBLE() {
    NimBLEDevice::init("WAQM"); // Intialize BLE device

    NimBLEServer* pServer = NimBLEDevice::createServer(); // Create a server
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID); // Create a service

    pCharacteristic = pService->createCharacteristic(   // Create a characteristic. Read: phone can request data and Notify: ESP32 can push updates
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pCharacteristic->setValue("Waiting for data");
    pService->start(); // Start service

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising(); // Start advertising
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    Serial.println("BLE advertising started");
}


// Checksum XOR function for BLE text payload
uint8_t calculateChecksum(const char* data) {
    uint8_t checksum = 0;

    while (*data) {
        checksum ^= (uint8_t)(*data);  // XOR each byte
        data++;
    }

    return checksum;
}
//Send BLE Update
void sendBleUpdate() {
    char basePayload[180];
    char finalPayload[220];

    sprintf(basePayload, "O2=%.2f,CO=%.1f,PM2.5=%.2f,PM10=%.2f,Temp=%.2f,RH=%.2f", latestO2Percent, latestZE07ppm, latestPm2p5, latestPm10, latestTemp, latestRh);

    //Compute checksum
    uint8_t checksum = calculateChecksum(basePayload);

    //Add checksum to end of payload
    snprintf(finalPayload, sizeof(finalPayload), "%s,chk=%02X", basePayload, checksum);

    size_t payloadLen = strlen(finalPayload); // byte count
    
    //Send value over BLE
    pCharacteristic->setValue((uint8_t*)finalPayload, payloadLen);

    // Serial output
    Serial.println("----- BLE Payload -----");
    Serial.print("Data: ");
    Serial.println(finalPayload);

    Serial.print("Bytes sent: ");
    Serial.println(payloadLen);

    Serial.print("Checksum (HEX): 0x");
    Serial.println(checksum, HEX);

    if (deviceConnected) {
        pCharacteristic->notify();
        Serial.println("BLE update sent\n");
    } else {
        Serial.println("No phone connected\n"); 
    }
}

void setupSDCard() {
    Serial.println("Initializing SD card...");

    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    if (!SD.begin(SD_CS, sdSPI)) {
        Serial.println("SD card init failed");
        sdReady = false;
        return;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card detected");
        sdReady = false;
        return;
    }

    sdReady = true;
    Serial.println("SD card initialized");

    uint64_t cardSizeMB = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD card size: %llu MB\n", cardSizeMB);

    // Create ZE07 CSV header if file does not exist
    if (!SD.exists("/ze07_log.csv")) {
        File file = SD.open("/ze07_log.csv", FILE_WRITE);
        if (file) {
            file.println("timestamp_ms,elapsed_hr,ze07_ppm");
            file.close();
            Serial.println("Created /ze07_log.csv");
        } else {
            Serial.println("Failed to create /ze07_log.csv");
        }
    }

    // Create O2 CSV header if file does not exist
    if (!SD.exists("/o2_log.csv")) {
        File file = SD.open("/o2_log.csv", FILE_WRITE);
        if (file) {
            file.println("timestamp_ms,elapsed_hr,o2_adc_counts,o2_voltage_v,o2_current_ua,o2_percent");
            file.close();
            Serial.println("Created /o2_log.csv");
        } else {
            Serial.println("Failed to create /o2_log.csv");
        }
    }
}


void logZE07DataToSD(uint32_t timestampMs, float elapsedHours, float ze07ppm) {
    if (!sdReady) {
        return;
    }

    File file = SD.open("/ze07_log.csv", FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open /ze07_log.csv for append");
        return;
    }

    file.printf("%lu,%.3f,%.1f\n",
                (unsigned long)timestampMs,
                elapsedHours, ze07ppm);
    //Serial.println("SD write good"); //writing to sd card works

    file.close();
}

// Log O2 data to SD card
void logO2DataToSD(uint32_t timestampMs, float elapsedHours) {
    if (!sdReady) {
        return;
    }

    File file = SD.open("/o2_log.csv", FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open /o2_log.csv for append");
        return;
    }

    file.printf("%lu,%.3f,%u,%.4f,%.2f,%.2f\n", (unsigned long)timestampMs, 
                elapsedHours, latestO2Counts, latestO2Voltage, 
                latestO2CurrentUA, latestO2Percent);

    file.close();
}

// Alert logic
// Yellow LED = warning (CO >= 35 ppm)
// Red LED + buzzer = danger (CO >= 50 ppm OR O2 < 19.5%)
void updateAlerts() {

     // Danger conditions
    bool o2Danger = (latestO2Percent < O2_ALARM);
    bool coDanger = (latestZE07ppm >= CO_DANGER_PPM);

    // Warning condition
    bool coWarning = (latestZE07ppm >= CO_WARNING_PPM);

    // Reset all outputs first
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    noTone(BUZZER_PIN);

    // Danger has highest priority
    if (o2Danger || coDanger) {
        digitalWrite(LED_RED, HIGH);
        tone(BUZZER_PIN, 2000);
    }
    // Warning only
    else if (coWarning) {
        digitalWrite(LED_YELLOW, HIGH);
    }
}


void setup() {
   Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println();
    Serial.println("Starting full system test");
    
    //Serial.println("ZE07 + SEN54 + O2 + BLE + SD + alerts");

     // Alert outputs
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    noTone(BUZZER_PIN);

    // Setup sensors
    setupSen54();
    setupO2ADC();
    //setupBLE();
    //setupSDCard();

    // Start ZE07 UART
    // Datasheet default 9600 baud, 8N1
    ze07Serial.begin(9600, SERIAL_8N1, ZE07_RX_PIN, ZE07_TX_PIN);
    Serial.println("ZE07-CO UART started");
    Serial.println("Allow sensor to warm up (3-5 min)");

   // CSV-style serial headers
    //Serial.println("timestamp_ms,elapsed_hr,ze07_ppm,o2_percent");
}

void loop() {
    unsigned long now = millis();
    float elapsedHours = now / 3600000.0f; 
    
    if (now - lastSen54ReadMs >= 1000) {
        lastSen54ReadMs = now;
        readSen54();
    }
    // Read ZE07 every 1 second
    if (now - lastZE07ReadMs >= 1000) {
        lastZE07ReadMs = now;
        updateZE07();
    }

    // Read O2 every 1 second
    if (now - lastO2SampleMs >= 1000) {
        lastO2SampleMs = now;
        readO2Sensor(10);
    }

    // Update alarms continuously
    updateAlerts();
/*
    // Send BLE every 2 seconds
    if (now - lastBleSendMs >= 2000) {
        lastBleSendMs = now;
        sendBleUpdate();
    }
    // Log to SD every 1 second
    if (now - lastSdLogMs >= 1000) {
        lastSdLogMs = now;

        logZE07DataToSD(now, elapsedHours, latestZE07ppm);
        logO2DataToSD(now, elapsedHours);
    }
*/  if (now - lastPrintMs >= 1000) {
        lastPrintMs = now;
    // Keep serial print
        Serial.println("========== SYSTEM READINGS ==========");
        Serial.print("ZE07 CO ppm: ");
        Serial.println(latestZE07ppm, 1);

        Serial.print("O2 ADC Counts: ");
        Serial.println(latestO2Counts);

        Serial.print("O2 Voltage: ");
        Serial.println(latestO2Voltage, 4);

        Serial.print("O2 Current uA: ");
        Serial.println(latestO2CurrentUA, 2);

        Serial.print("O2 Percent: ");
        Serial.println(latestO2Percent, 2);

        Serial.print("PM2.5: ");
        Serial.println(latestPm2p5, 2);

        Serial.print("PM10: ");
        Serial.println(latestPm10, 2);

        Serial.print("Temp: ");
        Serial.println(latestTemp, 2);

        Serial.print("RH: ");
        Serial.println(latestRh, 2);

        Serial.println();
    }
}
