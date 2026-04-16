#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSen5x.h>
#include <NimBLEDevice.h>
#include <SPI.h>
#include <SD.h>

// ESP32-S3 specific I2C pins
#define I2C_SDA 8
#define I2C_SCL 9

// ESP32-S3 ADC channel pins
//#define CO_ADC_PIN 4
#define O2_ADC_PIN 5

// ZE07-CO UART pins
#define ZE07_RX_PIN 18  // ESP32 receives from ZE07 TXD. listens to sensor TXD wire
#define ZE07_TX_PIN 17  // Optional only for commands 

// SD card SPI pins
#define SD_SCK   12
#define SD_MISO  13
#define SD_MOSI  11
#define SD_CS    10

// BLE UUIDs
#define SERVICE_UUID "238d650a-4700-4cfb-a085-748030f31de5"
#define CHARACTERISTIC_UUID "b67f033d-ce15-4434-83db-31f8161afaea"

//globals
SPIClass sdSPI(FSPI);
bool sdReady = false;

// ADC configuration
const int ADC_MAX_COUNTS = 4095;     // Max count for 12-bit ADC
const float ADC_REF_VOLTAGE = 3.3f;  // ADC reference voltage

//Timing period 100ms = 10hz
const uint32_t ME2_TIMER_PERIOD = 100000;

// Number of ADC samples to average every 100 ms
const uint8_t ADC_SAMPLES_TO_AVG = 10;

// SEN54 and BLE objects
SensirionI2CSen5x sen5x;
NimBLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// Latest SEN54 values
float latestPm2p5 = 0.0f;
float latestPm10 = 0.0f;
float latestTemp = 0.0f;
float latestRh = 0.0f;

//Create UART1 for the ZE07 sensor
HardwareSerial ze07Serial(1);

//Latest ZE07-CO value
float latestZE07ppm = 0.0f;
bool ze07Valid = false;


// Latest ME2 ADC values
//uint16_t latestCoCounts = 0;
uint16_t latestO2Counts = 0;

//float latestCoVoltage   = 0.0f;
float latestO2Voltage   = 0.0f;


// Timers
unsigned long sensorTimer = 0;
unsigned long bleTimer = 0;

//Hardware timer ISR 
hw_timer_t* sampleTimer = nullptr; //holds the ESP32 hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool me2SampleFlag = false; // tells loop() that a sample is due
volatile uint32_t me2SampleIndex = 0; // Counts timer events

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

// ISR: hardware timer fires every 100 ms
// only set a flag/counter
void IRAM_ATTR onSampleTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    me2SampleFlag = true;
    me2SampleIndex++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ADC setup for ME2 sensor channels
void setupME2ADC() {
    // Set ADC resolution to 12-bit
    analogReadResolution(12);

    // Set 11dB attenuation for wider input voltage range
    //analogSetPinAttenuation(CO_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(O2_ADC_PIN, ADC_11db);

    // Configure pins as inputs
    //pinMode(CO_ADC_PIN, INPUT);
    pinMode(O2_ADC_PIN, INPUT);

    Serial.println("ME2 ADC setup complete");
}

// Hardware timer setup running every 100ms
void setupHardwareTimer() {
    sampleTimer = timerBegin(0, 80, true);

    if (sampleTimer == nullptr) {
        Serial.println("Timer init failed");
        while (true) {
            delay(1000);
        }
    }

    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    timerAlarmWrite(sampleTimer, ME2_TIMER_PERIOD, true);
    timerAlarmEnable(sampleTimer);

    Serial.println("Hardware timer started");
    Serial.println("Timer tick = 1 us");
    Serial.println("Timer period = 100000 us = 100 ms = 10 Hz");
}

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


// Read ME2 sensor channels
// This reads the TIA output voltage through the ESP32 ADC
void readME2SensorsAveraged(uint8_t numSamples) {
    // Read raw ADC counts
    //latestCoCounts = readAveragedADC(CO_ADC_PIN, numSamples);
    latestO2Counts = readAveragedADC(O2_ADC_PIN, numSamples);

    // Convert counts to voltage
    //latestCoVoltage = countsToVoltage(latestCoCounts);
    latestO2Voltage = countsToVoltage(latestO2Counts);
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

        Serial.println("----------------------------------");
        Serial.printf("PM2.5: %.2f ug/m3\n", latestPm2p5);
        Serial.printf("PM10 : %.2f ug/m3\n", latestPm10);
        Serial.printf("Temp : %.2f C\n", latestTemp);
        Serial.printf("RH   : %.2f %\n", latestRh);
    } else {
        char errorMessage[256];
        errorToString(sensorStatus, errorMessage, 256);
        Serial.printf("Sensor Error: %s\n", errorMessage);
    }
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
    char basePayload[100];
    char finalPayload[120];

    sprintf(basePayload, "PM2.5=%.2f ug/m3, PM10=%.2f ug/m3, Temp=%.2fC, Hum=%.2f % ", latestPm2p5, latestPm10, latestTemp, latestRh);

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

    // Create CSV header if file does not exist
    if (!SD.exists("/ze07_log.csv")) {
        File file = SD.open("/ze07_log.csv", FILE_WRITE);
        if (file) {
            file.println("timestamp_ms,elapsed_hr,sample_index,ze07_ppm");
            file.close();
            Serial.println("Created /ze07_log.csv with header");
        } else {
            Serial.println("Failed to create /ze07_log.csv");
        }
    }
}

void logZE07DataToSD(uint32_t timestampMs, float elapsedHours, uint32_t sampleIndex,
                    float ze07ppm) {
    if (!sdReady) {
        return;
    }

    File file = SD.open("/ze07_log.csv", FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open /ze07_log.csv for append");
        return;
    }

    file.printf("%lu,%.3f,%lu,%.1f\n",
                (unsigned long)timestampMs,
                elapsedHours,
                (unsigned long)sampleIndex,
                ze07ppm);
    //Serial.println("SD write good"); //writing to sd card works

    file.close();
}


void setup() {
   Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    Serial.println();
    Serial.println("Starting ZE07-CO test");
    Serial.println("Starting SEN54 + ME2 + BLE test");

    //setupSen54();
    //setupME2ADC();
    //setupBLE();
    //setupHardwareTimer();

    //setupSDCard();

    // Start ZE07 UART
    // Datasheet default 9600 baud, 8N1
    ze07Serial.begin(9600, SERIAL_8N1, ZE07_RX_PIN, ZE07_TX_PIN);
    Serial.println("ZE07-CO UART started");
    Serial.println("Allow sensor to warm up (3-5 min)");

   // CSV header for calibration logging over UART
   Serial.println("timestamp_ms,elapsed_hr,sample_index,ze07_ppm");
}

void loop() {
      
    // Read ZE07 UART 
    float ppm = 0.0f;

    if (readZE07ppm(ppm)) {
        latestZE07ppm = ppm;
        ze07Valid = true;

        uint32_t timestampMs = millis();
        float elapsedHours = timestampMs / 3600000.0f;

        // CSV line (for logging)
        Serial.printf("%lu,%.3f,%.1f\n",
              (unsigned long)timestampMs,
              elapsedHours,
              latestZE07ppm);

        Serial.print("CO ppm: ");
        Serial.println(latestZE07ppm, 1);
    }   
/*
     // Handle ME2 sampling flag using 100ms timer
    bool doSample = false;
    uint32_t localSampleIndex = 0;

    portENTER_CRITICAL(&timerMux);
    if (me2SampleFlag) {
        me2SampleFlag = false;
        doSample = true;
        localSampleIndex = me2SampleIndex;
    }
    portEXIT_CRITICAL(&timerMux);

    if (doSample) {
        uint32_t timestampMs = millis();

        // Average 10 ADC reads per channel every 100 ms
        //readME2SensorsAveraged(ADC_SAMPLES_TO_AVG);

    
    float elapsedHours = timestampMs / 3600000.0f;

    //Serial.printf("%lu,%.3f,%lu,%.1f\n",
     //         (unsigned long)timestampMs,
     //         elapsedHours,
     //         (unsigned long)localSampleIndex,
      //        latestZE07ppm);

    //logZE07DataToSD(timestampMs, elapsedHours, localSampleIndex, latestZE07ppm);
        Serial.printf("%lu,%lu,%u,%.4f,%u,%.4f\n",
                      (unsigned long)timestampMs,
                      (unsigned long)localSampleIndex,
                      latestCoCounts,
                      latestCoVoltage,
                      latestO2Counts,
                      latestO2Voltage);
                    
    }

    // Read and print SEN54 every 1 second
    if (millis() - sensorTimer >= 1000) {
        sensorTimer = millis(); // update sensortimer to miilis(1000)
        readSen54();
    }

    // Send BLE update every 5 seconds
    if (millis() - bleTimer >= 5000) {
        bleTimer = millis(); //upate bletimer to millis(5000)
        sendBleUpdate();
    }
*/
}