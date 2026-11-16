#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GPS.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BNO08x.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =====================================================
// MINI AVIONICS SENSOR COMPUTER — ESP-NOW LIVE DATA v1
//
// Board: Adafruit ESP32-S2 Feather with onboard BME280
// USB Serial Monitor: 115200 baud
// Wireless link: ESP-NOW broadcast, Wi-Fi channel 6
//
// Sensors:
//   PA1010D GPS     I2C 0x10
//   BME280 onboard  I2C 0x77
//   BNO085          I2C 0x4A / 0x4B
//
// No TX/RX or RS485 converter is required.
// Feather broadcasts AV1 packets directly to the Waveshare.
// =====================================================

constexpr uint8_t GPS_I2C_ADDRESS = 0x10;
constexpr uint8_t BME280_I2C_ADDRESS = 0x77;
constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25F;

constexpr uint8_t ESPNOW_CHANNEL = 6;
constexpr uint32_t PACKET_INTERVAL_MS = 100;      // 10 Hz wireless link
constexpr uint32_t BME_INTERVAL_MS = 500;         // 2 Hz environment
constexpr uint32_t DEBUG_INTERVAL_MS = 2000;
constexpr uint32_t GPS_TIMEOUT_MS = 3000;

Adafruit_GPS gps(&Wire);
Adafruit_BME280 bme280;
Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bnoEvent;

const uint8_t ESPNOW_BROADCAST_ADDRESS[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

bool espNowReady = false;
uint32_t espNowSendOkCount = 0;
uint32_t espNowSendFailCount = 0;

bool gpsStarted = false;
bool bmeOnline = false;
bool bnoOnline = false;

uint32_t lastGpsSentenceMs = 0;
uint32_t lastBmeReadMs = 0;
uint32_t lastPacketMs = 0;
uint32_t lastDebugMs = 0;
uint32_t sequenceNumber = 0;

float pressureHpa = 0.0F;
float temperatureC = 0.0F;
float humidityPercent = 0.0F;
float baroAltitudeM = 0.0F;

float rollDeg = 0.0F;
float pitchDeg = 0.0F;
float headingDeg = 0.0F;

// =====================================================
// CRC-16/CCITT-FALSE
// =====================================================

uint16_t crc16Ccitt(const char *text)
{
    uint16_t crc = 0xFFFF;

    while (*text != '\0') {
        crc ^= static_cast<uint16_t>(
            static_cast<uint8_t>(*text++)
        ) << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

// =====================================================
// BNO085
// =====================================================

void quaternionToEuler(
    float qw,
    float qx,
    float qy,
    float qz,
    float &roll,
    float &pitch,
    float &yaw
) {
    const float sinrCosp = 2.0F * (qw * qx + qy * qz);
    const float cosrCosp = 1.0F - 2.0F * (qx * qx + qy * qy);
    roll = atan2f(sinrCosp, cosrCosp) * 180.0F / PI;

    float sinp = 2.0F * (qw * qy - qz * qx);
    sinp = constrain(sinp, -1.0F, 1.0F);
    pitch = asinf(sinp) * 180.0F / PI;

    const float sinyCosp = 2.0F * (qw * qz + qx * qy);
    const float cosyCosp = 1.0F - 2.0F * (qy * qy + qz * qz);
    yaw = atan2f(sinyCosp, cosyCosp) * 180.0F / PI;

    if (yaw < 0.0F) {
        yaw += 360.0F;
    }
}

bool enableBnoReports()
{
    return bno085.enableReport(
        SH2_ROTATION_VECTOR,
        50000                       // 20 Hz
    );
}

// =====================================================
// SENSOR SERVICE
// =====================================================

void serviceGps()
{
    if (!gpsStarted) {
        return;
    }

    gps.read();

    if (gps.newNMEAreceived()) {
        if (gps.parse(gps.lastNMEA())) {
            lastGpsSentenceMs = millis();
        }
    }
}

void serviceBno()
{
    if (!bnoOnline) {
        return;
    }

    if (bno085.wasReset()) {
        enableBnoReports();
    }

    while (bno085.getSensorEvent(&bnoEvent)) {
        if (bnoEvent.sensorId != SH2_ROTATION_VECTOR) {
            continue;
        }

        quaternionToEuler(
            bnoEvent.un.rotationVector.real,
            bnoEvent.un.rotationVector.i,
            bnoEvent.un.rotationVector.j,
            bnoEvent.un.rotationVector.k,
            rollDeg,
            pitchDeg,
            headingDeg
        );
    }
}

void serviceBme()
{
    if (!bmeOnline || millis() - lastBmeReadMs < BME_INTERVAL_MS) {
        return;
    }

    lastBmeReadMs = millis();

    temperatureC = bme280.readTemperature();
    humidityPercent = bme280.readHumidity();
    pressureHpa = bme280.readPressure() / 100.0F;
    baroAltitudeM = bme280.readAltitude(SEA_LEVEL_PRESSURE_HPA);
}

// =====================================================
// ESP-NOW SENSOR PACKET
//
// @AV1,
// sequence,featherUptime,
// gpsOnline,gpsFix,satellites,lat,lon,gsKt,track,gpsAltM,
// utcValid,hour,minute,second,
// bmeOnline,pressureHpa,tempC,humidityPct,baroAltM,
// bnoOnline,roll,pitch,heading
// *CRC16
// =====================================================

void transmitSensorPacket()
{
    if (millis() - lastPacketMs < PACKET_INTERVAL_MS) {
        return;
    }

    lastPacketMs = millis();
    sequenceNumber++;

    const bool gpsOnline =
        gpsStarted &&
        lastGpsSentenceMs != 0 &&
        millis() - lastGpsSentenceMs <= GPS_TIMEOUT_MS;

    const bool gpsFix = gpsOnline && gps.fix;
    const bool utcValid = gpsFix;

    const double latitude = gpsFix ? gps.latitudeDegrees : 0.0;
    const double longitude = gpsFix ? gps.longitudeDegrees : 0.0;

    const float groundSpeedKt = gpsFix ? gps.speed : 0.0F;
    const float trackDeg = gpsFix ? gps.angle : 0.0F;
    const float gpsAltitudeM = gpsFix ? gps.altitude : 0.0F;

    char payload[300];

    snprintf(
        payload,
        sizeof(payload),
        "AV1,%lu,%lu,%d,%d,%u,%.7f,%.7f,%.2f,%.2f,%.1f,%d,%u,%u,%u,%d,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%.2f,%.2f",
        static_cast<unsigned long>(sequenceNumber),
        static_cast<unsigned long>(millis()),
        gpsOnline ? 1 : 0,
        gpsFix ? 1 : 0,
        gpsFix ? static_cast<unsigned int>(gps.satellites) : 0U,
        latitude,
        longitude,
        groundSpeedKt,
        trackDeg,
        gpsAltitudeM,
        utcValid ? 1 : 0,
        utcValid ? static_cast<unsigned int>(gps.hour) : 0U,
        utcValid ? static_cast<unsigned int>(gps.minute) : 0U,
        utcValid ? static_cast<unsigned int>(gps.seconds) : 0U,
        bmeOnline ? 1 : 0,
        pressureHpa,
        temperatureC,
        humidityPercent,
        baroAltitudeM,
        bnoOnline ? 1 : 0,
        rollDeg,
        pitchDeg,
        headingDeg
    );

    const uint16_t crc = crc16Ccitt(payload);

    char packet[320];

    snprintf(
        packet,
        sizeof(packet),
        "@%s*%04X\r\n",
        payload,
        crc
    );

    if (!espNowReady) {
        espNowSendFailCount++;
        return;
    }

    const size_t packetLength = strlen(packet);

    if (packetLength > ESP_NOW_MAX_DATA_LEN) {
        espNowSendFailCount++;
        return;
    }

    const esp_err_t result = esp_now_send(
        ESPNOW_BROADCAST_ADDRESS,
        reinterpret_cast<const uint8_t *>(packet),
        packetLength
    );

    if (result == ESP_OK) {
        espNowSendOkCount++;
    } else {
        espNowSendFailCount++;
    }
}

void printDebugStatus()
{
    if (millis() - lastDebugMs < DEBUG_INTERVAL_MS) {
        return;
    }

    lastDebugMs = millis();

    const bool gpsOnline =
        gpsStarted &&
        lastGpsSentenceMs != 0 &&
        millis() - lastGpsSentenceMs <= GPS_TIMEOUT_MS;

    Serial.println();
    Serial.println("========== SENSOR COMPUTER ==========");

    Serial.print("ESP-NOW packet: AV1 #");
    Serial.println(sequenceNumber);

    Serial.print("ESP-NOW TX queued/fail: ");
    Serial.print(espNowSendOkCount);
    Serial.print("/");
    Serial.println(espNowSendFailCount);

    Serial.print("GPS link: ");
    Serial.println(gpsOnline ? "ONLINE" : "OFFLINE");

    Serial.print("GPS fix: ");
    Serial.println((gpsOnline && gps.fix) ? "VALID" : "NO FIX");

    if (gpsOnline && gps.fix) {
        Serial.print("SAT: ");
        Serial.println(gps.satellites);

        Serial.print("POS: ");
        Serial.print(gps.latitudeDegrees, 6);
        Serial.print(", ");
        Serial.println(gps.longitudeDegrees, 6);
    }

    Serial.print("BME280: ");
    Serial.println(bmeOnline ? "ONLINE" : "OFFLINE");

    if (bmeOnline) {
        Serial.print("PRESS: ");
        Serial.print(pressureHpa, 1);
        Serial.println(" hPa");
    }

    Serial.print("BNO085: ");
    Serial.println(bnoOnline ? "ONLINE" : "OFFLINE");

    if (bnoOnline) {
        Serial.print("ATT: R ");
        Serial.print(rollDeg, 1);
        Serial.print(" P ");
        Serial.print(pitchDeg, 1);
        Serial.print(" H ");
        Serial.println(headingDeg, 1);
    }

    Serial.println("=====================================");
}

// =====================================================
// ESP-NOW
// =====================================================

bool setupEspNow()
{
    WiFi.mode(WIFI_STA);
    delay(100);

    if (
        esp_wifi_set_channel(
            ESPNOW_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        ) != ESP_OK
    ) {
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t peer = {};

    memcpy(
        peer.peer_addr,
        ESPNOW_BROADCAST_ADDRESS,
        sizeof(ESPNOW_BROADCAST_ADDRESS)
    );

    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t addResult = esp_now_add_peer(&peer);

    if (
        addResult != ESP_OK &&
        addResult != ESP_ERR_ESPNOW_EXIST
    ) {
        return false;
    }

    return true;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

#if defined(PIN_I2C_POWER)
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, HIGH);
    delay(20);
#endif

    Wire.begin();
    Wire.setClock(100000);

    espNowReady = setupEspNow();

    Serial.println();
    Serial.println("MINI AVIONICS SENSOR COMPUTER — ESP-NOW LIVE");

    gpsStarted = gps.begin(GPS_I2C_ADDRESS);

    if (gpsStarted) {
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    }

    bmeOnline = bme280.begin(BME280_I2C_ADDRESS, &Wire);

    if (bno085.begin_I2C(0x4A, &Wire)) {
        bnoOnline = true;
    } else if (bno085.begin_I2C(0x4B, &Wire)) {
        bnoOnline = true;
    }

    if (bnoOnline) {
        enableBnoReports();
    }

    // Prime BME values immediately.
    if (bmeOnline) {
        temperatureC = bme280.readTemperature();
        humidityPercent = bme280.readHumidity();
        pressureHpa = bme280.readPressure() / 100.0F;
        baroAltitudeM = bme280.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    }

    Serial.print("GPS: ");
    Serial.println(gpsStarted ? "STARTED" : "OFFLINE");

    Serial.print("BME280: ");
    Serial.println(bmeOnline ? "ONLINE" : "OFFLINE");

    Serial.print("BNO085: ");
    Serial.println(bnoOnline ? "ONLINE" : "OFFLINE");

    Serial.print("ESP-NOW: ");
    Serial.println(espNowReady ? "READY" : "FAILED");

    Serial.print("ESP-NOW channel: ");
    Serial.println(ESPNOW_CHANNEL);

    Serial.print("Feather Wi-Fi MAC: ");
    Serial.println(WiFi.macAddress());

    Serial.println("Wireless AV1 packets: 10 Hz broadcast");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    serviceGps();
    serviceBno();
    serviceBme();

    transmitSensorPacket();
    printDebugStatus();

    delay(1);
}
