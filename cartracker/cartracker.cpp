/**
 * @file      cartracker.ino
 * @author    Rui Romao
 * @license   MIT
 * @copyright Copyright (c) 2024
 * @date      2024-03-11
 * @note      
 */

#include "utilities.h"
#include <TinyGsmClient.h>
#include <esp_adc_cal.h>
#include <Arduino.h>

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#define SMS_TARGET  "+380xxxxxxxxx" //Change the number you want to send sms message

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
    #include <StreamDebugger.h>
    StreamDebugger debugger(SerialAT, SerialMon);
    TinyGsm modem(debugger);
#else
    TinyGsm modem(SerialAT);
#endif

// Define the serial console buffer for debug prints, if needed
char buf[256];

void initializeBluetooth() {
    
}

void initializeGSM() {

    // Set modem baud
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    Serial.println("Start modem...");
    delay(3000);

    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println(".");
        if (retry++ > 10) {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println();
    delay(200);
    Serial.print("GSM Modem init success, lets check the SIM card now.");

    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
        sim = modem.getSimStatus();
        switch (sim) {
        case SIM_READY:
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            break;
        default:
            break;
        }
        delay(1000);
    }
    
    //SIM7672G Can't set network mode
    #ifndef TINY_GSM_MODEM_SIM7672
        if (!modem.setNetworkMode(MODEM_NETWORK_AUTO)) {
            Serial.println("Set network mode failed!");
        }
        String mode = modem.getNetworkModes();
        Serial.print("Current network mode : ");
        Serial.println(mode);
    #endif

    // Check network registration status and network signal status
    int16_t sq ;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
        status = modem.getRegistrationStatus();
        switch (status) {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            sq = modem.getSignalQuality();
            Serial.printf("[%lu] Signal Quality:%d", millis() / 1000, sq);
            delay(1000);
            break;
        case REG_DENIED:
            Serial.println("Network registration was rejected, please check if the APN is correct");
            return ;
        case REG_OK_HOME:
            Serial.println("Online registration successful");
            break;
        case REG_OK_ROAMING:
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();
    
    Serial.printf("Registration Status:%d\n", status);
    delay(1000);

    String ueInfo;
    if (modem.getSystemInformation(ueInfo)) {
        Serial.print("Inquiring UE system information:");
        Serial.println(ueInfo);
    }

    if (!modem.enableNetwork()) {
        Serial.println("Enable network failed!");
    }

    delay(5000);

    String ipAddress = modem.getLocalIP();
    Serial.print("Network IP:"); Serial.println(ipAddress);
}

void initializeGPS() {
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO)) {
        Serial.print(".");
    }
    Serial.println();
    Serial.println("GPS Enabled");
}

String getGPSCoordinates() { 
    float lat2      = 0;
    float lon2      = 0;
    float speed2    = 0;
    float alt2      = 0;
    int   vsat2     = 0;
    int   usat2     = 0;
    float accuracy2 = 0;
    int   year2     = 0;
    int   month2    = 0;
    int   day2      = 0;
    int   hour2     = 0;
    int   min2      = 0;
    int   sec2      = 0;
    uint8_t    fixMode   = 0;

    for (int8_t i = 15; i; i--) {
        Serial.println("Requesting current GPS/GNSS/GLONASS location");
        if (modem.getGPS(&fixMode, &lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                         &year2, &month2, &day2, &hour2, &min2, &sec2)) {

            Serial.print("FixMode:"); Serial.println(fixMode);
            Serial.print("Latitude:"); Serial.print(lat2, 6); Serial.print("\tLongitude:"); Serial.println(lon2, 6);
            Serial.print("Speed:"); Serial.print(speed2); Serial.print("\tAltitude:"); Serial.println(alt2);
            Serial.print("Visible Satellites:"); Serial.print(vsat2); Serial.print("\tUsed Satellites:"); Serial.println(usat2);
            Serial.print("Accuracy:"); Serial.println(accuracy2);

            Serial.print("Year:"); Serial.print(year2);
            Serial.print("\tMonth:"); Serial.print(month2);
            Serial.print("\tDay:"); Serial.println(day2);

            Serial.print("Hour:"); Serial.print(hour2);
            Serial.print("\tMinute:"); Serial.print(min2);
            Serial.print("\tSecond:"); Serial.println(sec2);
            break;
        } else {
            Serial.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
            delay(15000L);
        }
    }

    Serial.println("Retrieving GPS/GNSS/GLONASS location again as a string");
    String gps_raw = modem.getGPSraw();
    Serial.print("GPS/GNSS Based Location String:");
    Serial.println(gps_raw);

    return gps_raw;
}

uint16_t readBatteryLevel() {
    
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    uint16_t battery_voltage = esp_adc_cal_raw_to_voltage(analogRead(BOARD_BAT_ADC_PIN), &adc_chars) * 2;
    #ifdef BOARD_SOLAR_ADC_PIN
            uint16_t solar_voltage = esp_adc_cal_raw_to_voltage(analogRead(BOARD_SOLAR_ADC_PIN), &adc_chars) * 2;
    #else
            uint16_t solar_voltage = 0;
    #endif

    snprintf(buf, 256, "Battery:%umV \tSolar:%umV", battery_voltage, solar_voltage);
    Serial.println(buf);

    //TODO return either battery or solar voltage
    return battery_voltage;

}


void setup()
{
    Serial.begin(115200);
    // Turn on DC boost to power on the modem
    #ifdef BOARD_POWERON_PIN
        pinMode(BOARD_POWERON_PIN, OUTPUT);
        digitalWrite(BOARD_POWERON_PIN, HIGH);
    #endif

    // Set modem reset pin ,reset modem
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

    // Set ring pin input
    pinMode(MODEM_RING_PIN, INPUT_PULLUP);

    //initializeGSM();
    initializeGPS();


    //Send text with GPS coordinates to SMS_TARGET
    //if (modem.sendSMS(SMS_TARGET, gps_raw)) {
    //    Serial.println("SMS sent successfully");    
    //} else {
    //    Serial.println("SMS failed to send");
    //}

}

void loop() {
    // Wait 5 seconds before getting the coordinates again
    delay(5000);
    String gps_raw = getGPSCoordinates();

    //print gps coordinates on the serial monitor
    Serial.println(gps_raw);

    if (SerialAT.available()) {
        Serial.write(SerialAT.read());
    }
    if (Serial.available()) {
        SerialAT.write(Serial.read());
    }

    //if (modem.sendSMS(SMS_TARGET, gps_raw)) {
    //    Serial.println("SMS sent successfully");    
    //} else {
    //    Serial.println("SMS failed to send");
    //}
}

