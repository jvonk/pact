/**
 * @brief BLE iBeacon advertisement or scanning for Intel Curie based Arduino 101
 **/
// Copyright © 2020, Johan Vonk
// SPDX-License-Identifier: MIT

#include <arduino.h>
#include <CurieBLE.h>
#include <internal/BLEUtils.h>

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))
#define ALIGN( type ) __attribute__((aligned( __alignof__( type ) )))
#define PACK( type )  __attribute__((aligned( __alignof__( type ) ), packed ))
#define PACK8  __attribute__((aligned( __alignof__( uint8_t ) ), packed ))

typedef uint8_t uint8_t;
char myName[] = "arduino####";
uint8_t const * myMacAddress;

struct iBeacon_t {
    uint8_t header[4];
    uint8_t uuid[16];
    uint8_t major[2];
    uint8_t minor[2];
    uint8_t txpwr;
} PACK8;

static iBeacon_t iBeacon = {
    .header = {0x4c, 0x00, 0x02, 0x15}, // beacon type = Apple proximity
    .uuid = {},                         // proximity UUID defines the beacon region
    .major = {0x01, 0x00},              // major value from the beacon identity constraint that defines the beacon region
    .minor = {0x01, 0x00},              // minor value from the beacon identity constraint that defines the beacon region
    .txpwr = 0xF6                       // meaured power at 1 meter: 0xF6 = -75 dBm
};

#if 0
void removeColons(char const * src, char * dst) {

    do {
        if (*src != ':') {
            *dst++ = *src;
        }
    } while (*src++);
}
#endif

void bleCentralDiscoverHandler(BLEDevice peripheral) {

    if (peripheral.hasManufacturerData()) {


        uint8_t manu_data[0xff];
        uint8_t manu_data_length;

        if (peripheral.getManufacturerData(manu_data, manu_data_length) && manu_data_length >= sizeof(iBeacon.header) && memcmp(manu_data, iBeacon.header, sizeof(iBeacon.header)) == 0) {
            char json[256];
            int len = 0;
            //int const BD_ADDR_LEN = 6;
            //char address[BD_ADDR_LEN * 2 + 1];
            //removeColons(peripheral.address().c_str(), address);
            char const * const address = peripheral.address().c_str();

            int8_t pwr = 0;
            if (manu_data_length >= sizeof(iBeacon)) {
                pwr =(int8_t)manu_data[offsetof(iBeacon_t, txpwr)];
            }
            len += sprintf(json + len, "{ \"Address\": \"%s\"", address);
            len += sprintf(json + len, ", \"txPwr\": %d", pwr);
            len += sprintf(json + len, ", \"RSSI\": %d }", peripheral.rssi());
            len += sprintf(json + len, "\n");
            Serial.write(json);
        }
    }
}

void setup()
{
    Serial.begin(115200, SERIAL_8N1);
    for (uint8_t i = 0; i < sizeof(iBeacon.uuid); i++) {
        iBeacon.uuid[i] = rand() & 0xFF;
        if (i == 6) {
            iBeacon.uuid[i] = (iBeacon.uuid[i] & 0x0F) | 0x40;
        } else if (i == 8) {
            iBeacon.uuid[i] = (iBeacon.uuid[i] & 0x3F) | 0x80;
        }
    }
    BLE.setManufacturerData((uint8_t *)&iBeacon, sizeof(iBeacon));
    BLE.begin();
    BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);
    BLE.scan(true);
    BLE.advertise();
    myMacAddress = BLEUtils::bleGetLoalAddress()->val;

    char * hexAddr = strchr(myName, '#');
    sprintf(hexAddr, "%02X%02X", myMacAddress[4], myMacAddress[5]);
    BLE.setLocalName(myName);
    BLE.setDeviceName(myName);
    while (!Serial)
        /* wait for serial port to connect. Needed for native USB */ ;
}

void loop() {

    BLE.poll();
}

