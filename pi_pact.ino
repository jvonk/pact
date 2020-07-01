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

//char _bdaStr[64];
typedef uint8_t uint8_t;
size_t const _devName_len = 32;
char _devName[_devName_len];
int const BD_ADDR_LEN = 6;

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

void _bdaStr2bda(char const * const src, uint8_t * const dst) {

    sscanf(src, "%hhd:%hhd:%hhd%hhd:%hhd:%hhd", dst, dst+1, dst+2, dst+3, dst+4, dst+5);
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {

    if (peripheral.hasManufacturerData()) {

        uint8_t manu_data[0xff];
        uint8_t manu_data_length;

        if (peripheral.getManufacturerData(manu_data, manu_data_length) && manu_data_length >= sizeof(iBeacon.header) && memcmp(manu_data, iBeacon.header, sizeof(iBeacon.header)) == 0) {
            char json[256];
            int len = 0;
            //char address[BD_ADDR_LEN * 2 + 1];
            //removeColons(peripheral.address().c_str(), address);
            char const * const address = peripheral.address().c_str();

            int8_t pwr = 0;
            if (manu_data_length >= sizeof(iBeacon)) {
                pwr =(int8_t)manu_data[offsetof(iBeacon_t, txpwr)];
            }

            //len += sprintf(json + len, "%s, ", _bdaStr);

            len += sprintf(json + len, "%s ", _devName);
            len += sprintf(json + len, "{ \"Address\": \"%s\"", address);
            len += sprintf(json + len, ", \"txPwr\": %d", pwr);
            len += sprintf(json + len, ", \"RSSI\": %d }", peripheral.rssi());
            len += sprintf(json + len, "\n");
            Serial.write(json);
        }
    }
}

void _reversedBda2bda(uint8_t const * const reversed_bda, uint8_t * const bda) {

    for (uint ii=0; ii < BD_ADDR_LEN; ii++) {
        bda[ii] = reversed_bda[BD_ADDR_LEN - ii - 1];
    }
}

void _bda2name(uint8_t const * const bda, char * const name, size_t name_len) {
	typedef struct {
		uint8_t const bda[BD_ADDR_LEN];
		char const * const name;
	} PACK8 knownBrd_t;
	static knownBrd_t knownBrds[] = {
        { {0x98, 0x4f, 0xee, 0x0d, 0x11, 0x53}, "curie101-1"},
        { {0x98, 0x4f, 0xee, 0x0d, 0x0e, 0x38}, "curie101-2"},
        { {0x98, 0x4f, 0xee, 0x0d, 0x0e, 0xc6}, "curie101-3"},
        { {0x98, 0x4f, 0xee, 0x0d, 0x04, 0x8e}, "curie101-4"},
        { {0x98, 0x4f, 0xee, 0x0d, 0x04, 0xa4}, "curie101-5"},
        { {0x98, 0x4f, 0xee, 0x0d, 0x0f, 0x34}, "curie101-6"},
        { {0x98, 0x4f, 0xee, 0x0d, 0x11, 0x4c}, "curie101-7"},
#if 0
		{ {0x53, 0x11, 0x0d, 0xee, 0x4f, 0x98}, "curie101-1"},
		{ {0x38, 0x0e, 0x0d, 0xee, 0x4f, 0x98}, "curie101-2"},
		{ {0xc6, 0x0e, 0x0d, 0xee, 0x4f, 0x98}, "curie101-3"},
		{ {0x8e, 0x04, 0x0d, 0xee, 0x4f, 0x98}, "curie101-4"},
		{ {0xa4, 0x04, 0x0d, 0xee, 0x4f, 0x98}, "curie101-5"},
		{ {0x34, 0x0f, 0x0d, 0xee, 0x4f, 0x98}, "curie101-6"},
		{ {0x4c, 0x11, 0x0d, 0xee, 0x4f, 0x98}, "curie101-7"}
#endif
	};

	for (uint ii=0; ii < ARRAYSIZE(knownBrds); ii++) {
		if (memcmp(bda, knownBrds[ii].bda, BD_ADDR_LEN) == 0) {
			strncpy(name, knownBrds[ii].name, name_len);
			return;
		}
	}
	snprintf(name, name_len, "curie101_%02x%02x",
			 bda[BD_ADDR_LEN-2], bda[BD_ADDR_LEN-1]);
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

    uint8_t const * reversed_bda = BLEUtils::bleGetLoalAddress()->val;    
    uint8_t bda[BD_ADDR_LEN];
    _reversedBda2bda(reversed_bda, bda);
    _bda2name(bda, _devName, _devName_len);

    BLE.setLocalName(_devName);
    BLE.setDeviceName(_devName);
    while (!Serial)
        /* wait for serial port to connect. Needed for native USB */ ;
}

void loop() {

    BLE.poll();
}
