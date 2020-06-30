#include <CurieBLE.h> // #include <ArduinoBLE.h>
#include <internal/BLEUtils.h>

char name[] = "arduino_";
unsigned char* address;
unsigned char iBeaconData[] = {
  0x4C, 0x00, 0x02, 0x15, //Apple iBeacon
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //blank UUID
  0x01, 0x00, // major
  0x01, 0x00, // minor
  0xF6 // meaured power at 1 meter: 0xF6 = -75 dBm
};

void setup() {
  Serial.begin(9600);
  for (int i = 4; i < 20; i++) {
    iBeaconData[i]=rand()%0xFF;
    if (i==10) {
      iBeaconData[i]=(iBeaconData[i]&0x0F)|0x40;
    } else if (i==12) {
      iBeaconData[i]=(iBeaconData[i]&0x3F)|0x80;
    }
  }
  BLE.begin();
  BLE.setManufacturerData(iBeaconData, sizeof(iBeaconData));
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);
  BLE.scan(true);
  BLE.advertise();
  address = BLEUtils::bleGetLoalAddress()->val;
  char* name_ptr = name+strlen(name);
  name_ptr+=sprintf(name_ptr, "%02X", (unsigned int)address[4]);
  name_ptr+=sprintf(name_ptr, "%02X", (unsigned int)address[5]);
  BLE.setLocalName(name);
  BLE.setDeviceName(name);
  while (!Serial);
}

void loop() {
  BLE.poll();
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  if (peripheral.hasManufacturerData()) {
    unsigned char manu_data[255];
    unsigned char manu_data_length;
    bool success = peripheral.getManufacturerData(manu_data, manu_data_length);
    if (success && memcmp(manu_data, "\x4c\x00\x02\x15", 4) == 0) {
      Serial.write(address, 6);
      unsigned char mac[6];
      sscanf(peripheral.address().c_str(), "%hhu:%hhu:%hhu:%hhu:%hhu:%hhu", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
      Serial.write(mac, 6);
      Serial.write((unsigned char)peripheral.rssi());
      Serial.print('\n');
    }
  }
}
