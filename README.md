# 1-Wire library
[![Framework Badge mbed](https://img.shields.io/badge/framework-mbed-008fbe.svg)](https://os.mbed.com/)
[![build](https://github.com/pilotak/DS248X/actions/workflows/build.yml/badge.svg)](https://github.com/pilotak/DS248X/actions/workflows/build.yml)

Supports following ICs:
- DS2484 *(single channel)*
- DS2482-100 *(single channel)*
- DS2482-800 *(8-channel)*

## Basic example
```cpp
#include "mbed.h"
#include "DS248X.h"

DS248X oneWire(I2C_SDA, I2C_SCL);

int main() {
    if (!oneWire.init()) {
        debug("Init failed\n");
        return 0;
    }

    debug("At least one device on the bus: %u\n", oneWire.reset());

    return 0;
}
```

## Example searching the bus
```cpp
#include "mbed.h"
#include "DS248X.h"

DS248X oneWire(I2C_SDA, I2C_SCL);

bool oneWireInit() {
    if (!oneWire.init()) {
        debug("Init failed\n");
        return false;
    }

    if (!oneWire.setConfig(DS248X::ActivePullUp)) {
        debug("Config failed\n");
        return false;
    }

    return true;
}

void oneWireCb(char error) {
    if (error & DS248X_CB_RESET_CONDITION) {
        debug("1-Wire reset\n");

    } else if (error & DS248X_CB_SHORT_CONDITION) {
        debug("1-Wire short\n");

    }  else if(error & DS248X_CB_DEVICE_RESET_NEEDED){
        debug("Device reset needed\n");
        oneWire.deviceReset();
        oneWireInit();
    }
}

int main() {
    char rom[8];
    uint8_t device_count = 0;

    oneWire.attach(oneWireCb);

    if (!oneWireInit()) {
        return 0;
    }

    while (1) {
        while (oneWire.search(rom)) {
            device_count++;

            debug("Found device: ");

            for (size_t i = 0; i < sizeof(rom); i++) {
                debug("%02X", rom[i]);
            }

            debug("\n");
        }

        oneWire.resetSearch();

        debug("Total devices on the bus: %u\n\n", device_count);
        device_count = 0;

        ThisThread::sleep_for(5s);
    }
}
```

## Example reading DS18B20 and passing I2C object
```cpp
#include "mbed.h"
#include "DS248X.h"

I2C i2c(I2C_SDA, I2C_SCL);
DS248X oneWire;

int main() {
    char rom[8];
    char data[9];

    if (!oneWire.init(&i2c)) {
        debug("Init failed\n");
        return 0;
    }

    if (!oneWire.setConfig(DS248X::ActivePullUp)) {
        debug("Config failed\n");
        return 0;
    }

    while (1) {
        if (!oneWire.search(rom)) {
            debug("No devices on the bus\n");
            ThisThread::sleep_for(1s);
            continue;
        }

        oneWire.resetSearch();

        if (rom[0] != 0x10 && rom[0] != 0x28) { // DS18S20 or DS18B20
            debug("Not a temperature sensor\n");
            continue;
        }

        debug("Temperature sensor found\n");

        while (1) {
            if (!oneWire.reset()) {
                debug("Sensor is no longer on the bus\n");
                break;
            }

            oneWire.select(rom);

            // start conversion
            data[0] = 0x44;
            oneWire.writeBytes(data, 1, true); // use SPU if in parasitic mode

            // wait for conversion
            ThisThread::sleep_for(750ms); // default conversion (12bit) time is 750ms

            oneWire.reset();
            oneWire.select(rom);

            // Read Scratchpad
            data[0] = 0xBE;
            oneWire.writeBytes(data, 1, true); // use SPU if in parasitic mode
            oneWire.readBytes(data, 9, true); // use SPU if in parasitic mode

            if (!oneWire.crc8(data, 9)) {
                debug("Invalid CRC\n");
                continue;
            }

            int16_t raw = (data[1] << 8) | data[0];

            for (auto i = 0; i < 9; i++) {
                printf("%02X ", data[i]);
            }

            printf("\n");

            switch (rom[0]) {
                case 0x10: { // DS18S20
                    raw = raw << 3;

                    if (data[7] == 0x10) {
                        raw = (raw & 0xFFF0) + 12 - data[6];
                    }
                }
                break;

                case 0x28: { // DS18B20
                    char cfg = (data[4] & 0x60);  // default is 12 bit resolution, 750 ms conversion time

                    if (cfg == 0x00) { // 9 bit resolution, 93.75 ms
                        raw &= ~7;

                    } else if (cfg == 0x20) { // 10 bit res, 187.5 ms
                        raw &= ~3;

                    } else if (cfg == 0x40) { // 11 bit res, 375 ms
                        raw &= ~1;
                    }
                }

                default:
                    break;
            }

            printf("Temperature: %i *mC\n", ((int32_t)raw * 100) >> 4);
        }
    }
}
```
