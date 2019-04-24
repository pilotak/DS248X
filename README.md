# DS2482
A OneWire library using the DS2482 (1-Wire Master) for mbed. Supports both DS2482-100 (Single channel) and DS2482-800 (8-channel).

## Basic example
```cpp
#include "mbed.h"
#include "DS2482.h"

I2C i2c(PB_7, PB_6);
DS2482 oneWire(&i2c);

int main() {
    if (oneWire.init()) {
        printf("At least one device on the bus: %i\n", oneWire.reset());

    } else {
        printf("Could not init\n");
    }

    return 0;
}
```

## Example searching the bus
```cpp
#include "mbed.h"
#include "DS2482.h"

I2C i2c(PB_7, PB_6);
DS2482 oneWire(&i2c);

int main() {
    char rom[8];

    if (oneWire.init()) {
        oneWire.set_config(DS2482::ActivePullUp);

        while (oneWire.search(rom)) {
            printf("Device on the bus: ");

            for (int i = 0; i < 8; ++i) {
                printf("%02X ", rom[i]);
            }

            printf("\n");
        }

    } else {
        printf("Could not init\n");
    }

    return 0;
}
```

## Example reading DS18B20 at channel 6
```cpp
#include "mbed.h"
#include "DS2482.h"

DS2482 oneWire(PB_7, PB_6);

int main() {
    char rom[8];
    char data[9];

    if (oneWire.init()) {
        oneWire.set_config(DS2482::StrongPullUp);

        if (oneWire.select_channel(6)) {
            if (oneWire.search(rom)) {
                oneWire.reset();
                oneWire.select(rom);
                oneWire.write(0x44);  // start conversion

                ThisThread::sleep_for(1000);

                oneWire.reset();
                oneWire.select(rom);
                oneWire.write(0xBE);  // Read Scratchpad

                oneWire.read_bytes(data, 9);

                if (oneWire.crc8(data, 9)) {
                    int16_t raw = (data[1] << 8) | data[0];

                    if (rom[0] == 0x10) {  // DS18S20
                        raw = raw << 3;

                        if (data[7] == 0x10) {
                            raw = (raw & 0xFFF0) + 12 - data[6];
                        }

                    } else if (rom[0] == 0x28) {  // DS18B20
                        char cfg = (data[4] & 0x60);  // default is 12 bit resolution, 750 ms conversion time

                        if (cfg == 0x00) raw = raw & ~7;       // 9 bit resolution, 93.75 ms
                        else if (cfg == 0x20) raw = raw & ~3;  // 10 bit res, 187.5 ms
                        else if (cfg == 0x40) raw = raw & ~1;  // 11 bit res, 375 ms

                    } else {
                        printf("Not a temperature sensor\n");
                    }

                    printf("Temperature: %.2f\n", raw / 16.0);

                } else {
                    printf("CRC8 failed\n");
                }
            }

        } else {
            printf("Select channel failed\n");
        }

    } else {
        printf("Could not init\n");
    }

    return 0;
}
```
