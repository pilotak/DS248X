# DS2482
DS2482 library for mbed

```cpp
#include "mbed.h"
#include "DS2482.h"

I2C i2c(PB_7, PB_6);
DS2482 oneWire(&i2c);

int main() {
    if (oneWire.init()) {
        printf("reset: %i\n", oneWire.reset());

    } else {
        printf("Could not init\n");
    }

    return 0;
}
```
