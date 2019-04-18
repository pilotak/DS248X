/*
MIT License

Copyright (c) 2019 Pavel Slama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "mbed.h"
#include "DS2482.h"

DS2482::DS2482(I2C * i2c_obj, char address):
    _address(address),
    _config(UCHAR_MAX),
    _callback(NULL) {
    _i2c = i2c_obj;
}

DS2482::DS2482(PinName sda, PinName scl, char address, uint32_t frequency):
    _address(address),
    _config(UCHAR_MAX),
    _callback(NULL) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
}

DS2482::~DS2482(void) {
    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool DS2482::init() {
    if (_i2c) {
        _config = get_config();

        if (_config != UCHAR_MAX) {
            return true;
        }
    }

    return false;
}

void DS2482::device_reset() {
    write(DS2482_COMMAND_RESET);
}

char DS2482::get_config() {
    if (set_read_pointer(DS2482_POINTER_CONFIG)) {
        return read();
    }

    return UCHAR_MAX;
}

bool DS2482::send_config() {
    char buf[2];

    buf[0] = DS2482_COMMAND_WRITECONFIG;
    buf[1] = _config | (~_config) << 4;

    if (write_bytes(buf, 2)) {
        return true;
    }

    return false;
}

bool DS2482::set_config(DS2482_config type) {
    _config |= type;
    return send_config();
}

bool DS2482::clear_config(DS2482_config type) {
    _config &= ~(type);
    return send_config();
}

bool DS2482::select_channel(char channel) {
    char buf[2];
    // char read_channel;
    // read_channel = (channel | (~channel) << 3) & ~(1 << 6);

    channel |= (~channel) << 4;

    buf[0] = DS2482_COMMAND_CHSL;
    buf[1] = channel;

    if (write_bytes(buf, 2)) {
        _config = get_config();

        if (_config != UCHAR_MAX) {
            return true;
        }
    }

    return false;
}

bool DS2482::set_read_pointer(char read_pointer) {
    char buf[2];
    buf[0] = DS2482_COMMAND_SRP;
    buf[1] = read_pointer;

    if (write_bytes(buf, 2)) {
        return true;
    }

    return false;
}

bool DS2482::reset() {
    char status = UCHAR_MAX;

    wait_busy();

    bool spu = _config & DS2482_CONFIG_SPU;

    if (spu && !clear_config(StrongPullUp)) {
        return false;
    }

    wait_busy();

    if (write(DS2482_COMMAND_RESETWIRE)) {
        status = wait_busy();

        if (status != UCHAR_MAX) {
            if (spu && !set_config(StrongPullUp)) {
                return false;
            }

            return (status & DS2482_STATUS_PPD);
        }
    }

    return false;
}

char DS2482::read() {
    char buf[1];

    if (read_bytes(buf, 1)) {
        return buf[0];
    }

    return UCHAR_MAX;
}

bool DS2482::read_bit() {
    write_bit(1);
    uint8_t status = wait_busy();

    return status & DS2482_STATUS_SBR ? 1 : 0;
}

bool DS2482::read_bytes(char* data, uint16_t len) {
    int32_t ack;

    _i2c->lock();
    ack = _i2c->read(_address, data, len);
    _i2c->unlock();

    if (ack == 0) {
        return true;
    }

    return false;
}


bool DS2482::write(char data) {
    char buf[1];
    buf[0] = data;

    return write_bytes(buf, 1);
}

bool DS2482::write_bit(bool data) {
    char buf[2];
    wait_busy();

    buf[0] = DS2482_COMMAND_SINGLEBIT;
    buf[1] = data ? 0x80 : 0x00;

    return write_bytes(buf, 2);
}

bool DS2482::write_bytes(const char* data, uint16_t len) {
    int32_t ack;

    _i2c->lock();
    ack = _i2c->write(_address, data, len);
    _i2c->unlock();

    if (ack == 0) {
        return true;
    }

    return false;
}


char DS2482::search(char *address) {
    return 0;
}

void DS2482::reset_search() {
    /*searchLastDiscrepancy = 0;
    searchLastDeviceFlag = 0;

    memset(searchAddress, 0, sizeof(searchAddress));*/
}

void DS2482::select(const char rom[8]) {

}

char DS2482::wait_busy() {
    char status = UCHAR_MAX;
    bool cb_sent = false;

    for (char i = 0; i < 20; i++) {
        if (set_read_pointer(DS2482_POINTER_STATUS)) {
            status = read();

            // debug("Status: %c%c%c%c%c%c%c%c\n",
            //       (status & 0x80 ? '1' : '0'),
            //       (status & 0x40 ? '1' : '0'),
            //       (status & 0x20 ? '1' : '0'),
            //       (status & 0x10 ? '1' : '0'),
            //       (status & 0x08 ? '1' : '0'),
            //       (status & 0x04 ? '1' : '0'),
            //       (status & 0x02 ? '1' : '0'),
            //       (status & 0x01 ? '1' : '0'));

            if ((status & DS2482_STATUS_SD) && !cb_sent) {
                if (_callback) {
                    _callback.call(DS2482_STATUS_SD);
                }

                cb_sent = true;

            } else if ((status & DS2482_STATUS_RST) && !cb_sent) {
                if (_callback) {
                    _callback.call(DS2482_STATUS_RST);
                }

                cb_sent = true;
            }

            if (!(status & DS2482_STATUS_BUSY)) {
                // debug("1-Wire ready\n");
                break;
            }

        } else {
            return UCHAR_MAX;
        }

        ThisThread::sleep_for(1);
    }

    return status;
}

void DS2482::attach(Callback<void(uint8_t)> function) {
    if (function) {
        _callback = function;

    } else {
        _callback = NULL;
    }
}
