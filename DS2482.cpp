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
    _callback(NULL),
    _last_discrepancy(0),
    _last_device_flag(false) {
    _i2c = i2c_obj;
}

DS2482::DS2482(PinName sda, PinName scl, char address, uint32_t frequency):
    _address(address),
    _config(UCHAR_MAX),
    _callback(NULL),
    _last_discrepancy(0),
    _last_device_flag(false) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
}

DS2482::~DS2482(void) {
    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool DS2482::init(I2C * i2c_obj) {
    if (i2c_obj != NULL) {
        _i2c = i2c_obj;
    }

    if (_i2c) {
        if (wait_busy() != UCHAR_MAX) {
            _config = get_config();

            if (_config != UCHAR_MAX) {
                return true;
            }
        }
    }

    return false;
}

void DS2482::device_reset() {
    device_write(DS2482_COMMAND_RESET);
}

char DS2482::device_read() {
    char buf[1];

    if (device_read_bytes(buf, 1)) {
        return buf[0];
    }

    return UCHAR_MAX;
}

bool DS2482::device_read_bytes(char* data, uint16_t len) {
    int32_t ack;

    _i2c->lock();
    ack = _i2c->read(_address, data, len);
    _i2c->unlock();

    if (ack == 0) {
        return true;
    }

    return false;
}

bool DS2482::device_write(char data) {
    return device_write_bytes(&data, 1);
}

bool DS2482::device_write_bytes(const char* data, uint16_t len) {
    int32_t ack;

    _i2c->lock();
    ack = _i2c->write(_address, data, len);
    _i2c->unlock();

    if (ack == 0) {
        return true;
    }

    return false;
}

char DS2482::get_config() {
    if (set_read_pointer(DS2482_POINTER_CONFIG)) {
        char config = device_read();

        if ((config & DS2482_CONFIG_PPM) == 0) {
            return config;
        }
    }

    return UCHAR_MAX;
}

bool DS2482::send_config() {
    char buf[2];

    buf[0] = DS2482_COMMAND_WRITECONFIG;
    buf[1] = _config | (~_config) << 4;

    if (device_write_bytes(buf, 2)) {
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
    char read_channel = (channel | (~channel) << 3) & ~(1 << 6);

    if (channel < 8) {
        channel |= (~channel) << 4;

        buf[0] = DS2482_COMMAND_CHSL;
        buf[1] = channel;

        if (device_write_bytes(buf, 2)) {
            if (device_read() == read_channel) {
                reset_search();
                reset();
                return true;
            }
        }
    }

    return false;
}

bool DS2482::set_read_pointer(char read_pointer) {
    char buf[2];
    buf[0] = DS2482_COMMAND_SRP;
    buf[1] = read_pointer;

    if (device_write_bytes(buf, 2)) {
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

    if (device_write(DS2482_COMMAND_RESETWIRE)) {
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
    wait_busy();

    if (device_write(DS2482_COMMAND_READBYTE)) {
        wait_busy();
        set_read_pointer(DS2482_POINTER_DATA);
        return device_read();
    }

    return UCHAR_MAX;
}

bool DS2482::read_bit() {
    write_bit(1);
    uint8_t status = wait_busy();

    return status & DS2482_STATUS_SBR ? 1 : 0;
}

bool DS2482::read_bytes(char* data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        data[i] = read();
    }

    return true;
}

bool DS2482::write_bit(bool bit) {
    char buf[2];
    wait_busy();

    buf[0] = DS2482_COMMAND_SINGLEBIT;
    buf[1] = bit ? 0x80 : 0x00;

    return device_write_bytes(buf, 2);
}

bool DS2482::write(char data) {
    char buf[2];
    buf[0] = DS2482_COMMAND_WRITEBYTE;
    buf[1] = data;

    wait_busy();

    return device_write_bytes(buf, 2);
}

bool DS2482::write_bytes(const char* data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        if (!write(data[i])) {
            return false;
        }
    }

    return true;
}

bool DS2482::skip() {
    wait_busy();

    return write(DS2482_WIRE_COMMAND_SKIP);
}

bool DS2482::search(char *address) {
    uint8_t direction;
    uint8_t last_zero = 0;
    char buf[2];

    if (!_last_device_flag) {
        if (!reset()) {
            reset_search();
            return false;
        }

        wait_busy();

        write(DS2482_WIRE_COMMAND_SEARCH);

        for (uint8_t i = 0; i < 64; i++) {
            uint8_t searchByte = i >> 3;
            uint8_t searchBit = 1 << (i & 7);

            if (i < _last_discrepancy) {
                direction = _search_address[searchByte] & searchBit;

            } else {
                direction = i == _last_discrepancy;
            }

            wait_busy();

            buf[0] = DS2482_COMMAND_TRIPLET;
            buf[1] = direction ? 0x80 : 0x00;

            if (device_write_bytes(buf, 2)) {
                uint8_t status = wait_busy();

                uint8_t id = status & DS2482_STATUS_SBR;
                uint8_t comp_id = status & DS2482_STATUS_TSB;
                direction = status & DS2482_STATUS_DIR;

                if (id && comp_id) {  // no devices on the bus
                    return false;

                } else {
                    if (!id && !comp_id && !direction) {
                        last_zero = i;
                    }
                }

                if (direction) {
                    _search_address[searchByte] |= searchBit;

                } else {
                    _search_address[searchByte] &= ~searchBit;
                }
            }
        }

        _last_discrepancy = last_zero;

        if (last_zero == 0) {
            _last_device_flag = true;
        }

        if (crc8(_search_address, sizeof(_search_address))) {
            memcpy(address, _search_address, sizeof(_search_address));
            return true;
        }
    }

    return false;
}

void DS2482::reset_search() {
    _last_discrepancy = 0;
    _last_device_flag = false;

    memset(_search_address, 0, sizeof(_search_address));
}

bool DS2482::select(const char rom[8]) {
    if (write(DS2482_WIRE_COMMAND_SELECT)) {
        return write_bytes(rom, 8);
    }

    return false;
}

char DS2482::wait_busy() {
    bool cb_sent[2] = {false, false};
    char status = UCHAR_MAX;

    for (uint16_t i = 0; i < 500; i++) {
        if (set_read_pointer(DS2482_POINTER_STATUS)) {
            status = device_read();

            // debug("Status: %c%c%c%c%c%c%c%c\n",
            //       (status & 0x80 ? '1' : '0'),
            //       (status & 0x40 ? '1' : '0'),
            //       (status & 0x20 ? '1' : '0'),
            //       (status & 0x10 ? '1' : '0'),
            //       (status & 0x08 ? '1' : '0'),
            //       (status & 0x04 ? '1' : '0'),
            //       (status & 0x02 ? '1' : '0'),
            //       (status & 0x01 ? '1' : '0'));

            if ((status & DS2482_STATUS_SD) && !cb_sent[0]) {
                if (_callback) {
                    _callback.call(DS2482_STATUS_SD);
                }

                cb_sent[0] = true;

            } else if ((status & DS2482_STATUS_RST) && !cb_sent[1]) {
                if (_callback) {
                    _callback.call(DS2482_STATUS_RST);
                }

                cb_sent[1] = true;
            }

            if (!(status & DS2482_STATUS_BUSY)) {
                // debug("1-Wire ready\n");
                break;
            }

        } else {
            return UCHAR_MAX;
        }

        // ThisThread::sleep_for(1);
        wait_us(20);
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

char DS2482::get_crc8(const char* data, uint8_t len) {
    MbedCRC<0x31, 8> ct(0, 0, true, true);
    uint32_t crc = 0;

    if (ct.compute((void *)data, len, &crc) == 0) {
        return static_cast<char>(crc);
    }

    return UCHAR_MAX;
}

bool DS2482::crc8(const char* data, uint8_t len) {
    char crc = get_crc8(data, len - 1);

    if (data[len - 1] == crc) {
        return true;
    }

    return false;
}
