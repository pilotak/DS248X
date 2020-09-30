/*
MIT License

Copyright (c) 2020 Pavel Slama

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
#include "DS248X.h"

DS248X::DS248X(uint8_t address):
    _address(address) {
}

DS248X::DS248X(PinName sda, PinName scl, uint8_t address, uint32_t frequency):
    _address(address) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
}

DS248X::~DS248X(void) {
    if (_i2c == reinterpret_cast<I2C *>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool DS248X::init(I2C *i2c_obj) {
    if (i2c_obj != nullptr) {
        _i2c = i2c_obj;
    }

    MBED_ASSERT(_i2c);

    if (!waitBusy()) {
        return false;
    }

    if (!getConfig()) {
        tr_error("Could not get config");
        return false;
    }

    tr_info("Init successful, config: %02X", _config);
    return true;
}

bool DS248X::setConfig(ds248x_config_t type) {
    _config |= type;
    return sendConfig();
}

bool DS248X::clearConfig(ds248x_config_t type) {
    _config &= ~(type);
    return sendConfig();
}

bool DS248X::selectChannel(uint8_t channel) {
    char buf[2];
    uint8_t read_channel = (channel | (~channel) << 3) & ~(1 << 6);

    if (channel >= 8) {
        tr_error("Invalid channel: %u", channel);
        return false;
    }

    channel |= (~channel) << 4;

    buf[0] = (char)CMD_CHSL;
    buf[1] = channel;

    if (!deviceWriteBytes(buf, sizeof(buf))) {
        return false;
    }

    // TODO
    /*if (!deviceReadBytes(buf)) {
        return false;
    }*/

    if (buf[0] != read_channel) {
        tr_error("Channel not selected");
        return false;
    }

    resetSearch();

    tr_info("Channel set to: %u", channel);
    return true;
}

bool DS248X::deviceReset() {
    char buf[1];
    buf[0] = (char)CMD_DRST;

    tr_info("Device reset");

    if (!deviceWriteBytes(buf, sizeof(buf))) {
        return false;
    }

    return true;
}

char DS248X::computeCRC(const char *data, size_t len) {
    MbedCRC<0x31, 8> ct(0, 0, true, true);
    uint32_t crc = 0;

    if (ct.compute(data, len, &crc) == 0) {
        return static_cast<char>(crc);
    }

    return UCHAR_MAX;
}

bool DS248X::crc8(const char *data, size_t len) {
    char crc = computeCRC(data, len - 1);

    if (data[len - 1] == crc) {
        tr_debug("Checksum OK");
        return true;
    }

    tr_error("Checksum failed");
    return false;
}

void DS248X::attach(Callback<void(char)> function) {
    if (function) {
        _callback = function;

    } else {
        _callback = nullptr;
    }
}

bool DS248X::writeBytes(const char *data, size_t len) {
    if (data == nullptr || len == 0) {
        tr_error("Invalid input data");
        return false;
    }

    for (size_t i = 0; i < len; ++i) {
        if (!write(data[i])) {
            return false;
        }
    }

    return true;
}

bool DS248X::readBytes(char *buffer, size_t len) {
    if (buffer == nullptr || len == 0) {
        tr_error("Invalid input data");
        return false;
    }

    char buf[1];

    for (size_t i = 0; i < len; ++i) {
        if (!read(buf)) {
            return false;
        }

        buffer[i] = buf[0];
    }

    return true;
}

bool DS248X::writeBit(bool bit) {
    char buf[2];

    if (!waitBusy()) {
        return false;
    }

    buf[0] = (char)CMD_1WSB;
    buf[1] = bit ? 0x80 : 0x00;

    return deviceWriteBytes(buf, sizeof(buf));
}

bool DS248X::readBit() {
    char buf[1];

    if (!writeBit(true)) {
        return false;
    }

    if (!waitBusy(buf)) {
        return false;
    }

    return buf[0] & DS248X_STATUS_SBR;
}

bool DS248X::reset() {
    char buf[1];

    tr_info("Reset");

    if (!waitBusy()) {
        return false;
    }

    bool spu = _config & DS248X_CONFIG_SPU;

    if (spu && !clearConfig(StrongPullUp)) {
        return false;
    }

    if (!waitBusy()) {
        return false;
    }

    buf[0] = (char)CMD_1WRS;

    if (!deviceWriteBytes(buf, 1)) {
        return false;
    }

    if (!waitBusy(buf)) {
        return false;
    }

    if (spu && !setConfig(StrongPullUp)) {
        return false;
    }

    return (buf[0] & DS248X_STATUS_PPD);
}

bool DS248X::skip() {
    tr_info("Skip");

    if (!waitBusy()) {
        return false;
    }

    return write(WIRE_COMMAND_SKIP);
}

bool DS248X::select(const char *rom) {
    tr_info("Selecting: %s", tr_array(reinterpret_cast<const uint8_t *>(rom), 8));

    if (!write(WIRE_COMMAND_SELECT)) {
        return false;
    }

    return writeBytes(rom, 8);
}

bool DS248X::search(char *rom) {
    char buf[2];
    uint8_t rom_byte_counter = 0;
    uint8_t id_bit_counter = 1;
    uint8_t last_zero = 0;
    uint8_t rom_byte_mask = 1;
    bool id_bit;
    bool cmp_id_bit;
    bool search_direction;

    if (_last_device_flag || !reset()) {
        tr_warning("No %sdevices on the bus", _last_device_flag ? "more " : "");
        resetSearch();
        return false;
    }

    if (!write(WIRE_COMMAND_SEARCH)) {
        return false;
    }

    if (!waitBusy()) {
        resetSearch();
        return false;
    }

    while (rom_byte_counter < 8) {
        if (id_bit_counter < _last_discrepancy) {
            search_direction = ((rom[rom_byte_counter] & rom_byte_mask) > 0);

        } else {
            search_direction = (id_bit_counter == _last_discrepancy);
        }

        buf[0] = (char)CMD_1WT;
        buf[1] = search_direction ? 0x80 : 0x00;

        if (!deviceWriteBytes(buf, 2)) {
            resetSearch();
            return false;
        }

        if (!waitBusy(buf)) {
            resetSearch();
            return false;
        }

        id_bit = (buf[0] & DS248X_STATUS_SBR);
        cmp_id_bit = (buf[0] & DS248X_STATUS_TSB);
        search_direction = (buf[0] & DS248X_STATUS_DIR);

        // check for no devices
        if (id_bit && cmp_id_bit) {
            resetSearch();
            return false;
        }

        if (!id_bit && !cmp_id_bit && !search_direction) {
            last_zero = id_bit_counter;
        }

        if (search_direction) {
            rom[rom_byte_counter] |= rom_byte_mask;

        } else {
            rom[rom_byte_counter] &= ~rom_byte_mask;
        }

        id_bit_counter++;
        rom_byte_mask <<= 1;

        if (rom_byte_mask == 0) {
            rom_byte_counter++;
            rom_byte_mask = 1;
        }
    }

    _last_discrepancy = last_zero;

    // check for last device
    if (_last_discrepancy == 0) {
        _last_device_flag = true;
    }

    // compare CRC
    if (!crc8(rom, sizeof(rom))) {
        return false;
    }

    tr_info("Found device: %s", tr_array(reinterpret_cast<uint8_t *>(rom), sizeof(rom)));
    return true;
}

void DS248X::resetSearch() {
    tr_debug("Seach reset");

    _last_discrepancy = 0;
    _last_device_flag = false;
}

bool DS248X::deviceWriteBytes(const char *data, size_t len) {
    int32_t ack;

    tr_debug("Sending[%u]: %s", len, tr_array(reinterpret_cast<const uint8_t *>(data), len));

    _i2c->lock();
    ack = _i2c->write(_address, data, len);
    _i2c->unlock();

    if (ack != 0) {
        tr_error("Error write");
        return false;
    }

    return true;
}

bool DS248X::deviceReadBytes(ds248x_pointer_t address, char *buffer, size_t len) {
    int32_t ack;

    if (!setReadPointer(address)) {
        return false;
    }

    _i2c->lock();
    ack = _i2c->read(_address, buffer, len);
    _i2c->unlock();

    if (ack != 0) {
        tr_error("Error read");
        return false;
    }

    tr_debug("Read:[%u]: %s", len, tr_array(reinterpret_cast<uint8_t *>(buffer), len));

    return true;
}

bool DS248X::waitBusy(char *status) {
    static bool cb_sent[2] = {false, false};
    char buf[1];

    for (auto i = 0; i < MBED_CONF_DS248X_RETRY; i++) {
        if (!deviceReadBytes(POINTER_STATUS, buf, sizeof(buf))) {
            return false;
        }

        // tr_debug("Status: %c%c%c%c%c%c%c%c",
        //       (buf[0] & 0x80 ? '1' : '0'),
        //       (buf[0] & 0x40 ? '1' : '0'),
        //       (buf[0] & 0x20 ? '1' : '0'),
        //       (buf[0] & 0x10 ? '1' : '0'),
        //       (buf[0] & 0x08 ? '1' : '0'),
        //       (buf[0] & 0x04 ? '1' : '0'),
        //       (buf[0] & 0x02 ? '1' : '0'),
        //       (buf[0] & 0x01 ? '1' : '0'));

        if (buf[0] & DS248X_STATUS_SD) {
            if (!cb_sent[0]) {
                tr_warning("Short condition detected");

                if (_callback) {
                    _callback.call(DS248X_STATUS_SD);
                }

                cb_sent[0] = true;
            }

        } else {
            cb_sent[0] = false;
        }

        if (buf[0] & DS248X_STATUS_RST) {
            if (!cb_sent[1]) {
                tr_warning("Reset condition detected");

                if (_callback) {
                    _callback.call(DS248X_STATUS_RST);
                }

                cb_sent[1] = true;
            }

        } else {
            cb_sent[1] = false;
        }

        if (!(buf[0] & DS248X_STATUS_BUSY)) {
            tr_debug("1-Wire ready");

            if (status) {
                memcpy(status, buf, sizeof(buf));
            }

            return true;
        }

        ThisThread::sleep_for(1ms);
    }

    tr_error("Device not ready");

    deviceReset();
    return false;
}

bool DS248X::sendConfig() {
    char buf[2];

    buf[0] = (char)CMD_WCFG;
    buf[1] = _config | (~_config) << 4;

    tr_info("Sending config: %02X", _config);

    if (!deviceWriteBytes(buf, sizeof(buf))) {
        return false;
    }

    return true;
}

bool DS248X::getConfig() {
    char buf[1];

    if (!deviceReadBytes(POINTER_CONFIG, buf, sizeof(buf))) {
        return false;
    }

    _config = buf[0];
    tr_info("Got config: %02X", _config);

    return true;
}

bool DS248X::write(char data) {
    char buf[2];
    buf[0] = (char)CMD_1WWB;
    buf[1] = data;

    if (!waitBusy()) {
        return false;
    }

    return deviceWriteBytes(buf, sizeof(buf));
}

bool DS248X::read(char *buffer) {
    if (!waitBusy()) {
        return false;
    }

    buffer[0] = (char)CMD_1WRB;

    if (!deviceWriteBytes(buffer, 1)) {
        return false;
    }

    if (!waitBusy()) {
        return false;
    }

    return deviceReadBytes(POINTER_DATA, buffer, 1);
}

bool DS248X::setReadPointer(ds248x_pointer_t address) {
    char buf[2];
    buf[0] = CMD_SRP;
    buf[1] = (char)address;

    tr_debug("Setting read pointer to: %02X", address);

    if (!deviceWriteBytes(buf, sizeof(buf))) {
        tr_error("Setting read pointer failed");
        return false;
    }

    return true;
}