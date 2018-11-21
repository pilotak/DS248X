/*
MIT License

Copyright (c) 2018 Pavel Slama

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

DS2482::DS2482(I2C * i2c_obj, EventQueue * queue, int8_t address):
    _i2c_addr(address),
    searchLastDiscrepancy(0),
    searchLastDeviceFlag(0),
    _try_counter(0),
    _config(0),
    _stage(Init) {
    _i2c = i2c_obj;
    _queue = queue;
}

DS2482::DS2482(PinName sda, PinName scl, EventQueue * queue, int8_t address, uint32_t frequency):
    _i2c_addr(address),
    searchLastDiscrepancy(0),
    searchLastDeviceFlag(0),
    _try_counter(0),
    _config(0),
    _stage(Init) {
    _i2c = new (_i2c_buffer) I2C(sda, scl);
    _i2c->frequency(frequency);
    _queue = queue;
}

DS2482::~DS2482(void) {
    if (_i2c == reinterpret_cast<I2C*>(_i2c_buffer)) {
        _i2c->~I2C();
    }
}

bool DS2482::init() {
    if (_i2c && _queue) {
        _wait_buf[0] = DS2482_COMMAND_SRP;
        _wait_buf[1] = DS2482_POINTER_STATUS;

        _config = getConfig();

        if (_config < 0xFF) {
            printf("config: %u\n", _config);
            resetSearch();
            _stage = Ready;
            return true;
        }
    }

    return false;
}

bool DS2482::reset() {
    /* TASK LIST
    - waitOnBusy
    - clear strong pull up if set
    - waitOnBusy
    - send reset command
    - waitOnBusy
    - get data from waitOnBusy, compare and return
     */

    uint32_t flag;
    bool pull_up = _config & StrongPullUp;

    /*if (pull_up && !clearConfig(StrongPullUp)) {
        return false;
    }*/

    _try_counter = 0;
    waitOnBusy();
    flag = _event.wait_any(ERROR_FLAG | READY_FLAG, DS2482_DEFAULT_TIMEOUT);

    if (flag == READY_FLAG) {
        _tx_buf[0] = DS2482_COMMAND_RESETWIRE;

        if (send(1, 0)) {
            flag = _event.wait_any(ERROR_FLAG | OK_FLAG, DS2482_DEFAULT_TIMEOUT);

            if (flag == OK_FLAG) {
                _try_counter = 0;
                waitOnBusy();
                flag = _event.wait_any(ERROR_FLAG | READY_FLAG, DS2482_DEFAULT_TIMEOUT);

                if (flag == READY_FLAG) {
                    printf("ret: %u\n", _rx_buf[0]);
                }
            }
        }
    }


    /*_tx_buf[0] = DS2482_COMMAND_RESETWIRE;

    if (sendData(1, 0)) {
        flag = _event.wait_any(ERROR_FLAG | DONE_FLAG, DS2482_DEFAULT_TIMEOUT);

        if (flag == DONE_FLAG) {
            _try_counter = 0;
            waitOnBusy();
            flag = _event.wait_any(ERROR_FLAG | READY_FLAG, DS2482_DEFAULT_TIMEOUT);

            if (flag == READY_FLAG) {
                printf("ret: %u\n", _rx_buf[0]);
                return (_rx_buf[0] & DS2482_STATUS_PPD) ? true : false;
            }
        }
    }*/


    /*if (pull_up && !setConfig(StrongPullUp)) {
        return false;
    }*/

    return false;
}

void DS2482::deviceReset() {
    _tx_buf[0] = DS2482_COMMAND_RESET;
    send(1, 0);
}

uint8_t DS2482::getConfig() {
    _tx_buf[0] = DS2482_COMMAND_SRP;
    _tx_buf[1] = DS2482_POINTER_CONFIG;

    if (send(2, 1)) {
        uint32_t flag = _event.wait_any(ERROR_FLAG | OK_FLAG, DS2482_DEFAULT_TIMEOUT);

        if (flag == OK_FLAG) {
            return _rx_buf[0];
        }
    }

    return 0xFF;
}

bool DS2482::sendConfig() {
    _tx_buf[0] = DS2482_COMMAND_WRITECONFIG;
    _tx_buf[1] = _config | (~_config) << 4;

    if (sendData(2, 1)) {
        uint32_t flag = _event.wait_any(ERROR_FLAG | DONE_FLAG, DS2482_DEFAULT_TIMEOUT);

        if (flag == DONE_FLAG) {
            if (_config == _rx_buf[0]) {
                _config = _rx_buf[0];
                return true;
            }
        }
    }

    return false;
}

bool DS2482::setConfig(ds2482_config type) {
    _config |= type;
    return sendConfig();
}

bool DS2482::clearConfig(ds2482_config type) {
    _config &= ~(type);
    return sendConfig();
}

void DS2482::selectChannel(uint8_t channel) {

}

uint8_t DS2482::search(uint8_t *newAddr) {
    /*uint8_t direction;
    uint8_t last_zero = 0;

    if (searchLastDeviceFlag) {
        return 0;
    }

    if (!reset()) {
        return 0;
    }

    waitOnBusy();

    wireWriteByte(WIRE_COMMAND_SEARCH);

    for (uint8_t i = 0; i < 64; i++) {
        int searchByte = i / 8;
        int searchBit = 1 << i % 8;

        if (i < searchLastDiscrepancy) {
            direction = searchAddress[searchByte] & searchBit;

        } else {
            direction = i == searchLastDiscrepancy;
        }

        waitOnBusy();
        begin();
        writeByte(DS2482_COMMAND_TRIPLET);
        writeByte(direction ? 0x80 : 0x00);
        end();

        uint8_t status = waitOnBusy();

        uint8_t id = status & DS2482_STATUS_SBR;
        uint8_t comp_id = status & DS2482_STATUS_TSB;
        direction = status & DS2482_STATUS_DIR;

        if (id && comp_id) {
            return 0;

        } else {
            if (!id && !comp_id && !direction) {
                last_zero = i;
            }
        }

        if (direction)
            searchAddress[searchByte] |= searchBit;
        else
            searchAddress[searchByte] &= ~searchBit;

    }

    searchLastDiscrepancy = last_zero;

    if (!last_zero)
        searchLastDeviceFlag = 1;

    for (uint8_t i = 0; i < 8; i++)
        address[i] = searchAddress[i];*/

    return 1;
}
void DS2482::resetSearch() {
    searchLastDiscrepancy = 0;
    searchLastDeviceFlag = 0;

    memset(searchAddress, 0, sizeof(searchAddress));
}

void DS2482::select(const uint8_t rom[8]) {

}
void DS2482::write(uint8_t data) {

}

bool DS2482::sendData(uint8_t tx_len, uint16_t rx_len) {
    _try_counter = 0;
    waitOnBusy();

    uint32_t flag = _event.wait_any(ERROR_FLAG | READY_FLAG, DS2482_DEFAULT_TIMEOUT);

    if (flag == READY_FLAG) {
        if (send(tx_len, rx_len)) {
            flag = _event.wait_any(OK_FLAG | ERROR_FLAG, DS2482_DEFAULT_TIMEOUT);

            if (flag == OK_FLAG) {
                _event.set(DONE_FLAG);
                return true;
            }
        }
    }

    _event.set(ERROR_FLAG);
    return false;
}

bool DS2482::send(uint8_t tx_len, uint16_t rx_len, bool wait) {
    if (_i2c->transfer(
                _i2c_addr,
                (wait ? reinterpret_cast<char*>(_wait_buf) : reinterpret_cast<char*>(_tx_buf)),
                (wait ? 2 : tx_len),
                _rx_buf,
                (wait ? 1 : rx_len),
                event_callback_t(this, &DS2482::internalCb),
                I2C_EVENT_ALL) == 0) {
        return true;
    }

    _event.set(ERROR_FLAG);
    return false;
}

void DS2482::waitOnBusy() {
    if (send(2, 1, true)) {
        uint32_t flag = _event.wait_any(ERROR_FLAG | OK_FLAG, DS2482_DEFAULT_TIMEOUT);
        //printf("wait flag: %lu, %u\n", flag, _rx_buf[0]);

        if (flag == OK_FLAG) {
            if (!(_rx_buf[0] & DS2482_STATUS_BUSY)) {
                _event.set(READY_FLAG);
                printf("ready\n");

            } else if ((_try_counter + 1) < DS2482_DEFAULT_TIMEOUT) {
                _try_counter++;
                _queue->call_in(1, callback(this, &DS2482::waitOnBusy));

            } else {
                _event.set(ERROR_FLAG);
            }

        } else {
            _event.set(ERROR_FLAG);
        }

    } else {
        printf("waiting send error\n");
    }
}


void DS2482::internalCb(int event) {
    bool data_ok = false;

    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        printf("I2C_EVENT_ERROR_NO_SLAVE\n");

    } else if (event & I2C_EVENT_ERROR) {
        printf("I2C_EVENT_ERROR\n");

    } else {
        data_ok = true;

        // if (_task == None) {
        _event.set(OK_FLAG);

        /*} else if (_stage == BusyWait) {
            if (!(_rx_buf[0] & DS2482_STATUS_BUSY)) {
                if (_task == Reset) {
                    _stage = GetDataReady;

                    if (_task_id == 0) {
                        _config &= ~(StrongPullUp);
                        _tx_buf[0] = DS2482_COMMAND_WRITECONFIG;
                        _tx_buf[1] = _config | (~_config) << 4;

                        _queue->call(callback(this, &DS2482::send), 2, 1, false);

                    } else if (_task_id == 2) {
                        _tx_buf[0] = DS2482_COMMAND_RESETWIRE;

                        _queue->call(callback(this, &DS2482::send), 1, 0, false);

                    } else if (_task_id == 4) {
                        if (_rx_buf[0] & DS2482_STATUS_PPD) {
                            // doneCb.call(reset_ok);
                        }

                        _task = None;
                    }

                    _task_id++;
                }

            } else if ((_try_counter + 1) < DS2482_DEFAULT_TIMEOUT) {
                _try_counter++;
                _queue->call_in(1, callback(this, &DS2482::dataReadyReq));

            } else {
                // doneCb.call(error);
            }
        } else if (_stage == GetDataReady) {
            _task_id++;
            _queue->call(callback(this, &DS2482::dataReadyReq));
        }*/
    }

    if (!data_ok) {
        _event.set(ERROR_FLAG);
    }
}

