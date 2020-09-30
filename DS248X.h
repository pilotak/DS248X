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

#ifndef DS248X_H
#define DS248X_H

#include <climits>
#include <chrono>
#include "mbed.h"
using namespace std::chrono;

#include "mbed-trace/mbed_trace.h"
#ifndef TRACE_GROUP
    #define TRACE_GROUP "WIRE"
#endif

#define DS248X_DEFAULT_ADDRESS (0x18 << 1)

#define DS248X_CONFIG_APU (1<<0)
#define DS248X_CONFIG_PPM (1<<1)
#define DS248X_CONFIG_SPU (1<<2)
#define DS2484_CONFIG_WS  (1<<3)

#define DS248X_STATUS_BUSY (1<<0)
#define DS248X_STATUS_PPD  (1<<1) // present detect
#define DS248X_STATUS_SD   (1<<2)
#define DS248X_STATUS_LL   (1<<3)
#define DS248X_STATUS_RST  (1<<4)
#define DS248X_STATUS_SBR  (1<<5)
#define DS248X_STATUS_TSB  (1<<6)
#define DS248X_STATUS_DIR  (1<<7)

class DS248X {
  public:
    typedef enum {
        ActivePullUp = DS248X_CONFIG_APU,
        StrongPullUp = DS248X_CONFIG_SPU,
        OverdriveSpeed = DS2484_CONFIG_WS  // perform reset() after setting this
    } ds248x_config_t;

    DS248X(uint8_t address = DS248X_DEFAULT_ADDRESS);
    DS248X(PinName sda, PinName scl, uint8_t address = DS248X_DEFAULT_ADDRESS, uint32_t frequency = 400000);
    virtual ~DS248X(void);

    /**
    * @brief Initialise the chip
    *
    * @param i2c_obj pass I2C object if you didn't specify pins in constructor
    * @return true if successful, otherwise false
    */
    bool init(I2C *i2c_obj = nullptr);

    /**
     * @brief Set configuration
     *
     * @param config configuration type
     * @return true if successful, otherwise false
     */
    bool setConfig(ds248x_config_t config);

    /**
     * @brief Clear configuration
     *
     * @param config configuration type
     * @return true if successful, otherwise false
     */
    bool clearConfig(ds248x_config_t config);

    /**
     * @brief Select channel for DS248X-800 only
     *
     * @param channel
     * @return true if successful, otherwise false
     */
    bool selectChannel(uint8_t channel);

    /**
     * @brief Reset the device
     *
     * @return true if successful, otherwise false
     */
    bool deviceReset();

    /**
     * @brief Compute CRC
     *
     * @param data a pointer to the data block
     * @param len the size of the data
     * @return CRC
     */
    static char computeCRC(const char *data, size_t len = 8);

    /**
     * @brief Check if CRC math the data
     *
     * @param data a pointer to the data block where the last byte is the CRC
     * @param len the size of the data
     * @return true if CRC math, otherwise false
     */
    bool crc8(const char *data, size_t len);

    /**
     * @brief Attach callback for events
     *
     * @param function callback
     */
    void attach(Callback<void(char)> function);

    // 1-Wire commands

    /**
     * @brief Write multiple bytes to 1-wire bus
     *
     * @param data a pointer to the data block
     * @param len the size of the data to be written
     * @return true if successful, otherwise false
     */
    bool writeBytes(const char *data, size_t len);

    /**
     * @brief Read multiple bytes from 1-wire bus
     *
     * @param buffer place to put the reading
     * @param len size of data to read (make sure it fits into buffer)
     * @return true if successful, otherwise false
     */
    bool readBytes(char *buffer, size_t len);

    /**
     * @brief Write bit to 1-wire bus
     *
     * @param bit bit to send
     * @return true if successful, otherwise false
     */
    bool writeBit(bool bit);

    /**
     * @brief Write bit from 1-wire bus
     *
     * @return result bit
     */
    bool readBit();

    /**
    * @brief Creates reset condition on 1-wire bus
    *
    * @return true if device on the bus, false if no device/error
    */
    bool reset();

    /**
     * @brief Creates skip condition on 1-wire bus
     *
     * @return true if successful, otherwise false
     */
    bool skip();

    /**
     * @brief Select device on 1-wire bus
     *
     * @param rom unique address of device
     * @return true if successful, otherwise false
     */
    bool select(const char *rom);

    /**
     * @brief Search for device on 1-wire bus
     *
     * @param rom place to put the unique address of device (8 bytes)
     * @return true if successful, otherwise false
     */
    bool search(char *rom);

    /**
     * @brief Reset search for next usage
     *
     */
    void resetSearch();

  protected:
    typedef enum {
        CMD_1WT  = 0x78, // 1-Wire triplet
        CMD_1WSB = 0x87, // 1-Wire single bit
        CMD_1WRB = 0x96, // 1-Wire read byte
        CMD_1WWB = 0xA5, // 1-Wire write byte
        CMD_1WRS = 0xB4, // 1-Wire reset
        CMD_CHSL = 0xC3, // channel select
        CMD_WCFG = 0xD2, // write configuration
        CMD_SRP  = 0xE1, // set read pointer
        CMD_DRST = 0xF0, // device reset
    } ds248x_cmd_t;

    typedef enum {
        POINTER_CONFIG = 0xC3,
        POINTER_DATA   = 0xE1,
        POINTER_STATUS = 0xF0,
    } ds248x_pointer_t;

    typedef enum {
        WIRE_COMMAND_SELECT = 0x55,
        WIRE_COMMAND_SKIP   = 0xCC,
        WIRE_COMMAND_SEARCH = 0xF0
    } ds248x_wire_cmd_t;

    /**
     * @brief Write data to device
     *
     * @param data a pointer to the data block
     * @param len the size of the data to be written
     * @return true if successful, otherwise false
     */
    bool deviceWriteBytes(const char *data, size_t len);

    /**
     * @brief Read the data from device
     *
     * @param address where to read from
     * @param buffer place to put the reading
     * @param len size of data to read (make sure it fits into buffer)
     * @return true if successful, otherwise false
     */
    bool deviceReadBytes(ds248x_pointer_t address, char *buffer, size_t len);

    /**
     * @brief Wait until device not busy
     *
     * @param status place to put the reading (1 byte)
     * @return true if successful, otherwise false
     */
    bool waitBusy(char *status = nullptr);

    /**
     * @brief Send current config to device
     *
     * @return true if successful, otherwise false
     */
    bool sendConfig();

    /**
     * @brief Get config from device
     *
     * @return true if successful, otherwise false
     */
    bool getConfig();

  private:
    I2C *_i2c;
    uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];
    const char _address = DS248X_DEFAULT_ADDRESS;
    char _config = UCHAR_MAX;
    Callback<void(char)> _callback = nullptr;

    uint8_t _last_discrepancy = 0;
    bool _last_device_flag = false;

    /**
     * @brief Write single byte to 1-wire bus
     *
     * @param data byte to send
     * @return true if successful, otherwise false
     */
    bool write(char data);

    /**
     * @brief Read single byte from 1-wire bus
     *
     * @param buffer place to put the reading (1 byte)
     * @return true if successful, otherwise false
     */
    bool read(char *buffer);

    /**
     * @brief Set the read pointer from where to read the data
     *
     * @param address where to read from
     * @return true if successful, otherwise false
     */
    bool setReadPointer(ds248x_pointer_t address);
};

#endif  // DS248X_H
