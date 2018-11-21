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

#ifndef DS2482_H
#define DS2482_H

#include "mbed.h"

#define ERROR_FLAG (1UL << 0)
#define OK_FLAG (1UL << 1)
#define READY_FLAG (1UL << 2)
#define DONE_FLAG (1UL << 3)

#define DS2482_DEFAULT_ADDRESS (0x18<<1)
#define DS2482_TX_BUFFER_SIZE 8
#define DS2482_RX_BUFFER_SIZE 32
#define DS2482_DEFAULT_TIMEOUT 2000  // ms

#define DS2482_POINTER_CONFIG 0xC3
#define DS2482_COMMAND_SRP  0xE1  // Set read pointer
#define DS2482_POINTER_STATUS 0xF0

#define DS2482_COMMAND_RESETWIRE  0xB4
#define DS2482_COMMAND_WRITEBYTE  0xA5
#define DS2482_COMMAND_READBYTE   0x96
#define DS2482_COMMAND_SINGLEBIT  0x87
#define DS2482_COMMAND_TRIPLET    0x78

#define DS2484_CONFIG_WS   (1 << 3)

#define DS2482_CONFIG_PPM  (1 << 1)

#define DS2482_STATUS_BUSY  (1 << 0)
#define DS2482_STATUS_PPD   (1 << 1)
#define DS2482_STATUS_SD   (1 << 2)
#define DS2482_STATUS_LL   (1 << 3)
#define DS2482_STATUS_RST  (1 << 4)
#define DS2482_STATUS_SBR  (1 << 5)
#define DS2482_STATUS_TSB  (1 << 6)
#define DS2482_STATUS_DIR  (1 << 7)

#define WIRE_COMMAND_SKIP     0xCC
#define WIRE_COMMAND_SELECT     0x55
#define WIRE_COMMAND_SEARCH     0xF0

#define DS2482_ERROR_TIMEOUT    (1<<0)
#define DS2482_ERROR_SHORT      (1<<1)
#define DS2482_ERROR_CONFIG     (1<<2)

#define DS2482_CMD_CHSL   0xC3  //< DS2482 Channel Select
// DS2482 channel selection code for defines
#define DS2482_CH_IO0   0xF0  //< DS2482 Select Channel IO0
#define DS2482_CH_IO1   0xE1  //< DS2482 Select Channel IO1
#define DS2482_CH_IO2   0xD2  //< DS2482 Select Channel IO2
#define DS2482_CH_IO3   0xC3  //< DS2482 Select Channel IO3
#define DS2482_CH_IO4   0xB4  //< DS2482 Select Channel IO4
#define DS2482_CH_IO5   0xA5  //< DS2482 Select Channel IO5
#define DS2482_CH_IO6   0x96  //< DS2482 Select Channel IO6
#define DS2482_CH_IO7   0x87  //< DS2482 Select Channel IO7

// DS2482 channel selection read back code for defines
#define DS2482_RCH_IO0    0xB8  //< DS2482 Select Channel IO0
#define DS2482_RCH_IO1    0xB1  //< DS2482 Select Channel IO1
#define DS2482_RCH_IO2    0xAA  //< DS2482 Select Channel IO2
#define DS2482_RCH_IO3    0xA3  //< DS2482 Select Channel IO3
#define DS2482_RCH_IO4    0x9C  //< DS2482 Select Channel IO4
#define DS2482_RCH_IO5    0x95  //< DS2482 Select Channel IO5
#define DS2482_RCH_IO6    0x8E  //< DS2482 Select Channel IO6
#define DS2482_RCH_IO7  0x87  //< DS2482 Select Channel IO7



class DS2482 {
 public:
  typedef enum {
    ActivePullUp = (1 << 0),
    StrongPullUp = (1 << 2)
  } ds2482_config;

  explicit DS2482(I2C * i2c_obj, EventQueue * queue, int8_t address = DS2482_DEFAULT_ADDRESS);
  DS2482(PinName sda, PinName scl, EventQueue * queue, int8_t address = DS2482_DEFAULT_ADDRESS, uint32_t frequency = 400000);
  virtual ~DS2482(void);
  bool init();
  bool reset();
  void deviceReset();
  void selectChannel(uint8_t channel);
  bool setConfig(ds2482_config type);
  bool clearConfig(ds2482_config type);
  void resetSearch();
  uint8_t search(uint8_t *newAddr);
  void select(const uint8_t rom[8]);
  void write(uint8_t data);
  uint8_t getConfig();

 protected:
  typedef enum {
    DS2482_COMMAND_WRITECONFIG = 0xD2,  // set config
    DS2482_COMMAND_RESET =  0xF0,  // device reset
  } DS2482Command;
  bool sendData(uint8_t tx_len, uint16_t rx_len);

 private:
  I2C * _i2c;
  EventQueue * _queue;
  event_callback_t _done_cb;
  EventFlags _event;

  typedef enum {
    Init,
    Ready,
    BusyWait,
    GetDataReady
  } Stage;


  const int8_t _i2c_addr;

  char _rx_buf[DS2482_RX_BUFFER_SIZE];
  char _tx_buf[DS2482_TX_BUFFER_SIZE];
  char _wait_buf[2];
  uint32_t _i2c_buffer[sizeof(I2C) / sizeof(uint32_t)];

  uint8_t searchAddress[8];
  uint8_t searchLastDiscrepancy;
  uint8_t searchLastDeviceFlag;
  uint8_t _try_counter;

  uint8_t _config;
  Stage _stage;

  bool send(uint8_t tx_size, uint16_t rx_size = 1, bool wait = false);
  void internalCb(int event);
  bool sendConfig();
  void waitOnBusy();
  void dataReadyReq();
};

#endif  // DS2482_H
