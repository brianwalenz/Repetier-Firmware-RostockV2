/**
 * Copyright (c) 20011-2017 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
/**
 * \file
 * \brief SpiDriver classes
 */
#ifndef SdSpiDriver_h
#define SdSpiDriver_h

#include <Arduino.h>
#include "SPI.h"
#include "../SdFatConfig.h"



#if 1

class SdSpiLibDriver {
 public:
  /** Activate SPI hardware. */
  void activate() {
    SPI.beginTransaction(m_spiSettings);
  }
  /** Deactivate SPI hardware. */
  void deactivate() {
    SPI.endTransaction();
  }
  /** Initialize the SPI bus.
   *
   * \param[in] csPin SD card chip select pin.
   */
  void begin(uint8_t csPin) {
    m_csPin = csPin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    //SPI.begin();
  }
  /** Receive a byte.
   *
   * \return The byte.
   */
  uint8_t receive() {
    return SPI.transfer( 0XFF);
  }
  /** Receive multiple bytes.
  *
  * \param[out] buf Buffer to receive the data.
  * \param[in] n Number of bytes to receive.
  *
  * \return Zero for no error or nonzero error code.
  */
  uint8_t receive(uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; i++) {
      buf[i] = SPI.transfer(0XFF);
    }
    return 0;
  }
  /** Send a byte.
   *
   * \param[in] data Byte to send
   */
  void send(uint8_t data) {
    SPI.transfer(data);
  }
  /** Send multiple bytes.
   *
   * \param[in] buf Buffer for data to be sent.
   * \param[in] n Number of bytes to send.
   */
  void send(const uint8_t* buf, size_t n) {
    for (size_t i = 0; i < n; i++) {
      SPI.transfer(buf[i]);
    }
  }
  /** Set CS low. */
  void select() {
    digitalWrite(m_csPin, LOW);
  }
  /** Save SPISettings.
   *
   * \param[in] spiSettings SPI speed, mode, and byte order.
   */
  void setSpiSettings(SPISettings spiSettings) {
    m_spiSettings = spiSettings;
  }
  /** Set CS high. */
  void unselect() {
    digitalWrite(m_csPin, HIGH);
  }

 private:
  SPISettings m_spiSettings;
  uint8_t m_csPin;
};

typedef SdSpiLibDriver SdFatSpiDriver;
typedef SdFatSpiDriver SdSpiDriver;


#else


//-----------------------------------------------------------------------------
/**
 * \class SdSpiAltDriver
 * \brief Optimized SPI class for access to SD and SDHC flash memory cards.
 */
class SdSpiAltDriver {
public:

  /** Activate SPI hardware. */
  void activate() {
    SPI.beginTransaction(m_spiSettings);
  };

  /** Deactivate SPI hardware. */
  void deactivate() {
    SPI.endTransaction();
  };

  /** Initialize the SPI bus.
   *
   * \param[in] csPin SD card chip select pin.
   */
  void begin(uint8_t csPin) {
    m_csPin = csPin;
    pinMode(m_csPin, OUTPUT);
    digitalWrite(m_csPin, HIGH);
    //SPI.begin();
  };

  /** Receive a byte.
   *
   * \return The byte.
   */
  uint8_t receive() {
    SPDR = 0XFF;
    while (!(SPSR & (1 << SPIF))) {}
    return SPDR;
  };

  /** Receive multiple bytes.
   *
   * \param[out] buf Buffer to receive the data.
   * \param[in] n Number of bytes to receive.
   *
   * \return Zero for no error or nonzero error code.
   */
  uint8_t receive(uint8_t* buf, size_t n) {
    if (n-- == 0) {
      return 0;
    }
    SPDR = 0XFF;
    for (size_t i = 0; i < n; i++) {
      while (!(SPSR & (1 << SPIF))) {}
      uint8_t b = SPDR;
      SPDR = 0XFF;
      buf[i] = b;
    }
    while (!(SPSR & (1 << SPIF))) {}
    buf[n] = SPDR;
    return 0;
  };

  /** Send a byte.
   *
   * \param[in] data Byte to send
   */
  void send(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF))) {}
  };

  /** Send multiple bytes.
   *
   * \param[in] buf Buffer for data to be sent.
   * \param[in] n Number of bytes to send.
   */
  void send(const uint8_t* buf, size_t n) {
    if (n == 0) {
      return;
    }
    SPDR = buf[0];
    if (n > 1) {
      uint8_t b = buf[1];
      size_t i = 2;
      while (1) {
        while (!(SPSR & (1 << SPIF))) {}
        SPDR = b;
        if (i == n) {
          break;
        }
        b = buf[i++];
      }
    }
    while (!(SPSR & (1 << SPIF))) {}
  };

  /** Set CS low. */
  void select() {
    digitalWrite(m_csPin, LOW);
  }

  /** Save SPISettings.
   *
   * \param[in] spiSettings SPI speed, mode, and byte order.
   */
  void setSpiSettings(SPISettings spiSettings) {
    m_spiSettings = spiSettings;
  }

  /** Set CS high. */
  void unselect() {
    digitalWrite(m_csPin, HIGH);
  }

private:
  SPISettings m_spiSettings;
  uint8_t m_csPin;
};


typedef SdSpiAltDriver SdFatSpiDriver;
typedef SdFatSpiDriver SdSpiDriver;

#endif

#endif  // SdSpiDriver_h
