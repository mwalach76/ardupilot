/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  u-blox DDC (I2C) port presented as a UART, so the standard u-blox
  driver can talk to a receiver that is wired only over I2C.

  The receiver exposes the number of bytes waiting in registers
  0xFD/0xFE (big endian) and its message stream at register 0xFF. Writes
  of two or more bytes are message data; a one-byte write only sets the
  register address, so this port never sends a single byte.
 */
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_UBLOX_I2C_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/RingBuffer.h>

class AP_GPS_UBLOX_I2C_Port : public AP_HAL::UARTDriver {
public:
    AP_GPS_UBLOX_I2C_Port(AP_HAL::I2CDevice *dev);

    // return a port for a u-blox receiver at bus/address, or nullptr
    static AP_GPS_UBLOX_I2C_Port *create(uint8_t bus, uint8_t address);

    bool is_initialized() override { return true; }
    bool tx_pending() override { return tx_buffer.available() > 0; }
    uint32_t txspace() override { return tx_buffer.space(); }

    // there is no baud rate, but report the one requested so detection
    // logic that reads it back behaves as it does on a UART
    uint32_t get_baud_rate() const override { return baud; }

    uint32_t get_total_tx_bytes() const override { return total_tx_bytes; }
    uint32_t get_total_rx_bytes() const override { return total_rx_bytes; }

protected:
    void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;
    void _end() override {}
    void _flush() override {}
    uint32_t _available() override { return rx_buffer.available(); }
    bool _discard_input() override;

private:
    // runs on the I2C bus thread
    void update();
    void send_pending();
    void receive_pending();

    static constexpr uint8_t REG_BYTES_AVAILABLE = 0xFD;
    static constexpr uint8_t REG_DATA_STREAM = 0xFF;
    static constexpr uint16_t MAX_TRANSFER = 128;

    AP_HAL::I2CDevice *dev;
    ByteBuffer rx_buffer{2048};
    ByteBuffer tx_buffer{1024};
    uint32_t baud;
    uint32_t total_tx_bytes;
    uint32_t total_rx_bytes;
};

#endif  // AP_GPS_UBLOX_I2C_ENABLED
