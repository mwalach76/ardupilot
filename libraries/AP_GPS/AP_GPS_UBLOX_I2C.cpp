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

#include "AP_GPS_UBLOX_I2C.h"

#include <AP_Math/AP_Math.h>

#if AP_GPS_UBLOX_I2C_ENABLED

extern const AP_HAL::HAL &hal;

AP_GPS_UBLOX_I2C_Port::AP_GPS_UBLOX_I2C_Port(AP_HAL::I2CDevice *_dev) :
    dev(_dev)
{
}

AP_GPS_UBLOX_I2C_Port *AP_GPS_UBLOX_I2C_Port::create(uint8_t bus, uint8_t address)
{
    AP_HAL::I2CDevice *dev = hal.i2c_mgr->get_device_ptr(bus, address);
    if (dev == nullptr) {
        return nullptr;
    }
    auto *port = NEW_NOTHROW AP_GPS_UBLOX_I2C_Port(dev);
    if (port == nullptr) {
        delete dev;
        return nullptr;
    }
    // Poll at 100Hz. The receiver may still be starting when this is
    // created, so there is no probe: until it answers, nothing is read and
    // the GPS driver simply sees no data.
    dev->register_periodic_callback(10000, FUNCTOR_BIND(port, &AP_GPS_UBLOX_I2C_Port::update, void));
    return port;
}

void AP_GPS_UBLOX_I2C_Port::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (b != 0) {
        baud = b;
    }
}

size_t AP_GPS_UBLOX_I2C_Port::_write(const uint8_t *buffer, size_t size)
{
    return tx_buffer.write(buffer, size);
}

ssize_t AP_GPS_UBLOX_I2C_Port::_read(uint8_t *buffer, uint16_t count)
{
    return rx_buffer.read(buffer, count);
}

bool AP_GPS_UBLOX_I2C_Port::_discard_input()
{
    // advance is a reader-side operation, so this is safe while the bus
    // thread is filling the buffer
    rx_buffer.advance(rx_buffer.available());
    return true;
}

void AP_GPS_UBLOX_I2C_Port::update()
{
    send_pending();
    receive_pending();
}

void AP_GPS_UBLOX_I2C_Port::send_pending()
{
    uint32_t pending = tx_buffer.available();
    while (pending >= 2) {
        uint32_t n = MIN(pending, uint32_t(MAX_TRANSFER));
        if (pending - n == 1) {
            // never leave a single byte behind: on its own it would be
            // taken as a register address, not message data
            n--;
        }
        uint8_t buf[MAX_TRANSFER];
        if (tx_buffer.peekbytes(buf, n) != n || !dev->transfer(buf, n, nullptr, 0)) {
            return;
        }
        tx_buffer.advance(n);
        total_tx_bytes += n;
        pending -= n;
    }
}

void AP_GPS_UBLOX_I2C_Port::receive_pending()
{
    uint8_t count_be[2];
    if (!dev->read_registers(REG_BYTES_AVAILABLE, count_be, sizeof(count_be))) {
        return;
    }
    uint16_t waiting = (uint16_t(count_be[0]) << 8) | count_be[1];
    if (waiting == 0xFFFF) {
        // bus floating high: no receiver answering
        return;
    }
    waiting = MIN(waiting, uint16_t(rx_buffer.space()));
    while (waiting > 0) {
        const uint16_t n = MIN(waiting, MAX_TRANSFER);
        uint8_t buf[MAX_TRANSFER];
        if (!dev->read_registers(REG_DATA_STREAM, buf, n)) {
            return;
        }
        rx_buffer.write(buf, n);
        total_rx_bytes += n;
        waiting -= n;
    }
}

#endif  // AP_GPS_UBLOX_I2C_ENABLED
