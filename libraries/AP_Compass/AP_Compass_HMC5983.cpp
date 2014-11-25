/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_Compass_HMC5983.cpp - Arduino Library for HMC5983 magnetometer
 *       Code by John Williams based on HMC5843 driver
 *
 *       Sensor is conected to SPI port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

// AVR LibC Includes
#include <AP_Math.h>
#include <AP_HAL.h>

#include "AP_Compass_HMC5983.h"
#include "HMC5xx3.h"

extern const AP_HAL::HAL& hal;

// read_register - read a register value
bool AP_Compass_HMC5983::read_register(uint8_t address, uint8_t *value)
{
    uint8_t tx[2], rx[2];
    tx[0] = address | REG_READ;
    tx[1] = 0x0;
    _spi->transaction(tx, rx, 2);
    *value = rx[1];

/*

    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
    }
*/
    return true;
}

// write_register - update a register value
bool AP_Compass_HMC5983::write_register(uint8_t address, uint8_t value)
{
    uint8_t tx[2], rx[2];
    tx[0] = address;
    tx[1] = value;
    _spi->transaction(tx, rx, 2);

/*
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _healthy[0] = false;
        return false;
    }
*/
    return true;
}

// Read Sensor data
bool AP_Compass_HMC5983::read_raw()
{
    uint8_t tx_buff[7],rx_buff[7];

    tx_buff[0] = 0x03 | REG_READ | REG_MS;
    memset(tx_buff+1,0,6);
    memset(rx_buff,0,7);

    _spi->transaction(tx_buff, rx_buff, 7);

    int16_t rx, ry, rz;
    rx = (((int16_t)rx_buff[1]) << 8) | rx_buff[2];
    rz = (((int16_t)rx_buff[3]) << 8) | rx_buff[4];
    ry = (((int16_t)rx_buff[5]) << 8) | rx_buff[6];
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_HMC5983::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
   uint32_t tnow = hal.scheduler->micros();
   if (_healthy[0] && _accum_count != 0 && (tnow - _last_accum_time) < 13333) {
	  // the compass gets new data at 75Hz
	  return;
   }

   if (!_spi_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _spi_sem->give();

   if (result) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
	  _mag_x_accum += _mag_x;
	  _mag_y_accum += _mag_y;
	  _mag_z_accum += _mag_z;
	  _accum_count++;
	  if (_accum_count == 14) {
		 _mag_x_accum /= 2;
		 _mag_y_accum /= 2;
		 _mag_z_accum /= 2;
		 _accum_count = 7;
	  }
	  _last_accum_time = tnow;
   }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_HMC5983::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_HMC5983::init()
{
    int numAttempts = 0, good_count = 0;
    bool success = false;
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    float gain_multiple = 1.0;

    hal.scheduler->suspend_timer_procs();
    hal.scheduler->delay(10);

    _spi = hal.spi->device(AP_HAL::SPIDevice_HMC5983);
    if (_spi == NULL) {
        hal.scheduler->panic(PSTR("PANIC: AP_Compass_HMC5983 did not get "
                     "valid SPI device driver!"));
        return false;
    }

    _spi_sem = _spi->get_semaphore();
    if (!_spi_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get HMC5983 semaphore"));
        return false;
    }

    // confirm we are using a 5983
    _base_config = 0;
    if (!write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        _healthy[0] = false;
        _spi_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }
    if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        // a 5983 supports the sample averaging config
        product_id = AP_COMPASS_TYPE_HMC5983;
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        gain_multiple = 660.0 / 1090;  // adjustment for runtime vs calibration gain
    } else {
        // don't recognise this compass type
        _spi_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }

    calibration[0] = 0;
    calibration[1] = 0;
    calibration[2] = 0;

    while ( success == 0 && numAttempts < 25 && good_count < 5)
    {
        // record number of attempts at initialisation
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!write_register(ConfigRegA, PositiveBiasConfig))
            continue;      // compass not responding on the bus
        hal.scheduler->delay(50);

        // set gains
        if (!write_register(ConfigRegB, calibration_gain) ||
            !write_register(ModeRegister, SingleConversion))
            continue;

        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        hal.scheduler->delay(10);

        float cal[3];

        // hal.console->printf_P(PSTR("mag %d %d %d\n"), _mag_x, _mag_y, _mag_z);
        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

        // hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f\n"), cal[0], cal[1], cal[2]);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts > 2 &&
            cal[0] > 0.7f && cal[0] < 1.35f &&
            cal[1] > 0.7f && cal[1] < 1.35f &&
            cal[2] > 0.7f && cal[2] < 1.35f) {
            // hal.console->printf_P(PSTR("cal=%.2f %.2f %.2f good\n"), cal[0], cal[1], cal[2]);
            good_count++;
            calibration[0] += cal[0];
            calibration[1] += cal[1];
            calibration[2] += cal[2];
        }

#if 0
        /* useful for debugging */
        hal.console->printf_P(PSTR("MagX: %d MagY: %d MagZ: %d\n"), (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), cal[0], cal[1], cal[2]);
#endif
    }

    if (good_count >= 5) {
        /*
          The use of gain_multiple below is incorrect, as the gain
          difference between 2.5Ga mode and 1Ga mode is already taken
          into account by the expected_x and expected_yz values.  We
          are not going to fix it however as it would mean all
          APM1/APM2 users redoing their compass calibration. The
          impact is that the values we report on APM1/APM2 are lower
          than they should be (by a multiple of about 0.6). This
          doesn't have any impact other than the learned compass
          offsets
         */
        calibration[0] = calibration[0] * gain_multiple / good_count;
        calibration[1] = calibration[1] * gain_multiple / good_count;
        calibration[2] = calibration[2] * gain_multiple / good_count;
        success = true;
    } else {
        /* best guess */
        calibration[0] = 1.0;
        calibration[1] = 1.0;
        calibration[2] = 1.0;
    }

    // leave test mode
    if (!re_initialise()) {
        _spi_sem->give();
        hal.scheduler->resume_timer_procs();
        return false;
    }

    _spi_sem->give();
    hal.scheduler->resume_timer_procs();
    _initialised = true;

	// perform an initial read
	_healthy[0] = true;
	read();

#if 0
    hal.console->printf_P(PSTR("CalX: %.2f CalY: %.2f CalZ: %.2f\n"), 
                          calibration[0], calibration[1], calibration[2]);
#endif

    return success;
}

// Read Sensor data
bool AP_Compass_HMC5983::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return false;
    }
    if (!_healthy[0]) {
        if (hal.scheduler->millis() < _retry_time) {
            return false;
        }
        if (!re_initialise()) {
            _retry_time = hal.scheduler->millis() + 1000;
	    // FIXME do what now?
            return false;
        }
    }

	if (_accum_count == 0) {
	   accumulate();
	   if (!_healthy[0] || _accum_count == 0) {
		  // try again in 1 second, and set I2c clock speed slower
		  _retry_time = hal.scheduler->millis() + 1000;
		  // FIXME do what now?
		  return false;
	   }
	}

	_field[0].x = _mag_x_accum * calibration[0] / _accum_count;
	_field[0].y = _mag_y_accum * calibration[1] / _accum_count;
	_field[0].z = _mag_z_accum * calibration[2] / _accum_count;
	_accum_count = 0;
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    last_update = hal.scheduler->micros(); // record time of update

    // rotate to the desired orientation
    if (product_id == AP_COMPASS_TYPE_HMC5983) {
        _field[0].rotate(ROTATION_YAW_90);
    }

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _field[0].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _field[0].rotate((enum Rotation)_orientation[0].get());

    if (!_external[0]) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _field[0].rotate(_board_orientation);
    }

    apply_corrections(_field[0],0);

    _healthy[0] = true;

    return true;
}
