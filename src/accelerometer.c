/* 
Copyright 2013-2014 Brad Quick

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "bradwii.h"
#include "defs.h"
#include "rx.h"
#include "lib_i2c.h"
#include "accelerometer.h"
#include "lib_fp.h"
#include "lib_timers.h"

// when adding accelerometers, you need to include the following functions:
// void initacc() // initializes the accelerometer
// void readacc() // loads global.acc_g_vector with acc values in fixedpointnum g's

extern globalstruct global;

#define MPU6050_ADDRESS     0x68        // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board

void initacc(void)
{
    lib_i2c_writereg(MPU6050_ADDRESS, 0x1C, 0x10);      //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
}

void readacc(void)
{
    unsigned char data[6];

    lib_i2c_readdata(MPU6050_ADDRESS, 0x3B, (unsigned char *) &data, 6);

    // convert readings to fixedpointnum (in g's)
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
    ACC_ORIENTATION(global.acc_g_vector,
        (((int16_t) ((data[0] << 8) | data[1])) >> 2) * 64L,
        (((int16_t) ((data[2] << 8) | data[3])) >> 2) * 64L,
        (((int16_t) ((data[4] << 8) | data[5])) >> 2) * 64L);
      
}
