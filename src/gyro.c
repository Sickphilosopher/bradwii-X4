/* 
Copyright 2013 Brad Quick

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

#include "gyro.h"
#include "lib_i2c.h"
#include "lib_timers.h"
#include "rx.h"
#include "lib_fp.h"
#include "bradwii.h"

extern globalstruct global;

// when adding gyros, the following functions need to be included:
// initgyro() // initializes the gyro
// readgyro() // loads global.gyrorate with gyro readings in fixedpointnum degrees per second

#define MPU6050_ADDRESS     0x68        // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
#if (GYRO_LOW_PASS_FILTER<=6)
#define MPU6050_DLPF_CFG GYRO_LOW_PASS_FILTER
#else
#define MPU6050_DLPF_CFG   6
#endif

void initgyro(void)
{
    // Resetting the MPU6050 does not work for some reason.
    // But it is already in default mode at power up, so there is no need to reset it anyways

    //lib_i2c_writereg(MPU6050_ADDRESS, 0x6B, 0x80);      //PWR_MGMT_1    -- DEVICE_RESET 1
    //lib_timers_delaymilliseconds(10);
    lib_i2c_writereg(MPU6050_ADDRESS, 0x6B, 0x03);      //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    lib_i2c_writereg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG);  //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    lib_i2c_writereg(MPU6050_ADDRESS, 0x1B, 0x18);      //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void readgyro(void)
{
    unsigned char data[6];
    lib_i2c_readdata(MPU6050_ADDRESS, 0x43, (unsigned char *) &data, 6);
    // for (int i = 0; i < 6; i++)
    // {
    //   printf("%02X ", data[i]);
    // }
    // printf("\n");
    
    // convert to fixedpointnum, in degrees per second
    // the gyro puts out an int where each count equals 0.0609756097561 degrees/second
    // we want fixedpointnums, so we multiply by 3996 (0.0609756097561 * (1<<FIXEDPOINTSHIFT))
    GYRO_ORIENTATION(global.gyrorate,
        ((int16_t) ((data[0] << 8) | data[1])) * 3996L,       // range: +/- 8192; +/- 2000 deg/sec
        ((int16_t) ((data[2] << 8) | data[3])) * 3996L,
        ((int16_t) ((data[4] << 8) | data[5])) * 3996L);
}