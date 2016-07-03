/* 
Copyright 2013 Brad Quick

Some of this code is based on Multiwii code by Alexandre Dubus (www.multiwii.com)

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


#include <stdlib.h>

#include <string.h>
#include "lib_serial.h"
#include "lib_timers.h"
#include "lib_fp.h"

#include "bradwii.h"
#include "serial.h"
#include "defs.h"
#include "checkboxes.h"
#include "compass.h"
#include "eeprom.h"
#include "imu.h"
#include "gps.h"

#define MSP_VERSION 0
#define  VERSION  112           // version 1.12


extern globalstruct global;
extern usersettingsstruct usersettings;

extern const char checkboxnames[];
extern unsigned int lib_i2c_error_count;

void serialinit(void)
{
  lib_serial_initport(SERIAL_BAUD);
}

#if (MULTIWII_CONFIG_SERIAL_PORTS!=NOSERIALPORT)
#define SERIALSTATEIDLE 0
#define SERIALSTATEGOTDOLLARSIGN 1
#define SERIALSTATEGOTM 2
#define SERIALSTATEGOTLESSTHANSIGN 3
#define SERIALSTATEGOTDATASIZE 4
#define SERIALSTATEGOTCOMMAND 5
#define SERIALSTAGEGOTPAYLOAD 6

#define CAPABILITES 1 | ((BAROMETER_TYPE!=NO_BAROMETER)<<1) | ((COMPASS_TYPE!=NO_COMPASS)<<2) | ((GPS_TYPE!=NO_GPS)<<3)

// datagram format is $M<[data size][command][data...][checksum]
// response format is $M>[data size][command][data...][checksum]
//                 or $M![data size][command][data...][checksum] on error
unsigned char serialreceivestate = 0;

unsigned char serialcommand;
unsigned char serialdatasize;
unsigned char serialchecksum;

void sendandchecksumcharacter(unsigned char c)
{
    lib_serial_sendchar(c);
    serialchecksum ^= c;
}

void sendandchecksumdata(unsigned char *data, char length)
{
    for (int x = 0; x < length; ++x)
        sendandchecksumcharacter(data[x]);
}

void sendandchecksumint(unsigned int value)
{
    sendandchecksumdata((unsigned char *) &value, 2);
}

void sendandchecksumlong(unsigned long value)
{
    sendandchecksumdata((unsigned char *) &value, 4);
}

void sendgoodheader(unsigned char size)
{
    lib_serial_sendchar('$');
    lib_serial_sendchar('M');
    lib_serial_sendchar('>');
    lib_serial_sendchar(size);
    serialchecksum = size;
    sendandchecksumcharacter(serialcommand);
}

void senderrorheader()
{
    lib_serial_sendchar('$');
    lib_serial_sendchar('M');
    lib_serial_sendchar('!');
    lib_serial_sendchar(0);
    serialchecksum = 0;
    sendandchecksumcharacter(serialcommand);
}

void evaluatecommand(unsigned char *data)
{
    unsigned char command = serialcommand;
    if (command == MSP_IDENT) { // send rx data
        sendgoodheader(7);
        sendandchecksumcharacter(VERSION);
        sendandchecksumcharacter(AIRCRAFT_CONFIGURATION);
        sendandchecksumcharacter(MSP_VERSION);
        for (int x = 0; x < 4; ++x)
            sendandchecksumcharacter(0);    // 32 bit "capability"
    } else if (command == MSP_RC) {     // send rx data
        sendgoodheader(16);
        for (int x = 0; x < 8; ++x) {
            int value = 0;
            if (x < RXNUMCHANNELS)
                value = ((global.rxvalues[x] * 500L) >>FIXEDPOINTSHIFT) + 1500;

            sendandchecksumdata((unsigned char *) &value, 2);
        }
    } else if (command == MSP_ATTITUDE) {       // send attitude data
        sendgoodheader(6);
        // convert our estimated gravity vector into roll and pitch angles
        int value;
        value = (global.currentestimatedeulerattitude[0] * 10) >>FIXEDPOINTSHIFT;
        sendandchecksumdata((unsigned char *) &value, 2);
        value = (global.currentestimatedeulerattitude[1] * 10) >>FIXEDPOINTSHIFT;
        sendandchecksumdata((unsigned char *) &value, 2);
        value = (global.currentestimatedeulerattitude[2]) >>FIXEDPOINTSHIFT;
        sendandchecksumdata((unsigned char *) &value, 2);
    } else if (command == MSP_ALTITUDE) {       // send attitude data
        sendgoodheader(4);
        fixedpointnum fp = (global.altitude * 25) >>(FIXEDPOINTSHIFT - 2);
        sendandchecksumdata((unsigned char *) &fp, 4);
    } else if (command == MSP_MAG_CALIBRATION) {        // send attitude data
        if (!global.armed)
            calibratecompass();
        sendgoodheader(0);
    } else if (command == MSP_ACC_CALIBRATION) {        // send attitude data
        if (!global.armed)
            calibrategyroandaccelerometer(true);
        sendgoodheader(0);
    }

    else if (command == MSP_RAW_IMU) {  // send attitude data
        sendgoodheader(18);
        for (int x = 0; x < 3; ++x) {   // convert from g's to what multiwii uses
            int value = global.acc_g_vector[x] >> 8;
            sendandchecksumdata((unsigned char *) &value, 2);
        }
        for (int x = 0; x < 3; ++x) {   // convert from degrees per second to /8000
            int value = (global.gyrorate[x]) >>14;      // this is aproximate
            sendandchecksumdata((unsigned char *) &value, 2);
        }
        for (int x = 0; x < 3; ++x) {   // convert from
            int value = (global.compassvector[x]) >>8;
            sendandchecksumdata((unsigned char *) &value, 2);
        }
    } else if (command == MSP_STATUS) { // send attitude data
        sendgoodheader(10);
        sendandchecksumint((global.timesliver * 15) >>8);   // convert from fixedpointnum to microseconds
        sendandchecksumint(lib_i2c_error_count);    // i2c error count
        sendandchecksumint(CAPABILITES);    // baro mag, gps, sonar
        sendandchecksumdata((unsigned char *) &global.activecheckboxitems, 4);      // options1
    } else if (command == MSP_MOTOR) {  // send motor output data
        sendgoodheader(16);
        for (int x = 0; x < 8; ++x) {
            if (x < NUMMOTORS)
                sendandchecksumint(global.motoroutputvalue[x]);     // current motor value
            else
                sendandchecksumint(0);
        }
    } else if (command == MSP_PID) {    // send pid data
        sendgoodheader(3 * NUMPIDITEMS);
        for (int x = 0; x < NUMPIDITEMS; ++x) {
            if (x == ALTITUDEINDEX)
                sendandchecksumcharacter(usersettings.pid_pgain[x] >> 7);
            else if (x == NAVIGATIONINDEX)
                sendandchecksumcharacter(usersettings.pid_pgain[x] >> 11);
            else
                sendandchecksumcharacter(usersettings.pid_pgain[x] >> 3);
            sendandchecksumcharacter(usersettings.pid_igain[x]);
            if (x == NAVIGATIONINDEX)
                sendandchecksumcharacter(usersettings.pid_dgain[x] >> 8);
            else if (x == ALTITUDEINDEX)
                sendandchecksumcharacter(usersettings.pid_dgain[x] >> 9);
            else
                sendandchecksumcharacter(usersettings.pid_dgain[x] >> 2);
        }
    } else if (command == MSP_SET_PID) {
        for (int x = 0; x < NUMPIDITEMS; ++x) {
            if (x == ALTITUDEINDEX)
                usersettings.pid_pgain[x] = ((fixedpointnum) (*data++)) << 7;
            else if (x == NAVIGATIONINDEX)
                usersettings.pid_pgain[x] = ((fixedpointnum) (*data++)) << 11;
            else
                usersettings.pid_pgain[x] = ((fixedpointnum) (*data++)) << 3;
            usersettings.pid_igain[x] = ((fixedpointnum) (*data++));
            if (x == NAVIGATIONINDEX)
                usersettings.pid_dgain[x] = ((fixedpointnum) (*data++)) << 8;
            else if (x == ALTITUDEINDEX)
                usersettings.pid_dgain[x] = ((fixedpointnum) (*data++)) << 9;
            else
                usersettings.pid_dgain[x] = ((fixedpointnum) (*data++)) << 2;

        }
// while testing, make roll pid equal to pitch pid so I only have to change one thing.
//usersettings.pid_pgain[ROLLINDEX]=usersettings.pid_pgain[PITCHINDEX];
//usersettings.pid_igain[ROLLINDEX]=usersettings.pid_igain[PITCHINDEX];
//usersettings.pid_dgain[ROLLINDEX]=usersettings.pid_dgain[PITCHINDEX];
        sendgoodheader(0);
    } else if (command == MSP_DEBUG) {  // send debug data
        sendgoodheader(8);
        for (int x = 0; x < 4; ++x) {
            int value = global.debugvalue[x];
            sendandchecksumdata((unsigned char *) &value, 2);
        }
    } else if (command == MSP_BOXNAMES) {       // send names of checkboxes
        char length = strlen(checkboxnames);
        sendgoodheader(length);
        sendandchecksumdata((unsigned char *) checkboxnames, length);
    } else if (command == MSP_SET_BOX) {        // receive check box settings
        unsigned char *ptr = (unsigned char *) usersettings.checkboxconfiguration;
        for (int x = 0; x < NUMCHECKBOXES * 2; ++x) {
            *ptr++ = *data++;
        }
    } else if (command == MSP_BOX) {    // send check box settings
        sendgoodheader(NUMCHECKBOXES * 2);
        sendandchecksumdata((unsigned char *) usersettings.checkboxconfiguration, NUMCHECKBOXES * 2);
    } else if (command == MSP_RESET_CONF) {     // reset user settings
        sendgoodheader(0);
        defaultusersettings();
    } else if (command == MSP_EEPROM_WRITE) {   // reset user settings
        sendgoodheader(0);
        if (!global.armed)
            writeusersettingstoeeprom();
    } else if (command == MSP_RAW_GPS) {        // reset user settings
        sendgoodheader(14);
        sendandchecksumcharacter(0);        // gps fix
        sendandchecksumcharacter(global.gps_num_satelites);
        sendandchecksumlong(lib_fp_multiply(global.gps_current_latitude, 156250L)); //156250L is 10,000,000L>>LATLONGEXTRASHIFT); 
        sendandchecksumlong(lib_fp_multiply(global.gps_current_longitude, 156250L));
        sendandchecksumint(global.gps_current_altitude >> FIXEDPOINTSHIFT); // gps altitude
        sendandchecksumint((global.gps_current_speed * 100) >>FIXEDPOINTSHIFT);     // gps speed
    } else if (command == MSP_COMP_GPS) {       // reset user settings
        sendgoodheader(5);
        sendandchecksumint((global.navigation_distance) >>FIXEDPOINTSHIFT);
        sendandchecksumint((global.navigation_bearing) >>FIXEDPOINTSHIFT);
        sendandchecksumcharacter(0);        // gps update
    } else if (command == MSP_RC_TUNING) {      // user settings
        sendgoodheader(7);
        sendandchecksumcharacter(0);        // rcRate
        sendandchecksumcharacter(0);        // rcExpo
        sendandchecksumcharacter(usersettings.maxpitchandrollrate >> (FIXEDPOINTSHIFT + 3));        // rollPitchRate
        sendandchecksumcharacter(usersettings.maxyawrate >> (FIXEDPOINTSHIFT + 2)); // yawRate
        sendandchecksumcharacter(0);        // dynThrPID
        sendandchecksumcharacter(0);        // thrMid8
        sendandchecksumcharacter(0);        // thrExpo8
    } else if (command == MSP_SET_RC_TUNING) {  // user settings
        data++;                 //rcRate
        data++;                 //rcExpo
        usersettings.maxpitchandrollrate = ((fixedpointnum) (*data++)) << (FIXEDPOINTSHIFT + 3);        // rollPitchRate
        usersettings.maxyawrate = ((fixedpointnum) (*data++)) << (FIXEDPOINTSHIFT + 2); // yawRate
        data++;                 // dynThrPID
        data++;                 // thrMid8
        data++;                 // thrExpo8
        sendgoodheader(0);
    }

    else                        // we don't know this command
    {
        senderrorheader();
    }
    lib_serial_sendchar(serialchecksum);
}

#define MAXPAYLOADSIZE 64

void serialcheckportforaction()
{
    int numcharsavailable;
    while ((numcharsavailable = lib_serial_numcharsavailable()) != 0) {
        if (serialreceivestate == SERIALSTATEGOTCOMMAND) {
            // this is the only state where we have to read more than one byte, so do this first, even though it's not first in the sequence of events
            // we need to wait for data plus the checksum.  But don't process until we have enough space in the output buffer
            int spaceneeded = 40;
            if (serialcommand == MSP_BOXNAMES)
                spaceneeded = strlen(checkboxnames) + 10;

            if (numcharsavailable > serialdatasize && lib_serial_availableoutputbuffersize() >= spaceneeded) {
                unsigned char data[MAXPAYLOADSIZE + 1];
                lib_serial_getdata(data, serialdatasize + 1);
                for (int x = 0; x < serialdatasize; ++x)
                    serialchecksum ^= data[x];
                if (serialchecksum == data[serialdatasize]) {
                    evaluatecommand(data);
                }
                serialreceivestate = SERIALSTATEIDLE;
            } else
                return;
        } else {
            unsigned char c = lib_serial_getchar();

            if (serialreceivestate == SERIALSTATEIDLE) {
                if (c == '$')
                    serialreceivestate = SERIALSTATEGOTDOLLARSIGN;
            } else if (serialreceivestate == SERIALSTATEGOTDOLLARSIGN) {
                if (c == 'M')
                    serialreceivestate = SERIALSTATEGOTM;
                else
                    serialreceivestate = SERIALSTATEIDLE;
            } else if (serialreceivestate == SERIALSTATEGOTM) {
                if (c == '<')
                    serialreceivestate = SERIALSTATEGOTLESSTHANSIGN;
                else
                    serialreceivestate = SERIALSTATEIDLE;
            } else if (serialreceivestate == SERIALSTATEGOTLESSTHANSIGN) {
                serialdatasize = c;
                if (c > MAXPAYLOADSIZE)
                    serialreceivestate = SERIALSTATEIDLE;
                else {
                    serialchecksum = c;
                    serialreceivestate = SERIALSTATEGOTDATASIZE;
                }
            } else if (serialreceivestate == SERIALSTATEGOTDATASIZE) {
                serialcommand = c;
                serialchecksum ^= c;
                serialreceivestate = SERIALSTATEGOTCOMMAND;
            }
        }
    }
}

#define SERIALTEXTDEBUG
#ifdef SERIALTEXTDEBUG
void serialprintnumber(long num, int digits, int decimals, char usebuffer)
   // prints a int number, right justified, using digits # of digits, puting a
   // decimal decimals places from the end, and using blank
   // to fill all blank spaces
{
    char stg[12];
    char *ptr;
    int x;

    ptr = stg + 11;

    *ptr = '\0';
    if (num < 0) {
        num = -num;
        *(--ptr) = '-';
    } else
        *(--ptr) = ' ';

    for (x = 1; x <= digits; ++x) {
        if (num == 0)
            *(--ptr) = ' ';
        else {
            *(--ptr) = 48 + num - (num / 10) * 10;
            num /= 10;
        }
        if (x == decimals)
            *(--ptr) = '.';
    }
    lib_serial_sendstring(ptr);
}

void serialprintfixedpoint(fixedpointnum fp)
{
    serialprintnumber(lib_fp_multiply(fp, 1000), 7, 3, 1);
    lib_serial_sendstring("\n\r");
}

void serialcheckportforactiontest()
{
    int numcharsavailable = lib_serial_numcharsavailable();
    if (numcharsavailable) {
        char c = lib_serial_getchar();
        lib_serial_sendstring("got char\n\r");

        if (c == 'r') {         // receiver values
            for (int x = 0; x < 6; ++x) {
                serialprintfixedpoint(global.rxvalues[x]);
            }
        } else if (c == 'g') {  // gyro values
            for (int x = 0; x < 3; ++x) {
                serialprintfixedpoint(global.gyrorate[x]);
            }
        } else if (c == 'a') {  // acc g values
            for (int x = 0; x < 3; ++x) {
                serialprintfixedpoint(global.acc_g_vector[x]);
            }
        } else if (c == 't') {  // atttude angle values
            for (int x = 0; x < 3; ++x) {
                serialprintfixedpoint(global.currentestimatedeulerattitude[x]);
            }
        } else if (c == 'e') {  // atttude angle values
            serialprintfixedpoint(global.estimateddownvector[0]);
            serialprintfixedpoint(global.estimateddownvector[1]);
            serialprintfixedpoint(global.estimateddownvector[2]);
            serialprintfixedpoint(global.estimatedwestvector[0]);
            serialprintfixedpoint(global.estimatedwestvector[1]);
            serialprintfixedpoint(global.estimatedwestvector[2]);
        } else if (c == 'd') {  // debug values
            for (int x = 0; x < 3; ++x)
                serialprintfixedpoint(global.debugvalue[x]);
        } else if (c == 'l') {  // altitude 
            serialprintfixedpoint(global.altitude);
        } else if (c == 'p') {  // atttude angle values
            serialprintfixedpoint(usersettings.pid_pgain[0]);
            serialprintfixedpoint(usersettings.pid_igain[0]);
            serialprintfixedpoint(usersettings.pid_dgain[0]);
        }
        lib_serial_sendstring("\n\r");
    }
}
#endif
#endif

void serialcheckforaction(void)
{
  // to be called by the main program every cycle so that we can check to see if we need to respond to incoming characters
#if (MULTIWII_CONFIG_SERIAL_PORTS)
    serialcheckportforaction();
#endif
}
