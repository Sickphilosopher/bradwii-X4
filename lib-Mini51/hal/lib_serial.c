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

// This library manages serial ports.  It sets up input and output buffers and sends and receieves using interrupts.
// To use this code, define which ports to include and the size of their buffers in projectsettings.h
// Then use the library like this:
//
// lib_serial_initport(2,9600); // init serial port 2 to 9600 baud
// lib_serial_sendstring(2,"Send this string");
// lib_serial_sendchar(2,'.');
// lib_serial_senddata(2,"This is Data",4); // sends the first 4 characters of the string
// int count=lib_serial_numcharsavailable(2);
// if (count>0)
//      {
//      lib_serial_getdata(2,data,count);
//      }

#include "hal.h"
#include "lib_serial.h"
#include "projectsettings.h"
#include "drv_serial.h"

static serialPort_t *lib_serial_getport()
{
  return &serialPort1;
}

int lib_serial_availableoutputbuffersize()
{
    // returns how many more bytes can fit in the outputbuffer
    return 256; // TODO who cares
}

// TODO: not implemented
// used only in rx.c for decoding of some receivers
//void lib_serial_setrxcallback(serialcallbackfunctptr callback)
//{
//    serialPort_t *port = lib_serial_getport();
//    port->callback = callback;
//}

void lib_serial_initport(long baud)
{
  serialOpen(UART, NULL, baud, MODE_RXTX);
}

void lib_serial_sendchar(unsigned char c)
{
    // add a character to the send buffer
    serialPort_t *port = lib_serial_getport();
    uartWrite(port, c);
}
   
void lib_serial_sendstring(char *string)
{
    // adds the string to the output buffer.
    while (*string) 
        lib_serial_sendchar(*string++);
}

void lib_serial_senddata(unsigned char *data, int datalength)
{
    // send datalength bytes of data to the serial port
    while (datalength-- >0) 
        lib_serial_sendchar(*data++);
}

int lib_serial_numcharsavailable()
{
    // returns number of characters available in the rx buffer
    serialPort_t *port = lib_serial_getport();
    return uartAvailable(port);
}

unsigned char lib_serial_getchar()
{
    // get the next character from the serial port
    serialPort_t *port = lib_serial_getport();
    return uartRead(port);
}

void lib_serial_getdata(unsigned char *data, int numchars)
{
    int x;
    for (x = 0; x < numchars; ++x)
        *data++ = lib_serial_getchar();
}
