/* 
Copyright 2014 Victor Joukov

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


#include "hal.h"
#include "bradwii.h"
#include "rx.h"
#include "defs.h"
#include "lib_timers.h"
#include "nrf24l01.h"
//#include "lib_digitalio.h"
//#include "lib_serial.h"

// when adding new receivers, the following functions must be included:
// initrx()   // initializes the r/c receiver
// readrx()   // loads global.rxvalues with r/c values as fixedpointnum's from -1 to 1 (0 is the center).

unsigned char channel_index[] = { THROTTLEINDEX,PITCHINDEX,YAWINDEX,ROLLINDEX,AUX1INDEX,AUX2INDEX,AUX3INDEX,AUX4INDEX,8 };

extern globalstruct global;
extern usersettingsstruct usersettings;        // user editable variables


#define BV(x) (1 << (x))

///////////////////////////////////////////////////////////////////////
// Definitions for a group of nRF24L01 based protocols
enum Proto {
    PROTO_NONE = 0,
    PROTO_SYMAX
};

enum BindState {
  WAIT_BIND_PACKET = 0,
  JUST_BOUND,
  BINDING,
  LOST_BINDING,
  BOUND
};


#define BIND_COUNT			345		// 1.5 seconds
#define FIRST_PACKET_DELAY	12000
#define PACKET_PERIOD			4000	// Timeout for callback in uSec

#define RF_CHANNELS    	4

// flags going to packet[4]
#define FLAG_PICTURE  0x40
#define FLAG_VIDEO    0x80
// flags going to packet[6]
#define FLAG_FLIP     0x40
// flags going to packet[7]
#define FLAG_HEADLESS 0x80

#define PAYLOADSIZE		10
#define PACKET_SIZE		10

static const uint8_t bind_freq_hopping [RF_CHANNELS] =  {
  0x4b, 0x30, 0x40, 0x20
};

static const uint8_t main_freq_hopping_0[] = {0x0a, 0x1a, 0x2a, 0x3a};
static const uint8_t main_freq_hopping_1[] = {0x2a, 0x0a, 0x42, 0x22};
static const uint8_t main_freq_hopping_2[] = {0x1a, 0x3a, 0x12, 0x32};

static uint8_t txid[5];
static uint8_t rf_channels [RF_CHANNELS];
static uint8_t packet[PAYLOADSIZE];

static uint8_t rf_ch_num;
static uint8_t nfreqchannels;
static uint8_t bind_phase;
static uint8_t boundstate;
static uint32_t packet_timer;

//static uint32_t rx_timeout;
//static uint32_t valid_packets;
//static uint32_t missed_packets;
//static uint32_t bad_packets;
#define valid_packets (global.debugvalue[0])
#define missed_packets (global.debugvalue[1])
#define bad_packets (global.debugvalue[2])
#define rx_timeout (global.debugvalue[3])


uint8_t checksum(uint8_t *data)
{
    uint8_t sum = data[0];

    for (uint8_t i=1; i < PACKET_SIZE-1; i++)
      sum ^= data[i];
    return sum +  0x55;
}

static void set_main_fh(uint8_t address)
{
  uint8_t laddress = address & 0x1f;
  uint8_t i;
  uint32_t *pchans = (uint32_t *)rf_channels;   // avoid compiler warning
  if (laddress < 0x10)
  {
      if (laddress == 6)
          laddress = 7;
      for(i=0; i < RF_CHANNELS; i++)
          rf_channels[i] = main_freq_hopping_0[i] + laddress;
  }
  else
    if (laddress < 0x18)
    {
        for(i=0; i < RF_CHANNELS; i++)
            rf_channels[i] = main_freq_hopping_1[i] + (laddress & 0x07);
        if (laddress == 0x16)
        {
            rf_channels[0]++;
            rf_channels[1]++;
        }
    }
    else
      if (laddress < 0x1e)
      {
          for(i=0; i < RF_CHANNELS; i++)
          rf_channels[i] = main_freq_hopping_2[i] + (laddress & 0x07);
      }
      else
        if (laddress == 0x1e)
            *pchans = 0x38184121;
        else
            *pchans = 0x39194121;
}

static void set_tx_id(uint8_t *packet)
{
    txid[0] = packet[4];
    txid[1] = packet[3];
    txid[2] = packet[2];
    txid[3] = packet[1];
    txid[4] = packet[0];
    
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, txid, 5);
    set_main_fh(txid[0]);
}

static void prepare_to_bind(void)
{
  packet_timer = lib_timers_starttimer();
  for (int i = 0; i < RF_CHANNELS; ++i) {
    rf_channels[i] = bind_freq_hopping[i];
  }
  nfreqchannels = RF_CHANNELS;
  rx_timeout = 1000L;
}


static void switch_channel(void)
{
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[rf_ch_num]);
    if (++rf_ch_num >= nfreqchannels) rf_ch_num = 0;
    uint8_t config = BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PRIM_RX);
    config |= BV(NRF24L01_00_PWR_UP);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
}

// The Beken radio chip can be improperly reset
// We try to set it into Bank 0, before that we try
// to verify that it is there at all
static void reset_beken(void)
{
    NRF24L01_Activate(0x53); // magic for BK2421/BK2423 bank switch
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        NRF24L01_Activate(0x53); // switch to register set 0
    }
}

// Check for Beken BK2421/BK2423 chip
// It is done by using Beken specific activate code, 0x53
// and checking that status register changed appropriately
// There is no harm to run it on nRF24L01 because following
// closing activate command changes state back even if it
// does something on nRF24L01
// For detailed description of what's happening here see 
//   http://www.inhaos.com/uploadfile/otherpic/BK2423%20Datasheet%20v2.0.pdf
//   http://www.inhaos.com/uploadfile/otherpic/AN0008-BK2423%20Communication%20In%20250Kbps%20Air%20Rate.pdf
static void initialize_beken(void)
{
    NRF24L01_Activate(0x53); // magic for BK2421/BK2423 bank switch
    printf("Trying to switch banks\n");
    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & 0x80) {
        printf("BK2421 detected\n");
        // Beken registers don't have such nice names, so we just mention
        // them by their numbers
        // It's all magic, eavesdropped from real transfer and not even from the
        // data sheet - it has slightly different values
        NRF24L01_WriteRegisterMulti(0x00, (uint8_t *) "\x40\x4B\x01\xE2", 4);
        NRF24L01_WriteRegisterMulti(0x01, (uint8_t *) "\xC0\x4B\x00\x00", 4);
        NRF24L01_WriteRegisterMulti(0x02, (uint8_t *) "\xD0\xFC\x8C\x02", 4);
        NRF24L01_WriteRegisterMulti(0x03, (uint8_t *) "\x99\x00\x39\x21", 4);
        NRF24L01_WriteRegisterMulti(0x04, (uint8_t *) "\xF9\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x05, (uint8_t *) "\x24\x06\x7F\xA6", 4);
        NRF24L01_WriteRegisterMulti(0x0C, (uint8_t *) "\x00\x12\x73\x00", 4); // PLL locking time 130us, like nRF24L01
        NRF24L01_WriteRegisterMulti(0x0D, (uint8_t *) "\x46\xB4\x80\x00", 4);
        NRF24L01_WriteRegisterMulti(0x0E, (uint8_t *) "\xFF\xEF\x7D\xF2\x08\x08\x20\x82\x04\x10\x41", 11);
        NRF24L01_WriteRegisterMulti(0x0E, (uint8_t *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
        NRF24L01_WriteRegisterMulti(0x04, (uint8_t *) "\xFF\x96\x82\x1B", 4);
        NRF24L01_WriteRegisterMulti(0x04, (uint8_t *) "\xF9\x96\x82\x1B", 4);
    } else {
        printf("nRF24L01 detected\n");
    }
    NRF24L01_Activate(0x53); // switch bank back
}

void initrx(void)
{
    NRF24L01_Initialize();
    
    reset_beken();

    // 2-bytes CRC, radio off
    uint8_t config = BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PRIM_RX);

    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);      // Channel 8 - bind
    NRF24L01_SetBitrate(NRF24L01_BR_250K);                          // 1Mbps
    NRF24L01_SetPower(TXPOWER_100mW);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, PAYLOADSIZE);  // bytes of data payload for pipe 0
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    uint8_t bind_rx_addr[] = {0xAB, 0xAC, 0xAD, 0xAE, 0xAF};
    //AB AC AD AE AF
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, "\xC3", 1);
    
    initialize_beken();

    lib_timers_delaymilliseconds(50);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    rf_ch_num = 0;

    // Turn radio power on
    config |= BV(NRF24L01_00_PWR_UP);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
    
    lib_timers_delaymilliseconds(1); // 6 times more than needed
    
    valid_packets = missed_packets = bad_packets = 0;
    
    bind_phase = WAIT_BIND_PACKET;
    prepare_to_bind();
    
    switch_channel();
}

static void decode_bind_packet(uint8_t *packet)
{
  if (packet[5] == 0xAA && packet[6] == 0xAA && packet[7] == 0xAA) {
    set_tx_id(&packet[0]);
    boundstate = BOUND;
    switch_channel();
  }
}

// Returns whether the data was successfully decoded
static bool decode_packet(uint8_t *packet, uint16_t *data)
{
    switch (boundstate) {
    case WAIT_BIND_PACKET:
      decode_bind_packet(packet);
      return false;
    case BOUND:
      uint8_t sum = checksum(packet);
      if (packet[9] != sum)
      {
        bad_packets++;
        printf("wrong checksum");
        return false;
      }
      rx_timeout = 10000L; // 4ms interval, duplicate packets, (8ms unique) + 25%
      // TREA order in packet to MultiWii order is handled by
      // correct assignment to channelindex
      // Throttle 0..255 to 1000..2000
      data[channel_index[0]] = ((uint16_t)packet[0]) * 1000 / 255 + 1000;
      for (int i = 1; i < 4; ++i) {
          uint8_t a = packet[i];
          data[channel_index[i]] = ((uint16_t)(a < 0x80 ? 0x7f - a : a)) * 1000 / 255 + 1000;
      }
      data[PITCHINDEX] = 3000 - data[PITCHINDEX];//reverse pitch
      return true;
    }
    return false;
}

void readrx(void)
{
    int chan;
    uint16_t data[8];
    if (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR))) {
        uint32_t t = lib_timers_gettimermicroseconds(packet_timer);
        if (t > rx_timeout)
        {
          // if (boundstate == BOUND)
          // {
          //   if (++missed_packets > 500 && boundstate == JUST_BOUND)
          //   {
          //     valid_packets = missed_packets = bad_packets = 0;
          //     bind_phase = LOST_BINDING;
          //     prepare_to_bind();
          //   }
          // }
          // else
          // {
          //   switch_channel();
          // }
          switch_channel();
          packet_timer = lib_timers_starttimer();
        }
        return;
    }
    packet_timer = lib_timers_starttimer();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BV(NRF24L01_07_RX_DR));
    NRF24L01_ReadPayload(packet, PAYLOADSIZE);
    NRF24L01_FlushRx();
    switch_channel();
    // for (int i = 0; i < 10; i++)
    // {
    //   printf("%02X ", packet[i]);
    // }
    // printf("\n");
    
    if (!decode_packet(packet, data))
        return;
    
    for (chan = 0; chan < 8; ++chan) 
    {
        // convert from 1000-2000 range to -1 to 1 fixedpointnum range and low pass filter to remove glitches
        lib_fp_lowpassfilter(&global.rxvalues[chan], ((fixedpointnum) data[chan] - 1500) * 131L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
    }
    // reset the failsafe timer
    global.failsafetimer = lib_timers_starttimer();
}
