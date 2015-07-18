
/* From Project Swift - High altitude balloon flight software            */
/*=======================================================================*/
/* Copyright 2010-2015 Philip Heron <phil@sanslogic.co.uk>               */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#ifdef APRS_DATA

#include <util/crc16.h>
#include <avr/pgmspace.h>

#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (50)
#define REST_BYTES     (5)

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

#define APRS_DEVID "APEHAB"

/* External variables */
extern int DS18B20_Temperatures[];
extern unsigned int Channel0Average;
/* These and an extern for GPS should be in a common include file */

// Our variables

unsigned long NextAPRS=0;
int aprstxstatus=0;
unsigned int APRSSentenceCounter;
volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;
static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

// Code

void SetupAPRS(void)
{
#ifdef APRS_ENABLE
  pinMode(APRS_ENABLE, OUTPUT);
  digitalWrite(APRS_ENABLE, 0);
#endif

  // Fast PWM mode, non-inverting output on OC2A
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  
  pinMode(APRS_DATA, OUTPUT);
}

void CheckAPRS(void)
{
  if ((millis() >= NextAPRS) && (GPS.Satellites >= 4))
  {
    // Set time for next transmission
    NextAPRS = millis() + (unsigned long)APRS_TX_INTERVAL * 60000L;

    Serial.println("Sending APRS Packet");
    
    tx_aprs();
  }
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid, char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Key the radio and enable the AX25 timer interrupt */
  digitalWrite(APRS_ENABLE, 1);
  TIMSK2 |= _BV(TOIE2);
}

void tx_aprs(void)
{
  aprstxstatus=1;
  
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  int32_t aprs_lat, aprs_lon, aprs_alt;

  // Convert the coordinates to the APRS compressed format
  aprs_lat = 380926 * (90.0 - GPS.Latitude);
  aprs_lon = 190463 * (180.0 - GPS.Longitude);
  aprs_alt = GPS.Altitude * 32808 / 10000;

  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);
  ax25_base91enc(stlm + 2, 2, GPS.Satellites);
  ax25_base91enc(stlm + 4, 2, DS18B20_Temperatures[0]);
  ax25_base91enc(stlm + 6, 2, Channel0Average);
  ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    APRS_DEVID, 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
    "!/%s%sO   /A=%06ld|%s|%s",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, APRS_COMMENT
  );

  /* Send the telemetry definitions every 10 packets */
  if(seq % (APRS_TELEM_INTERVAL) == 0)
  {
    char s[10];

    /* Need CALLSIGN-SSID as string */
    strncpy_P(s, PSTR(APRS_CALLSIGN), 7);
    if(APRS_SSID) snprintf_P(s + strlen(s), 4, PSTR("-%i"), APRS_SSID);

    /* Transmit telemetry definitions */
    ax25_frame(
      APRS_CALLSIGN, APRS_SSID,
      APRS_DEVID, 0,
      0, 0, 0, 0,
      ":%-9s:PARM.Satellites,Temperature,Battery", s
    );
    ax25_frame(
      APRS_CALLSIGN, APRS_SSID,
      APRS_DEVID, 0,
      0, 0, 0, 0,
      ":%-9s:UNIT.Sats,deg.C,Volts", s
    );
    ax25_frame(
      APRS_CALLSIGN, APRS_SSID,
      APRS_DEVID, 0,
      0, 0, 0, 0,
      ":%-9s:EQNS.0,1,0,0,1,0,0,0.001,0", s
    );
  }

  seq++;
}

ISR(TIMER2_OVF_vect)
{
  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;
  uint8_t sin;

  /* Update the PWM output */
  sin = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  if(step == PHASE_DELTA_1200) sin = (sin >> 1) + 64; /* Crude pre-emphasis */
  OCR2B = sin;

  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        // Disable radio and interrupt

        digitalWrite(APRS_ENABLE, 0);
        aprstxstatus=0;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;
        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

#endif
