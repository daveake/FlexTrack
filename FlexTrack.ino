/*------------------------------------------------------------------------------------------------------*\
|                                                                                                        |
| New tracker code that can be simply rebuilt for various hardware designs.  e.g. serial or i2c GPS,     |
| NTX2B or LoRa radio.  Using LoRa, it can configured to use TDMA in which case it will accept uplinked  |
| messages and/or repeat messages from other balloons.                                                   |
|                                                                                                        |
| Configuration is using #defines near the top of the file.  These control which other modules get       |
| used and linked in, and configure those modules (e.g. radio frequency).                                |
|                                                                                                        |
| V0.00   First stab                                                                                     |
|                                                                                                        |
\*------------------------------------------------------------------------------------------------------*/

#include <avr/pgmspace.h>

//------------------------------------------------------------------------------------------------------

// CONFIGURATION SECTION.

// Edit this section to choose the hardware design and set your payload ID etc

// CHOOSE BOARD (comment out one of these only)
#define HABDUINO
// #define UAVANUT_LORA
// #define HS_APRS_300
// #define HS_RTTY_300      

// RTTY settings
#define RTTY_PAYLOAD_ID   "CHANGE_ME"          // Do not use spaces.
#define RTTY_FREQUENCY    434.65               // For devices that are frequency-agile
#define RTTY_BAUD          50               // Comment out if not using RTTY
#define RTTY_SHIFT        425                // Only used on boards where PWM is used for RTTY.

// Power settings
// #define POWERSAVING	                      // Comment out to disable GPS power saving

// LORA settings
#define LORA_PAYLOAD_ID   "OO5"            // Do not use spaces.
#define LORA_SLOT            11
#define LORA_REPEAT_SLOT_1   -1
#define LORA_REPEAT_SLOT_2   -1

#define LORA_TIME_INDEX      2
#define LORA_TIME_MUTLIPLER  2
#define LORA_TIME_OFFSET     1
#define LORA_PACKET_TIME    500
#define LORA_FREQUENCY       434.45

#define LORA_ID              0
#define LORA_CYCLETIME       20                // Set to zero to send continuously
#define LORA_MODE            2
#define LORA_BINARY          0

// APRS settings
#define APRS_CALLSIGN    "CHANGE"               // Max 6 characters
#define APRS_SSID            11
#define APRS_PATH_ALTITUDE   1500              // Below this altitude, ** in metres **, path will switch to WIDE1-1, WIDE2-1.  Above it will be or path or WIDE2-1 (see below)
#define APRS_HIGH_USE_WIDE2    1                 // 1 means WIDE2-1 is used at altitude; 0 means no path is used

#define APRS_TX_INTERVAL      1                 // APRS TX Interval in minutes
#define APRS_PRE_EMPHASIS                      // Comment out to disable 3dB pre-emphasis.
#define APRS_RANDOM          30                // Adjusts time to nexr transmission by up to +/1 this figure, in seconds.
                                               // So for interval of 1 (minute), and random(30), each gap could be 30 - 90 seconds.
                                               // Set to 0 to disable
#define APRS_COMMENT     "www.daveakerman.com"   
#define APRS_TELEM_INTERVAL  2                // How often to send telemetry packets.  Comment out to disable

//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------

// HARDWARE DEFINITIONS

// For unsupported hardware, add your own section here

#ifdef HS_APRS_300
  #define GPS_I2C
  #define LED_STATUS         A2
  #define LED_TX             A3
  #define APRS_ENABLE         6
  #define APRS_DATA           3         
  
  #define A0_MULTIPLIER      4.9
  
  #define WIREBUS             5
#endif

#ifdef HS_RTTY_300
  #define GPS_I2C
  #define LED_STATUS         A2
  #define LED_TX             A3
  #define RTTY_ENABLE         6
  #define RTTY_DATA           3 
  #define A0_MULTIPLIER      4.9
  
  #define WIREBUS             5
#endif

#ifdef UAVANUT_LORA
  #define GPS_I2C             1                // Comment out if using serial GPS
  #define LORA_NSS           10                // Comment out to disable LoRa code
  #define LORA_RESET          7                // Comment out if not connected
  #define LORA_DIO0           3                
  #define LORA_DIO5           2
  #define LED_WARN            5
  #define LED_OK              6
  #define A0_MULTIPLIER      4.9
#endif

#ifdef HABDUINO
  #define LED_WARN           12
  #define LED_OK             13
  #define GPS_ON              2
  #define RTTY_ENABLE         7
  #define RTTY_DATA           4
  #define APRS_ENABLE         6
  #define APRS_DATA           3                // Comment out to disable APRS  
  #define A0_MULTIPLIER      4.9
  #define WIREBUS             5
  #define MTX2
#endif

#ifdef UAB
  #define LED_OK              7
  #define APRS_ENABLE         6
  #define APRS_DATA           3         
  #define GPS_SERIAL          Serial1
  #define DEBUG_SERIAL        Serial
#endif

//------------------------------------------------------------------------------------------------------

// Default serial port usage
#ifndef GPS_SERIAL
  #ifndef GPS_I2C
    #define GPS_SERIAL Serial
  #endif
#endif

#ifndef DEBUG_SERIAL
  #define DEBUG_SERIAL Serial
#endif

#define EXTRA_FIELD_FORMAT    ",%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define EXTRA_FIELD_LIST           ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites

// #define EXTRA_FIELD_FORMAT    ""   // ",%d,%d,%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
// #define EXTRA_FIELD_LIST           // ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites, DS18B20_Temperatures[0], Channel0Average
                                                                // List of variables/expressions for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define SENTENCE_LENGTH      100                  // This is more than sufficient for the standard sentence.  Extend if needed; shorten if you are tight on memory.

    /*
            "$%s,%d,%02d:%02d:%02d,%s,%s,%05.5u,%d,%d,%d",
            PAYLOAD_ID,
            SentenceCounter,
	    GPS.Hours, GPS.Minutes, GPS.Seconds,
            LatitudeString,
            LongitudeString,
            GPS.Altitude,
            (int)((GPS.Speed * 13) / 7),
            GPS.Direction,
            GPS.Satellites);
    */


//------------------------------------------------------------------------------------------------------
//
//  Globals

struct TBinaryPacket
{
	uint8_t 	PayloadIDs;
	uint16_t	Counter;
	uint16_t	BiSeconds;
	float		Latitude;
	float		Longitude;
	int32_t  	Altitude;
};  //  __attribute__ ((packed));

struct TGPS
{
  int Hours, Minutes, Seconds;
  unsigned long SecondsInDay;					// Time in seconds since midnight
  float Longitude, Latitude;
  long Altitude;
  unsigned int Satellites;
  int Speed;
  int Direction;
  byte FixType;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte FlightMode;
  byte PowerMode;
} GPS;


int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------

void setup()
{
  // Serial port(s)
  
  #ifdef GPS_SERIAL
    GPS_SERIAL.begin(9600);
  #endif
  
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(9600);
    Serial.println("");
    Serial.print("FlexTrack Flight Computer, payload ID(s)");
    #ifdef RTTY_DATA
      Serial.print(' ');
      Serial.print(RTTY_PAYLOAD_ID);
    #endif  
    #ifdef LORA_NSS
      Serial.print(' ');
      Serial.print(LORA_PAYLOAD_ID);
    #endif  
    #ifdef APRS_DATA
      Serial.print(' ');
      Serial.print(APRS_CALLSIGN);
    #endif  
      
    Serial.println("");
    Serial.println("");
  #endif

#ifdef GPS_I2C
  Serial.println(F("I2C GPS"));
#else
  Serial.println(F("Serial GPS"));
#endif

#ifdef LORA_NSS
  Serial.println(F("LoRa telemetry enabled"));
#endif

#ifdef RTTY_BAUD
  #ifdef RTTY_DATA
    Serial.println(F("RTTY telemetry enabled"));
  #endif
#endif

#ifdef APRS_DATA 
  Serial.println(F("APRS telemetry enabled"));
#endif

  Serial.print(F("Free memory = "));
  Serial.println(freeRam());

  SetupLEDs();
  
  SetupGPS();
  
  SetupADC();
  
#ifdef LORA_NSS
  SetupLoRa();
#endif

#ifdef RTTY_BAUD
#ifdef RTTY_DATA
  SetupRTTY();
#endif
#endif

#ifdef APRS_DATA
  SetupAPRS();
#endif
  
#ifdef WIREBUS
  Setupds18b20();
#endif
}


void loop()
{  
  CheckGPS();

#ifdef RTTY_BAUD
#ifdef RTTY_DATA
  CheckRTTY();
#endif
#endif
  
#ifdef LORA_NSS
  CheckLoRa();
#endif
  
#ifdef APRS_DATA
  CheckAPRS();
#endif
  
  CheckADC();
  
  CheckLEDs();

#ifdef WIREBUS
  Checkds18b20();
#endif
}


int freeRam(void)
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
