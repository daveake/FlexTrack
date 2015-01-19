/*------------------------------------------------------------------------------------------------------*\
|                                                                                                        |
| New tracker code that can be simply rebuilt for various hardware designs.  e.g. serial or i2c GPS,     |
| NTX2B or LoRa radio.  Using LoRa, it can configured to use TDMA in which case it will acceput uplink   |
| messages and/or repeat messages from other balloons.                                                   |
|                                                                                                        |
| Configuration is using #defines near the top of the file.  These control which other modules get       |
| used and linked in, and configure those modules (e.g. radio frequency).                                |
|                                                                                                        |
| V0.00   First stab                                                                                     |
|                                                                                                        |
\*------------------------------------------------------------------------------------------------------*/

// CONFIGURATION SECTION.
//
// Confine edits to this section only.

#define PAYLOAD_ID  "CHANGE_ME"                 // Do not use spaces.

#define APRS_CALLSIGN    "ME_TOO"           
#define APRS_SSID            11
#define APRS_TX_INTERVAL      1                // APRS TX Interval in minutes

#define HABDUINO

#ifdef HABDUINO
  #define LED_WARN           12
  #define LED_OK             13
  #define GPS_ON              2
  #define RTTY_BAUD          50
  #define RTTY_ENABLE         7
  #define RTTY_DATA          11
  #define RTTY_SHIFT        425                // Only define if you want to use PWM control. Do not define if shift is set by resistor network.

  #define APRS_ENABLE         6
  #define APRS_DATA           3                // Comment out to disable APRS
  
  #define A0_MULTIPLIER      4.9
  
  #define WIREBUS             5
  
  #define MTX2
  #define RTTY_FREQUENCY    434.65
#else
  // GPS Section
  #define  GPS_I2C  1                          // Comment out if using serial GPS

  // Radio section.
  #define  LORA_NSS           5                // Comment out to disable LoRa code
  #define  LORA_DIO0          3                
  #define  LORA_DIO5          2
  #define  LORA_ID            2
  #define  LORA_SLOT         12
  #define  LORA_REPEAT_SLOT  14
  #define  LORA_CYCLETIME    15                  // Set to zero to send continuously
  #define  LORA_MODE          2
  #define  LORA_BINARY        1
#endif

#define EXTRA_FIELD_FORMAT    ",%d,%d,%d,%d,%d"          // List of formats for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define EXTRA_FIELD_LIST      ,(int)((GPS.Speed * 13) / 7), GPS.Direction, GPS.Satellites, DS18B20_Temperatures[0], Channel0Average
                                                  // List of variables/expressions for extra fields. Make empty if no such fields.  Always use comma at start of there are any such fields.
#define SENTENCE_LENGTH      100                  // This is more than sufficient for the standard sentence.  Extend if needed; shorten if you are tight on memory.

    /*
            "$$%s,%d,%02d:%02d:%02d,%s,%s,%05.5u,%d,%d,%d",
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
	uint16_t	Altitude;
};  //  __attribute__ ((packed));

struct TGPS
{
  int Hours, Minutes, Seconds;
  long SecondsInDay;					// Time in seconds since midnight
  float Longitude, Latitude;
  unsigned int Altitude;
  unsigned int Satellites;
  int Speed;
  int Direction;
  byte GotTime;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
} GPS;


int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);

  Serial.println("");
  Serial.print("FlexTrack Flight Computer, payload ID ");
  Serial.println(PAYLOAD_ID);
  Serial.println("");

#ifdef LED_WARN
  pinMode(LED_WARN, OUTPUT);
  digitalWrite(LED_WARN, 1);
#endif

#ifdef LED_OK
  pinMode(LED_OK, OUTPUT);
  digitalWrite(LED_OK, 0);
#endif

#ifdef GPS_I2C
  Serial.println("I2C GPS");
#else
  Serial.println("Serial GPS");
#endif
  
#ifdef LORA_NSS
  Serial.println("LoRa telemetry enabled");
#endif

#ifdef RTTY_BAUD
  Serial.println("RTTY telemetry enabled");
 #endif

#ifdef APRS_DATA 
  Serial.println("APRS telemetry enabled");
#endif

  Serial.print("Free memory = ");
  Serial.println(freeRam());
  
  SetupGPS();
  
  SetupADC();
  
  #ifdef LORA_NSS
    SetupLoRa();
  #endif
  
  #ifdef RTTY_BAUD 
    SetupRTTY();
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
    CheckRTTY();
  #endif
  
  #ifdef LORA_NSS
    CheckLoRa();
  #endif
  
  #ifdef APRS_DATA
    CheckAPRS();
  #endif
  
  CheckADC();

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
