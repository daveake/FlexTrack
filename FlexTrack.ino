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

// CONFIGURATION SECTION.
//f
// Confine edits to this section only.

// RTTY settings
#define RTTY_PAYLOAD_ID   "CHANGE_ME"          // Do not use spaces.
#define RTTY_FREQUENCY    434.65               // For devices that are frequency-agile
#define RTTY_BAUD          50               // Comment out if not using RTTY
#define RTTY_SHIFT        425                // Only used on boards where PWM is used for RTTY.

// Power settings
#define POWERSAVING	1		        // GPS power saving

// LORA settings
#define LORA_PAYLOAD_ID   "CHANGE_ME"            // Do not use spaces.
#define LORA_SLOT            0
#define LORA_REPEAT_SLOT_1   0
#define LORA_REPEAT_SLOT_2   0
#define LORA_FREQUENCY       434.45
#define LORA_ID              1
#define LORA_CYCLETIME       0                // Set to zero to send continuously
#define LORA_MODE            0
#define LORA_BINARY          0

// APRS settings
#define APRS_CALLSIGN    "CHANGE"              // Comment out to disable APRS         
#define APRS_SSID            11
#define APRS_TX_INTERVAL      1                 // APRS TX Interval in minutes


// CHOOSE BOARD (comment out one of these only)
// #define HABDUINO
// #define UAVANUT-LORA


// PRESET BOARD CONFIGURATIONS - THESE DEFINE THE ATTACHED HARDWARE

#ifdef UAVANUT-LORA
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
  #define RTTY_DATA          11
  #define RTTY_PWM            1
  #define APRS_ENABLE         6
  #define APRS_DATA           3                // Comment out to disable APRS
  
  #define A0_MULTIPLIER      4.9
  
  #define WIREBUS             5
  
  #define MTX2
#endif

#ifdef SPECIAL
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
  byte Lock;
  byte psm_status;
  float InternalTemperature;
  float BatteryVoltage;
  float ExternalTemperature;
  float Pressure;
  unsigned int BoardCurrent;
  unsigned int errorstatus;
  byte navmode;
} GPS;


int SentenceCounter=0;

//------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);

  Serial.println("");
  Serial.print("FlexTrack Flight Computer, payload ID ");
  Serial.println(RTTY_PAYLOAD_ID);
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

#ifdef WIREBUS
  Checkds18b20();
#endif
  
#ifdef POWERSAVING
  CheckPowerSaving();
#endif
  
  checkDynamicModel();  
}


int freeRam(void)
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
