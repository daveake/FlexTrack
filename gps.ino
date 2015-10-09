/* ========================================================================== */
/*   gps.ino                                                                  */
/*                                                                            */
/*   Serial and i2c code for ublox on AVR                                     */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */

// Include i2c library, if using it

#ifdef GPS_I2C
//  #include <Wire.h>
  #include <I2C.h>
#endif


unsigned long TimeForFlightMode=0;

char Hex(char Character)
{
  char HexTable[] = "0123456789ABCDEF";
	
  return HexTable[Character];
}

int GPSChecksumOK(char *Buffer, int Count)
{
  unsigned char XOR, i, c;

  XOR = 0;
  for (i = 1; i < (Count-4); i++)
  {
    c = Buffer[i];
    XOR ^= c;
  }

  return (Buffer[Count-4] == '*') && (Buffer[Count-3] == Hex(XOR >> 4)) && (Buffer[Count-2] == Hex(XOR & 15));
}

void SendUBX(unsigned char *Message, int Length)
{ 
#ifdef GPS_I2C  
  I2c.write(0x42, 0, Message, Length);
#else
  int i;

  for (i=0; i<Length; i++)
  {
    Serial.write(Message[i]);
  }
#endif
}

void SetFlightMode(void)
{
    // Send navigation configuration command
    unsigned char setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
    SendUBX(setNav, sizeof(setNav));
    Serial.println("Setting flight mode\n");
}

void SetGNSSMode(void)
 {
  // Sets CFG-GNSS to disable everything other than GPS GNSS
  // solution. Failure to do this means GPS power saving 
  // doesn't work. Not needed for MAX7, needed for MAX8's
  uint8_t setGNSS[] = {
    0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00, 0x00, 0x00,
    0x20, 0x05, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
    0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00,
    0x01, 0x01, 0xFC, 0x11   };
    SendUBX(setGNSS, sizeof(setGNSS));
} 

float FixPosition(float Position)
{
  float Minutes, Seconds;
	
  Position = Position / 100;
	
  Minutes = trunc(Position);
  Seconds = fmod(Position, 1);

  return Minutes + Seconds * 5 / 3;
}

void ProcessLine(char *Buffer, int Count)
{
  int Satellites, date;
  char ns, ew;
  char TimeString[16], LatString[16], LongString[16], Temp[4];
	
  if (GPSChecksumOK(Buffer, Count))
  {
    Satellites = 0;
	
    if (strncmp(Buffer+3, "GGA", 3) == 0)
    {
      int lock;
      char hdop[16], Altitude[16];
      
      Serial.print(Buffer+1);
      #ifdef LED_WARN
        digitalWrite(LED_WARN, GPS.Altitude < 1000);
      #endif
      
      if (sscanf(Buffer+7, "%16[^,],%16[^,],%c,%[^,],%c,%d,%d,%[^,],%[^,]", TimeString, LatString, &ns, LongString, &ew, &lock, &Satellites, hdop, Altitude) >= 1)
      {	
        // $GPGGA,124943.00,5157.01557,N,00232.66381,W,1,09,1.01,149.3,M,48.6,M,,*42
        Temp[0] = TimeString[0]; Temp[1] = TimeString[1]; Temp[2] = '\0';
        GPS.Hours = atoi(Temp);
        Temp[0] = TimeString[2]; Temp[1] = TimeString[3]; Temp[2] = '\0';
        GPS.Minutes = atoi(Temp);
        Temp[0] = TimeString[4]; Temp[1] = TimeString[5]; Temp[2] = '\0';
        GPS.Seconds = atoi(Temp);
        GPS.SecondsInDay = (unsigned long)GPS.Hours * 3600L + (unsigned long)GPS.Minutes * 60L + (unsigned long)GPS.Seconds;

        #ifdef LED_OK        
          digitalWrite(LED_OK, GPS.Altitude < 1000);
        #endif

        if (Satellites >= 4)
        {
          GPS.Latitude = FixPosition(atof(LatString));
          if (ns == 'S') GPS.Latitude = -GPS.Latitude;
          GPS.Longitude = FixPosition(atof(LongString));
          if (ew == 'W') GPS.Longitude = -GPS.Longitude;
          GPS.Altitude = (unsigned int)atof(Altitude);
        }
        
        GPS.Lock = lock;
        GPS.Satellites = Satellites;
        GPS.GotTime = 1;
      }
      else
      {
        GPS.GotTime = 0;
      }
    }
    else if (strncmp(Buffer+3, "RMC", 3) == 0)
    {
      // $GPRMC,224008.00,A,5157.01406,N,00232.65882,W,0.087,,070115,,,A*64
      char *ptr, *ptr2;
      int i;
      
      Serial.print(Buffer+1);

      for (i=0,ptr=Buffer; i<7; i++)
      {
          ptr = strchr(ptr, ',') + 1;
      }
      
      ptr2 = strchr(ptr, ',');
      if (ptr2)
      {
        *ptr2 = '\0';
        GPS.Speed = (int)atof(ptr);

        ptr = ptr2 + 1;
        ptr2 = strchr(ptr, ',');
        if (ptr2)
        {
          *ptr2 = '\0';
        }
      }
    }
    else if (strncmp(Buffer+3, "GSV", 3) == 0)
    {
      // Disable GSV
      Serial.println("Disabling GSV\r\n");
      unsigned char setGSV[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
      SendUBX(setGSV, sizeof(setGSV));
    }
    else if (strncmp(Buffer+3, "GLL", 3) == 0)
    {
      // Disable GLL
      Serial.println("Disabling GLL\r\n");
      unsigned char setGLL[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
      SendUBX(setGLL, sizeof(setGLL));
    }
    else if (strncmp(Buffer+3, "GSA", 3) == 0)
    {
      // Disable GSA
      Serial.println("Disabling GSA\r\n");
      unsigned char setGSA[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };
      SendUBX(setGSA, sizeof(setGSA));
    }
    else if (strncmp(Buffer+3, "VTG", 3) == 0)
    {
      // Disable VTG
      Serial.println("Disabling VTG\r\n");
      unsigned char setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
      SendUBX( setVTG, sizeof(setVTG));
    }
    else
    {
      // digitalWrite(LED_WARN, 1);
      // Serial.print("Unknown NMEA sentence: ");
      // Serial.println(Buffer+1);
    }
  }
  else
  {
    Serial.println("Bad checksum");
  }
}


void SetupGPS(void)
{
#ifdef GPS_ON
  pinMode(GPS_ON, OUTPUT);
  digitalWrite(GPS_ON, 1);
#endif
  
#ifdef GPS_I2C
  // Init i2c library
  // Wire.begin();
  I2c.begin();
#endif
  TimeForFlightMode = millis() + 10000L;
#ifdef POWERSAVING
  Serial.println("Disabling non GPS GNSS Solutions");
  SetGNSSMode();
#endif  
}

void OpenGPS(void)
{
#ifdef GPS_I2C
  // Wire.begin();
  // I2c.begin();
  
//  Wire.requestFrom(0x42, 80);    // request 80 bytes from slave device 0x42 (Ublox default)
  I2c.read(0x42, 32);    // request 80 bytes from slave device 0x42 (Ublox default)
#endif
}

int GPSAvailable(void)
{
#ifdef GPS_I2C
  // return Wire.available();
  return I2c.available();
#else  
  return Serial.available();
#endif
}

char ReadGPS(void)
{
#ifdef GPS_I2C
  // return Wire.read();        
  return I2c.receive();        
#else
  return Serial.read();
#endif
}

void CheckGPS(void)
{
  static char Line[100];
  static int Length=0;
  char Character;

  #ifdef LED_OK
    digitalWrite(LED_OK, 0);
  #endif
  #ifdef LED_WARN
    digitalWrite(LED_WARN, 0);
  #endif
  OpenGPS();

  while(GPSAvailable())
  { 
    Character = ReadGPS();    

    if (Character == 0xFF)
    {
    }
    else if (Character == '$')
    {
      Line[0] = Character;
      Length = 1;
    }
    else if (Length > 90)
    {
      Length = 0;
    }
    else if ((Length > 0) && (Character != '\r'))
    {
      Line[Length++] = Character;
      if (Character == '\n')
      {
        Line[Length] = '\0';
        ProcessLine(Line, Length);
        Length = 0;
      }
    }
  }
  
  if (millis() >= TimeForFlightMode)
  {
    SetFlightMode();
    TimeForFlightMode = millis() + 60000L;
  }
}

void setGPS_PowerSaveMode(void)
{
  // Power Save Mode

  uint8_t setPSM[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92}; // Setup for Power Save Mode (Default Cyclic 1s)

  SendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}


void setGps_MaxPerformanceMode()
{
  //Set GPS for Max Performance Mode

  uint8_t setMax[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91}; // Setup for Max Power Mode

  SendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}


// Main loop (note conditions). 
// If these conditions aren’t met you need to get it back in max performance mode asap.
// I did this with a messy but effective routine that flagged if sats dropped below 4 or the time froze.
// If either of these conditions occurred within 10 loops I put it back in max performance mode.
// Time freezing is a sure fire guarantee you have issues but I’ve never seen it happen as long as you get back in max performance mode if the sats drops <4

 

#ifdef POWERSAVING
void CheckPowerSaving(void)
{
  if((GPS.Lock==3) && (GPS.psm_status==0) && (GPS.Satellites>=5) &&((GPS.errorstatus & (1 << 0))==0)&&((GPS.errorstatus & (1 << 1))==0)) // Check we aren't in an error condition
  {
    setGPS_PowerSaveMode();

    // wait(1000);

    GPS.psm_status=1;

    GPS.errorstatus &= ~(1 << 4); // Set Bit 4 Indicating PSM is on
  }
}
#endif


void checkDynamicModel()
{
  if ((GPS.Altitude<=1000) && (GPS.Satellites>4))
  {
    if(GPS.navmode != 3)
    {
      setGPS_DynamicModel3();

      GPS.errorstatus |=(1 << 3);  // Set Bit 3 indicating we are in pedestrian mode   
    }
  }
  else
  {
    if (GPS.navmode != 6)
    {
      setGPS_DynamicModel6();

      GPS.errorstatus &= ~(1 << 3); // Unset bit 3 indicating we are in flight mode
    }
  }
}

int getUBX_ACK(uint8_t *Command)
{
  return 1;
}

void setGPS_DynamicModel3()
{
  int OK=0;

  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76};

  while(!OK)
  {
    SendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));

    OK=getUBX_ACK(setdm3);
  }
}


void setGPS_DynamicModel6()
{
  int OK=0;

  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};

  while(!OK)
  {
    SendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));

    OK=getUBX_ACK(setdm6);
  }
}

