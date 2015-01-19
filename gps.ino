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
  #include <Wire.h>
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
  int i;
  
#ifdef GPS_I2C  
  Wire.beginTransmission(0x42);
  
  for (i=0; i<Length; i++)
  {
    Wire.write(Message[i]);
  }
  
  Wire.endTransmission();     
#else
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
  static byte LED_Status=0;
  int lock, Satellites, date;
  char ns, ew;
  char TimeString[16], LatString[16], LongString[16], Temp[4];
	
  if (GPSChecksumOK(Buffer, Count))
  {
    Satellites = 0;
	
    if (strncmp(Buffer+3, "GGA", 3) == 0)
    {
      char lock;
      char hdop[16], Altitude[16];
      
      Serial.print(Buffer+1);
      
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
        
        if (Satellites >= 4)
	{
          GPS.Latitude = FixPosition(atof(LatString));
          if (ns == 'S') GPS.Latitude = -GPS.Latitude;
          GPS.Longitude = FixPosition(atof(LongString));
          if (ew == 'W') GPS.Longitude = -GPS.Longitude;
          GPS.Altitude = (unsigned int)atof(Altitude);
        }
        
        GPS.Satellites = Satellites;
        GPS.GotTime = 1;
      }
      else
      {
        GPS.GotTime = 0;
      }
      
      #ifdef LED_WARN
        digitalWrite(LED_WARN, (!GPS.GotTime) && LED_Status);
      #endif

      #ifdef LED_OK
        digitalWrite(LED_OK, (!GPS.GotTime) || (GPS.Altitude > 1000) ? 0 : LED_Status);
      #endif
      
      LED_Status ^= 1;
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
  Wire.begin();
#endif
  TimeForFlightMode = millis() + 10000L;
}

void OpenGPS(void)
{
#ifdef GPS_I2C
  Wire.begin();
  Wire.requestFrom(0x42, 80);    // request 80 bytes from slave device 0x42 (Ublox default)
#endif
}

int GPSAvailable(void)
{
#ifdef GPS_I2C
  return Wire.available();
#else  
  return Serial.available();
#endif
}

char ReadGPS(void)
{
#ifdef GPS_I2C
  return Wire.read();        
#else
  return Serial.read();
#endif
}

void CheckGPS(void)
{
  static char Line[100];
  static int Length=0;
  char Character;

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


