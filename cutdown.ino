#ifdef CUTDOWN

void SetupCutdown(void)
{
    digitalWrite(CUTDOWN, 0);
    pinMode(CUTDOWN, OUTPUT);
    digitalWrite(CUTDOWN, 0);
}

void CutdownNow(void)
{
  Serial.println("CUTDOWN ON");
  digitalWrite(CUTDOWN, 1);
  delay(5000);
  digitalWrite(CUTDOWN, 0);
  Serial.println("CUTDOWN OFF");
}

void CheckCutdown(void)
{
  // Don't do anything unless we have GPS
  if (GPS.Satellites >= 4)
  {
    // Arm ?
    
    if ((GPS.Altitude > 2000) && (GPS.CutdownStatus == 0))
    {
      GPS.CutdownStatus = 1;      // Armed
    }

    // Trigger only if armed
    if (GPS.CutdownStatus == 1)
    {
      // Uncomment/modify the following code to trigger the cutdown appropriately for your flight
    
      // ALTITUDE TEST
      /*
      if (GPS.Altitude > 12000)
      {
        GPS.CutdownStatus = 2;      // Altitude trigger
        CutdownNow();
      }
      */
    
      // LONGITUDE TEST
      /*
      if ((GPS.Longitude < -6.5)
      {
        GPS.CutdownStatus = 3;      // Longitude trigger
        CutdownNow();
      }
      */
    }
  }
}

#endif
