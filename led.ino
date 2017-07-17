unsigned long NextLEDs=0;

void SetupLEDs(void)
{
#ifdef LED_WARN
  pinMode(LED_WARN, OUTPUT);
  digitalWrite(LED_WARN, 1);
#endif

#ifdef LED_OK
  pinMode(LED_OK, OUTPUT);
  digitalWrite(LED_OK, 0);
#endif

#ifdef LED_STATUS
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, 0);
#endif

#ifdef LED_TX
  pinMode(LED_TX, OUTPUT);
  digitalWrite(LED_TX, 0);
#endif
}

void ControlLEDs(int LEDStatus, int LEDOK, int LEDWarn)
{
  #ifdef LED_STATUS
    digitalWrite(LED_STATUS, LEDStatus);
  #endif
  #ifdef LED_OK
    digitalWrite(LED_OK, LEDOK);
  #endif
  #ifdef LED_WARN
    digitalWrite(LED_WARN, LEDWarn);
  #endif
}

void CheckLEDs(void)
{
  if (millis() >= NextLEDs)
  {
    static byte Flash=0;
    
    // This would normally be the only LED for status (i.e. no OK or WARN LEDs)
    if (GPS.Altitude > 1000)
    {
      // All off
      ControlLEDs(0,0,0);
    }
    else if ((GPS.FixType == 3) && (GPS.Satellites >= 4))
    {
      ControlLEDs(Flash, Flash, 0);
    }
    else
    {
      ControlLEDs(1, 0, Flash);
    }       
    
    NextLEDs = millis() + 500L;
    Flash = 1-Flash;
  }
}

