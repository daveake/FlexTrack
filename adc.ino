/* ========================================================================== */
/*   adc.ino                                                                  */
/*                                                                            */
/*   Code for reading/averaging ADC channels                                  */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */

// Variables

unsigned long CheckADCChannels=0;
#ifdef A0_MULTIPLIER
  unsigned int Channel0Readings[5];
  unsigned int Channel0Average;
#endif

void SetupADC(void)
{
  #ifdef A0_MULTIPLIER
    analogReference(DEFAULT);
    // Serial.println("Setup A0");
    pinMode(A0, INPUT);
  #endif
}

void CheckADC(void)
{
  if (millis() >= CheckADCChannels)
  {
    #ifdef A0_MULTIPLIER
      Channel0Average = ReadADC(A0, A0_MULTIPLIER, Channel0Readings);
      // Serial.print("Average=");Serial.println(Channel0Average);
    #endif
  
    CheckADCChannels = millis() + 1000L;
  }
}

unsigned int ReadADC(int Pin, float Multiplier, unsigned int *Readings)
{
  int i;
  unsigned int Result;
  
  for (i=0; i<4; i++)
  {
    Readings[i] = Readings[i+1];
  }

  Readings[4] = analogRead(Pin);
  //Serial.print("A0=");Serial.println(Readings[4]);
  
  Result = 0;
  for (i=0; i<5; i++)
  {
    Result += Readings[i];
  }
  
  return (float)Result * Multiplier / 5.0;
}

