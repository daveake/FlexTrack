/* ========================================================================== */
/*   ds18b20.ino                                                              */
/*                                                                            */
/*   Code for reading OneWire Temperature devices/averaging ADC channels      */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */

// Variables

#ifdef WIREBUS

#include <OneWire.h>
#include <DallasTemperature.h>

#define MAX_SENSORS  4
// Variables

int SensorCount=0;       // Number of temperature devices found
unsigned long CheckDS18B20s=0;
int GettingTemperature=0;
OneWire oneWire(WIREBUS);                    // OneWire port
DallasTemperature sensors(&oneWire);   // Pass oneWire reference to Dallas Temperature object
int DS18B20_Temperatures[MAX_SENSORS];

void Setupds18b20(void)
{
  sensors.begin();
  // Grab a count of devices on the wire
  SensorCount = sensors.getDeviceCount();
  Serial.print(SensorCount);
  Serial.println(" DS18B20's on bus");
  SensorCount = min(SensorCount, MAX_SENSORS);
  if (SensorCount > 0)
  {
    sensors.setResolution(9);
  }
}

void Checkds18b20(void)
{
  if (millis() >= CheckDS18B20s)
  {
    if (GettingTemperature)
    {
      int i;
  
      for (i=0; i<SensorCount; i++)
      {
        DS18B20_Temperatures[i] = sensors.getTempCByIndex(i);
        Serial.print("Temperature "); Serial.print(i); Serial.print(" = "); Serial.print(DS18B20_Temperatures[i]); Serial.println("degC");
      }
      CheckDS18B20s = millis() + 10000L;
    }
    else
    {
      sensors.requestTemperatures();          // Send the command to get temperature
      CheckDS18B20s = millis() + 1000L;        // Leave 1 second (takes 782ms) for readings to happen
    }
    GettingTemperature = !GettingTemperature;
  }
}


#endif
