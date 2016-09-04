// Adds a menu accessible via the serial port
// Menu allows the user to change the payload ID etc
// Values are stored in the AVR's flash data memory

#ifdef ENABLE_MENU

#include <EEPROM.h>     // Used for EE storage

int MenuFunction=0;
int LineIndex=0;
int MaxLength=0;
char Line[32];
enum TLineType {ltUNSIGNED, ltSIGNED, ltUPPERDIGITS, ltPRINT} LineType;

void SetupMenu(void)
{
  if ((EEPROM.read(0) != 'D') || (EEPROM.read(1) != 'A'))
  {
    // Store current (default) settings
    StoreSettings();
  }

  // Load settings from EEPROM
  LoadSettings();

  Serial.println(F("To enter menu, press ESC twice"));
}

void StoreSettings(void)
{
  int i;
  unsigned char *ptr;
  
  // Signature
  EEPROM.write(0, 'D');
  EEPROM.write(1, 'A');

  // Settings
  ptr = (unsigned char *)(&Settings);
  for (i=0; i<sizeof(Settings); i++, ptr++)
  {
    EEPROM.write(i+2, *ptr);
  }
  
  Serial.println(F("EEPROM settings updated"));
}

void LoadSettings(void)
{
  int i;
  unsigned char *ptr;

  ptr = (unsigned char *)(&Settings);
  for (i=0; i<sizeof(Settings); i++, ptr++)
  {
    *ptr = EEPROM.read(i+2);
  }
  
  Serial.println(F("Settings loaded from EEPROM"));
}

void CheckMenu(void)
{
  while (Serial.available())
  {
    char Character;

    Character = Serial.read();

    if (Character == 27)
    {
      // ESC key
      if (MenuLevel < 2)
      {
        MenuLevel++;
        DisplayMenu();
      }
      else if (MenuLevel == 2)
      {
        MenuLevel = 0;
        DisplayMenu();
      }
      else
      {
        MenuLevel--;
        DisplayMenu();
      }
    }
    else if (MenuLevel == 2)
    {
      switch (Character)
      {
        case '0':
          LoadDefaults();
          StoreSettings();
        break;

        case '1':
          Serial.print(F("Callsign = '"));
          Serial.print(Settings.APRS_Callsign);
          Serial.println("'");
          Serial.println(F("Please enter new callsign then ENTER to save or ESC to cancel"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 1;
          MaxLength = 6;
          LineIndex = 0;
          LineType = ltUPPERDIGITS;
        break;

        case '2':
          Serial.print(F("SSID = '"));
          Serial.print(Settings.APRS_ID);
          Serial.println("'");
          Serial.println(F("Please enter new SSID then ENTER to save or ESC to cancel"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 2;
          MaxLength = 2;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;

        case '3':
          Serial.print(F("Path Changeover Altitude = '"));
          Serial.print(Settings.APRS_PathAltitude);
          Serial.println("'");
          Serial.println(F("Please enter new altitude in metresm, then ENTER to save or ESC to cancel"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 3;
          MaxLength = 4;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;

        case '4':
          Serial.print(F("Path At High Altitude = "));
          Serial.print(Settings.APRS_UsePathWhenHigh ? F("WIDE2-1") : F("NONE"));
          Serial.println("");
          Serial.println(F("Please enter 1 for WIDE2-1 or 0 for NONE (recommended)"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 4;
          MaxLength = 1;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;

        case '5':
          Serial.print(F("Time Between Transmissions = "));
          Serial.print(Settings.APRS_TxInterval);
          Serial.println("m");
          Serial.println(F("Please enter time in minutes between transmissions"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 5;
          MaxLength = 2;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;        
        
        case '6':
          Serial.print(F("Pre-emphasis = "));
          Serial.print(Settings.APRS_PreEmphasis ? F("ON") : F("OFF"));
          Serial.println("");
          Serial.println(F("Please enter 1 for ON or 0 for OFF"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 6;
          MaxLength = 1;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;

        case '7':
          Serial.print(F("Random Timing Delay = +/-"));
          Serial.print(Settings.APRS_Randomize);
          Serial.println("s");
          Serial.println(F("Please enter new value between 0 and 30 seconds"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 7;
          MaxLength = 2;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;

        case '8':
          Serial.print(F("APRS Comment = "));
          Serial.println(Settings.APRS_Comment);
          Serial.println(F("Please enter new comment up to 30 characters"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 8;
          MaxLength = 30;
          LineIndex = 0;
          LineType = ltPRINT;
        break;

        case '9':
          Serial.print(F("Send Telemetry Packet After This Many Packets: "));
          Serial.println(Settings.APRS_TelemetryInterval);
          Serial.println(F("Please enter number of packets between telemetry transmissions (0 to disable)"));
          Serial.println();
          Serial.print(F("> "));
          MenuLevel = 3;
          MenuFunction = 9;
          MaxLength = 2;
          LineIndex = 0;
          LineType = ltUNSIGNED;
        break;        

      }
      DisplayMenu();
    }
    else if (MenuLevel == 3)
    {
      // User typing in a value
      if (Character == 8)
      {
        // backspace
        if (LineIndex)
        {
          LineIndex--;
          Serial.print(Character);
          Serial.print(' ');
          Serial.print(Character);
        }
      }
      else if (isprint(Character) && (LineIndex < MaxLength))
      {
        int OK=0;

        if (LineType == ltUNSIGNED) OK = isdigit(Character);
        if (LineType == ltSIGNED) OK = isdigit(Character) || (Character == '-');
        if (LineType == ltUPPERDIGITS) OK = isdigit(Character) || isupper(Character);
        if (LineType == ltPRINT) OK = 1;

        if (OK)
        {
          Line[LineIndex++] = Character;
          Line[LineIndex] = '\0';
          Serial.print(Character);
        }
      }
      else if (Character == 13)
      {
        // CR
        Serial.println();
        Serial.println();
        if (LineIndex > 0)
        {
          // Save setting
          Line[LineIndex] = '\0';
          switch (MenuFunction)
          {
              case 1:   // Callsign
              strcpy(Settings.APRS_Callsign, Line);
              break;
              
              case 2:   // SSID
              Settings.APRS_ID = atoi(Line);
              break;

              case 3: // Path change altitude
              Settings.APRS_PathAltitude = atoi(Line);
              break;

              case 4: // Path at altitude
              Settings.APRS_UsePathWhenHigh = atoi(Line) ? 1 : 0;
              break;

              case 5: // Minutes between transmissions
              Settings.APRS_TxInterval = atoi(Line);
              if (Settings.APRS_TxInterval <= 0) Settings.APRS_TxInterval = 0;
              break;

              case 6: // Pre-emphasis
              Settings.APRS_PreEmphasis = atoi(Line) ? 1 : 0;
              break;

              case 7: // Random delay
              Settings.APRS_Randomize = atoi(Line);
              if (Settings.APRS_Randomize > 30) Settings.APRS_Randomize = 30;
              break;

              case 8: // Comment
              strcpy(Settings.APRS_Comment, Line);
              break;

              case 9: // Telemetry packet count
              Settings.APRS_TelemetryInterval = atoi(Line);
              break;
          }
          StoreSettings();
          Serial.println();
          Serial.println();
          Serial.println(F("SAVED !!"));
        }
        MenuLevel--;
        DisplayMenu();
      }
    }
  }
}

void DisplayMenu(void)
{ 
  if (MenuLevel == 1)
  {
    Serial.println(F("Press ESC again to enter main menu"));
  }
  else if (MenuLevel == 2)
  {
    Serial.println();
    Serial.println(F("APRS MENU"));
    Serial.println(F("========="));
    Serial.println();
    Serial.println(F("  0 - Reset to defaults"));
    Serial.print  (F("  1 - Callsign: "));                        Serial.println(Settings.APRS_Callsign);
    Serial.print  (F("  2 - SSID: "));                            Serial.println(Settings.APRS_ID);
    Serial.print  (F("  3 - Altitude that path changes at: "));   Serial.print(Settings.APRS_PathAltitude);                                   Serial.println('m');
    Serial.print  (F("  4 - Path at high altitude: "));           Serial.println(Settings.APRS_UsePathWhenHigh ? F("WIDE2-1") : F("None"));
    Serial.print  (F("  5 - Time between transmissions: "));      Serial.print(Settings.APRS_TxInterval);                                     Serial.println("m");
    Serial.print  (F("  6 - Pre-emphasis: "));                    Serial.println(Settings.APRS_PreEmphasis ? F("ON") : F("OFF"));
    Serial.print  (F("  7 - Random timing delay: +/-"));          Serial.print(Settings.APRS_Randomize);                                      Serial.println("s");
    Serial.print  (F("  8 - APRS Comment: "));                    Serial.println(Settings.APRS_Comment);
    Serial.print  (F("  9 - Telemetry send after "));             Serial.print(Settings.APRS_TelemetryInterval);                              Serial.println(F(" packets"));
    Serial.println(F("ESC - Exit Mdnu"));
    Serial.println();
    Serial.println('>');
    Serial.println();
  }
}


#endif
