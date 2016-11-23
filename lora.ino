/*---------------------------------------------------*\
|                                                     |
| LoRa radio code, for downlink, uplink and repeating |
|                                                     |
| Messages can be timed using a GPS reference, to     |
| comply with the TDMA timing requirements.           |
|                                                     |
| Connections:                                        |
|                                                     |
|               Arduino  X - RFM98W DIO5              |
|               Arduino  X - RFM98W DIO0              |
|                                                     |
|               Arduino  X  - RFM98W NSS              |
|               Arduino 11 - RFM98W MOSI              |
|               Arduino 12 - RFM98W MISO              |
|               Arduino 13 - RFM98W CLK               |
|                                                     |
\*---------------------------------------------------*/

#ifdef LORA_NSS

#include <SPI.h>
#include <string.h>

// RFM98 registers
#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D 
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_HOP_PERIOD              0x24
#define REG_FREQ_ERROR              0x28
#define REG_DETECT_OPT              0x31
#define	REG_DETECTION_THRESHOLD     0x37
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41

// MODES
#define RF98_MODE_RX_CONTINUOUS     0x85
#define RF98_MODE_TX                0x83
#define RF98_MODE_SLEEP             0x80
#define RF98_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              255

// Modem Config 1
#define EXPLICIT_MODE               0x00
#define IMPLICIT_MODE               0x01

#define ERROR_CODING_4_5            0x02
#define ERROR_CODING_4_6            0x04
#define ERROR_CODING_4_7            0x06
#define ERROR_CODING_4_8            0x08

#define BANDWIDTH_7K8               0x00
#define BANDWIDTH_10K4              0x10
#define BANDWIDTH_15K6              0x20
#define BANDWIDTH_20K8              0x30
#define BANDWIDTH_31K25             0x40
#define BANDWIDTH_41K7              0x50
#define BANDWIDTH_62K5              0x60
#define BANDWIDTH_125K              0x70
#define BANDWIDTH_250K              0x80
#define BANDWIDTH_500K              0x90

// Modem Config 2

#define SPREADING_6                 0x60
#define SPREADING_7                 0x70
#define SPREADING_8                 0x80
#define SPREADING_9                 0x90
#define SPREADING_10                0xA0
#define SPREADING_11                0xB0
#define SPREADING_12                0xC0

#define CRC_OFF                     0x00
#define CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define REG_PA_CONFIG               0x09
#define PA_MAX_BOOST                0x8F    // 100mW (max 869.4 - 869.65)
#define PA_LOW_BOOST                0x81
#define PA_MED_BOOST                0x8A
#define PA_MAX_UK                   0x88    // 10mW (max 434)
#define PA_OFF_BOOST                0x00
#define RFO_MIN                     0x00

// 20DBm
#define REG_PA_DAC                  0x4D
#define PA_DAC_20                   0x87

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23  // 0010 0011
#define LNA_OFF_GAIN                0x00

typedef enum {lmIdle, lmListening, lmSending} tLoRaMode;

tLoRaMode LoRaMode;
byte currentMode = 0x81;
int TargetID;
struct TBinaryPacket PacketToRepeat;
byte SendRepeatedPacket, RepeatedPacketType=0;
int ImplicitOrExplicit;
int GroundCount;
int AirCount;
int BadCRCCount;
unsigned char Sentence[SENTENCE_LENGTH];
unsigned long LastLoRaTX=0;
unsigned long TimeToSendIfNoGPS=0;

void SetupLoRa(void)
{
  setupRFM98();
}

void setupRFM98(void)
{
  int ErrorCoding;
  int Bandwidth;
  int SpreadingFactor;
  int LowDataRateOptimize;
  int PayloadLength;
  
  // initialize the pins
  #ifdef LORA_RESET
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(10);          // Module needs this before it's ready
  #endif
  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  pinMode(LORA_DIO5, INPUT);

  SPI.begin();
  
  // LoRa mode 
  setLoRaMode();

  // Frequency
  setFrequency(LORA_FREQUENCY);

  // LoRa settings for various modes.  We support modes 2 (repeater mode), 1 (normally used for SSDV) and 0 (normal slow telemetry mode).
  #if LORA_MODE == 2
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_62K5;
    SpreadingFactor = SPREADING_8;
    LowDataRateOptimize = 0;		
  #endif

  #if LORA_MODE == 1
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_6;
    LowDataRateOptimize = 0;    
  #endif

  #if LORA_MODE == 0  
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0x08;		
  #endif

  PayloadLength = ImplicitOrExplicit == IMPLICIT_MODE ? 255 : 0;

  writeRegister(REG_MODEM_CONFIG, ImplicitOrExplicit | ErrorCoding | Bandwidth);
  writeRegister(REG_MODEM_CONFIG2, SpreadingFactor | CRC_ON);
  writeRegister(REG_MODEM_CONFIG3, 0x04 | LowDataRateOptimize);									// 0x04: AGC sets LNA gain
  
  // writeRegister(REG_DETECT_OPT, (SpreadingFactor == SPREADING_6) ? 0x05 : 0x03);					// 0x05 For SF6; 0x03 otherwise
  writeRegister(REG_DETECT_OPT, (readRegister(REG_DETECT_OPT) & 0xF8) | ((SpreadingFactor == SPREADING_6) ? 0x05 : 0x03));  // 0x05 For SF6; 0x03 otherwise
  
  writeRegister(REG_DETECTION_THRESHOLD, (SpreadingFactor == SPREADING_6) ? 0x0C : 0x0A);		// 0x0C for SF6, 0x0A otherwise  
  
  writeRegister(REG_PAYLOAD_LENGTH, PayloadLength);
  writeRegister(REG_RX_NB_BYTES, PayloadLength);
  
  // Change the DIO mapping to 01 so we can listen for TxDone on the interrupt
  writeRegister(REG_DIO_MAPPING_1,0x40);
  writeRegister(REG_DIO_MAPPING_2,0x00);
  
  // Go to standby mode
  setMode(RF98_MODE_STANDBY);
  
  Serial.println("Setup Complete");
}

void setFrequency(double Frequency)
{
  unsigned long FrequencyValue;
    
  Serial.print("Frequency is ");
  Serial.println(Frequency);

  Frequency = Frequency * 7110656 / 434;
  FrequencyValue = (unsigned long)(Frequency);

  Serial.print("FrequencyValue is ");
  Serial.println(FrequencyValue);

  writeRegister(0x06, (FrequencyValue >> 16) & 0xFF);    // Set frequency
  writeRegister(0x07, (FrequencyValue >> 8) & 0xFF);
  writeRegister(0x08, FrequencyValue & 0xFF);
}

void setLoRaMode()
{
  Serial.println("Setting LoRa Mode");
  setMode(RF98_MODE_SLEEP);
  writeRegister(REG_OPMODE,0x80);
   
  Serial.println("LoRa Mode Set");
}

/////////////////////////////////////
//    Method:   Change the mode
//////////////////////////////////////
void setMode(byte newMode)
{
  if(newMode == currentMode)
    return;  
  
  switch (newMode) 
  {
    case RF98_MODE_TX:
      writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
      writeRegister(REG_PA_CONFIG, PA_MAX_UK);
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      
      break;
    case RF98_MODE_RX_CONTINUOUS:
      writeRegister(REG_PA_CONFIG, PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
      writeRegister(REG_LNA, LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_SLEEP:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    case RF98_MODE_STANDBY:
      writeRegister(REG_OPMODE, newMode);
      currentMode = newMode; 
      break;
    default: return;
  } 
  
  
  if(newMode != RF98_MODE_SLEEP){
    while(digitalRead(LORA_DIO5) == 0)
    {
    } 
  }
   
  return;
}


/////////////////////////////////////
//    Method:   Read Register
//////////////////////////////////////

byte readRegister(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

/////////////////////////////////////
//    Method:   Write Register
//////////////////////////////////////

void writeRegister(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80); // OR address with 10000000 to indicate write enable;
  SPI.transfer(value);
  unselect();
}

/////////////////////////////////////
//    Method:   Select Transceiver
//////////////////////////////////////
void select() 
{
  digitalWrite(LORA_NSS, LOW);
}

/////////////////////////////////////
//    Method:   UNSelect Transceiver
//////////////////////////////////////
void unselect() 
{
  digitalWrite(LORA_NSS, HIGH);
}

void CheckLoRaRx(void)
{
  if (LoRaMode == lmListening)
  {
    if (digitalRead(LORA_DIO0))
    {
      // unsigned char Message[32];
      int Bytes;
					
      Bytes = receiveMessage(Sentence, sizeof(Sentence));
      Serial.print("Rx "); Serial.print(Bytes); Serial.println(" bytes");
      RepeatedPacketType = 0;
      
      Bytes = min(Bytes, sizeof(Sentence));
					
      if (Bytes > 0)
      {
        if (Sentence[0] == '$')
        {
          // ASCII telemetry
          Serial.println("Rx ASCII");
          if (memcmp(Sentence+2, LORA_PAYLOAD_ID, strlen(LORA_PAYLOAD_ID)) != 0)
          {
            RepeatedPacketType = 3;
          }

          // Get timing from this message
          if ((LORA_TIME_INDEX > 0) && (LORA_TIME_MUTLIPLER > 0))
          {
            unsigned char Slot;
            long Offset;

            Slot = (Sentence[LORA_TIME_INDEX+2] - '0') * LORA_TIME_MUTLIPLER + LORA_TIME_OFFSET;
            Offset = (LORA_SLOT - Slot) * 1000L - LORA_PACKET_TIME;
            if (Offset < 0) Offset += LORA_CYCLETIME * 1000L;

            Serial.print("Rx Slot = "); Serial.println(Slot);
            Serial.print(" Offset = "); Serial.println(Offset);

            TimeToSendIfNoGPS = millis() + Offset;
          }
        }
        /*
        else if ((Sentence[0] & 0xC0) == 0xC0)
        {
          // Binary downlink message
          char Payload[32];
          int SourceID;
							
          SourceID = Message[0] & 0x07;
						
          if (SourceID == LORA_ID)
          {
            Serial.println("Ignoring Binary Repeat");
          }
          else
          {
            Serial.print("Balloon Binary Message from sender "); Serial.println(SourceID);
            
            // Replace the sender ID with ours
            Sentence[0] = Sentenceage[0] & 0xC7 | (LORA_ID << 3);
            memcpy(&PacketToRepeat, Sentence, sizeof(PacketToRepeat));
            RepeatedPacketType = 1;
							
            AirCount++;
          }
        }
        else if ((Message[0] & 0xC0) == 0x80)
        {
          int SenderID, TargetID;
						
          TargetID = Message[0] & 0x07;
          SenderID = (Message[0] >> 3) & 0x07;

          Serial.print("Uplink from "); Serial.print(SenderID); Serial.print(" to "); Serial.print(TargetID); Serial.print(", Message "); Serial.println((char *)Message+1);
									
          if (TargetID == LORA_ID)
          {
            Serial.println("Message was for us!");
            Serial.print("Message is "); Serial.println((char *)Message+1);
              
            GroundCount++;
          }
          else
          {
            Serial.println("Message is for another balloon");
            Message[0] = Message[0] & 0xC7 | (LORA_ID << 3);
            memcpy(&PacketToRepeat, Message, sizeof(PacketToRepeat));
            RepeatedPacketType = 2;
          }
        }
        else
        {
          Serial.print("Unknown message "); Serial.println((int)Message[0]);
        }
        */
      }
    }
  }
}

int TimeToSend(void)
{
  int CycleSeconds;
	
  SendRepeatedPacket = 0;

  if (LORA_CYCLETIME == 0)
  {
    // Not using time to decide when we can send
    return 1;
  }

  if ((millis() > (LastLoRaTX + LORA_CYCLETIME*1000+2000)) && (TimeToSendIfNoGPS == 0))
  {
    // Timed out
    Serial.println("Using Timeout");
    return 1;
  }
  
  if (GPS.Satellites > 0)
  {
    static int LastCycleSeconds=-1;

    // Can't Tx twice at the same time
    CycleSeconds = (GPS.SecondsInDay+LORA_CYCLETIME-17) % LORA_CYCLETIME;   // Could just use GPS time, but it's nice to see the slot agree with UTC
    
    if (CycleSeconds != LastCycleSeconds)
    {
      LastCycleSeconds = CycleSeconds;
      
      if (CycleSeconds == LORA_SLOT)
      {
        Serial.println("Using GPS Timing");
        SendRepeatedPacket = 0;
        return 1;
      }

      if (RepeatedPacketType && ((CycleSeconds == LORA_REPEAT_SLOT_1) || (CycleSeconds == LORA_REPEAT_SLOT_2)))
      {
        Serial.println("Time to repeat");
        SendRepeatedPacket = RepeatedPacketType;
        RepeatedPacketType = 0;
        return 1;
      }
    }
  }
  else if ((TimeToSendIfNoGPS > 0) && (millis() >= TimeToSendIfNoGPS))
  {
    Serial.println("Using LoRa Timing");
    SendRepeatedPacket = 0;
    return 1;
  }
    
  return 0;
}


int LoRaIsFree(void)
{
  if ((LoRaMode != lmSending) || digitalRead(LORA_DIO0))
  {
    // Either not sending, or was but now it's sent.  Clear the flag if we need to
    if (LoRaMode == lmSending)
    {
      // Clear that IRQ flag
      writeRegister( REG_IRQ_FLAGS, 0x08); 
      LoRaMode = lmIdle;
    }
				
    // Now we test to see if we're doing TDM or not
    // For TDM, if it's not a slot that we send in, then we should be in listening mode
    // Otherwise, we just send
				
    if (TimeToSend())
    {
      // Either sending continuously, or it's our slot to send in
      // printf("Channel %d is free\n", Channel);
					
      return 1;
    }
    
    if (LORA_CYCLETIME > 0)
    {
      // TDM system and not time to send, so we can listen
      if (LoRaMode == lmIdle)
      {
        startReceiving();
      }
    }
  }
  
  return 0;
}

void SendLoRaPacket(unsigned char *buffer, int Length)
{
  int i;
  
  LastLoRaTX = millis();
  TimeToSendIfNoGPS = 0;
  
  Serial.print("Sending "); Serial.print(Length);Serial.println(" bytes");
  
  setMode(RF98_MODE_STANDBY);

  writeRegister(REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 maps DIO0 to TxDone
  writeRegister(REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
  writeRegister(REG_FIFO_ADDR_PTR, 0x00); 
  if (ImplicitOrExplicit == EXPLICIT_MODE)
  {
    writeRegister(REG_PAYLOAD_LENGTH, Length);
  }
  select();
  // tell SPI which address you want to write to
  SPI.transfer(REG_FIFO | 0x80);

  // loop over the payload and put it on the buffer 
  for (i = 0; i < Length; i++)
  {
    SPI.transfer(buffer[i]);
  }
  unselect();

  // go into transmit mode
  setMode(RF98_MODE_TX);
  
  LoRaMode = lmSending;
}

void startReceiving(void)
{
  writeRegister(REG_DIO_MAPPING_1, 0x00);		// 00 00 00 00 maps DIO0 to RxDone
	
  writeRegister(REG_FIFO_RX_BASE_AD, 0);
  writeRegister(REG_FIFO_ADDR_PTR, 0);
	  
  // Setup Receive Continuous Mode
  setMode(RF98_MODE_RX_CONTINUOUS); 
		
  LoRaMode = lmListening;
}

int receiveMessage(unsigned char *message, int MaxLength)
{
  int i, Bytes, currentAddr, x;

  Bytes = 0;
	
  x = readRegister(REG_IRQ_FLAGS);
  
  // clear the rxDone flag
  writeRegister(REG_IRQ_FLAGS, 0x40); 
    
  // check for payload crc issues (0x20 is the bit we are looking for
  if((x & 0x20) == 0x20)
  {
    // CRC Error
    writeRegister(REG_IRQ_FLAGS, 0x20);		// reset the crc flags
    BadCRCCount++;
  }
  else
  {
    currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    Bytes = readRegister(REG_RX_NB_BYTES);
    Bytes = min(Bytes, MaxLength-1);

    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);   
		
    for(i = 0; i < Bytes; i++)
    {
      message[i] = (unsigned char)readRegister(REG_FIFO);
    }
    message[Bytes] = '\0';

    // Clear all flags
    writeRegister(REG_IRQ_FLAGS, 0xFF); 
  }
  
  return Bytes;
}


int BuildLoRaPositionPacket(unsigned char *TxLine)
{
  struct TBinaryPacket BinaryPacket;

  SentenceCounter++;

  BinaryPacket.PayloadIDs = 0xC0 | (LORA_ID << 3) | LORA_ID;
  BinaryPacket.Counter = SentenceCounter;
  BinaryPacket.BiSeconds = GPS.SecondsInDay / 2L;
  BinaryPacket.Latitude = GPS.Latitude;
  BinaryPacket.Longitude = GPS.Longitude;
  BinaryPacket.Altitude = GPS.Altitude;

  memcpy(TxLine, &BinaryPacket, sizeof(BinaryPacket));
	
  return sizeof(struct TBinaryPacket);
}

void CheckLoRa(void)
{
  /*
   if (LORA_CYCLETIME > 0)
   {
     // TDMA
     static long LastCheckAt=-1;
    int CurrentSeconds;

    if (GPS.Seconds != LastCheckAt)
    {
      LastCheckAt = GPS.Seconds;
      CurrentSeconds = GPS.Seconds % LORA_CYCLETIME;

      if (CurrentSeconds == LORA_SLOT)
      {
        // Send our telemetry
        char *Message = "$Hello world";
      
        SendLoRaPacket(Message, strlen(Message)+1);
      }
      else if (UplinkMessage[0] && (CurrentSeconds == LORA_REPEATSLOT))
      {
        SendLoRaPacket(UplinkMessage, strlen(UplinkMessage));
        UplinkMessage[0] = '\0';
      }
    }
  }
  
  */

  CheckLoRaRx();
		
  if (LoRaIsFree())
  {		
    Serial.println("LoRa is free");
    if (SendRepeatedPacket == 3)
    {
      // Repeat ASCII sentence
      SendLoRaPacket(Sentence, strlen((char *)Sentence)+1);
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else if (SendRepeatedPacket == 2)
    {
      Serial.println(F("Repeating uplink packet"));
				
        // 0x80 | (LORA_ID << 3) | TargetID
      SendLoRaPacket((unsigned char *)&PacketToRepeat, sizeof(PacketToRepeat));
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else if (SendRepeatedPacket == 1)
    {
      Serial.println(F("Repeating balloon packet"));
				
        // 0x80 | (LORA_ID << 3) | TargetID
      SendLoRaPacket((unsigned char *)&PacketToRepeat, sizeof(PacketToRepeat));
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else			
    {
      int PacketLength;
      // unsigned char Sentence[SENTENCE_LENGTH];
				
      if (LORA_BINARY)
      {
        
        // 0x80 | (LORA_ID << 3) | TargetID
        PacketLength = BuildLoRaPositionPacket(Sentence);
	      Serial.println(F("LoRa: Tx Binary packet"));
      }
      else
      {
        PacketLength = BuildSentence((char *)Sentence, LORA_PAYLOAD_ID);
	      Serial.println(F("LoRa: Tx ASCII Sentence"));
      }
							
      SendLoRaPacket(Sentence, PacketLength);		
    }
  }
}

#endif

