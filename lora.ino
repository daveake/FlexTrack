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

// FSK stuff
#define REG_PREAMBLE_MSB_FSK        0x25
#define REG_PREAMBLE_LSB_FSK        0x26
#define REG_PACKET_CONFIG1          0x30
#define REG_PAYLOAD_LENGTH_FSK      0x32
#define REG_FIFO_THRESH             0x35
#define REG_FDEV_MSB                0x04
#define REG_FDEV_LSB                0x05
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_BITRATE_MSB             0x02
#define REG_BITRATE_LSB             0x03
#define REG_IRQ_FLAGS2              0x3F

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

#ifdef LORA_NSS

#include <SPI.h>
#include <string.h>

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
int CallingCount=0;
int RTTYCount=0;
int InRTTYMode=0;
int SendingRTTY=0;
int RTTYIndex, RTTYMask, RTTYLength;
int FSKBitRate, FSKOverSample, RTTYBitLength;

void SetupLoRa(void)
{
  setupRFM98(LORA_FREQUENCY, LORA_MODE);

  if (LORA_RTTY_BAUD == 50)
  {
    FSKBitRate = 40000;
    FSKOverSample = 2;
    RTTYBitLength = 7;
  }
  else
  {
    // 300 baud
    FSKBitRate = 13333;
    FSKOverSample = 1;
    RTTYBitLength = 8;
  }
}

void setupRFM98(double Frequency, int Mode)
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
  setFrequency(Frequency + LORA_OFFSET / 1000.0);

  // LoRa settings for various modes.  We support modes 2 (repeater mode), 1 (normally used for SSDV) and 0 (normal slow telemetry mode).
  
  if (Mode == 5)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_41K7;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0;		
  }
  else if (Mode == 2)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_62K5;
    SpreadingFactor = SPREADING_8;
    LowDataRateOptimize = 0;		
  }
  else if (Mode == 1)
  {
    ImplicitOrExplicit = IMPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_5;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_6;
    LowDataRateOptimize = 0;    
  }
  else if (Mode == 0)
  {
    ImplicitOrExplicit = EXPLICIT_MODE;
    ErrorCoding = ERROR_CODING_4_8;
    Bandwidth = BANDWIDTH_20K8;
    SpreadingFactor = SPREADING_11;
    LowDataRateOptimize = 0x08;		
  }
  
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

void SendLoRaPacket(unsigned char *buffer, int Length, int CallingPacket)
{
  int i;
  
  LastLoRaTX = millis();
  TimeToSendIfNoGPS = 0;

//  if (InRTTYMode != 0)
//  {
//    setupRFM98(LORA_FREQUENCY, LORA_MODE);
//    InRTTYMode = 0;
//  }
  InRTTYMode = 0;

  if (CallingPacket)
  {
    setupRFM98(LORA_CALL_FREQ, LORA_CALL_MODE);
  }
  else
  {
    setupRFM98(LORA_FREQUENCY, LORA_MODE);
  }
   
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
  SendingRTTY = 0;
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

int FSKPacketSent(void)
{
  return ((readRegister(REG_IRQ_FLAGS2) & 0x48) != 0);
}

int FSKBufferLow(void)
{
  return (readRegister(REG_IRQ_FLAGS2) & 0x20) == 0;
}


void CheckFSKBuffer(void)
{
  if ((LoRaMode == lmSending) && SendingRTTY)
  {
    // Check if packet sent
    if (FSKPacketSent())
    {
      LoRaMode = lmIdle;
      SendingRTTY = 0;
    }
    else if (FSKBufferLow())
    {
      AddBytesToFSKBuffer(64 - 20);
    }
  }
}

void AddBytesToFSKBuffer(int MaxBytes)
{
  unsigned char data[65], temp;
  int i;
  uint8_t BytesWritten = 0;

  if (RTTYIndex < RTTYLength)
  {
    data[BytesWritten++] = REG_FIFO | 0x80;
    
    while((BytesWritten <= (MaxBytes-FSKOverSample+1)) &&
        (RTTYIndex < RTTYLength))
    {
      if (RTTYMask < 0)  //start bit
      {
        temp = 0xFF;
        RTTYMask++;
      }
      else if (RTTYMask == 0)  //start bit
      {
        temp = 0;
        RTTYMask = 1;
      }
      else if (RTTYMask >= (1<<RTTYBitLength))  //stop bits
      {
        RTTYMask <<= 1;
        temp = 0xFF;
        if (RTTYMask >= (1<<(RTTYBitLength+2)))
        {
          RTTYMask = 0;
          RTTYIndex++;
        }
      }
      else  //databits
      {
        if (Sentence[RTTYIndex] & RTTYMask)
        {
          temp = 0xFF;
        }
        else
        {
          temp = 0x0;
        }
        RTTYMask <<= 1;
      }

      for (i = 0; i < FSKOverSample; i++)
      {
        data[BytesWritten++] = temp;
      }
    }


    select();

    for (i = 0; i < BytesWritten; i++)
    {
      SPI.transfer(data[i]);
    }
    unselect();
  }
}

void SwitchToFSKMode(void)
{
  unsigned long FrequencyValue;
  
  uint8_t mode = readRegister(REG_OPMODE);
  mode |= (1<<3);                           //set LF range

  if (mode & (1<<7))  //if in lora mode
  {
    writeRegister(REG_OPMODE,(mode & ~(uint8_t)7));    //set to sleep mode so fsk bit can be written
  }
  else
  {
    writeRegister(REG_OPMODE,(mode & ~(uint8_t)7) | 1);  //set to standby mode so various settings can be written  
  }

  mode = readRegister(REG_OPMODE);
  writeRegister(REG_OPMODE, mode & ~(uint8_t)(7<<5));         //set to FSK

  writeRegister(REG_LNA, LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMIT
  writeRegister(REG_PA_CONFIG, PA_MAX_UK);
    
  // Frequency
  FrequencyValue = (unsigned long)((LORA_RTTY_FREQ + (LORA_OFFSET / 1000.0)) * 7110656 / 434);
  writeRegister(REG_FRF_MSB, (FrequencyValue >> 16) & 0xFF);   // Set frequency
  writeRegister(REG_FRF_MID, (FrequencyValue >> 8) & 0xFF);
  writeRegister(REG_FRF_LSB, FrequencyValue & 0xFF);
  
  //write modem config
  writeRegister(REG_BITRATE_LSB, FSKBitRate & 0xFF);
  writeRegister(REG_BITRATE_MSB, (FSKBitRate >> 8) & 0xFF);
  writeRegister(REG_FDEV_LSB, (LORA_RTTY_SHIFT / 122) & 0xFF);
  writeRegister(REG_FDEV_MSB, 0);
  writeRegister(REG_PREAMBLE_LSB_FSK, 0);    // Preamble
  writeRegister(REG_PREAMBLE_MSB_FSK, 0);
  
  // writeRegister(REG_PACKET_CONFIG1, 0x80);    // variable length, no DC fix, no CRC, no addressing
  writeRegister(REG_PACKET_CONFIG1, 0x00);   // fixed length, no DC fix, no CRC, no addressing
  writeRegister(REG_PAYLOAD_LENGTH_FSK, 0);
}

void SendLoRaRTTY(int Length)
{
  if (InRTTYMode != 1)
  {
    // Set RTTY mode
    SwitchToFSKMode();
    InRTTYMode = 1;
  }

  // Fill in RTTY buffer
  // memcpy(RTTYBuffer, buffer, Length);
  RTTYLength = Length;
  RTTYIndex = 0;
  RTTYMask = -LORA_RTTY_PREAMBLE;
  
  // Set FIFO threshold
  uint8_t r = readRegister(REG_FIFO_THRESH); 
  writeRegister(REG_FIFO_THRESH,(r&(~0x3F)) | 20);     // 20 = FIFO threshold
  
  writeRegister(REG_OPMODE, 0x0B);   // Tx mode

  // Populate FIFO
  AddBytesToFSKBuffer(64);
  
  // Set channel state
  LoRaMode = lmSending;
  SendingRTTY = 1;
}

void CheckLoRa(void)
{
  CheckFSKBuffer();
  
  CheckLoRaRx();
		
  if (LoRaIsFree())
  {		
    Serial.println("LoRa is free");
    if (SendRepeatedPacket == 3)
    {
      // Repeat ASCII sentence
      Sentence[0] = '%';
      SendLoRaPacket(Sentence, strlen((char *)Sentence)+1, 0);
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else if (SendRepeatedPacket == 2)
    {
      Serial.println(F("Repeating uplink packet"));
				
        // 0x80 | (LORA_ID << 3) | TargetID
      SendLoRaPacket((unsigned char *)&PacketToRepeat, sizeof(PacketToRepeat), 0);
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else if (SendRepeatedPacket == 1)
    {
      Serial.println(F("Repeating balloon packet"));
				
      // 0x80 | (LORA_ID << 3) | TargetID
				
      RepeatedPacketType = 0;
      SendRepeatedPacket = 0;
    }
    else			
    {
      int PacketLength;

      if (++RTTYCount >= (LORA_RTTY_COUNT + LORA_RTTY_EVERY))
      {
        RTTYCount = 0;
      }
            
      if (RTTYCount < LORA_RTTY_COUNT)
      {
        // Send RTTY packet
        PacketLength = BuildSentence((char *)Sentence, LORA_PAYLOAD_ID);
        Serial.println(F("LoRa: Tx RTTY packet"));
        SendLoRaRTTY(PacketLength);    
      }
      else
      {
        if ((LORA_CALL_COUNT > 0) && (++CallingCount > LORA_CALL_COUNT))
  	    {
  		    CallingCount = 0;
  		    setupRFM98(LORA_CALL_FREQ, LORA_CALL_MODE);
          PacketLength = BuildLoRaCall(Sentence);
  		    Serial.println(F("LoRa: Calling Mode"));
          SendLoRaPacket(Sentence, PacketLength, 1);
  	    }
        else
  	    {
//  		    if ((LORA_CALL_COUNT > 0) && (CallingCount == 1))
//  		    {
//  			    setupRFM98(LORA_FREQUENCY, LORA_MODE);
//  		    }
  		
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
          SendLoRaPacket(Sentence, PacketLength, 0);  
        }
      }
    }
  }
}

#endif
