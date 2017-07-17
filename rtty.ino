/* ========================================================================== */
/*   rtty.ino                                                                 */
/*                                                                            */
/*   Interrupt-driven RTTY transmission for MTX2                              */
/*                                                                            */
/*                                                                            */
/*                                                                            */
/* ========================================================================== */

#ifdef RTTY_BAUD
#ifdef RTTY_DATA

// Our variables

char TxLine[SENTENCE_LENGTH];
volatile int SendIndex = -1;
volatile int SendBit = 0;
int StopBits, DataBits;
int Timer2Count;
int RTTY_Counter;
int SettingFrequency;

// Code

void SetupRTTY(void)
{
  pinMode(RTTY_DATA, OUTPUT);
  digitalWrite(RTTY_DATA, 1);
  
#ifdef RTTY_ENABLE
  pinMode(RTTY_ENABLE, OUTPUT);
  digitalWrite(RTTY_ENABLE, 1);
  #ifdef MTX2    
    #ifdef RTTY_FREQUENCY
      SetMTX2Frequency(RTTY_FREQUENCY);
    #endif
  #endif
#endif
 
  SetupTimer1(RTTY_BAUD);
  
  DataBits = 7;
  StopBits = 2;

  #ifdef RTTY_PWM   
    TCCR2B = TCCR2B & 0b11111000 | 1; // Sets fast PWM on pin 11  
  #endif
}

void SetupTimer1(int Baud) 
{
  Serial.print("Setting baud rate of ");Serial.println(Baud);
  
  if (Baud > 1200)
  {
    Serial.print("OCR1A=");Serial.println(F_CPU / 8 / Baud - 1);
  }
  else
  {
    Serial.print("OCR1A=");Serial.println(F_CPU / 1024 / Baud - 1);
  }
  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  if (Baud > 1200)
  {
    // Prescaler of 8
    OCR1A = F_CPU / 8 / Baud - 1;  // set compare match register to desired timer count:
    TCCR1B |= (1 << WGM12);   // turn on CTC mode:
    // Set CS11 bit for prescaler of /8
    TCCR1B |= (1 << CS11);
  }
  else
  {
    // Prescaler of 1024
    OCR1A = F_CPU / 1024 / Baud - 1;  // set compare match register to desired timer count:
    TCCR1B |= (1 << WGM12);   // turn on CTC mode:
    // Set CS10 and CS12 bits for prescaler of /1024
    TCCR1B |= (1 << CS10);
    TCCR1B |= (1 << CS12);
  }
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void CheckRTTY(void)
{
  if (SendIndex == -1)
  {
    BuildSentence(TxLine, RTTY_PAYLOAD_ID);    
    Serial.print(TxLine);
    Serial.print('\r');
    SendIndex = 0;
  }
}

void rtty_txbit(int bit)
{
  if (SettingFrequency)
  {
    digitalWrite(RTTY_ENABLE, bit);
  }
  else
  {
    #ifdef RTTY_PWM  
    // PWM control
      if (bit)
      {
        analogWrite(RTTY_DATA, (RTTY_SHIFT *1.8) / 16); // High
      }
      else
      {
        analogWrite(RTTY_DATA, 0); // Low
      }
    #else
      digitalWrite(RTTY_DATA, bit);
    #endif
  }
}

ISR(TIMER1_COMPA_vect)
{ 
    // Are we sending?
    if (SendIndex >= 0)
    {
      // Send correct bit or start or stop bit
      if (SendBit == 0)
      {
        rtty_txbit(0);
      }
      else if (SendBit <= DataBits)
      {
        rtty_txbit(TxLine[SendIndex] & (1 << (SendBit-1)));
      }
      else
      {
        rtty_txbit(1);  // Stop bit(s)
      }
      
      if (++SendBit > (DataBits+StopBits))
      {
        SendBit = 0;
        if (TxLine[++SendIndex] == 0)
        {
          SendIndex = -1;
        }
      }
    }
}

#ifdef MTX2    
  #ifdef RTTY_FREQUENCY
void SetMTX2Frequency(float Frequency)
{
  float _mtx2comp;
  int _mtx2int;
  long _mtx2fractional;

  SendIndex = -1;
  DataBits = 8;
  StopBits = 1;

  SetupTimer1(9600);
  digitalWrite(RTTY_ENABLE, 1);
  delay(100);
		
  _mtx2comp=(Frequency+0.0015)/6.5;
  _mtx2int=_mtx2comp;
  _mtx2fractional = ((_mtx2comp-_mtx2int)+1) * 524288;
  sprintf(TxLine, "@PRG_%02X%06lX\r", _mtx2int-1, _mtx2fractional);

  // Serial.print("MTX2 command is "); Serial.println(TxLine);

  SettingFrequency = 1;
  SendIndex = 0;
  
  while (SendIndex >= 0)
  {
    delay(10);
  }
  
  SettingFrequency = 0;
  delay(100);
  digitalWrite(RTTY_ENABLE, 1);
  delay(100);
  
  // Serial.print("RTTY Frequency set to "); Serial.println(Frequency);
}
    #endif
  #endif

#endif
#endif

