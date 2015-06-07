#define MSG_SIZE 6          //the size of the payload
#define HEADER1 0xF6      // the first header
#define HEADER2 0x99      // the second header
#define PWM_OUT_PORT 9      // The digital port of the PWM (D9)
#define LIGHTS_SWITCH 6      //digital ouput port for turning on and off the lights
#define HORN_SWITCH 5      //digital ouput port for turning on and off the horn
#define AUX_SWITCH 4      //digital ouput port for turning on and off - AUX
//#define NO_DATA_LED 13    // a led that idicates that no data/corrupted had been recieved

/**
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 *
 * Configurable:
 * CE -> 8
 * CSN -> 7
 */

#include <Servo.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

byte payload[MSG_SIZE];

// loop timer
unsigned long time = 0;

// create servo object to control the 2 ECSs
Servo servoMotor;  

int i=0;

//the throttle value (900 to 2100);
int throttle;

volatile  byte Packet =0;

// the number of the previuos sequence from the previuos loop
byte prev_seq;

//count the number of the frame that where lost
//lost counts as corrupted (bad checksum) or
//didn't make it al all
volatile int framelost = 0;

void setup()
{
  // for debugging
  //Serial.begin(9600);
  
  attachInterrupt(0, rcvPacket , RISING);
  
  //led used for error reporting of no data/corrupted data
  //pinMode(NO_DATA_LED,OUTPUT);
  pinMode(LIGHTS_SWITCH,OUTPUT);
  pinMode(HORN_SWITCH,OUTPUT);
  pinMode(AUX_SWITCH,OUTPUT);
  
  // attachs the PWM output port to the PWM calbe of the motors
  servoMotor.attach(PWM_OUT_PORT);
  
  // Setup pins / SPI.
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  
  //Configure reciving address 
  // mbglv - mountainboard glove
  // mbrx1 - mountainboard rx1
  Mirf.setRADDR((byte *)"mbrx1");
  
   /*
   * Set the payload length = 5 Bytes
   * Byte0 = Header 1 = 0xF6
   * Byte1 = Header2 = 0x99
   * Byte2 = PWM = (0-255)
   * Byte3 = Options = 8bit options
   * Byte4 = sequence
   * Byte5 = Checksum
   */
   
   // zeros the sequence counter
  prev_seq = 0;
  
  // sets the headers
  payload[0] = HEADER1; 
  payload[1] = HEADER2;
 
  // the payloaf size
  Mirf.payload = MSG_SIZE;
  
  // sets channel for the RF module
  Mirf.channel = 16;
   
  // configures the RF module
  Mirf.config(); 
  
  // makes sure the LED error report is low 
  // so if it will turns or we'll know that there 
  // was a no data/corrupted data incedent
  //digitalWrite(NO_DATA_LED,LOW);
}

void loop()
{
  
 
  
    if ( ( millis() - time ) > 20 ) 
    {
      time = millis();      
      Packet++;
    }
    
    if(!Mirf.isSending() && Mirf.dataReady())
    {
       Mirf.getData(payload); 

       /*
       Serial.println(payload[0]);
       Serial.println(payload[1]);
       Serial.println(payload[2]);
       Serial.println(payload[3]);
       Serial.println(payload[4]);
       Serial.println(payload[5]);
       Serial.println("----------------------------");
       Serial.print("packet ");
       Serial.println(Packet);
       */
       Mirf.setTADDR((byte *)"mbglv");
       
       // count the frame lost for safty - or Packet > 10 
       if ((calc_checksum() != payload[MSG_SIZE - 1])  || (Packet > 5))
       {
         framelost++;
       }
       else
       {
         framelost = 0;
         prev_seq = payload[4] + 1;
       }
    }
    else
    {
      //framelost++;
    }
    if (Packet < 200 )
    {
      Packet++;
    }
    
  
    // if bad corrupted frame were recieved for 200ms or
    // not recieved att all then breaks the mountainboard
    if (framelost >= 10 or payload[3] == 0x01 or Packet > 10)
    {
      // in order to not overflow the frame lost counter
      framelost = 10;

      // natural - SLOW BREAKS!!!!
      servoMotor.writeMicroseconds(1500);
      // turn on the LED error report - no data/corrupted data
      //digitalWrite(NO_DATA_LED,HIGH);
    } 
    else 
    // every thing is OK.. zeros the framelost counter
    // and transmit the PWM data from the payload to the ESCs
    {
      //framelost = 0;
      throttle = map(payload[2], 0, 255, 1000, 2000);
      if (payload[3] == 0x01)
      {
        digitalWrite(LIGHTS_SWITCH, HIGH);
      }
      if (payload[3] == 0x02)
      {
        digitalWrite(HORN_SWITCH, HIGH);
      }
      // D/R, Expo and deadband are to added here
      
      digitalWrite(LIGHTS_SWITCH, HIGH);
      digitalWrite(HORN_SWITCH, LOW);
      digitalWrite(AUX_SWITCH, HIGH);
      
      servoMotor.writeMicroseconds(throttle);    
      //framelost = 0;
      
    }
      /*
      Serial.print( "seq: ");
      Serial.println(payload[4]);
      Serial.print( "PWM: ");
      Serial.println(payload[2]);
      Serial.print( "framelost: ");
      Serial.println(framelost);
      Serial.print( "Packet: ");
      Serial.println(Packet);
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
  */
}

//interrupt function for recieiving bytes
void rcvPacket()
{
  Packet=0;  
  //Serial.print("hit");
}

// check sum data check up 
byte calc_checksum()
{
  byte calc_cs = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++)
  {
    calc_cs += payload[i];
  }
  return calc_cs;
}
