#define MSG_SIZE 6          //the size of the payload
#define HEADER1 0xF6      // the first header
#define HEADER2 0x99      // the second header
#define KILLSWITCH 4    // the digital pin number of the kill switch
#define FLEXSENSOR 0    // the analog pin muber of the flex sensor

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

/*
  The keypad circuit:
  SparkFun Keypad                Arduino
  -----------------------------------------
    Wire 1 (NC) ------------- No connection
   Wire 2 (LEDR) ----------------- D3
    Wire 3 (GND) ------------------GND
   Wire 4 (P5.1) ----------------- D5
   Wire 5 (P5.3) ----------------- D6
   Wire 6 (P5.2) ----------------- D7
*/

// Pin definitions
int p51 = 5;
int p53 = 6;
int p52 = 9;

//deadband for the flex sensor to PWM
// 256 = 8 bit, the middle is 128 so 10% deadband is ~13 to have an equad i deceided 14 bit >> 128 - 7 = 121, 128 + 7 = 135
int startDeadband =  121;
int endDeadband =  135;


// Global variables
int button = 0;
int keypressed = 0;


//MIRF
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

byte payload[MSG_SIZE];
int intFlexRead;
boolean bolKSW;          //killswitch on/off
boolean StateKSW  = HIGH;
boolean LastStateKSW = HIGH;

// loop timer
unsigned long time = 0;

void setup(){
  // for debugging
  //Serial.begin(9600);
  
  // Initially set up the pins for the keypad
  pinMode(p51, INPUT);
  pinMode(p53, INPUT);
  pinMode(p52, INPUT);
  digitalWrite(p51, HIGH);
  digitalWrite(p52, HIGH);
  digitalWrite(p53, HIGH);
  
  // sets the D0 pin for detect the killswitch
  pinMode(KILLSWITCH,INPUT);
  digitalWrite(KILLSWITCH,HIGH);
  
  // sets the A0 pin for reading the flex sensor
  pinMode(FLEXSENSOR,INPUT);
  
  // Setup pins / SPI.
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  
  //Configure reciving address 
  // mbglv - mountainboard glove
  // mbglv - mountainboard rx1
  Mirf.setRADDR((byte *)"mbglv");
  
  
   /*
   * Set the payload length = 5 Bytes
   * Byte0 = Header 1 = 0xF6
   * Byte1 = Header2 = 0x99
   * Byte2 = PWM = (0-255)
   * Byte3 = Options = 8bit options
   * Byte4 = sequence
   * Byte5 = Checksum
   */
  
  //Byte 3 - options: (LSB)  
  //                  bit 1 = lights 
  //                  bit 2 = horn
   
   
  // sets the headers
  payload[0] = HEADER1; 
  payload[1] = HEADER2;
  
  payload[4] = 0;
 
  // the payloaf size
  Mirf.payload = MSG_SIZE;
  
  // sets channel for the RF module
  Mirf.channel = 16;
   
  // configures the RF module
  Mirf.config();
  
  
}

void loop(){
  
  button = getButtonState();  // Get button status
  if (button == 0x04)  // FLAME
  {
    Serial.println("flame");
  }
  else if (button == 0x02)  // UP
  {
    Serial.println("up");
  }
  else if (button == 0x01)  // DOWN
  {
    Serial.println("down");
  }
  else if (button == 0x08)  // RIGHT
  {
    Serial.println("right");
  }
  else if (button == 0x10)  // LEFT
  {
    Serial.println("left");
  }
  
  // sets target RF module
  Mirf.setTADDR((byte *)"mbrx1");
  
  // the flex sensor readings 
  intFlexRead = analogRead(FLEXSENSOR);
  
  //Serial.println(intFlexRead);
  // maps it into 256 bit resulotion (1byte PWM)
  payload[2] = map(intFlexRead, 450, 780, 0, 255);
  if (payload[2] >= startDeadband and payload[2] <= endDeadband)
 {
   payload[2] = 128;
 } 
 //Serial.println(intFlexRead);
  
  // the kill switch reading
  bolKSW = digitalRead(KILLSWITCH);
  
  // if the killswitch is not pressed
  
  if (bolKSW != LastStateKSW)
  {
    time = millis();
  }
  
  
  //check every 20ms if the killswitch is pressed (connecto to GND)
  if ( ( millis() - time ) > 20 ) {
        StateKSW = bolKSW;
        if (StateKSW == LOW)
        {
          payload[3] = 0x00;
        }
        else // sets the PWM to natural - SLOW BREAK!!!
        {
          payload[3] = 0x01;
          payload[2] = 128;
        }
    }
      SendToBoard();
    
      LastStateKSW = bolKSW;
} 
  
// transmite... 
void SendToBoard()
{
  // Generate sequence before sending
  payload[4] = payload[4]++;
  // calculate checksum  
  checksum();
  Mirf.send(payload);
  while(Mirf.isSending())
  {
    // waits to finish sending data
  }
  //Serial.println("data sent");
  
}
 
// check sum data check up 
void checksum()
{
  payload[MSG_SIZE-1] = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++)
  {
    payload[MSG_SIZE-1] += payload[i];
  }  
}

/* getButtonState() will return a uint8_t representing the status
  of the SparkFun button pad. The meaning of the return value is:
  0x01: Down
  0x02: Up
  0x04: Flame
  0x08: Right
  0x10: Left
*/
uint8_t getButtonState()
{
  // Initially set all buttons as inputs, and pull them up
  pinMode(p52, INPUT);
  digitalWrite(p52, HIGH);
  pinMode(p51, INPUT);
  digitalWrite(p51, HIGH);
  pinMode(p53, INPUT);
  digitalWrite(p53, HIGH);
  
  // Read the d/u/flame buttons
  if (!digitalRead(p53))
    return 0x01;  // Down
  if (!digitalRead(p52))
    return 0x02;  // Up
  if (!digitalRead(p51))
    return 0x04;  // Flame
    
  // Read right button
  pinMode(p52, OUTPUT);  // set p52 to output, set low
  digitalWrite(p52, LOW);
  if (!digitalRead(p53))
    return 0x08;  // Right
  pinMode(p52, INPUT);  // set p52 back to input and pull-up
  digitalWrite(p52, HIGH);
  
  // Read left button
  pinMode(p51, OUTPUT);  // Set p51 to output and low
  digitalWrite(p51, LOW);
  if (!digitalRead(p53))
    return 0x10;  // Left
  pinMode(p51, INPUT);  // Set p51 back to input and pull-up
  pinMode(p51, HIGH);
  
  return 0;
}
