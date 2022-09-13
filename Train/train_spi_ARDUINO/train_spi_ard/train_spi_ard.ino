/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *

 */

#include <SPI.h>
//https://www.makerguides.com/master-slave-spi-communication-arduino/
/*SPCR
| 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
| SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |

SPIE - Enables the SPI interrupt when 1
SPE - Enables the SPI when 1
DORD - Sends data least Significant Bit First when 1, most Significant Bit first when 0
MSTR - Sets the Arduino in controller mode when 1, peripheral mode when 0
CPOL - Sets the data clock to be idle when high if set to 1, idle when low if set to 0
CPHA - Samples data on the falling edge of the data clock when 1, rising edge when 0
SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz) 11 is slowest (250KHz)*/

/*SPSR: status register
Bit 7 – SPIF: SPI interrupt flag bit.
When the serial data transfer is complete, this flag gets set. It also gets set when the SS pin is driven low in Master mode.
Bit 6 – WCOL: Write Collision Flag bit. This bit is set when SPI data register writes occur during previous data transfer.
Bit 5:1 – Reserved Bits. These bits are reserved bits, and they will always read as zero.
Bit 0 – SPI2X: Double SPI Speed bit. If  SPI speed (SCK Frequency) gets doubled when set this bit.
 */

/*SPDR: data register
The SPI Data Register is a read/write register. It is used for data transfer between the Register File and the SPI Shift Register.
Writing to the register initiates data transmission.
Reading the register causes the Shift Register Receive buffer to be read.
 */

/*const byte led = 9;           // Slave LED digital I/O pin.

boolean ledState = HIGH;      // LED state flag.*/

uint8_t dataBuff[255];

uint8_t board_id[10] = "ARDUINOUNO";

#define NACK 0xB1
#define ACK 0xA1


//command codes
/*#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54*/

#define SPI_COMMAND_PRINT    0x50
#define SPI_COMMAND_LED   0x51

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  //make SPI as slave
  
  // Enable SPI (bit 6=1) as slave (bit 4=0).no interrupts
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  Serial.println("in SlaveReceive");
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  Serial.print("SPDR,received:");
  Serial.println(SPDR);
  Serial.println("ending SlaveReceive");
  return SPDR;
  
}


//sends one byte of data 
void SPI_SlaveTransmit(uint8_t data)
{
  Serial.print("in SlaveTransmit. SPDR=data transmitted=");
  Serial.println(data);
  /* Start transmission */
  SPDR = data;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
  Serial.println("ending SlaveTransmit");
}
  

// The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
  
  // Initialize slave LED pin.
  /*pinMode(led, OUTPUT);
  
  digitalWrite(led,LOW);*/
  
  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}


byte checkCommandPrint(byte inputcommand)
{
  Serial.println("in checkData");
  //todo
  if (inputcommand == SPI_COMMAND_PRINT){
    return ACK;
  }
  else {
    return NACK;
  }
}

// The loop function runs continuously after setup().
void loop() 
{
  byte data,command,len,ackornack=NACK;
  int i = 0;
  
  //1. fist make sure that ss is low . so lets wait until ss is low 
  Serial.println("Slave waiting for ss to go low");
  Serial.print("SS read: ");
  Serial.println(digitalRead(SS));
  while(digitalRead(SS)){
    //Serial.println("SS read: "+digitalRead(SS));
  }
  
  //2. now lets wait until rx buffer has a byte
  Serial.println("1.Received command");
  command = SPI_SlaveReceive();
  ackornack = checkCommandPrint(command);
  
  Serial.println("2.command to be transmitted when dummy");
  SPI_SlaveTransmit(ackornack);
  
  Serial.println("3.Received dummy");
  data = SPI_SlaveReceive(); //dummy byte from STM32 so that ACK/NACK is sent
  
  if ( command == SPI_COMMAND_PRINT)
  {
    Serial.println("4.Received length");
    //uint8_t len = SPI_SlaveReceive(); 
    len = SPI_SlaveReceive(); 
    for(i=0 ; i < len ; i++)
    {
      Serial.print(i);
      Serial.println(" byte position received:");
      dataBuff[i] = SPI_SlaveReceive();
    }
    /*Serial.print("length:");
    Serial.println(len);
    Serial.print("dataBuff: ");
    Serial.println((char*)dataBuff);
    Serial.println("RCVD:COMMAND_PRINT");*/
  }
  else{
    Serial.println("Other than PRINT");
  }
 

}
