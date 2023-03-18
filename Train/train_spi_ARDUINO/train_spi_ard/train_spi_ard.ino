/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *

 */
#include <LiquidCrystal.h>
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

 //REMOVE AS MANY SERIAL.PRINT AS POSSIBLE! I USE IT FOR TESTING BUT IT TAKES TIME FOR COMMUNICATION

//uint8_t dataBuff[255];
char dataBuff[32];//32 max because of LCD screen size (16x2)

//ACK/NACK for SPI
#define NACK 0xB1
#define ACK 0xA1
//command codes for SPI
#define SPI_COMMAND_PRINT    0x50

//LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2
//initialize library by indicating pins to be used.(RS[5 in LCD], enable[6 in LCD], d4, d5, d6, d7 [7-14]). (rs,enable,d1,d2,d3,d4)
LiquidCrystal lcd(7, 6, 5, 4, 3, 2); 

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
  //Serial.println(SPDR); //better not read here
  return SPDR;
  
}

//This function receives a message of many bytes
void SPI_SlaveReceive_Message(byte messLen){
  for(int i=0 ; i < messLen ; i++)
    {
      //Serial.print(i);
      //Serial.println(" byte position received:");
      while(!(SPSR & (1<<SPIF)));
      dataBuff[i] = SPSR;
      //Serial.println(dataBuff[i]);
      lcd.setCursor(i%(LCD_COLUMNS),i/LCD_COLUMNS); // Position the cursor where to write (column,row)
      lcd.write(dataBuff[i]);
      //in case of lack of synchronization,don't use setCursor or write here. Instead, lcd.setCursor(0,0) and lcd.write(dataBuff) after returning from here
    }
}

//sends one byte of data 
void SPI_SlaveTransmit(uint8_t data)
{
  //Serial.print("in SlaveTransmit. SPDR=data transmitted=");
  //Serial.println(data);
  /* Start transmission */
  SPDR = data;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
  //Serial.println("ending SlaveTransmit");
}
  

// The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
  
  // Initialize slave LED pin.
  /*pinMode(led, OUTPUT);
  
  digitalWrite(led,LOW);*/

  //LCD
  lcd.begin(LCD_COLUMNS,LCD_ROWS);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setup");
  lcd.setCursor(0,1);
  lcd.print("working");
  
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
  byte dataRec,len,ackornack=NACK;
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
  dataRec = SPI_SlaveReceive();
  Serial.println(dataRec);
  ackornack = checkCommandPrint(dataRec);
  
  Serial.println("2.command to be transmitted when dummy");
  Serial.println(ackornack);
  SPI_SlaveTransmit(ackornack);
  
  Serial.println("3.Received dummy");
  dataRec = SPI_SlaveReceive(); //dummy byte from STM32 so that ACK/NACK is sent
  //Serial.println(data);
  
  if ( dataRec == SPI_COMMAND_PRINT)
  {
    Serial.println("4.Received length");
    //uint8_t len = SPI_SlaveReceive(); 
    len = SPI_SlaveReceive();
    Serial.println(len);
    lcd.clear(); //clear the LCD screen (first time)
    SPI_SlaveReceive_Message(len); //read all the message in this function rather than looping here,which delays time due to calls
    //lcd.print(dataBuff);

  }
  else{
    Serial.println("Other than PRINT");
  }
 

}
