#include <Arduino.h>
#include <SPI.h>

#define DRDY 6 // ADC tells us when it is ready
#define ADCRST 7 // Resets the ADC
#define SPICLOCK 9
#define SS 0 // Not yet connected electrically

void setup()
{
  delay(100);
  pinMode(SS, OUTPUT);
  pinMode(DRDY, INPUT);
  pinMode(ADCRST, OUTPUT);
  
  digitalWrite(SS, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  digitalWrite(SPICLOCK, HIGH);
  digitalWrite(ADCRST, HIGH);
  delay(1000);
  digitalWrite(ADCRST, LOW);  // LOW value triggers ADC reset.
  delay(1000);
  digitalWrite(ADCRST, HIGH);
  delay(1000);
  
  Serial.begin(9600);
  
  delay(300);
}

void MAX1416_Config()//You can modify it for handle channels
{
  //series of commandbit    
  digitalWrite(SS, LOW); // Enable ADC SPI
  
  //Write OP
  SPI.transfer(0x20);//command for comm reg to select ch1 and write to clock register 
  delay(100);
  SPI.transfer(0xA7);//command for clock reg to set 2,4576Mhz                                  
  //End Write OP
  delay(100);
  //Write OP
  SPI.transfer(0x10);//command for comm reg to write setup register 
  delay(100);
  SPI.transfer(0x44);//command for setup reg to self calibration,unipolar,unbuffered,     
  //End Write OP

  digitalWrite(SS,HIGH); // Disable ADC SPI
}

void MAX1416_WaitForData_Soft() 
{
      char DataNotReady = 0x80;
      
      digitalWrite(SS, LOW); // Enable ADC SPI
      
      while(DataNotReady) // wait for end of conversion 
      {
          // Read OP
          SPI.transfer(0x08);//command for comm reg to read  (dec   8)
          DataNotReady =SPI.transfer(0x00); // Read comm register
          // End Read OP
          Serial.println(DataNotReady, BIN);
          DataNotReady &= 0x80;
      }
      
      digitalWrite(SS, HIGH); // Disable ADC SPI
}

void MAX1416_WaitForData_Hard() 
{
      char DataNotReady = 1;
      char value;

      while(DataNotReady) // wait for end of conversion 
      {
          // Read OP
          value = digitalRead(DRDY); // Read comm register
           if (value == LOW)
             DataNotReady = 0;
           else
             DataNotReady = 1;
          // End Read OP
          //Serial.println("NOT READY");
      }
}

byte MAX1416_ReadSetupReg() //You can modify it to read other channels
{

      byte myByte;
      
      digitalWrite(SS, LOW); // Enable ADC SPI
      
      // READ Data OPERATION
      SPI.transfer(0x18);//command for the comm to read register register 00011000
      //read 8bit of data
      myByte = SPI.transfer(0x00);
      // End Read Data
      Serial.print(myByte,BIN);
      //delay(2000);
      digitalWrite(SS, HIGH); // Disable ADC SPI
    
      
      return myByte;
}


unsigned int MAX1416_ReadCH0Data() //You can modify it to read other channels
{
      unsigned int uiData;
      byte highByte;
      byte lowByte;
      
      digitalWrite(SS, LOW); // Enable ADC SPI
      
      // READ Data OPERATION
      SPI.transfer(0x38);//command for the comm to read data register for channel 1 (dec  56)
      //read 16bit of data ADC
      highByte = SPI.transfer(0x00);
      lowByte = SPI.transfer(0x00);
      // End Read Data
      
      digitalWrite(SS,HIGH); // Disable ADC SPI
    
      
      uiData = highByte;
      uiData <<= 8;
      uiData |= lowByte;
      
      return uiData;
}

void loop()
{
  //digitalWrite(ss,LOW); // Enable ADC SPI
  delay(100); 
  MAX1416_Config();
  delay(100);
  unsigned int adcValue;
  double volt;
  MAX1416_ReadSetupReg();
  while(1)
  {
      
      //MAX1416_WaitForData_Soft() ;
      MAX1416_WaitForData_Hard() ;
      delay(10);
      adcValue = MAX1416_ReadCH0Data();
      Serial.print("analog value =");
      Serial.print(adcValue);
      volt=double(adcValue)*5/65535;
      Serial.print(" volt =");
      Serial.println(volt,4);
      
  }
  digitalWrite(SS,HIGH); // Enable ADC SPI

}
