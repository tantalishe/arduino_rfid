#include "rdm630.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD's i2c adress
byte LCD_I2C = 0x3F;

LiquidCrystal_I2C lcd(LCD_I2C,16,2);

rdm630 rfid(6, 0);  //TX-pin of RDM630 connected to Arduino pin 6

unsigned long uidDec, uidLast;
byte data[6];
byte length;

void setup() {
  Serial.begin(9600);

  rfid.begin();

  // init lcd
  lcd.init();
  lcd.backlight();
  
  uidDec = 0;
  uidLast = 0;
}
void loop() {
  // display last tag's uid
  lcd.setCursor(0,0);
  lcd.print("Last tag's UID:");
  lcd.setCursor(0,1);
  lcd.print(uidLast);

  // get tag's uid and decode it
  if(rfid.available()){
    rfid.getData(data,length);
 
    uidDec = 
          ((unsigned long int)data[1]<<24) + 
          ((unsigned long int)data[2]<<16) + 
          ((unsigned long int)data[3]<<8) + 
          data[4];
          
    if ( uidDec == uidLast ) {
      return;
    }
    uidLast = uidDec;
    // send uid to serial port
    Serial.println(uidLast);
    delay(100);
  }
}
