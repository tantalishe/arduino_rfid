#include <SPI.h>
#include <MFRC522.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SS_PIN 10
#define RST_PIN 9

// LCD's i2c adress
byte LCD_I2C = 0x3F;

LiquidCrystal_I2C lcd(LCD_I2C,16,2);

MFRC522 mfrc522(SS_PIN, RST_PIN);

unsigned long uidDec, uidDecTemp, uidLast;

void setup() {
  Serial.begin(9600);
  
  // Init SPI bus.
  SPI.begin();
  // Init MFRC522 card.
  mfrc522.PCD_Init();
  // max antenna power
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  // init lcd
  lcd.init();
  lcd.backlight();
  
//  Serial.println("ready");
  
  uidDec = 0;
  uidLast = 0;
}
void loop() {
  // display last tag's uid
  lcd.setCursor(0,0);
  lcd.print("Last tag's UID:");
  lcd.setCursor(0,1);
  lcd.print(uidLast);
  
  // find frid tag
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // get tag's uid and decode it
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    uidDecTemp = mfrc522.uid.uidByte[i];
    uidDec = uidDec * 256 + uidDecTemp;
  }
  if ( uidDec == uidLast ) {
    return;
  }
  uidLast = uidDec;
  // send uid to serial port
  Serial.println(uidDec);
  delay(100);
}
