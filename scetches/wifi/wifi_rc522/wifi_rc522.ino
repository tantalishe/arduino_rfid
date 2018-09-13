#include <SPI.h>
#include <MFRC522.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SS_PIN 53
#define RST_PIN 5

// wifi settings
String SSID = "CSI-Robotics"; 
String PASS = "csi-robotics";
int PORT = 80;

bool DEBUG = true;   //show more logs
int responseTime = 100; //communication timeout
byte LCD_I2C = 0x27; // 0x3F; // LCD's i2c adress
int readerHysteresisTime = 2000; 

LiquidCrystal_I2C lcd(LCD_I2C,16,2); //sda - a4, scl - a5

MFRC522 mfrc522(SS_PIN, RST_PIN);

int buzzerPin = 11; //Define buzzer pin

unsigned long uidDec, uidDecTemp, lastReadTime;
String uidCur, uidLast;

void setup() {
  // Open serial for debugging
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial) {
      ;
    }
  }
  // Open serial communications and wait for port to open esp8266:
  Serial1.begin(115200);
  while (!Serial1) {
    ;
  }
  
  // Init SPI bus.
  SPI.begin();
  // Init MFRC522 card.
  mfrc522.PCD_Init();
  // max antenna power
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
  // init lcd
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting to");
  lcd.setCursor(0,1);
  lcd.print("network...");
  
//  bool wifi_conection = false;
  sendToEsp("AT+RST", responseTime, DEBUG);
  delay(15000); // if esp cant find previous network, starting can take long time
  WifiInit(SSID, PASS, PORT);
  // smart connection that doeset work
//  while(wifi_conection == false){
//    Serial.println("trying to connect...");
//    if(Serial1.available()){
//      if (Serial1.find("ready")) {
//        WifiInit(SSID, PASS, PORT);
//        wifi_conection = true;
//        Serial.println("WiFi connected");
//      }
//    }
//    delay(1000);
//  }
}

void loop() {
  uidLast = uidCur;
  uidCur = checkReader();

  // display current tag's uid
  if (uidCur != uidLast) {
    if (uidCur == "0") {
      beepdown(35);
    }
    else {
      beepup(50);
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Current tag UID:");
    lcd.setCursor(0,1);
    lcd.print(uidCur);
  }

  String data = uidCur;
  if(Serial1.available()){
    // checking for request
    if (Serial1.find("0,CONNECT")) {
      sendData(data);
    }
    if (Serial1.find("ready")) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Tryin to");
      lcd.setCursor(0,1);
      lcd.print("reconnect...");
      WifiInit(SSID, PASS, PORT);
    }
  }
}


void WifiInit(String ssid, String pass, int port) {
  sendToEsp("AT+CWMODE=3", responseTime, DEBUG); // configure as access point
  delay(1000);
  sendToEsp("AT+CWQAP", responseTime, DEBUG);  // disconnect from previous network
  delay(1000);
  sendToEsp("AT+CWJAP=\"" + ssid + "\",\"" + pass + "\"",5000,DEBUG); // connect to new netwok
  delay(1000);
  sendToEsp("AT+CIFSR", responseTime, DEBUG); // check for ip
  delay(1000);
  sendToEsp("AT+CIPMUX=1", responseTime, DEBUG); // multiconnection mode
  delay(1000);
  sendToEsp("AT+CIPSERVER=1,80", responseTime, DEBUG); // turn on server on port 80
}

// function for sending data via esp
void sendData(String data) {
  String len = "";
  len += data.length();
  sendToEsp("AT+CIPSEND=0," + len, responseTime, DEBUG);
  delay(100);
  sendToEsp(data, responseTime, DEBUG);
  delay(100);
  sendToEsp("AT+CIPCLOSE=0", responseTime, DEBUG);
}

String sendToEsp(String command, const int timeout, boolean debug) {
  String response = "";
  if (debug)
  {
    Serial.println("command: " + command + ",  timeout:" + timeout);
  }
  Serial1.println(command); // send the read character to the esp8266
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (Serial1.available())
    {
      // The esp has data so display its output to the serial window
      char c = Serial1.read(); // read the next character.
      response += c;
    }
  }
  if (debug)
  {
    Serial.println("response: " + response);
  }

  return response;
}

String checkReader() {
  // find frid tag
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    if (millis() - lastReadTime > readerHysteresisTime) {
      return "0";
    }
  }
  else {
    lastReadTime = millis();
  }
  
  // get tag's uid and decode it
  uidDec = 0;
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    uidDecTemp = mfrc522.uid.uidByte[i];
    uidDec = uidDec * 256 + uidDecTemp;
  }
  delay(50);
  return String(uidDec);
}

// melody for getting tag
void beepup(unsigned char delayms) {
  analogWrite(buzzerPin, 10);
  delay(delayms); //Delaying
  analogWrite(buzzerPin, 0);
  delay(delayms); //Delaying
  analogWrite(buzzerPin, 20);
  delay(delayms); //Delaying
  analogWrite(buzzerPin, 10);
  delay(delayms); //Delaying
  analogWrite(buzzerPin ,0);
  
}

// melody for losing tag
void beepdown(unsigned char delayms) {
  analogWrite(buzzerPin, 20);
  delay(delayms); //Delaying
  analogWrite(buzzerPin, 0);
  delay(delayms); //Delaying
  analogWrite(buzzerPin, 10);
  delay(delayms); //Delaying
  analogWrite(buzzerPin ,0);
  
}
