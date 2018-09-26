#include <SPI.h>
#include <RH_RF95.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h> //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
I2CGPS myI2CGPS; //Hook object to the library
#include <TinyGPS++.h> //From: https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps; //Declare gps object

// ESP32
//const byte RFM95_CS = 16; 
//const byte RFM95_INT = 26;
//const byte LED = 23;
//const byte SWITCHES[5] = {A0, A3, A4, A5, A6};

// Pro Micro
const byte RFM95_CS = A5;
const byte RFM95_INT = 7;
const byte LED = 13; 
const byte SWITCHES[5] = {A0, A1, A2, A3, A4};

RH_RF95 rf95(RFM95_CS, RFM95_INT);
int packetCounter = 0; //Counts the number of packets sent

void setup() 
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  for (int i = 0; i <= sizeof(SWITCHES);i++) {
     pinMode(SWITCHES[i], INPUT);
  };

  Serial.begin(115200);
//  while (Serial == false);
  Serial.println("RFM Test");

  if (rf95.init() == false)
  {
    Serial.println("Radio Init Failed - Freezing");
    while (1);
  }
  Serial.println("LoRa module found!");

  Serial.println("GTOP Test");
  if (myI2CGPS.begin() == false)
  {
    Serial.println("Module failed to respond. Please check wiring.");
    while (1); //Freeze!
  }
  Serial.println("GPS module found!");

  //  RH_RF95::Bw31_25Cr48Sf512, < Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
  //  RH_RF95::Bw125Cr48Sf4096,  < Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
  rf95.setFrequency(909.2);
  rf95.setTxPower(23, false);
}

void loop()
{
  digitalWrite(LED, LOW);

  char toSend[48];
  sprintf(toSend, "N=%d", packetCounter++);
  addGPSInfo(toSend); // 40 characters
  addButtonInfo(toSend); // 7 characters

  rf95.send((const uint8_t*)toSend, sizeof(toSend));
  rf95.waitPacketSent();
  Serial.println(toSend);
  
  digitalWrite(LED, HIGH); //Turn off status LED
  delay(500);
}

void addButtonInfo(char* info) {
  int buttonStates[5];
  for (int i = 0; i <= sizeof(SWITCHES);i++) {
    int buttonState = digitalRead(SWITCHES[i]);
    if (buttonState == HIGH) {
      buttonStates[i] = 1;
    } else {
      buttonStates[i] = 0;
    }
  };

  char buttonText[7];
  sprintf(buttonText, " %d%d%d%d%d", buttonStates[0], buttonStates[1], buttonStates[2], buttonStates[3], buttonStates[4]);
  strcat(info, buttonText);
}

//appends GPS info to response
void addGPSInfo(char* info) {
  while (myI2CGPS.available()) { //available() returns the number of new bytes available from the GPS module
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }

  uint32_t gpsTime = 0;
  char timeString[9];
  if (gps.time.isValid()) {
     sprintf(timeString, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  double gpsSpeed = 0;
  char speedString[7];
  if (gps.speed.isValid()) {
     gpsSpeed = gps.speed.mph();
  }
  dtostrf(gpsSpeed, 3, 2, speedString);

  double gpsLatitude = 0;
  double gpsLongitude = 0;
  char latitudeString[12];
  char longitudeString[12];
  if (gps.location.isValid()) {
    gpsLatitude = gps.location.lat();
    gpsLongitude = gps.location.lng();
  }
  dtostrf(gpsLatitude, 3, 6, latitudeString);
  dtostrf(gpsLongitude, 3, 6, longitudeString);
  sprintf(info, "%s %s %s:%s", timeString, speedString, latitudeString, longitudeString);
}
