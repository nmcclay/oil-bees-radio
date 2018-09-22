#include <SPI.h>
#include <RH_RF95.h>
#include <SparkFun_I2C_GPS_Arduino_Library.h> //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
I2CGPS myI2CGPS; //Hook object to the library
#include <TinyGPS++.h> //From: https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps; //Declare gps object

RH_RF95 rf95(A5, 7);
const byte LED = 13; // Pro Micro
const byte SWITCH = A0;

int packetCounter = 0; //Counts the number of packets sent

long timeSinceLastPacket = 0; //Tracks the time stamp of last packet received

void setup() 
{
  pinMode(LED, OUTPUT);
  pinMode(SWITCH, INPUT);
  digitalWrite(LED, LOW);

  Serial.begin(115200);
  while (Serial == false);
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
  
  char toSend[50];
  // sprintf(toSend, "N=%d", packetCounter++);
  addGPSInfo(toSend); // 46 characters
  
  int buttonState = digitalRead(SWITCH);
  char *buttonText;
  if (buttonState == HIGH) {
    buttonText = " X=1";
  } else {
    buttonText = " X=0";
  }
  strcat(toSend, buttonText);

  rf95.send((const uint8_t*)toSend, sizeof(toSend));
  rf95.waitPacketSent();
  Serial.println(toSend);
  
  digitalWrite(LED, HIGH); //Turn off status LED
  delay(500);
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
  sprintf(info, "T=%s S=%s G=%s:%s", timeString, speedString, latitudeString, longitudeString);
}
