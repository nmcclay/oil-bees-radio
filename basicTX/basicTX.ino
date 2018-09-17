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

  Serial.begin(9600);
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

  rf95.setFrequency(909.2);
  rf95.setTxPower(23, false);
}

void loop()
{
  digitalWrite(LED, LOW);
  
  char toSend[50];
  sprintf(toSend, "COUNT=%d ", packetCounter++);
  
  while (myI2CGPS.available()) { //available() returns the number of new bytes available from the GPS module
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }
  char *GPSInfo = getGPSInfo();
  strcat(toSend, GPSInfo);
  
  int buttonState = digitalRead(SWITCH);
  char *buttonText;
  if (buttonState == HIGH) {
    buttonText = " SWITCH=ON";
    strcat(toSend, buttonText);
  }

  rf95.send((const uint8_t*)toSend, sizeof(toSend));
  rf95.waitPacketSent();
  Serial.println(toSend);
  
  digitalWrite(LED, HIGH); //Turn off status LED
  delay(500);
}

//Display new GPS info
char* getGPSInfo()
{
  uint32_t gpsTime = 0;
  double gpsSpeed = 0;
  double gpsLatitude = 0;
  double gpsLongitude = 0;

  if (gps.time.isValid()) {
    if (gps.time.isUpdated()) {
      gpsTime = gps.time.value();
    }

    if (gps.speed.isUpdated()) {
      gpsSpeed = gps.speed.mph();
    }
  }

  if (gps.location.isValid()) {
    gpsLatitude = gps.location.lat();
    gpsLongitude = gps.location.lng();
  }

  char info[100];
  sprintf(info, "TIME=%d MPH=%f GPS=%f:%f", gpsTime, gpsSpeed, gpsLatitude, gpsLongitude);
  Serial.println(info);
  Serial.println(gps.time.value());
  Serial.println(gpsTime);
  Serial.println(gps.speed.mph());
  Serial.println(gpsSpeed);
  Serial.println(gps.location.lat(), 6);
  Serial.println(gpsLatitude);
  Serial.println(gps.location.lng(), 6);
  Serial.println(gpsLongitude);
  return info;
}
