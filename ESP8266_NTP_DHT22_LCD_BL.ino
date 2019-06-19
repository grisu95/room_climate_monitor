#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Define I2C pins
LiquidCrystal_I2C lcd(0x3F,20,4);
#define DHTPIN 12     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

const char ssid[] = "zwickau.freifunk.net";  //  your network SSID (name)
const char pass[] = "";       // your network password

// NTP Servers:
//static const char ntpServerName[] = "10.23.1.51";   //NTP Server Arbeit
static const char ntpServerName[] = "de.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = 2;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void printLCD(int digits);
void sendNTPpacket(IPAddress &address);

const byte BUTTON=D7; // our button pin
const byte LED=D4;   // LED (built-in on Uno)
 
unsigned long buttonPushedMillis;  // when button was released
unsigned long ledTurnedOnAt;  // when led was turned on
unsigned long turnOnDelay = 100; // wait to turn on LED
unsigned long turnOffDelay = 5000; // turn off LED after this time
bool ledReady = false; // flag for when button is let go
bool ledState = false; // for LED is on or not.


void setup() {
  // put your setup code here, to run once:

 lcd.begin();                      // initialize the lcd 
  //lcd.backlight();
  lcd.clear();

  pinMode(BUTTON,INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
Serial.begin(112500);
  // Initialize device.
 Serial.print("Connecting to ");
 lcd.setCursor(0,0);
 lcd.print("Connecting to ");
  Serial.println(ssid);
  lcd.setCursor(0,1);
  lcd.print(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());
  lcd.setCursor(0,2);
  lcd.print("IP: ");
  lcd.print(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  lcd.setCursor(0,3);
  lcd.print("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);

  delay(2000);
  lcd.clear();
  
  dht.begin();
 
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("*C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("*C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("*C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
//  delayMS = sensor.min_delay / 1000;



}

time_t prevDisplay = 0; // when the digital clock was displayed


void loop() {
  // put your main code here, to run repeatedly:
   if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
      digitalClockDisplay();
    }
  }
 // read the state of the pushbutton value:
  unsigned long currentMillis = millis();
 
  // check the button
  if (digitalRead(BUTTON) == LOW) {
   // update the time when button was pushed
   buttonPushedMillis = currentMillis;
   //ledReady = true;
  }
 
  // make sure this code isn't checked until after button has been let go
  //if (ledReady) {
    //this is typical millis code here:
    if ((unsigned long)(currentMillis - buttonPushedMillis) >= turnOnDelay) {
       // okay, enough time has passed since the button was let go.
       digitalWrite(LED, HIGH);
       //Serial.print("AN");
       // setup our next "state"
       ledState = true;
       // save when the LED turned on
       ledTurnedOnAt = currentMillis;
       // wait for next button press
       ledReady = false;
    }
  
 
  // see if we are watching for the time to turn off LED
  //if (ledState) {
    // okay, led on, check for now long
    if ((unsigned long)(currentMillis - ledTurnedOnAt) >= turnOffDelay) {
      ledState = false;
      digitalWrite(LED, LOW);
    }
  
  
}
void digitalClockDisplay()
{
  
  lcd.setCursor(0,0);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {

    Serial.print(F("Temperature: "));
    lcd.print(F("Temperature: "));
    Serial.print(event.temperature);
    lcd.print(event.temperature);
    Serial.println(F("'C"));
    lcd.print(F("'C"));
 
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    lcd.setCursor(0,1);
    lcd.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    lcd.print(event.relative_humidity);
    Serial.println(F("%"));
    lcd.print(F("%"));
  }
  
  // digital clock display of the time
  Serial.print(hour());
  lcd.setCursor(0,2);
  printLCD(hour());
  printDigits(minute());
  lcd.print(":");
  printLCD(minute());
  printDigits(second());
  lcd.print(":");
  printLCD(second());
  Serial.print(" ");
  lcd.print(" ");
  Serial.print(day());
  printLCD(day());
  Serial.print(".");
  lcd.print(".");
  Serial.print(month());
  printLCD(month());
  Serial.print(".");
  lcd.print(".");
  Serial.print(year());
  lcd.print(year());
  Serial.println();


}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
   Serial.print('0');
  Serial.print(digits);
}

void printLCD(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  if (digits < 10)
    lcd.print('0');
  lcd.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
