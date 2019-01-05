// CONNECTIONS:
// DS1307 SDA --> SDA (D2)
// DS1307 SCL --> SCL (D1)
// DS1307 VCC --> 5v
// DS1307 GND --> GND

#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <time.h>
#include <Time.h>
#include <TimeLib.h>

char ssid[] = "";
char pass[] = "";
// char ssid[] = "FIXME-NAT";
// char pass[] = "hs1337_FIXME";
char HH = 1;
char MM = 0;

unsigned int localPort = 2390;

const char* ntpServerName = "pool.ntp.org"; // time.nist.gov
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP udp;
IPAddress timeServerIP;

const int relayPin1 = D7;
const int relayPin2 = D8;
int currentRelay = relayPin1;

tmElements_t arduinoTime;
tmElements_t ntpTime;
bool isArduinoTimeDefined = false;

void setup() {
  // We start by connecting to a WiFi network
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Connecting to ");
  Serial.print(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH);

  udp.begin(localPort);
  syncNtp();
}

void printTime(tmElements_t tm) {
  Serial.println(String(1970 + tm.Year) + "-" + tm.Month + "-" + tm.Day + ", " + tm.Hour + ":" + tm.Minute + ":" + tm.Second);
}

bool NtpUpdate() {
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no packet yet");
    return false;
  } else {
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900: ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time: ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);

    if (!isArduinoTimeDefined) {
      breakTime(epoch, arduinoTime);
      Serial.println("Waiting " + String(59 - arduinoTime.Second) + "s for the full minute");
      delay(1000 * (59 - arduinoTime.Second));
      epoch += (59 - arduinoTime.Second);
      breakTime(epoch, arduinoTime);
      isArduinoTimeDefined = true;
    }

    breakTime(epoch, ntpTime);

    return true;
  }
}

void syncNtp() {
  // Sync with NTP
  bool successful = NtpUpdate();

  if (successful) {
    Serial.print("NtpTime: ");
    printTime(ntpTime);
    Serial.print("ArduinoTime: ");
    printTime(arduinoTime);

    // synchronize with ticks or delays
    syncTimes();
  }
}

void syncTimes() {
  time_t arduinoEpoch = makeTime(arduinoTime);
  time_t ntpEpoch = makeTime(ntpTime);

  if (arduinoEpoch < ntpEpoch) {
    Serial.println("More ticks");
    // more ticks
    long diffMin = (ntpEpoch - arduinoEpoch) / 60.0;
    Serial.println("Time diff: " + String(diffMin));

    tick(diffMin);
    // Catch up minutes that we were missing
    breakTime(makeTime(arduinoTime) + (diffMin * 60), arduinoTime);
  } else if (arduinoEpoch > ntpEpoch) {
    Serial.println("Delay");
    Serial.println("Time diff: " + String(makeTime(arduinoTime) - makeTime(ntpTime)) + "s");
    // delay
    delay(1000 * (makeTime(arduinoTime) - makeTime(ntpTime)));
    breakTime(makeTime(ntpTime), arduinoTime);
  }
}

void loop() {
  long target = millis() + 60000L; // PROD
  breakTime(makeTime(arduinoTime) + 60, arduinoTime);

  syncNtp();

  tick();


  while (millis() < target) {
    delay(1);
  }
}

//  send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress & address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;   // Stratum, or type of clock
  packetBuffer[2] = 6;   // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:

  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void tick() {
  changeMinute();
}

void tick(long diffMin) {
  // Avoid spamming the clock in the event of a super long disconnection
  if (diffMin > 1000) {
    diffMin = 1;
  }

  for (long i = 0; i < diffMin; i++) {
    changeMinute();
  }
}

void changeMinute() {
  if (currentRelay == relayPin1) {
    currentRelay = relayPin2;
    Serial.println("Tick!");
  } else {
    currentRelay = relayPin1;
    Serial.println("Tack!");
  }

  digitalWrite(currentRelay, LOW);
  delay(1000);
  digitalWrite(currentRelay, HIGH);
  delay(2000);
}
