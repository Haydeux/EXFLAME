
// Begins an LAN connection on the arduino. Ping at 192.168.1.67

#include <SPI.h>
#include <Ethernet.h>

// Network configuration
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte ip[] = {192,168,1,67};

#define POT_1 A0_5
#define POT_2 A0_6

#define LED Q1_2
#define RELAY Q1_0

void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);

  pinMode(LED,OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(POT_1, OUTPUT);
  pinMode(POT_2, OUTPUT);
  analogWrite(POT_1, 127);
  analogWrite(POT_2, 127);
  
    Serial.begin(9600);
    Serial.println("Beginning ethernet");

    Ethernet.begin(mac, ip);
}

void loop() {
  
}
