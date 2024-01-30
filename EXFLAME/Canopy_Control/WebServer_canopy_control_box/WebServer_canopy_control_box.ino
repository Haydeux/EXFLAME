
// Go to web address http://192.168.1.102:80 to connect
// Switches the LED on and off by pressing the button. Done through ethernet connection


#include <SPI.h>
#include <Ethernet.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //physical mac address
byte ip[] = { 192, 168, 1, 102 }; // ip in lan

EthernetServer server(80);; //server port

String readString; 

//////////////////////

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

  
  //start Ethernet
  Ethernet.begin(mac, ip);
  server.begin();

  //enable serial data print 
  Serial.begin(9600); 
  Serial.println("servertest1"); // so I can keep track of what is loaded
}

void loop(){
  // Create a client connection
  EthernetClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        //read char by char HTTP request
        if (readString.length() < 100) {

          //store characters to string 
          readString += c; 
          //Serial.print(c);
        } 

        //if HTTP request has ended
        if (c == '\n') {

          ///////////////
          Serial.println(readString);

          // now output HTML data header

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();

          client.println("<HTML>");
          client.println("<HEAD>");
          client.println("<TITLE>Arduino GET test page</TITLE>");
          client.println("</HEAD>");
          
          client.println("<BODY>");
          client.println("<H1>LED</H1>");

          client.println("<FORM ACTION=\"http://192.168.1.102:84\" method=get >");
          client.println("<INPUT TYPE=SUBMIT NAME=\"submit\" VALUE=\"on\">");
          client.println("</FORM>");

          client.println("<FORM ACTION=\"http://192.168.1.102:84\" method=get >");
          client.println("<INPUT TYPE=SUBMIT NAME=\"submit\" VALUE=\"off\">");
          client.println("</FORM>");

          client.println("");
          client.println("</BODY>");
          client.println("</HTML>");

          delay(1);
          //stopping client
          client.stop();

          /////////////////////
          if(readString.indexOf("on") >0)//checks for on
          {
            digitalWrite(LED, HIGH);
            Serial.println("Led On");
          }
          if(readString.indexOf("off") >0)//checks for off
          {
            digitalWrite(LED, LOW);
            Serial.println("Led Off");
          }
          //clearing string for next read
          readString="";

        }
      }
    }
  }
}
