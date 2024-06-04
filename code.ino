#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include "Arduino.h"
#include "PCF8574.h"
#include <EthernetSPI2.h>


//Define Ethernet connection in SPI2

// ESP32-DEV  -> W5500
// GPIO12     -> MISO
// GPIO13     -> MOSI
// GPIO14     -> SCLK
//  GPIO2      -> SCS
// GND        -> GND
 
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x02, 0xAB, 0xCD, 0xEF, 0x00, 0x01
};

IPAddress ip(192, 168, 100, 211);
#define LED 33
#define FLED 34


const int sMachine = P0; //Main Machine  
const int sPause = P1; //Pause 
const int sCounterL = P2;  // Counter Left
const int sCounterR = P3;  // Counter Right
const int sQuality = P4;  // Problem1
const int sMaintenance = P5;  
const int sProblem = P6;  // the number of the pushbutton pin
const int sOption = P7;  // 
const int sWOL=4;
const int sWOR=25;

// variable for storing the Input Signal status 
int stMachine = 0;
int stPause = 0;
int stCounterL = 0;
int stCounterR = 0;
int stMaintenance = 0;
int stQuality = 0;
int stProblem = 0;
int stOption = 0;
int stWOL=0;
int stWOR=0;

#define DEBOUNCE_TIME  50 // the debounce time in millisecond, increase this time if it still chatters

int lastSteadyState = LOW;       // the previous steady state from the input pin
int lastFlickerableState = LOW;  // the previous flickerable state from the input pin
unsigned long lastDebounceTime = 0; 

int lastSteadyStateR = LOW;       // the previous steady state from the input pin
int lastFlickerableStateR = LOW;  // the previous flickerable state from the input pin
unsigned long lastDebounceTimeR = 0; 


//Define RFID connection

#define RST_PIN         26           
#define SS_PIN          5          
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance
// Set i2c address
PCF8574 pcf8574(0x20);
unsigned long timeElapsed;


bool latch;


// Replace with your network credentials
// const char* ssid = "AFD-W30";
// const char* password = "afdhome1";
const char* ssid = "METINDO-DOJO";
const char* password = "Metindo@Dojo";
long now = millis();

const char* mqtt_server = "192.168.3.11";
//const char* mqtt_server = "192.168.100.12";


String oprname="";

EthernetClient ethClient; //connection with Ethernet
WiFiClient espClient;    //connection with WiFi
PubSubClient client(ethClient);

const char* topic = "Machine1"; //publish topic

// This functions connects your ESP to your router

long cntL=0;
long cntR=0;

void setup_wifi() {
  delay(10);
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - IP address: ");
  Serial.println(WiFi.localIP());
}


// This functions reconnects  to your MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("MQTTClient")) {
      Serial.println("connected");       
       client.subscribe("machine1/stWOL");
       client.subscribe("machine1/stWOR");
    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 2 seconds");
        delay(2000);
    }
  }
}

//
void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

 // If a message to subscribed is received 
  if(topic=="machine1/stWOL"){
      Serial.print("WO Status L : ");
      if(messageTemp == "on"){
       digitalWrite(sWOL, LOW);
       Serial.println("ON");
       stWOL=1;
       cntL=0;
      }
      else if(messageTemp == "off"){
        digitalWrite(sWOL, HIGH);
        Serial.println("Off");
        stWOL=0;
      }    
  }

// If a message to subscribed is received 
  if(topic=="machine1/stWOR"){
      Serial.print("WO Status R : ");
      if(messageTemp == "on"){
       digitalWrite(sWOR, LOW);
       Serial.println("ON");
       stWOR=1;
       cntR=0;
      }
      else if(messageTemp == "off"){
        digitalWrite(sWOR, HIGH);
        Serial.println("Off");
        stWOR=0;
      }    
  }

  
}


void setup_eth() {
  Ethernet.init(2);
  //Ethernet.begin(mac, ip); // start the Ethernet connection and the server:
  Ethernet.begin(mac); 

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(200);
      digitalWrite(LED, HIGH);
      digitalWrite(FLED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      digitalWrite(FLED, LOW);
    }
  }
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is NOT connected.");
      delay(500);
      digitalWrite(LED, HIGH);
      digitalWrite(FLED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      digitalWrite(FLED, LOW);    
  }

  Serial.println("Ethernet cable is now connected.");
  
  delay(500);
  Serial.println(Ethernet.localIP());
}



void setup() {
  Serial.begin(115200);
  pcf8574.pinMode(sMachine, INPUT);
  pcf8574.pinMode(sPause, INPUT);
  pcf8574.pinMode(sCounterL, INPUT);
  pcf8574.pinMode(sCounterR, INPUT);
  pcf8574.pinMode(sQuality, INPUT);
  pcf8574.pinMode(sMaintenance, INPUT);
  pcf8574.pinMode(sProblem, INPUT);
  pcf8574.pinMode(sOption, INPUT);
  Serial.print("Init pcf8574...");
  
  pinMode(sWOL,OUTPUT);
  digitalWrite(sWOL, HIGH);

  pinMode(sWOR,OUTPUT);
  digitalWrite(sWOR, HIGH);

  if (pcf8574.begin()){
    Serial.println("OK");
  }else{
    Serial.println("KO");
  }
  
//  setup_wifi();
  
  setup_eth();
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
}


void loop() {

  static unsigned long lastpub=millis();

//Connection to MQTT Broker

  if (!client.connected()) {
    reconnect();
  }

  if(!client.loop())
    client.connect("MQTTClient");

  uint8_t val0 = pcf8574.digitalRead(sMachine);            // Read the value of pin P0        
  uint8_t val1 = pcf8574.digitalRead(sPause);
  uint8_t val2 = pcf8574.digitalRead(sCounterL);
  uint8_t val3 = pcf8574.digitalRead(sCounterR);
  uint8_t val4 = pcf8574.digitalRead(sQuality);            // Read the value of pin P0        
  uint8_t val5 = pcf8574.digitalRead(sMaintenance);
  uint8_t val6 = pcf8574.digitalRead(sProblem);
  uint8_t val7 = pcf8574.digitalRead(sOption);


  if (val0 == LOW)  {
    stMachine=1;
 //   Serial.println("Machine ON");
    
  }
  else {
    stMachine=0; 
 //   Serial.println("Machine OFF");
    cntL=0;
    cntR=0;
  }

  if (val1 == LOW) {
    stPause=1;
 //   Serial.println("Pause OFF"); 
  }  
  else
  { 
 //   Serial.println("Pause ON");
    stPause=0;
  }
  
  if (val2 != lastFlickerableState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the the last flickerable state
    lastFlickerableState = val2;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
    // if the button state has changed:
    if(lastSteadyState == HIGH && val2 == LOW) {
  //     Serial.println("Counter Left ON");
       stCounterL=1;
       cntL=cntL+1;
       Serial.print("Counter Left : ");
       Serial.println(cntL);
 
    }
    else if(lastSteadyState == LOW && val2 == HIGH) {
  //    Serial.println("Counter Left OFF");
      stCounterL=0;
    }
    // save the the last steady state
    lastSteadyState = val2;
    stCounterL=val2;
  }

  if (val3 != lastFlickerableStateR) {
    // reset the debouncing timer
    lastDebounceTimeR = millis();
    // save the the last flickerable state
    lastFlickerableStateR = val3;
  }

  if ((millis() - lastDebounceTimeR) > DEBOUNCE_TIME) {
    // if the button state has changed:
    if(lastSteadyStateR == HIGH && val3 == LOW) {
       cntR=cntR+1;
       
       Serial.print("Counter Right : ");
       Serial.println(cntR);
       stCounterR=1;
  
    }
    else if(lastSteadyStateR == LOW && val3 == HIGH) {
  //    Serial.println("Counter Left OFF");
      stCounterR=0;
    }
    // save the the last steady state
    lastSteadyStateR = val3;
    stCounterR=val3;
  }
  
  
  if (val4 == LOW) {
 //   Serial.println("Quality Check On");
    stQuality=1;
  }
  else { 
 //   Serial.println("Quality Check Off");
    stQuality=0;
  }
  if (val5 == LOW){  
    stMaintenance=1;
 //   Serial.println("Maintenance ON");
  }
  else { 
 //   Serial.println("Maintenance OFF");
    stMaintenance=0;
  }
   if (val6 == LOW) {  
//    Serial.println("PROBLEM ON");
    stProblem=1;
   }
  else {
//    Serial.println("No Problem");
    stProblem=0;
  }
  if (val7 == LOW) {  
//    Serial.println("Option On");
    stOption=1;
  }
  else { 
//    Serial.println("Option Off");
    stOption=0;
  }
//  Serial.println(" ");
  
  PCF8574::DigitalInput di = pcf8574.digitalReadAll();
//  Serial.print("READ VALUE FROM PCF P1: ");
 // Serial.print(di.p0);
//  Serial.print(" - ");
//  Serial.print(di.p1);
 // Serial.print(" - ");
 // Serial.print(di.p2);
//  Serial.print(" - ");
//  Serial.print(di.p3);
//  Serial.print(" - ");
//  Serial.print(di.p4);
//  Serial.print(" - ");
//  Serial.print(di.p5);
//  Serial.print(" - ");
//  Serial.print(di.p6);
//  Serial.print(" - ");
//  Serial.println(di.p7);
  

  if (millis() - lastpub > 1000) {
      lastpub=millis();
    //publish to MQTT 
    client.publish("machine1/sMachine", String(stMachine).c_str());
    client.publish("machine1/sPause", String(stPause).c_str());
    client.publish("machine1/sCounterL", String(stCounterL).c_str());
    client.publish("machine1/sCounterR", String(stCounterR).c_str());
    client.publish("machine1/sMaintenance", String(stMaintenance).c_str());
    client.publish("machine1/sQuality", String(stQuality).c_str());
    client.publish("machine1/sProblem", String(stProblem).c_str());
    client.publish("machine1/sOption", String(stOption).c_str());
    client.publish("machine1/counterL", String(cntL).c_str());        
    client.publish("machine1/counterR", String(cntR).c_str());     
    Serial.print("Counter : ");
    Serial.print(cntL);
    Serial.print(" - ");
    Serial.println(cntR);
  }      
  delay(50);
              
}


void readRFID() {
// Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  //some variables we need
  byte block;
  byte len;
  MFRC522::StatusCode status;

  //-------------------------------------------

  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  Serial.println(F("**Card Detected:**"));

  //-------------------------------------------

  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); //dump some details about the card

  //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));      //uncomment this to see all blocks in hex

  //-------------------------------------------

  Serial.print(F("Name: "));

  byte buffer1[18];

  block = 4;
  len = 18;

  //------------------------------------------- GET FIRST NAME
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  //PRINT FIRST NAME
  for (uint8_t i = 0; i < 16; i++)
  {
    if (buffer1[i] != 32)
    {
      Serial.write(buffer1[i]);
    }
  }
  Serial.print(" ");
  oprname = (char*)buffer1;
  //---------------------------------------- GET LAST NAME

  byte buffer2[18];
  block = 1;

  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(mfrc522.uid)); //line 834
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  status = mfrc522.MIFARE_Read(block, buffer2, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }

  //PRINT LAST NAME
  for (uint8_t i = 0; i < 16; i++) {
    Serial.write(buffer2[i] );
  }


  //----------------------------------------

  Serial.println(F("\n**End Reading**\n"));

  delay(1000); //change value if you want to read cards faster

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}
