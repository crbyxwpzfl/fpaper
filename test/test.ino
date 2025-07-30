
// to implement slash fix
// implement mqtt wildcard see https://github.com/theelims/PsychicMqttClient/issues/6#issuecomment-2530154457
// primary peer one click
// duble click to all peers
// set custom positions for each peer
// 
// ditch idle pulse do manual connection check
// 
// clarify commands in help









//  switched to arduinocli since platformio does not really support arduino core v3 eventually I really want to switch to idf and ditch arduino in the long term
//  `arduino-cli sketch new all-together-arduino`    init sketch
//  'arduino-cli core update-index'    fetches latest core index
//  'arduino-cli board search Adafruit Feather esp32s3'    find board
//  'arduino-cli core install esp32:esp32'    install matching core see Platform ID
//  'arduino-cli lib install -v --git-url 'url' '    install library from git set environment variable 'export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true' use verbos to find liberary dir for edits
//  'arduino-cli compile -v --fqbn esp32:esp32:adafruit_feather_esp32s3 --build-path ./firmware -upload -p /dev/tty.usbmodem101 '    compile for fqbn form board search esp32:esp32:adafruit_feather_esp32s3 also puts binaries into firmware folder
//  use 'merged.bin' at adress 0x0 with https://espressif.github.io/esptool-js/ for web programming



#include <Arduino.h>    // all this is arduino for an esp32    so compared to c some delacrations are missing but im not sure 
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <HTTPUpdate.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <WiFi.h>

#include <AsyncTCP.h>    //  https://github.com/mathieucarbou/AsyncTCP.git MOVED to https://github.com/ESP32Async/AsyncTCP.git
#include <ESPAsyncWebServer.h>     //  https://github.com/mathieucarbou/ESPAsyncWebServer.git  MOVED to https://github.com/ESP32Async/ESPAsyncWebServer.git
#include <MycilaWebSerial.h>  // https://github.com/mathieucarbou/MycilaWebSerial.git customize portal 'cd library/dirs/MycilaWebSerial/portal/' customize html delete 'library/dirs/MycilaWebSerial/src/MycilaWebSerialPage.h' regenerate it 'pnpm i' 'pnpm build' node and pnpm is required
#include "InterruptButton.h"    //  https://github.com/rwmingis/InterruptButton.git
#include <PsychicMqttClient.h>    //  https://github.com/theelims/PsychicMqttClient.git


#include <ChaChaPoly.h>    // https://github.com/rweather/arduinolibs.git
#include <SHA256.h>
#include <HKDF.h>


#include <GxEPD2_BW.h>  //  https://github.com/ZinggJM/GxEPD2.git + https://github.com/adafruit/Adafruit-GFX-Library.git + https://github.com/adafruit/Adafruit_BusIO.git for epaper GDEY042T81 4.2" b/w 400x300, SSD1683 on elecorw CrowPanel ESP32 E-Paper HMI 4.2-inch Display
#include <xpwallpaper.h>  //  test image bitmap
GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(/*CS=D8*/ 45, /*DC=D3*/ 46, /*RST=D4*/ 47, /*BUSY=D2*/ 48));
static uint8_t volatileShowBuff[15000];  // global show buffer no malloc/free necessary images are of static size
static uint8_t curriv[12];  // global iv buffer for chachapoly encryption and to ignore own message echos




// --- TODO solwley get rid of these these just was for testing
//Preferences prefs;    //  commented so no redfinition error
#define IMAGE_WIDTH 400
#define IMAGE_HEIGHT 300
#define IMAGE_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT / 8) // 1-bit per pixel

uint8_t* imageBuffer = nullptr; // Pointer to store the uploaded image
size_t imageBufferOffset = 0;   // Offset to track the current position in the buffer

uint8_t* receivedImageBuffer = nullptr; // Pointer to store received image data
size_t receivedImageSize = 0;           // Size of received image data
  







Preferences prefs;    //  first declaration of preferences as perfs
TaskHandle_t showTasHandle;
QueueHandle_t sendmqttQueue;    //  handle for mqtt message queue see task belowus
QueueHandle_t showQueue;    //  handle for servo queue
//static uint8_t volatileShowBuff[15000];  // global show buffer no malloc/free necessary images are of static size
void showTas(void *parameter) {    //  this handles servo movement
  showQueue = xQueueCreate(5, 15);    // create queue with buffer of 5 with length of nvsalias so 15 chars
 
  char buff[] = "nvsalias length";    //  buffer to read from queue

  pinMode(7, OUTPUT); digitalWrite(7, HIGH);   //  give power to the panel
  display.init(115200);    // init epd with 115200 baud rate
  display.setRotation(0);    //  TODO make this a setting in preferences but also change selection/ditthered overlay aspect accordingly



  //String *peerArray = prefs.getString("peers", "local").split(' ');    //  splits peers into array attentione this is unsafe and for changed peers there hast to be a restart
  //uint8_t peerindex = 0, totalpeers = prefs.getString("peers", "local").split(' ').size();    //  this leaks memory clean this up man perhpas with peerString.substring(peerString.indexOf('local')+1, peerString.indexOf(' ', peerString.indexOf('local')+1) )
  
  //String* tempArray = peerString.split(' ');
  //uint8_t totalpeers = peerString.split(' ').size(); // This creates another array!
  
  // Copy to fixed array
  //String peerArray[10];
  //for (int i = 0; i < min(totalpeers, 10); i++) {
  //  peerArray[i] = tempArray[i];
  //}
  
  //delete[] tempArray; // Clean up


  String peerString = prefs.getString("peers", "local");
  uint8_t sendScreen = 0;    //  homescreen is default otherwise sendScreen
  String currpeer = "local";    //  start locally

  while(true){
    if(!xQueueIsQueueEmptyFromISR( showQueue )){    //  just do sth when queue not empty
      xQueueReceive(showQueue, &buff, 0);

      if (!sendScreen){     //  while in send screen dont let others show stuff directly
        prefs.getBytes(buff, volatileShowBuff, 15000);    //  for invalid nvs lookups this rturns null and leaves volatileShowBuff so to directly show volatileShowBuff here as per convetion just queue 'showVolatile'
      }

      if (!strcmp(buff, "sendScreen")) {    //  here the volatileShowBuff already was preped inside upload so nothing to do here
        sendScreen = 1;
      }

      if (!strcmp(buff, "homeScreen")) {    //  this is the home screen so show latest photo of first peer
        currpeer = "local";    //  reset current peer to local 
        sendScreen = 0;
        if (!prefs.getBytes((currpeer + "L").c_str(), volatileShowBuff, 15000)) Serial.println("nothing found for " + currpeer + "L");    //  read foto from nvs
      }

      if (!strcmp(buff, "proceede")) {    //  show 'P'rofile picture of next peer and perhaps 'L'atest foto of peer
        //if (peerString.indexOf(' ', peerString.indexOf(currpeer)+currpeer.length()+1) == -1)
        //if (peerString.indexOf(' ', peerString.indexOf(currpeer)+currpeer.length()+1) != -1)
        if (peerString.indexOf(currpeer)+currpeer.length()+1+1 > peerString.length()) currpeer = "local";   //  account for trailing space here test for last peer
        else currpeer = peerString.substring(peerString.indexOf(currpeer)+currpeer.length()+1, peerString.indexOf(' ', peerString.indexOf(currpeer)+currpeer.length()+1) );


        Serial.println("got proceede sendscreen is " + String(sendScreen) + " try to find " + currpeer + "P in nvs");

        if (!prefs.getBytes((currpeer + "P").c_str(), volatileShowBuff, 15000)) Serial.println("nothing found for " + currpeer + "P");    //  TODO print to feedlog here !!   prep 'P'rofile picture of peer
        if (!sendScreen) xQueueSend(showQueue, (currpeer + "L").c_str(), 0);    //  prep 'L'atest foto of peer
      }

      if(!strcmp(buff, "send")) {    //  kickoff send of volatileShowBuff or ping current peer
        if ( sendScreen) xQueueSend(sendmqttQueue, ("sendf " + currpeer).c_str(), 0); xQueueSend(showQueue, "homeScreen", 0);    //  send foto to peer
        if (!sendScreen) xQueueSend(sendmqttQueue, ("annoy " + currpeer).c_str(), 0); xQueueSend(showQueue, "homeScreen", 0);    //  just ping peer
      }

      display.setFullWindow();    
      display.firstPage();
      do {
        display.fillScreen(GxEPD_BLACK);
        display.drawBitmap(0, 0, volatileShowBuff, display.width(), display.height(), GxEPD_WHITE);
      } while (display.nextPage());

      display.hibernate();   //  hibernate display to save power

    }
    vTaskDelay(1000);    //  no flicker just show every seconds
  }
}


TaskHandle_t ledTasHandle;
QueueHandle_t ledQueue, ackQueue;
void ledTas(void *parameter) {    //  this handles led user feedback
  
  /* no led with new board 
  Adafruit_NeoPixel neoled(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800); neoled.begin(); neoled.clear();    //  init led and clear buffer
  */
  struct ledfeedback{ int r; int g; int b; int t; }; ledQueue = xQueueCreate(5, sizeof(ledfeedback));    // create queue passing the struct with rgb and time with a buffer of 5 and
  ackQueue = xQueueCreate(1, sizeof(int)); int initack = 0; xQueueOverwrite(ackQueue, &initack);    // create queue with buffer of 1 to keep track of open acks
  int idlepulse = 0, allack = 0, openacks; unsigned long lastpulse = millis();    //  idlepulse for pulse led when nothing to do either white so no acks pending or green all acks closed and lastpulse for interval of idelpulse


  while(true){
    if(!xQueueIsQueueEmptyFromISR( ledQueue )) {    //  just do sth when queue not empty
      ledfeedback ledc; xQueueReceive(ledQueue, &ledc, 0);    //  reed rgbt into struct also this clears one buffer of queue
      
      /* no led with new board
      neoled.fill(neoled.Color(ledc.r, ledc.g, ledc.b)); neoled.show(); vTaskDelay(ledc.t); neoled.clear(); neoled.show();    //  show color for t ms
      */

    } else {        //  idel pulse led while nothing to do
      xQueuePeek(ackQueue, &openacks, 0);    //  this looks for open acks also leaves value in queueand
      
      // idel pulse disabled for now its just anoying
      
      // if (millis() - lastpulse > 60000) { idlepulse++; neoled.fill(neoled.Color(idlepulse * (openacks != 0), idlepulse, idlepulse * (openacks != 0))); neoled.show(); if(idlepulse == 40) { idlepulse = -40; lastpulse = millis(); }; vTaskDelay(30);}    //  this inits green pulse when no openacks and white pulse when still open acks every 60s 
      // if (idlepulse < 0) { idlepulse++; neoled.fill(neoled.Color(-idlepulse * (openacks != 0), -idlepulse, -idlepulse * (openacks != 0))); neoled.show(); vTaskDelay(40);}    //  this stops pulse when idlepulse reaches 0
    }
    vTaskDelay(1);
  }
}


WebSerial WebSerial;  //  first delclartion of webserial not static anymore since v8.0.0
//Preferences prefs;    //  commented so no redfinition error
void feedlog(String text, int r = 0, int g = 0, int b = 0, int t = 0, String level = "info") {    //  print to serial and webserial and forward led feedback to ledTas
  if (r != 0 || g != 0 || b != 0 || t != 0) { struct ledfeedback{ int r; int g; int b; int t; }; } // no led feedback! for now ledfeedback ledc = {r, g, b, t}; xQueueSend(ledQueue, &ledc, 0); }    //  send led color to queue
  if (prefs.getString("debuglevel", "info") == level || level == "info" ) { 
    Serial.print(text);    // TODO add \r\n here so each line is printed correctly
    WebSerial.print(text.c_str()); 
  }    //  always print info and just debug when debug level
}


//Preferences prefs;    //  commented so no redfinition error
TaskHandle_t servoTasHandle;
QueueHandle_t servoQueue;    //  handle for servo queue
void servoTas(void *parameter) {    //  this handles servo movement
  servoQueue = xQueueCreate(5, sizeof("sit"));    // create queue with buffer of 5 
  ledcAttach(38, 50, 12);    //  50hz pwm at pin 38 with 12 bit resolution so 0-4095
  char buf[] = "sit";    //  why does char buf[4] error help
  
  while(true){
    if(!xQueueIsQueueEmptyFromISR( servoQueue )){
      xQueueReceive(servoQueue, &buf, 0);    //  just do sth when queue not empty
      //ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); String(buf) == "top" ? ledcWrite(38, prefs.getInt("top", 0)) : ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0);    //  move servo to poses in preferences also cool c ternary operator
      if (String(buf) == "top") { ledcWrite(38, prefs.getInt("top", 0)); vTaskDelay(500); ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }   //  wigle servo to poses in preferences always top and back to sit pose
      if (String(buf) == "sit") { ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }  // move servo to sit pose 
    }
    vTaskDelay(1);
  }
}


//Preferences prefs;    //  commented so no redfinition error
ChaChaPoly chachapoly;
PsychicMqttClient mqttClient;    //  first declaration of mqttClient
TaskHandle_t sendmqttHandle;
//QueueHandle_t sendmqttQueue;    //  comented so no redfinition error
void sendmqttTas(void *parameter) {    //  this handles outgoing mqtt messages
  sendmqttQueue = xQueueCreate( 5, sizeof("  long ass peer name  says look here  ") );    // create queue with buffer of 5
  char buff[] = "  long ass peer name  says look here  ";    //  this hard coded finite length stresses me in python me no have to worry me miss python
  
  uint8_t hkdf[32];
  uint8_t cyphy[15000];    //  for encrypted bytes
  uint8_t tag[16];

  while (true) {
  if(!xQueueIsQueueEmptyFromISR( sendmqttQueue )){    //  just do sth when queue not empty
    xQueueReceive(sendmqttQueue, &buff, 0);    //  reads first word out of queue

    if (strcmp(buff + 6, "local") == 0) {
      prefs.putBytes("localL", volatileShowBuff, sizeof(volatileShowBuff));    //  just save to local

      Serial.println("saved image to localL and going back to homescreen");    //  TODO make this a feedlog

      continue;    //  jump back to while and dont process further
    }

    uint8_t *payload = (uint8_t*)malloc(sizeof(curriv) + sizeof(tag) + sizeof(cyphy));    //  allocate memory for payload and build payload
    uint32_t payloadlen = 12 + 16 + 15000;    //  assume full message iv + tag + cyphy

    if (strncmp(buff, "annoy ", 6) == 0) {
      esp_fill_random(curriv, sizeof(curriv));    //  fill curriv with noise here this only is to later in recieve mqtt determine wether message is a echo

      memcpy(payload, curriv, sizeof(curriv));    //  first iv
      memcpy(payload + sizeof(curriv), reinterpret_cast<const uint8_t*>("look here"), 9);    //  just add 'look here' to payload
      
      payloadlen = 12 + 9;    //  length of cypher 9 bytes + length of iv 12 bytes
      
      Serial.println("packed look here to payload try sending now to " + String(buff + 6));
    }

    if (strncmp(buff, "sendf ", 6) == 0) {
      esp_fill_random(curriv, sizeof(curriv));    //  fill curriv with noise
      prefs.getBytes((String(buff + 6) + "H").c_str(), hkdf, 32);
      
      chachapoly.setIV(curriv, 12);
      chachapoly.setKey(hkdf, 32);
      // this is optional right chachapoly.addAuthData(authdata, 12);    //  add auth data to chachapoly
      chachapoly.encrypt(cyphy, volatileShowBuff, 15000);    //  encrypt current foto

      chachapoly.computeTag(tag, 16);
      chachapoly.clear();

      memcpy(payload, curriv, sizeof(curriv));    //  first iv
      memcpy(payload + sizeof(curriv), tag, sizeof(tag));    //  then tag
      memcpy(payload + sizeof(curriv) + sizeof(tag), cyphy, sizeof(cyphy));    //  last data
      
      Serial.println("packed image to payload try sending now to " + String(buff + 6));
    }

    bool result = mqttClient.publish( (prefs.getString("mqtop", "/fpaper/") + String(buff + 6)).c_str() , 0, 0, reinterpret_cast<const char*>(payload), payloadlen, true);

    Serial.printf("Publish result %s\n", result ? "success" : "failed");    //  TODO make this a feedlog message
  
    free(payload);
    
    xQueueSend(showQueue, "homeScreen", 0);


    /*  this was test stuff 
    Serial.println("try to publ");
    bool result1 = mqttClient.publish("/fpaper/test", 0, 0, "Hello World!");
    Serial.printf("Publish result hello world: ", result1 ? "success" : "failed");

    Serial.println(buf);
    if (strcmp(buf, "test") == 0) {  // TODO for testing this should only send when publ test
      // TODO only publish to our topic
      if (imageBuffer && imageBufferOffset > 0) {
        
        Serial.printf("imageBufferOffset: %zu\n", imageBufferOffset);
        Serial.printf("First few bytes: %02X %02X %02X %02X\n", imageBuffer[0], imageBuffer[1], imageBuffer[2], imageBuffer[3]);
      
    
        //bool result = mqttClient.publish("/fpaper/test", 0, false, (const char*)imageBuffer, imageBufferOffset, false); // this crashes or the function returns false so sending did fail
        int size = prefs.getInt("top", 10);    //  get size from preferences or default to 10
        Serial.println(size);
        size_t testSize = min(size, (int)imageBufferOffset);
        bool result = mqttClient.publish("/fpaper/test", 0, 0, reinterpret_cast<const char*>(imageBuffer), testSize, true);
        Serial.printf("Publish result (first %zu bytes): %s\n", testSize, result ? "success" : "failed");

        //mqttClient.publish("/fpaper/test", 0, false, (const char*)imageBuffer, imageBufferOffset, true);
        Serial.println("did it");
      } else {
        Serial.println("eeeeeeee");
      }
    }
  */



  }
  vTaskDelay(1000);    // just send every second
  }
}


//Preferences prefs;    //  commented so no redfinition error
//PsychicMqttClient mqttClient;    //  commented so no redfinition error
void initmqtt(){    //  handle incoming mqtt
  String serverAddress = prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work
  
  String peerString = prefs.getString("peers", "local");    //  get peers from preferences

  mqttClient.onTopic( prefs.getString("mqtop", "/fpaper/+").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    // wildcards should work here listen level deep for now TODO change this to only subscribe to peers
    if (peerString.indexOf(String(topic).substring(8)) == -1) return;    //  this is dangars this matches partial string too topic still contains /fpaper/ so strip it see weather we want to listen to peer message 
    if (memcmp(curriv, payload, 12)) return;    //  when message was our own message ignore it

      
      
      // decode message here topic minus /fpaper/ is the nvsalias so to get correct key use getBytes() with topic.substring(8)+'H' this gives hkdf of corosponding peer
    
  });



  mqttClient.onTopic( prefs.getString("mqtop", "/fpaper/+").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    // wildcards should work here so listen to everything on level deep 
       Serial.printf("Received message on topic: %s\n", topic);

            // Check if this is binary image data (not text messages)
      if (strstr(topic, "/fpaper/test") != NULL) {
        // This is binary image data
        Serial.printf("Received binary data on topic: %s\n", topic);
        
        // For binary data, we need to determine actual size differently
        // Since strlen() stops at null bytes, let's assume 15KB for now
        const size_t EXPECTED_IMAGE_SIZE = 15000; // 15KB
        
        // Print first few bytes to serial console
        Serial.print("First bytes (hex): ");
        for (int i = 0; i < min(16, (int)strlen(payload)); i++) {
          Serial.printf("%02X ", (uint8_t)payload[i]);
        }
        Serial.println();
        
        // Allocate PSRAM buffer for received image
        if (receivedImageBuffer) {
          free(receivedImageBuffer); // Free previous buffer
        }
        
        //size_t payloadSize = strlen(payload); // For binary data, you might need a different approach
        //receivedImageBuffer = (uint8_t*)ps_malloc(payloadSize);
        receivedImageBuffer = (uint8_t*)ps_malloc(EXPECTED_IMAGE_SIZE);
        
        if (receivedImageBuffer) {
          // Copy binary data to PSRAM buffer
          //memcpy(receivedImageBuffer, payload, payloadSize);
          //receivedImageSize = payloadSize;

          memcpy(receivedImageBuffer, payload, EXPECTED_IMAGE_SIZE);
          receivedImageSize = EXPECTED_IMAGE_SIZE;
          
          Serial.printf("Stored %zu bytes in PSRAM buffer\n", receivedImageSize);
          
          // Optional: Display received image on e-paper
          if (receivedImageSize >= IMAGE_SIZE) {
            display.setFullWindow();
            display.firstPage();
            do {
              display.fillScreen(GxEPD_BLACK);
              display.drawBitmap(0, 0, receivedImageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT, GxEPD_WHITE);
            } while (display.nextPage());
            display.hibernate();
            Serial.println("Received image displayed on e-paper");
          }
        } else {
          Serial.println("Failed to allocate PSRAM for received image");
        }
        return; // Skip text message processing
      }

      if (strstr(payload, " says look here") != NULL) {
        String sender = String(payload).substring(0, strstr(payload, " says look here") - payload), peers = prefs.getString("peers", "");    //  peer always is before ' says' so subtracting the base pointer of buf pointer gives length of substring containing peer
        if ( ( peers.indexOf(sender) >= 0 || peers == "" ) && sender != prefs.getString("publ", String(ESP.getEfuseMac())) ) { xQueueSend(servoQueue, "top", 0); xQueueSend(sendmqttQueue, "shit", 0); }    //  when no peers set this moves servo with any sender but our self so we dont finger our self. alternative 'if (strstr(prefs.getString("peers").c_str(), sender.c_str()) != NULL)'
      }
      if (strstr(payload, " says shit") != NULL) { 
        String sender = String(payload).substring(0, strstr(payload, " says shit") - payload), peers = prefs.getString("peers", "");
        if (peers.indexOf(sender) >= 0) { int openacks; xQueuePeek(ackQueue, &openacks, 0); openacks--; xQueueOverwrite(ackQueue, &openacks); }    //  with ' || peers == "" ' this self acconoledges when no peers set
      }
      feedlog("mqtt recived " + String(payload), 0, 0, 0, 0, "debug");
  
  
  
  
  
  
  
    });

    // only listen to everything but our self 

  // Get the eFuse MAC as a string (12 hex digits, uppercase, no delimiters)
  char macStr[13]; // 12 hex digits + null terminator
  snprintf(macStr, sizeof(macStr), "%012llX", ESP.getEfuseMac());
  // Build the topic string "/fpaper/<efusemac>"
  String myTopic = String("/fpaper/") + macStr;
  // Unsubscribe from "/fpaper/<efusemac>"
  mqttClient.unsubscribe(myTopic.c_str());
  
  //mqttClient.unsubscribe("/fpaper/" + String(ESP.getEfuseMac()));

  xTaskCreate( sendmqttTas, "sendmqttTas", 2048, NULL, 1, &sendmqttHandle );    //  spawn mqtt message sender task
  mqttClient.connect();
}


TaskHandle_t dnsServHandle;
DNSServer dnsServer;
void dnsServTas(void *parameter) {    //  this is the dns response task this only is called in ap mode
  dnsServer.start(53, "*", WiFi.softAPIP());    //  init dns server on port 53 with wildcard domain to map all requests to ap ip for captive portal
  while(true){
    dnsServer.processNextRequest();
    feedlog("hold led blue while in ap mode", 0, 0, 70, 100, "debug");
    vTaskDelay(10);
  }
}


//Preferences prefs;    //  commented so no redfinition error
WiFiClientSecure secureClient;
HTTPUpdate up;
void tryair() {    //  this works with redirects and insecure https source 'https://github.com/espressif/arduino-esp32/issues/9530#issuecomment-2090034699' improve this with checking here 'https://api.github.com/repos/crbyxwpzfl/mini/releases/latest' or 'https://api.github.com/repos/crbyxwpzfl/mini/tags' befor download and then use 'https://github.com/crbyxwpzfl/mini/releases/latest/download/adafruit-feather-esp32s3-4flash-2psram.bin'
  String airlink = prefs.getString("airlink", "https://github.com/crbyxwpzfl/mini/releases/download/v9/adafruit-feather-esp32s3-4flash-2psram.bin"); prefs.putString("airlink", "https://github.com/crbyxwpzfl/mini/releases/download/v9/adafruit-feather-esp32s3-4flash-2psram.bin" );  //  usually try fixed link or try custom link only once
  secureClient.setInsecure();    //  this is to ignore ssl so theoretically some one can spoof github this is not good 
  up.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);    //  this is to follw link redirects other options are eg 'up.rebootOnUpdate(false);' or 'secureClient.setTimeout(5);'
  up.onStart([]() { feedlog("overwrite firmware init download \n"); });
  up.onEnd([]() { feedlog("firmware download success so restart to overwrite \n"); });
  up.onError([](int err) { feedlog(  up.getLastErrorString() + " \n"); });
  up.onProgress([](int current, int total) { feedlog(  String(100.0 * current / total) + "% \n", 117, 133, 3, 100 ); });    //  to print percentage of download and pulse led yellow while updating perhaps prgressbar is cooler instead but have ro figure out how to do same line prints in webserial
  HTTPUpdateResult result = up.update(secureClient, airlink, "", [](HTTPClient *http) { });    //  to add sth to the http header use 'http->addHeader("Authorization", "{\"token\":\"noInitYet\"}");'
  feedlog("auto firmware error (" + String(up.getLastError()) + ") " + up.getLastErrorString().c_str() + " check " + airlink.c_str() + " \n");    //  usually auto restart prevents this line so just prints when no restart cause error
}


TaskHandle_t watermarkHandle;
void printWatermarkTas(void *count){
  int iter = *(int*) count; feedlog ("printing stack high watermark for tasks for " + String(iter) + " seconds \n");
  for (int i = 0; i < iter; i++) {
      feedlog(String(i+1) + "/" + String(iter) + ", dnsTas '" + String(uxTaskGetStackHighWaterMark(dnsServHandle)) + "', servoTas '" + String(uxTaskGetStackHighWaterMark(servoTasHandle)) + "', sendmqttTas '" + String(uxTaskGetStackHighWaterMark(sendmqttHandle)) + "', ledTas '" + String(uxTaskGetStackHighWaterMark(ledTasHandle)) + "'\n");
      vTaskDelay(1000);
  }
  feedlog("\n\n");
  vTaskDelete(watermarkHandle);
}


//Preferences prefs;    //  commented so no redfinition error
void recv( String msg ){    //  this uses string likely char array is better see https://github.com/asjdf/WebSerialLite/blob/545465b009a06a4a7d2da4247c9af2a821391beb/examples/demo/demo.ino#L27
  if ( msg.indexOf("help") >= 0 ) {
    feedlog("\n \n"
         "\nwhen wlan fails an access point spawns \n"
         " ssid 'ssid'         sets wlan '" + prefs.getString("ssid", "N.A.") + "' \n"
         " pass 'password'     sets password \n"
                                
         "\nmqtt config. tell others to add '" + prefs.getString("publ", String(ESP.getEfuseMac()) ) + "' \n"
         " user 'you'          sets your peer name \n"
         " peer 'others'       adds peer to '" + prefs.getString("peers", "local") + "' \n"
         " serv 'mqtt://url'   sets server '" + prefs.getString("mqserv", "mqtt://broker.emqx.io") + "' \n"
         " topic 'mqtt/topic'  sets topic '" + prefs.getString("mqtop", "/fpaper") + "' \n"

         "\nservo config. please take finger off before \n"
         " top  'servo pos'    sets top pos '"  + prefs.getInt("top", 0) + "' \n"
         " sit  'servo pos'    sets sit pos '"  + prefs.getInt("sit", 0) + "' \n"

         "\nother stuff \n"
         " help                prints this\n"
         " info 'seconds'      see some info \n"
         " publ 'text'         publish to mqtt \n"
         " debug 'level'       sets debug level '" + prefs.getString("debuglevel", "info") + "' \n"
         " restart             well this restarts \n"
         " apt upgrade 'link'  sets firmware url for next restart \n"
         " rm -rf              chill this just clears preferences\n\n\n" ); return;
  }
  if (msg.indexOf("topic ") == 0) {
    prefs.putString("mqtop", msg.substring(6)); feedlog("mqtt topic set to '" + msg.substring(6) + "'\n"); return;
  }
  if (msg.indexOf("debug ") == 0) {
    prefs.putString("debuglevel", msg.substring(6)); feedlog("debug level set to '" + msg.substring(6) + "'\n"); return;
  }
  if (msg.indexOf("publ ") == 0) {
    xQueueSend(sendmqttQueue, msg.substring(5).c_str(), 0); return;
  }
  if ( msg.indexOf("serv ") == 0 ) {
    prefs.putString("mqserv", msg.substring(5)); feedlog("mqtt server set to '" + msg.substring(5) + "'\n"); return;
  }
  if ( msg.indexOf("user ") == 0 ) {
    prefs.putString("publ", msg.substring(5)); feedlog("name set to '" + msg.substring(5) + "'\n"); return;
  }
  if( msg.indexOf("profile ") == 0){
    prefs.putBytes("localP", volatileShowBuff, sizeof(volatileShowBuff));    //  store personal profile picture in nvs as 'localP'
    feedlog("saved your profile picture");
    xQueueSend(showQueue, "homeScreen", 0); return;    // return to home screen this cycles through latest fotos
  }
  if ( msg.indexOf("peer ") == 0 ) {  // TODO rename peers to secrets and error if secret contains space or is longer than 15 chars because this is max nvs key length
    uint8_t hkdfbuff[32]; hkdf<SHA256>( hkdfbuff, 32, msg.substring(5).c_str(), msg.substring(5).length(), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 15 bytes from secret for nvs alias
    uint8_t aliasbuff[15]; hkdf<SHA256>( aliasbuff, 14, msg.substring(5).c_str(), msg.substring(5).length(), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 14 bytes from secret for nvs alias and leave one byte for specifing associated information like nvsaliasP for profile foto or nvsaliasH for encryption hkdf

    String nvsalias = ""; for (size_t i = 0; i < 14; i++) {    //  nvs only allowes alphanumeric perhaps hex encoding is better since this has distribution bias but out of hkdf this should fine pls say if not
        nvsalias += (char)((aliasbuff[i] % 26) + 'a');
    }
    
    prefs.putString("peers", prefs.getString("peers", "local") + " " + nvsalias + " ");    //  here the trailing space is to find last peer correctly in showTas add new peer to peers list in preferences

    // obsolete now i guess  String peers = prefs.getString("peers", ""); prefs.putString("peers", (peers == "") ? nvsalias : peers + " " + nvsalias);    //  add new peer to peers list in preferences
    
    prefs.putBytes((nvsalias + "H").c_str(), hkdfbuff, sizeof(hkdfbuff));    //  store hkdf result in nvs under 'nvsaliasH'
    feedlog("added secret '" + msg.substring(5) + "' with alias '" + nvsalias + "'"); return;



    /*


    while uploading/choosing foto 
      (->  dont show incoming fotos while uploading/choosing foto just store them in nvs)
      ->  start pinging every nvsalias for their profile picture
        -> upload complete
          -> show uploaded foto
          -> exit with set personal profile picture command
          -> longpress cycles through profile fotos of pongs and own profile foto
          -> short click sends foto to profile picture with corosponding encryption or on own profile picture sets it to local display
     (-> exit unsresponsive state when user exits without uploading a foto)
 

    while not uploading/choosing foto
      ->  responde to pings with own profile picture
           -> move servo and when reciving a image and show image (flash sender profile picture)

      ->  when recieving a wave
          -> move servo and flash profile picture of sender

      ->  short click sends short wave to current picture
      ->  long press cycles through latest image of each peer (flash profile picture of sender on change)


    
    
    ->  store most recent uploaded foto either in buffer (or nvs)

    ->  Mqtt subscribe to every nvsalias
    
    ->  add command to set current photo as profile picture 
    
    ->  each nvsalias has to have these nvs: latest photo, profile picture, encryption hkdf , (hash of profile foto)
    
    ->  local profile has to have these nvs: latest photo, profile picture, (hash of profile foto)
    
    (->  How do i hash the profile foto and only send profile foto on change?)

    (-> posebilety to delet specific peer and all its related data)

    */








    /* -- debug helper to print hkdf result in hex TODO remove this
    String chachaKeyHex;
    for (size_t i = 0; i < sizeof(hkdfbuff); i++) {
      if (hkdfbuff[i] < 0x10) chachaKeyHex += "0";  // Add leading zero for single digit hex
      chachaKeyHex += String(hkdfbuff[i], HEX);
      if (i < sizeof(hkdfbuff) - 1) chachaKeyHex += " ";
    }
    feedlog("chacha key derived (hex): " + chachaKeyHex + "\n");
    
    uint8_t testbuff[32];
    prefs.getBytes(nvsalias.c_str(), testbuff, sizeof(testbuff));   //  read back hkdf result from nvs to test if it worked
    String testbuffHex;
    for (size_t i = 0; i < sizeof(testbuff); i++) {
      if (testbuff[i] < 0x10) testbuffHex += "0";  // Add leading zero for single digit hex
      testbuffHex += String(testbuff[i], HEX);
      if (i < sizeof(testbuff) - 1) testbuffHex += " ";
    }
    feedlog("readback derived (hex): " + testbuffHex + "\n");
    */

  }
  if ( msg.indexOf("ssid ") == 0 ) {
    prefs.putString("ssid", msg.substring(5)); feedlog("ssid set to '" + msg.substring(5) + "'\n"); return;
  }
  if ( msg.indexOf("pass ") == 0 ) {
    prefs.putString("pass", msg.substring(5)); feedlog("pass set to '" + msg.substring(5) + "'\n"); return;
  }
  if ( msg.indexOf("restart") == 0 ) {
    feedlog("restarting esp"); ESP.restart(); return;
  }
  if ( msg.indexOf("apt upgrade ") == 0 ) {
    feedlog("'restart' to init upgrade of of '" + msg.substring(12) + " '\n" ); prefs.putString("airlink", msg.substring(12)); return;
  }
  if ( msg.indexOf("rm -rf") == 0 ) {
    prefs.clear(); feedlog("cleared preferences"); return;
  }
  if ( msg.indexOf("top ") == 0 ) {
    prefs.putInt("top", msg.substring(4).toInt()); xQueueSend(servoQueue, "top", 0); feedlog("top angel set to '" + msg.substring(4) + "'\n"); return;
  }
  if ( msg.indexOf("sit ") == 0 ) {
    prefs.putInt("sit", msg.substring(4).toInt()); xQueueSend(servoQueue, "sit", 0); feedlog("sit angle set to '" + msg.substring(4) + "'\n"); return;
  }
  if ( msg.indexOf("info") == 0 ) {
    char nvsfree[30]; sprintf(nvsfree, "\n\nfree entries in nvs %d \n", prefs.freeEntries()); feedlog(nvsfree);
    feedlog("PSRAM " + (psramFound() ? "found " + String(ESP.getPsramSize()) + " bytes total, " + String(ESP.getFreePsram()) + " bytes free \n" : "Not found\n"));
    feedlog("auto firmware url is '" + prefs.getString("airlink", "error") + "' \n");
    int openacks; xQueuePeek(ackQueue, &openacks, 0); feedlog("open acks " + String(openacks) + " \n");
    if(WiFi.getMode() == WIFI_MODE_AP) { feedlog("local ip " + WiFi.softAPIP().toString() + " \n"); }
    if(WiFi.getMode() == WIFI_MODE_STA) { feedlog("local ip " + WiFi.localIP().toString() + " \n"); }
    char macStr[30]; sprintf(macStr, "eFuse mac %012llX \n", ESP.getEfuseMac() ); feedlog(macStr);    //  this is so tiedious pls help me do not know how to string
    feedlog("| Type | Sub |  Offset  |   Size   |       Label      | \n");    //  this prints current partition table just for your info
    feedlog("| ---- | --- | -------- | -------- | ---------------- | \n");
    esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    if (pi != NULL) {
      do {
        const esp_partition_t* p = esp_partition_get(pi);
          char buffer[128]; sprintf(buffer, "|  %02x  | %02x  | 0x%06X | 0x%06X | %-16s | \n", p->type, p->subtype, p->address, p->size, p->label); feedlog(buffer);    //  this sucks i hate strings i miss python
      } while (pi = (esp_partition_next(pi)));
    }
    int count = msg.substring(5).toInt() ; xTaskCreate( printWatermarkTas, "printWatermarkTas", 2048, (void*) &count, 1, &watermarkHandle ); return;   //  determine stack size just for your info 'xTaskCreate( function, name, stack size bytes, parameter to pass, priority, handle )'
  }
  feedlog("recived " + msg + " unknown try 'help' \n");
}


//Preferences prefs;    //  commented so no redfinition error
//WebSerial WebSerial;  // webserial not static anymore since v8.0.0
AsyncWebServer server(80);
void initWebSerial() {    //  either spwan ap or connect to wlan and init webserial
  WiFi.mode(WIFI_STA);
  WiFi.begin( prefs.getString("ssid", "fpaper"), prefs.getString("pass", "") );    //  return ssid from preferences nvs or return finger
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {    //  not able to connect to ssid from nvs so fall back to ap
    WiFi.mode(WIFI_AP);
    WiFi.softAP("fpaper", "");
    feedlog(prefs.getString("ssid", "fpaper") + " failed so fallback soft ap fpaper up so access webserial at http://" + WiFi.softAPIP().toString().c_str() + "/webserial \n");
    xTaskCreate( dnsServTas, "dnsServ", 2048, NULL, 1, &dnsServHandle );    //  begin dns serv 'xTaskCreate( function, name, stack size bytes, parameter to pass, priority, handle )'
  }
  if (WiFi.waitForConnectResult() == WL_CONNECTED) {    //  all good connected to ssid from nvs
    feedlog(prefs.getString("ssid", "fpaper") + " success so access webserial at http://" + WiFi.localIP().toString().c_str() + "/webserial \n");
  }

  
  WebSerial.onMessage([](const std::string& msg) { recv(msg.c_str()); });
  //WebSerial.onMessage([](const String& msg) { recv(msg); });    //  attach message callback
  WebSerial.begin(&server);    //  init webserial


  server.on("/file", HTTP_POST, 
    [](AsyncWebServerRequest* request) {},    // empty request handler - no response sent
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    static size_t totalSize = 0;    //  static so this is not reset on each chunck

    if (!index){
      totalSize = request->header("Content-Length").toInt();
    }
    if (len + index > sizeof(volatileShowBuff)) {
      feedlog("aw thats to grande for me"); return;    //  this is to prevent buffer overflow
    }
    else if (len) {
      feedlog("file " + filename + " " + String(index + len) + "/" + String(totalSize) + " bytes\r\n");
      memcpy(volatileShowBuff + index, data, len);    //  copy data to volatile buffer
    }
    if (final){
      xQueueSend(showQueue, "sendScreen", 0);
    }
  });


  server.onNotFound([](AsyncWebServerRequest* request) {    //  redirect all requests to webserial for captive portal request->redirect("/webserial"); does not work for captive portal
    request->send(200, "text/html", "<!DOCTYPE html><html><meta http-equiv='refresh' content='0; url=http://fpaper.local/webserial' /><head><title>Captive Portal</title></head><body><p>auto redirect failed http://" + WiFi.softAPIP().toString() + "/webserial </p></body></html>");
  });
  server.begin();
  if (MDNS.begin("fpaper")) { feedlog("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
}


//InterruptButton belowus(20, LOW);    //  default longpress is 750ms
InterruptButton belowus(20, LOW, GPIO_MODE_INPUT, 420);    //  why does this not work inside initflanks
void initflanks() {
  belowus.bind(Event_KeyPress, [](){    //  feedlog inside here does chrash perhaps this is 'm_RTOSservicerStackDepth' see here https://github.com/rwmingis/InterruptButton/tree/main?tab=readme-ov-file#known-limitations
    xQueueSend(showQueue, "proceede", 0);    //  show volatile buffer on longpress
    //xQueueSend(servoQueue, "top", 0); 
    //xQueueSend(sendmqttQueue, "look here", 0);
    //String peers = prefs.getString("peers", "none");
    //int openacks; xQueuePeek(ackQueue, &openacks, 0); for(int i=0; peers[i]; i++){if(peers[i] == ' '){openacks++;}}; xQueueOverwrite(ackQueue, &openacks);
  });
  belowus.bind(Event_DoubleClick, [](){
    xQueueSend(showQueue, "send", 0);    //  show volatile buffer on longpress
  });
}


//Preferences prefs;    //  commented so no redfinition error
void setup() {
  Serial.begin(115200);    //  serial requires delay or while(!Serial); so no output is lost
  prefs.begin("prefs", false);    //  open preferences with namespace prefs in read write mode this is for wifi creds and stuff  

  xTaskCreate( ledTas, "ledTas", 2048, NULL, 1, &ledTasHandle ); feedlog("init everything", 50, 50, 50, 2000, "debug");    //  spawn led task
  initWebSerial();   //  init wifi and webserial this is blocks until wifi is up

  //tryair();    //  TODO this should be a command thing to an auto thing try to upgrade firmware from hardcoded url fails in ap mode this blocks aswell
  
  xTaskCreate( servoTas, "servoTas", 2048, NULL, 1, &servoTasHandle );    //  now spawn async tasks
  initflanks();    //  this is asnyc per lib so no xTaskCreate nessesary
  initmqtt();    //  init mqtt this is asnyc per lib so no xTaskCreate nessesary
  xTaskCreate( showTas, "showTas", 2048, NULL, 1, &showTasHandle );    //  spawn show task to display images on epaper

  feedlog("init done");

  memcpy_P(volatileShowBuff, epd_bitmap_xpwallp, 15000);    //  copy boot foto from PROGMEM to volatile buffer for fast access
  xQueueSend(showQueue, "showboot", 0);    //  add volatile foto to show queue


  if (receivedImageBuffer) {
    free(receivedImageBuffer);
    receivedImageBuffer = nullptr;
  }
  

  //NEW
  if (imageBuffer) {
    free(imageBuffer);
    imageBuffer = nullptr;
  }
  //END

}


void loop() { }