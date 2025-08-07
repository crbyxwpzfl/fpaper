
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
static uint8_t sendBuff[15000];  // global show buffer no malloc/free necessary images are of static size
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
void showTas(void *parameter) {    //  this handles servo movement
  showQueue = xQueueCreate(5, 15);    // create queue with buffer of 5 with length of nvsalias so 15 chars
 
  char buff[] = "nvsalias length";    //  buffer to read from queue

  pinMode(7, OUTPUT); digitalWrite(7, HIGH);   //  give power to the panel
  display.init(115200);    // init epd with 115200 baud rate
  display.setRotation(0);    //  TODO make this a setting in preferences but also change selection/ditthered overlay aspect accordingly

  uint8_t showBuff[15000];

  while(true){
    if(!xQueueIsQueueEmptyFromISR( showQueue )){    //  just do sth when queue not empty
      xQueueReceive(showQueue, &buff, 0);

      if (!prefs.getBytes( buff, showBuff, 15000 ))  Serial.println("nothing found for " + String(buff));    //  for invalid nvs lookups this returns null and leaves showBuff

      display.setFullWindow();    
      display.firstPage();
      do {
        display.fillScreen(GxEPD_BLACK);
        display.drawBitmap(0, 0, showBuff, display.width(), display.height(), GxEPD_WHITE);
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

      if ( !strcmp(buff, "sendv local"   ) ) prefs.putBytes( "localL", sendBuff, sizeof(sendBuff));    //  ignore sendps here so no overwrites for annyos and only save once for usual send
      if ( !strcmp(buff, "sendv profile" ) ) prefs.putBytes( "localP", sendBuff, sizeof(sendBuff));    //  when recipient profile save to localP when local save to localL

      if (  strcmp(buff + 6, "profile") &&  strcmp(buff + 6, "local") ) {    //  here when recipient not profile and not local actually do send stuff either answer to look here with profile or annoy with just profile or send profile plus volatileShow    // TODO somehow dont send full profile everytime you want to annoy
        uint8_t *payload = (uint8_t*)malloc(sizeof(curriv) + sizeof(tag) + sizeof(cyphy) + 9);    //  allocate memory for payload
      
        esp_fill_random(curriv, sizeof(curriv));    //  fill curriv with noise here this only is to later in recieve mqtt determine wether message is a echo

        prefs.getBytes((String(buff + 6) + "H").c_str(), hkdf, 32);    //  find the peer hkdf

        chachapoly.setIV(curriv, 12);
        chachapoly.setKey(hkdf, 32);

        //if (strncmp(buff, "sendq ", 6) == 0) chachapoly.addAuthData("look here", 9);    //  TODO this is optional right to find listenig peers querey peers with 'sendq' this is authenticated but not encrypted
        //if (strncmp(buff, "senda ", 6) == 0) chachapoly.addAuthData("shit", 9);    //  to answer so we listening with 'senda'

        if (!strncmp(buff, "sendp ", 6)) { prefs.getBytes("localP", cyphy, 15000); chachapoly.encrypt(cyphy, cyphy, 15000); }     //  send profile
        if (!strncmp(buff, "sendv ", 6)) chachapoly.encrypt(cyphy, sendBuff, 15000);    //  with 'sendv' send current foto

        chachapoly.computeTag(tag, 16);    //  TODO chek that this can runn wihtout previously running encrypt for case "sendq " to just send look here
        chachapoly.clear();

        memcpy(payload, curriv, sizeof(curriv));    //  first iv
        memcpy(payload + sizeof(curriv), tag, sizeof(tag));    //  then tag
        memcpy(payload + sizeof(curriv) + sizeof(tag), cyphy, sizeof(cyphy));    //  then foto

        if (!strncmp(buff, "sendp ", 6)) memcpy(payload + sizeof(curriv) + sizeof(tag) + sizeof(cyphy), "look here", 9);    //  send our profile with 'look here' appendix    TODO send hash of peers profile to minimize messages
        if (!strncmp(buff, "sendv ", 6)) memcpy(payload + sizeof(curriv) + sizeof(tag) + sizeof(cyphy), "see this ", 9);    //  send foto with 'see this ' appendix
        
        Serial.println("packed payload try sending now to " + String(buff + 6));    //  TODO make this a feedlog message

        mqttClient.publish( (prefs.getString("mqtop", "fpaper/") + String(buff + 6)).c_str() , 0, 0, reinterpret_cast<const char*>(payload), 12 + 16 + 15000 + 9, true);    //  publish full length message to base topic + peer alias

        free(payload);
      }

    }
    vTaskDelay(4000);    // just send every two second so we have enugh time to filter out our echos with curriv
  }
}



//Preferences prefs;    //  commented so no redfinition error
//PsychicMqttClient mqttClient;    //  commented so no redfinition error
void initmqtt(){    //  handle incoming mqtt
  String serverAddress = prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work

  mqttClient.onTopic( prefs.getString("mqtop", "fpaper/+").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    // wildcards should work here listen one level deep for now TODO change this to only subscribe to peers
    if ( !prefs.getBytesLength( (String(topic).substring(7) + "H").c_str() ) ) return;    //  just listen to messages of our peers no sens to decode when no peer hkdf found
    if ( !memcmp(curriv, payload, 12) ) return;    //  when message was our own message ignore it
    
    feedlog("got message start decoding");

    uint8_t* hkdf = (uint8_t*)malloc(32); prefs.getBytes((String(topic).substring(7) + "H").c_str(), hkdf, 32);    //  find hkdf of sender peer
    uint8_t* iv = (uint8_t*)malloc(12); memcpy(iv, payload, 12);    //  iv starts at the beginning of payload and is 12 bytes long
    uint8_t* tag = (uint8_t*)malloc(16); memcpy(tag, payload + 12, 16);    //  tag starts after iv and is 16 bytes long
    uint8_t* cyphy = (uint8_t*)malloc(15000); memcpy(cyphy, payload + 12 + 16, 15000);    //  cypher text starts after iv and tag so and is 15000 bytes long

    feedlog("Received message on topic: " + String(topic) );
    
    chachapoly.setIV(iv, 12);     feedlog(" set iv");
    chachapoly.setKey(hkdf, 32);      feedlog(" set key");
    chachapoly.decrypt(cyphy, cyphy, 15000);   feedlog(" decrypted cypher text");
    bool tagValid = chachapoly.checkTag(tag, 16);    //  check tag after decryption so we can see if decryption was successfull

    if ( tagValid && !memcmp("look here", payload + 12 + 16 + 15000, 9) ) {    //  here compare recieved profile to saved profile and perhpas overwrite    also show recieved profile    also move servo 
      uint8_t* currentProfile = (uint8_t*)malloc(15000); prefs.getBytes( (String(topic).substring(7) + "P").c_str(), currentProfile, 15000 );    //  find current profile from nvsalias+'P' or leaves currentProfile as is
      
      feedlog("first decryption successfull");

      if ( memcmp(currentProfile, cyphy, 15000) ) prefs.putBytes( (String(topic).substring(7) + "P").c_str(), cyphy, 15000 );    //  when profile changes save recieved profile to nvsalias+'P'
      
      free(currentProfile);

      xQueueSend(showQueue, (String(topic).substring(7) + "P").c_str(), 0);    //  show recieved profile
      xQueueSend(servoQueue, "top", 0);    //  move servo to top position this wiggles screen
    }

    if ( tagValid && !memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here save recieved foto to nvsalias+'L'    also show this
      prefs.putBytes( (String(topic).substring(7) + "L").c_str(), cyphy, 15000 );    //  save foto to nvsalias+'L' so we can show it later
      
      feedlog("second decryption successfull");

      xQueueSend(showQueue, (String(topic).substring(7) + "L").c_str(), 0);    //  show recieved foto
    }
    chachapoly.clear(); free(hkdf); free(iv); free(tag); free(cyphy);


    /*
    if ( memcmp("you there", payload + 12 + 16 + 15000, 9) ) {    //  here always responde with our profile    and perhaps save recieved profile to nvsalias+'P'    and when in homeScreen show recieved profile
      xQueueSend(sendmqttQueue, "senda " + String(topic).substring(7) , 0);    //  responde to peer with our profile and apendix '..show me'

      return; 
    }


    
    if ( memcmp("sure sure", payload + 12 + 16 + 15000, 9) ) {    //  here queue recieved profile to sendScreen    and perhaps save recieved profile to nvsalias+'P'
    
      return; 
    }



    if ( memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here always save the recieved foto to nvsalias+'L'    and when in homeScreen show foto
    
      return;
    }



    if ( memcmp("look here", payload + 12 + 16 + 15000, 9) ) {    //  here when in homeScreen show recieved profile    and perhaps save recieved profile to nvsalias+'P'
    
      return;
    }

    




      
      // decode message here topic minus fpaper/ is the nvsalias so to get correct key use getBytes() with topic.substring(8)+'H' this gives hkdf of corosponding peer
   
      */


  });
  

  /*
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
  
  */

  xTaskCreate( sendmqttTas, "sendmqttTas", 32768, NULL, 1, &sendmqttHandle );    //  spawn mqtt message sender task apparently task has to have enough stack for every buffer so here > 15KB
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
         " serv 'mqtt://url'   sets server '" + prefs.getString("mqserv", "mqtt://broker.hivemq.com") + "' \n"
         " topic 'mqtt/topic'  sets topic '" + prefs.getString("mqtop", "fpaper/+") + "' \n"

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
  /*  TODO remove this this is not the way to set profile picture anymore
  if( msg.indexOf("profile ") == 0){
    prefs.putBytes("localP", sendBuff, sizeof(sendBuff));    //  store personal profile picture in nvs as 'localP'
    feedlog("saved your profile picture");
    xQueueSend(showQueue, "homeScreen", 0); return;    // return to home screen this cycles through latest fotos
  }
  */
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


  server.on("/queryPeers", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "profile " + prefs.getString("peers", "local ") );    //  send current peers list
  });


  server.on("/file", HTTP_POST,
    [](AsyncWebServerRequest* request) {},    // empty request handler - no response sent
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    static size_t totalSize = 0;    //  static so this is not reset on each chunck
    static String targetPeer = "";    // static to persist across chunks

    if (!index){
      totalSize = request->header("Content-Length").toInt();
      targetPeer = request->getParam("peer")->value();
      feedlog("file is for " + targetPeer);
    }
    if (len + index > sizeof(sendBuff)) {
      feedlog("aw thats to grande for me"); return;    //  this is to prevent buffer overflow
    }
    else if (len) {
      feedlog("file " + filename + " " + String(index + len) + "/" + String(totalSize) + " bytes\r\n");
      memcpy(sendBuff + index, data, len);    //  copy data to volatile buffer
    }
    if (final){
      xQueueSend(sendmqttQueue, ("sendp " + targetPeer).c_str(), 0);    //  send personal profile to peer
      xQueueSend(sendmqttQueue, ("sendv " + targetPeer).c_str(), 0);    //  send preped volatile buffer to peer
    }
  });


  server.onNotFound([](AsyncWebServerRequest* request) {    //  redirect all requests to webserial for captive portal request->redirect("/webserial"); does not work for captive portal
    request->send(200, "text/html", "<!DOCTYPE html><html><meta http-equiv='refresh' content='0; url=http://fpaper.local/webserial' /><head><title>Captive Portal</title></head><body><p>auto redirect failed http://" + WiFi.softAPIP().toString() + "/webserial </p></body></html>");
  });
  server.begin();
  if (MDNS.begin("fpaper")) { feedlog("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
}


//InterruptButton belowus(20, LOW);    //  default longpress is 750ms
String currpeer = "local";    //  start locally
//InterruptButton belowus(20, LOW, GPIO_MODE_INPUT, 420);    //  why does this not work inside initflanks
InterruptButton belowus(2, LOW, GPIO_MODE_INPUT, 420);    //  TODO reinstate above this is ony for testig we have her pin 2
void initflanks() {

  belowus.bind(Event_KeyPress, [](){    //  feedlog inside here does chrash perhaps this is 'm_RTOSservicerStackDepth' see here https://github.com/rwmingis/InterruptButton/tree/main?tab=readme-ov-file#known-limitations
    String peerString = prefs.getString("peers", "local");
    
    if (peerString.indexOf(currpeer)+currpeer.length()+1+1 > peerString.length()) currpeer = "local";   //  account for trailing space here test for last peer and wrap
    else currpeer = peerString.substring( peerString.indexOf(currpeer)+currpeer.length()+1, peerString.indexOf(' ', peerString.indexOf(currpeer)+currpeer.length()+1) );    //  advance to next peer in list so this is the next peer or local when no peers set

    Serial.println("current peer is " + currpeer + " now queueing currpeer p and L");    //  print current peer to serial

    xQueueSend(showQueue, (currpeer + "P").c_str(), 0);    //  queue 'P'rofile picture of peer
    xQueueSend(showQueue, (currpeer + "L").c_str(), 0);    //  queue 'L'atest foto of peer


    //xQueueSend(showQueue, "proceede", 0);    //  show volatile buffer on longpress
    //xQueueSend(servoQueue, "top", 0); 
    //xQueueSend(sendmqttQueue, "look here", 0);
    //String peers = prefs.getString("peers", "none");
    //int openacks; xQueuePeek(ackQueue, &openacks, 0); for(int i=0; peers[i]; i++){if(peers[i] == ' '){openacks++;}}; xQueueOverwrite(ackQueue, &openacks);
  });

  belowus.bind(Event_DoubleClick, [](){
    xQueueSend(sendmqttQueue, ("sendp " + currpeer).c_str(), 0);    //  just annoy peer with profile
    //xQueueSend(showQueue, "send", 0);    //  show volatile buffer on longpress
  });
}


//Preferences prefs;    //  commented so no redfinition error
void setup() {
  Serial.begin(115200);    //  serial requires delay or while(!Serial); so no output is lost
  prefs.begin("prefs", false);    //  open preferences with namespace prefs in read write mode this is for wifi creds and stuff  

  xTaskCreate( ledTas, "ledTas", 4096, NULL, 1, &ledTasHandle ); feedlog("init everything", 50, 50, 50, 2000, "debug");    //  spawn led task
  initWebSerial();   //  init wifi and webserial this is blocks until wifi is up

  //tryair();    //  TODO this should be a command thing to an auto thing try to upgrade firmware from hardcoded url fails in ap mode this blocks aswell
  
  xTaskCreate( servoTas, "servoTas", 4096, NULL, 1, &servoTasHandle );    //  now spawn async tasks
  initflanks();    //  this is asnyc per lib so no xTaskCreate nessesary
  initmqtt();    //  init mqtt this is asnyc per lib so no xTaskCreate nessesary
  xTaskCreate( showTas, "showTas", 32768, NULL, 1, &showTasHandle );    //  spawn show task to display images on epaper

  feedlog("init done");


  // TODO add a boot screen of some sort currently the showTas does not support this 
  //memcpy_P(sendBuff, epd_bitmap_xpwallp, 15000);    //  copy boot foto from PROGMEM to volatile buffer for fast access
  //xQueueSend(showQueue, "showboot", 0);    //  add volatile foto to show queue


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