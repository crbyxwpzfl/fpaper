
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
static uint8_t volatileBuff[15000];  // global show buffer no malloc/free necessary images are of static size
//static uint8_t curriv[12];  // global iv buffer for chachapoly encryption and to ignore own message echos


String currpeer = "local";    //  initally show local
String prep = "";    //  prepared peer when sendscreen timer runs out
uint32_t sendscreents = 0;    //  timestamp when user opened sendscreen



Preferences prefs;    //  first declaration of preferences as perfs
TaskHandle_t showTasHandle;
QueueHandle_t sendmqttQueue;    //  handle for mqtt message queue see task belowus
QueueHandle_t showQueue;    //  handle for servo queue
void showTas(void *parameter) {    //  this handles the epaper
  struct showstct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }; showQueue = xQueueCreate(5, sizeof(showstct));    // create queue with buffer of 5 with length of nvsalias so 20 chars

  //char buff[20] = "";    //  buffer to read from queue has length of 15 nvs chars plus 4 for full/part

  pinMode(7, OUTPUT); digitalWrite(7, HIGH);   //  give power to the panel
  display.init(115200);    // init epd with 115200 baud rate
  display.setRotation(0);    //  TODO make this a setting in preferences but also change selection/ditthered overlay aspect accordingly

  uint8_t showBuff[15000];
  char ocupado[5];    //  save the screen state either user or prog or empty

  while(true){
    if (!xQueueIsQueueEmptyFromISR( showQueue )){    //  just do sth when queue not empty
      showstct show; xQueueReceive(showQueue, &show, 0);

      if ( ocupado && strcmp(ocupado, show.ocupado) ) return;    //  ignore all requests while screen is ocupado with an somthing else

      strcpy(ocupado, show.ocupado);    //  save screen state

      if (!prefs.getBytes( show.nvsalias, showBuff, 15000 )) { Serial.println("nothing found for " + String(show.nvsalias)); esp_fill_random(showBuff, sizeof(showBuff)); }    //  for invalid nvs lookups this fills the showBuff with noise

      //if (strncmp(buff, "full", 4) == 0) {    //  show with full refresh
      if (!show.partial) {    //  show with full refresh

        ocupado = "";    //  a full refresh always indicates a free screen

        display.setFullWindow();
        display.firstPage();
        do {
          display.fillScreen(GxEPD_BLACK);
          display.drawBitmap(0, 0, showBuff, display.width(), display.height(), GxEPD_WHITE);
        } while (display.nextPage());
      }

      //if (display.epd2.hasFastPartialUpdate) {    //  this display has this
      //if (strncmp(buff, "part", 4) == 0) {    //  show picture in picture (center 100x100 of currently loaded showBuff)
      if ( show.partial) {    //  show picture in picture (center 100x100 of currently loaded showBuff)
        const int srcW = 400;
        const int srcH = 300;
        const int ovW = 230;
        const int ovH = 200;
        const int srcX0 = (srcW - ovW) / 2;  // 150
        const int srcY0 = (srcH - ovH) / 2;  // 100
        const int destX = 30;  // user requested coordinates
        const int destY = 30;

        display.setPartialWindow(destX, destY, ovW, ovH);    //  numbers are xpos ypos width height
        display.firstPage();
        do {
      // draw pixels copied bitwise from showBuff (1bpp linear, 400x300 -> 15000 bytes)
          for (int y = 0; y < ovH; y++) {
            int srcY = srcY0 + y;
            for (int x = 0; x < ovW; x++) {
              int srcX = srcX0 + x;
              int bitIndex = srcY * srcW + srcX;              // 1bpp linear index
              int byteIndex = bitIndex / 8;
              uint8_t bitMask = 0x80 >> (bitIndex % 8);
        uint8_t byteValue = showBuff[byteIndex];
              bool colored = byteValue & bitMask;             // bitmap as stored
              display.drawPixel(destX + x, destY + y, colored ? GxEPD_WHITE : GxEPD_BLACK);
            }
          }
        } while (display.nextPage());
      }



      display.hibernate();   //  hibernate display to save power
    }
    vTaskDelay(1);    //  befor one second so no flicker just show every seconds now we have to check timer continously


  }
}


WebSerial WebSerial;  //  first delclartion of webserial not static anymore since v8.0.0
//Preferences prefs;    //  commented so no redfinition error
void feedlog(String text, String level = "info") {    //  print to serial and webserial and forward led feedback to ledTas
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
  struct sendstct { char peer[16]; char load[16]; } sendmqttQueue = xQueueCreate( 5, sizeof(sendstct) );    // create queue with buffer of 5
      //  this hard coded finite length stresses me in python me no have to worry me miss python

  static uint8_t curriv[12];
  uint8_t hkdf[32];
  uint8_t cyphy[15000];    //  for encrypted bytes
  uint8_t tag[16];

  
  String serverAddress = prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work

  // TODO no topics anymore just one topic so dont check topic just try to decode with all stored peer hkdfs and find peer this way


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

      if (!sendscreents) xQueueSend(showQueue, ("part" + String(topic).substring(7) + "P").c_str(), 0);    //  show recieved profile picture in picture only when not in sendscreen
      xQueueSend(servoQueue, "top", 0);    //  move servo to top position this wiggles screen
    }

    if ( tagValid && !memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here save recieved foto to nvsalias+'L'    also show this
      prefs.putBytes( (String(topic).substring(7) + "L").c_str(), cyphy, 15000 );    //  save foto to nvsalias+'L' so we can show it later
      
      feedlog("second decryption successfull");

      if (!sendscreents) xQueueSend(showQueue, ("full" + String(topic).substring(7) + "L").c_str(), 0);    //  show recieved foto with full refresh to clear profile overlay
    }
    chachapoly.clear(); free(hkdf); free(iv); free(tag); free(cyphy);

  });

  mqttClient.connect();

  while (true) {
    if(!xQueueIsQueueEmptyFromISR( sendmqttQueue )){    //  just do sth when queue not empty
      sendstct send; xQueueReceive(sendmqttQueue, &send, 0);    //  reads first word out of queue

     if (!prefs.getBytes( send.load, cyphy, 15000 )) { Serial.println("nothing found for " + String(send.load)); return; }    //  for invalid nvs lookups this returns null and leaves cyphy


      //if ( !strcmp(nvsalias, "sendv local"   ) ) {
      //   prefs.putBytes( "localL", volatileBuff, sizeof(volatileBuff));    //  when recipient local save to localL
      //   xQueueSend(showQueue, ("full" + currpeer + "L").c_str(), 0);
      //}
      //if ( !strcmp(buff, "sendv profile" ) ) prefs.putBytes( "localP", volatileBuff, sizeof(volatileBuff));    //  when recipient profile save to localP when local save to localL  ignore sendps here so no overwrites for annyos and only save once for usual send
      if ( !send.peer ) {    //  local is "0" so falsy
         prefs.putBytes( "0latest", cyphy, sizeof(cyphy));    //  when recipient local save to local latest
      }

      if ( send.peer ) {    //  here when recipient not local actually do send stuff either answer to look here with profile or send profile plus volatileShow    // TODO somehow dont send full profile everytime you want to annoy

      //if (  strcmp(buff + 6, "profile") &&  strcmp(buff + 6, "local") ) {    //  here when recipient not profile and not local actually do send stuff either answer to look here with profile or annoy with just profile or send profile plus volatileShow    // TODO somehow dont send full profile everytime you want to annoy
      //if ( strcmp(nvsalias + 6, "local") ) {    //  here when recipient not local actually do send stuff either answer to look here with profile or send profile plus volatileShow    // TODO somehow dont send full profile everytime you want to annoy
        
        uint8_t *payload = (uint8_t*)malloc(sizeof(curriv) + sizeof(tag) + sizeof(cyphy) + 9);    //  allocate memory for payload
      
        esp_fill_random(curriv, sizeof(curriv));    //  fill curriv with noise here this only is to later in recieve mqtt determine wether message is a echo

        //prefs.getBytes((String(nvsalias + 6) + "H").c_str(), hkdf, 32);    //  find the peer hkdf
        prefs.getBytes( strcat(send.peer, "hkdf") , hkdf, 32);    //  find the peer hkdf

        chachapoly.setIV(curriv, 12);
        chachapoly.setKey(hkdf, 32);
        chachapoly.encrypt(cyphy, cyphy, 15000);    //  encrypt clear bytes of load this was loaded into cyphy befor

        //if (strncmp(buff, "sendq ", 6) == 0) chachapoly.addAuthData("look here", 9);    //  TODO this is optional right to find listenig peers querey peers with 'sendq' this is authenticated but not encrypted
        //if (strncmp(buff, "senda ", 6) == 0) chachapoly.addAuthData("shit", 9);    //  to answer so we listening with 'senda'

        //if (!strncmp(nvsalias, "sendp ", 6)) { prefs.getBytes("localP", cyphy, 15000); chachapoly.encrypt(cyphy, cyphy, 15000); }     //  send profile
        //if (!strncmp(nvsalias, "sendv ", 6)) chachapoly.encrypt(cyphy, volatileBuff, 15000);    //  with 'sendv' send current foto of volatile buffer

        chachapoly.computeTag(tag, 16);
        chachapoly.clear();

        memcpy(payload, curriv, sizeof(curriv));    //  pack payload with first iv
        memcpy(payload + sizeof(curriv), tag, sizeof(tag));    //  then tag
        memcpy(payload + sizeof(curriv) + sizeof(tag), cyphy, sizeof(cyphy));    //  then foto
        memcpy(payload + sizeof(curriv) + sizeof(tag) + sizeof(cyphy), strncmp(send.load, "0profile") ? "look here" : "see this ", 9);    //  send our profile with 'look here' appendix or send foto slot with 'see this'    TODO send hash of peers profile to minimize messages

        //if (!strncmp(nvsalias, "sendp ", 6)) memcpy(payload + sizeof(curriv) + sizeof(tag) + sizeof(cyphy), "look here", 9);    //  send our profile with 'look here' appendix    TODO send hash of peers profile to minimize messages
        //if (!strncmp(nvsalias, "sendv ", 6)) memcpy(payload + sizeof(curriv) + sizeof(tag) + sizeof(cyphy), "see this ", 9);    //  send foto with 'see this ' appendix
        
        Serial.println("packed payload try sending now to " + String(send.peer));    //  TODO make this a feedlog message

        mqttClient.publish( (prefs.getString("mqtop", "fpaper/") + String(nvsalias + 6)).c_str() , 0, 0, reinterpret_cast<const char*>(payload), 12 + 16 + 15000 + 9, true);    //  publish full length message to base topic + peer alias

        free(payload);
      }

    }
    vTaskDelay(4000);    // just send every two second so we have enugh time to filter out our echos with curriv   // TODO change this back to two seconds
  }
}


/*  todo rm this moved to mqtt task
//Preferences prefs;    //  commented so no redfinition error
//PsychicMqttClient mqttClient;    //  commented so no redfinition error
void initmqtt(){    //  handle incoming mqtt
  String serverAddress = prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work


  // TODO no topics anymore just one topic so dont check topic just try to decode with all stored peer hkdfs and find peer this way


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

      if (!sendscreents) xQueueSend(showQueue, ("part" + String(topic).substring(7) + "P").c_str(), 0);    //  show recieved profile picture in picture only when not in sendscreen
      xQueueSend(servoQueue, "top", 0);    //  move servo to top position this wiggles screen
    }

    if ( tagValid && !memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here save recieved foto to nvsalias+'L'    also show this
      prefs.putBytes( (String(topic).substring(7) + "L").c_str(), cyphy, 15000 );    //  save foto to nvsalias+'L' so we can show it later
      
      feedlog("second decryption successfull");

      if (!sendscreents) xQueueSend(showQueue, ("full" + String(topic).substring(7) + "L").c_str(), 0);    //  show recieved foto with full refresh to clear profile overlay
    }
    chachapoly.clear(); free(hkdf); free(iv); free(tag); free(cyphy);

  });

  xTaskCreate( sendmqttTas, "sendmqttTas", 32768, NULL, 1, &sendmqttHandle );    //  spawn mqtt message sender task apparently task has to have enough stack for every buffer so here > 15KB
  mqttClient.connect();
}
*/

TaskHandle_t dnsServHandle;
DNSServer dnsServer;
void dnsServTas(void *parameter) {    //  this is the dns response task this only is called in ap mode
  dnsServer.start(53, "*", WiFi.softAPIP());    //  init dns server on port 53 with wildcard domain to map all requests to ap ip for captive portal
  while(true){
    dnsServer.processNextRequest();
    feedlog("dns for ap mode", "debug");
    vTaskDelay(10);
  }
}


//Preferences prefs;    //  commented so no redfinition error
WiFiClientSecure secureClient;
HTTPUpdate up;
void tryair(String airlink) {    //  this works with redirects and insecure https source 'https://github.com/espressif/arduino-esp32/issues/9530#issuecomment-2090034699' improve this with checking here 'https://api.github.com/repos/crbyxwpzfl/mini/releases/latest' or 'https://api.github.com/repos/crbyxwpzfl/mini/tags' befor download and then use 'https://github.com/crbyxwpzfl/mini/releases/latest/download/adafruit-feather-esp32s3-4flash-2psram.bin'
  if( airlink ) {    //  only do this when airlink has value
    prefs.putString("airlink", "");    //  disable airlink for next boot
    //String airlink = prefs.getString("airlink", "https://github.com/crbyxwpzfl/mini/releases/download/v9/adafruit-feather-esp32s3-4flash-2psram.bin"); prefs.putString("airlink", "https://github.com/crbyxwpzfl/mini/releases/download/v9/adafruit-feather-esp32s3-4flash-2psram.bin" );  //  usually try fixed link or try custom link only once
    secureClient.setInsecure();    //  this is to ignore ssl so theoretically some one can spoof github this is not good 
    up.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);    //  this is to follw link redirects other options are eg 'up.rebootOnUpdate(false);' or 'secureClient.setTimeout(5);'
    up.onStart([]() { feedlog("overwrite firmware init download \n"); });
    up.onEnd([]() { feedlog("firmware download success so restart to overwrite \n"); });
    up.onError([](int err) { feedlog(  up.getLastErrorString() + " \n"); });
    up.onProgress([](int current, int total) { feedlog(  String(100.0 * current / total) + "% \n" ); });    //  to print percentage of download and pulse led yellow while updating perhaps prgressbar is cooler instead but have ro figure out how to do same line prints in webserial
    HTTPUpdateResult result = up.update(secureClient, airlink, "", [](HTTPClient *http) { });    //  to add sth to the http header use 'http->addHeader("Authorization", "{\"token\":\"noInitYet\"}");'
  }
  feedlog("auto firmware error (" + String(up.getLastError()) + ") " + up.getLastErrorString().c_str() + " check " + airlink.c_str() + " \n");    //  usually auto restart prevents this line so just prints when no restart cause error
}


TaskHandle_t watermarkHandle;
void printWatermarkTas(void *count){
  int iter = *(int*) count; feedlog ("printing stack high watermark for tasks for " + String(iter) + " seconds \n");
  for (int i = 0; i < iter; i++) {
      feedlog(String(i+1) + "/" + String(iter) + ", dnsTas '" + String(uxTaskGetStackHighWaterMark(dnsServHandle)) + "', servoTas '" + String(uxTaskGetStackHighWaterMark(servoTasHandle)) + "', sendmqttTas '" + String(uxTaskGetStackHighWaterMark(sendmqttHandle)) + "'\n");
      vTaskDelay(1000);
  }
  feedlog("\n\n");
  vTaskDelete(watermarkHandle);
}


//Preferences prefs;    //  commented so no redfinition error
void recv( String msg ){    //  this uses string likely char array is better see https://github.com/asjdf/WebSerialLite/blob/545465b009a06a4a7d2da4247c9af2a821391beb/examples/demo/demo.ino#L27
  if ( msg.indexOf("help") >= 0 ) {
    String peerstring = "";
    char i[2] = {'0', '\0'}; while (prefs.isKey(i)) { 
      peerstring += String(i) + "-" + prefs.getString(i, "N.A.") + " ";
      i[0]++;
    }

    feedlog("\n \n"
         "\nwhen wlan fails an access point spawns \n"
         " ssid 'ssid'         sets wlan '" + prefs.getString("ssid", "N.A.") + "' \n"
         " pass 'password'     sets password \n"
                                
         "\nmqtt config. tell others to add '" + prefs.getString("publ", String(ESP.getEfuseMac()) ) + "' \n"
         " peer 'name' 'secret' adds peer '" + peerstring + "' \n"
         " serv 'mqtt://url'    sets server '" + prefs.getString("mqserv", "mqtt://broker.hivemq.com") + "' \n"
         " topic 'mqtt/topic'   sets topic '" + prefs.getString("mqtop", "fpaper/+") + "' \n"
         //" slots 'count'        sets the available slots '" + prefs.getString("slotcount", "4") + "' \n"

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
  //if (msg.indexOf("slots ") == 0) {
  //  prefs.putString("slotcount", msg.substring(6)); feedlog("'" + msg.substring(6) + "' slots available\n"); return;
  //}
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

  // TODO add function to delete peer   delete all keys for peer like indexhkdf, indexprofile, index, indexlatest!   then move topmost peer to the index of deleted peer to keep iterable structure

  // TODO add function to delete slot   just overwrite the slot with the top most slot and update slotcount and restart

  if ( msg.indexOf("peer ") == 0 ) {    //  this adds peer name to nvs with ASCII index so that its easy to iterate over peers also this does hkdf with secret and puts it into nvs with 'peer name + hkdf'

    //char alias[16] = "";
    //strcpy(alias, msg.substring(5, msg.indexOf(" ", 5)).c_str());

    //if ( strlen(alias) > 7 ) { feedlog("alias too long max 7 chars \n"); return; }    //  check for max length of 7 since len("alias") + len("profile") < 15 nvs alias length 

    if (!prefs.isKey("0")) prefs.putString("0", "local");    //  when local peer not found do initialise local here

    //char i[2] = {'0', '\0'};  TODO rm
    //char i[16] = "0"; while (prefs.isKey(i)) i[0]++;    //  find first free nvs char sequentially this limits peer count to dec 48/ASCII 0 to dec 126/ASCII ~     //  theoretically with for esp32 platform this could do dec 0/ASCII NULL to dec 255/ASCII nbsp see here https://forum.arduino.cc/t/char-is-not-signed-no-reference-in-the-documentation/1297470
    
    uint16_t i = prefs.getUShort("peercount", 0) + 1;    //  read current peer count and add one
    prefs.putUShort("peercount", i);    //  put incremented peer count

    prefs.putString(i, msg.substring(5, msg.indexOf(" ", 5)));    //  put alias into nvs with ASCII index this will overwrite peers when ASCII rolesover

    uint32_t hkdfbuff[32]; hkdf<SHA256>( hkdfbuff, 32, msg.substring(msg.indexOf(" ", 5)+1).c_str(), msg.substring(5).length(), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 32 bytes as secret for encryption hkdf<SHA256>(outputbuff, sizeof(output), secret, sizeof(secret), salt, sizeof(salt), info, sizeof(info));

    strcat(i, "hkdf");    //  adds hkdf to peer

    prefs.putBytes(i, hkdfbuff, sizeof(hkdfbuff));    //  store hkdf result in nvs under 'index + hkdf'
    feedlog("added '" +  msg.substring(5, msg.indexOf(" ", 5)) + "' with '" + msg.substring(msg.indexOf(" ", 5)+1) + "'"); return;

    //uint8_t aliasbuff[15]; hkdf<SHA256>( aliasbuff, 14, msg.substring(5).c_str(), msg.substring(5).length(), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 14 bytes from secret for nvs alias and leave one byte for specifing associated information like nvsaliasP for profile foto or nvsaliasH for encryption hkdf


    // TODO replace nvsalias with user input eg peer kenny secret.... but check for length < ?? chars!!
    //      then make struct with name[??], hkdf[32], latest[150000], profile[15000] for kenny and save this in nvs with key "number of peers+1"
    //      update number of peers (or just iterate over keys starting form 0 until number is not found wich means this is next peer)

    //struct peerstct  { char name[16]; char hkdf[16]; char latest[16]; char profile[16]; char slots[5][16]; };    //  this is acts as a lut to avoid String concatination stuff but is this really better

   //prefs.getBytes( 1 , structbuff, len);    //  get peer struct wich has all the keys for this peers hkdf, latest, profile, name each with is a char[16]
   //prefs.getBytes( structbuff."value" , buf, bufLen);    //  get the actual value form the key

    //String nvsalias = ""; for (size_t i = 0; i < 14; i++) {    //  nvs only allowes alphanumeric perhaps hex encoding is better since this has distribution bias but out of hkdf this should fine pls say if not
    //    nvsalias += (char)((aliasbuff[i] % 26) + 'a');
    //}
    
    //prefs.putString("peers", prefs.getString("peers", "local") + " " + nvsalias + " ");    //  here the trailing space is to find last peer correctly in showTas add new peer to peers list in preferences

    // obsolete now i guess  String peers = prefs.getString("peers", ""); prefs.putString("peers", (peers == "") ? nvsalias : peers + " " + nvsalias);    //  add new peer to peers list in preferences
    
    //prefs.putBytes((nvsalias + "H").c_str(), hkdfbuff, sizeof(hkdfbuff));    //  store hkdf result in nvs under 'nvsaliasH'
    //feedlog("added secret '" + msg.substring(5) + "' with alias '" + nvsalias + "'"); return;



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
    feedlog("'restart' to init upgrade with '" + msg.substring(12) + " '\n" ); prefs.putString("airlink", msg.substring(12)); return;
    //feedlog("firmware link " + msg.substring(12) + " '\n" ); tryair( msg.substring(12) ); return;
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

  WebSerial.onMessage([](const std::string& msg) { recv(msg.c_str()); });    //  attach message callback

  WebSerial.begin(&server);    //  init webserial


  // TODO reinstante this and pass slot count +1 here so user always can add fotos
  //server.on("/querySlots", HTTP_GET, [](AsyncWebServerRequest *request) {
  //server.on("/queryPeers", HTTP_GET, [](AsyncWebServerRequest *request) {
  //  request->send(200, "text/plain", "profile 0 1 2 3 4 5 6 7");    //  send slot list
  //});


  server.on("/file", HTTP_POST,
    [](AsyncWebServerRequest* request) {},    // empty request handler - no response sent
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    static size_t totalSize = 0;    //  static so this is not reset on each chunck
    //static String destination = "";    // static to persist across chunks  // TODO rm
    static char slot[12];    // static to persist across chunks this max is 'profile'
    static uint8_t rcvbuff[15000];    // static buffer allocated once

    if (!index){
      totalSize = request->header("Content-Length").toInt();
      //destination = request->getParam("slot")->value();
      //destination = request->getParam("peer")->value();
      strncpy(slot, request->getParam("slot")->value().c_str(), 8);    //  max copy eight chars for 'profile' here
      feedlog("file is for slot " + String(slot));
    }
    if (len + index > sizeof(rcvbuff)) {
      feedlog("aw thats to grande for me"); return;    //  this is to prevent buffer overflow
    }
    else if (len) {
      feedlog("file " + filename + " " + String(index + len) + "/" + String(totalSize) + " bytes\r\n");
      memcpy(rcvbuff + index, data, len);    //  copy data to volatile buffer
    }
    if (final){    //  just save the recieved buffer to nvs
      prefs.putBytes( (strcmp(slot, "profile") ? strcat(slot, "slot") : "0profile") , rcvbuff, sizeof(rcvbuff));    //  save profile to 'local profile' or save foto to 0-7 for foto slots
      feedlog("file saved to " + String(slot));

      // TODO when prefs.get slotcount < slot then update slotcount to new value

      //xQueueSend(sendmqttQueue, ("sendp " + destination).c_str(), 0);    //  send personal profile to peer
      //xQueueSend(sendmqttQueue, ("sendv " + destination).c_str(), 0);    //  send preped volatile buffer to peer
    }
  });


  server.onNotFound([](AsyncWebServerRequest* request) {    //  redirect all requests to webserial for captive portal request->redirect("/webserial"); does not work for captive portal
    request->send(200, "text/html", "<!DOCTYPE html><html><meta http-equiv='refresh' content='0; url=http://fpaper.local/webserial' /><head><title>Captive Portal</title></head><body><p>auto redirect failed http://" + WiFi.softAPIP().toString() + "/webserial </p></body></html>");
  });
  server.begin();
  if (MDNS.begin("fpaper")) { feedlog("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
}


TaskHandle_t flanksTasHandle;
void flanksTas(void *parameter) {    //  this is hopefully the same as using this lib in default Asynchronous

//InterruptButton belowus(20, LOW);    //  default longpress is 750ms
//InterruptButton belowus(20, LOW, GPIO_MODE_INPUT, 420);    //  why does this not work inside initflanks
//InterruptButton belowus(2, LOW, GPIO_MODE_INPUT, 420);    //  TODO reinstate above this is ony for testig we have her pin 2
  InterruptButton belowus(2, LOW, GPIO_MODE_INPUT, 750, 250, 2000, 8000);    //  pin, pressed low, pin mode, longpress ms, autorepeat ms, doubleclick ms, debounce us
  InterruptButton::setMode(Mode_Synchronous);    // defaults to async wich executes immediate like an ISR, Synchronuse has to have a loop, hybrid does up/down events async and rest synchronous
//void initflanks() {

  static const uint16_t slotcount = prefs.getUShort("slotcount", 8);    //  TODO replace with prefs.getUShort("slotcount", 4); and also update site accordingly in the future

  static char prep[] = {slotcount, '\0'};    //  initially perp last slot so first increment shows peer
  static char currpeer[] = "0";    //  initially peer is local

  belowus.bind(Event_KeyDown, []() {    //  for each press prep the stuff to do
    prep[0] = ++prep[0] % (slotcount+2);    //  cycle trough eight slots plus one for current peer           
    if (prep[0] > slotcount) { xQueueSend(showQueue, &(struct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }){ "user", 1, strcat(currpeer, "profile") }, 0); }    //  show current peer with picture in picture onece every full cycle
    else {             xQueueSend(showQueue, &(struct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }){ "user", 1, strcat(prep, "slot")        }, 0); }    //  show foto slot with picture in picture
  });


  belowus.bind(Event_KeyPress, [](){    //  this is called after double click timeout so actually do the stuff here
    if (prep[0] > slotcount) {    //  when prep is a peer
      currpeer[0] = ++currpeer[0] % (prefs.getUShort("peercount", 0) + 1) ;    //  advance peer or wrap
      xQueueSend(showQueue, &(struct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }){ "user", 1, strcat(currpeer, "profile") }, 0);    //  show the advanced peers profile with picture in picture
      xQueueSend(showQueue, &(struct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }){ "user", 0, strcat(currpeer, "latest")  }, 0);    //  show current peers latest foto with full refresh
    } 
    else {    //  when perep is a foto slot
      xQueueSend(sendQueue, &(struct { char peer[16]; char load[16]; }){  currpeer, "0profile" }, 0);    //  first send our profile to current peer
      xQueueSend(sendQueue, &(struct { char peer[16]; char load[16]; }){  currpeer, strcat(prep, "slot") }, 0);    //  then send prepped foto slot to current peer
    }
    prep[0] = slotcount;    // reset prep so next time current peer shows up with first press
    xQueueSend(showQueue, &(struct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }){ "user", 0, strcat(currpeer, "latest")  }, 0);    //  show current peers latest foto with full refresh
  });

  belowus.bind(Event_DoubleClick, [](){ });    //  this has to be registered so the lib respects the doubleclick timeout

  while(true){
    InterruptButton::processSyncEvents();    //  only here for synchronous or hybrid
    vTaskDelay(1);
  }



  /*
    if (prep != currpeer)    //  when already in sendscreen or prep is not equal to currpeer only happens when we already are in sendscreen
      // this assumes char '0' to char '8' are slots and after that are peers






    // this blow could work i gues to iterate over slots and currentpeer

    //  for down event    this starts with 7 goes to 8/local then goes 0/slot - 7/slot and so on... and every loop this shows profile of current peer
    prep[0] = ++prep[0] % (currentpeer + 1);    //  wrap with currentpeer + one to show profile of current peer
    if (prep[0] == 8) prep[0] = prep[0] + currentpeer - 8;    //  for '8'/'local' this adds 0 so prep stays '8'/'local' for peer '9' this adds one
    xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){1, 1, prep}, 0);    //  show the preped slot in overlay

    //  for timerup event
    if (prep[0] < 8) // send slot to currentpeer
    if (prep[0] > 8) // make nextpeer the currentpeer then reset prep[0] = currpeer-1;







    
    //  for down event click this starts with 7 goes to 8/local then goes 0/slot - 7/slot and so on...
    prep[0] = ++prep[0] % 9;    //  wrap with max slot + one to show profile of current peer
    xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){1, 1, prep}, 0);    //  show the preped slot in overlay
    
    //  for timerup event
    if (prep[0] < 8) // send slot to currentpeer
    if (prep[0] > 8) // make nextpeer the currentpeer





    if ( sendscreen) {    //  when already in sendscreen this preps next slot to be sent after double click timeout

      //   ------- TODO ---------
      //   restructure async queses to use structs and TRY TO REMOVE STRINGS where possible eg prep should be char[] or string (not String) i think
      //   build a timer to shutoff webpage after a time eg. server.end() MDNS.stopp() webserial.end() and so on
      //   package falnk into its own task perhaps this is not a good idea but it seems more consice with the rest of this and likely the lib does the same thing i think perhaps chekc this and perhaps use hybrid mode instead of syncronous wit own rtos task



      xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){1, 1, prep}, 0);    //  show the preped slot in overlay



      prep = "slot" + String(prep.substring(4).toInt() + 1);    //  prep slot to be sent when sendscreen timout runs out
      if (prep != "slot6" ) {

        //struct showstct { uint8_t sendscreen = 0; uint8_t partial = 0; char nvsalias[15] = ""; };
        //xQueueSend(showQueue, &(showstct){1, 1, prep}, 0);

        xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){1, 1, prep}, 0);
        
        //xQueueSend(showQueue, ("part" + prep).c_str(), 0); 
        return; }    //  show slot in overlay and return eraly aslong as max slot is not reached
    }

    if (!sendscreen || prep == "slot6") {    //  when not in sendscreen or last slot is reached this preps the next peer and resets slot
      String peerString = prefs.getString("peers", "local");
      if (peerString.indexOf(currpeer)+currpeer.length()+1+1 > peerString.length()) prep = "local";   //  account for trailing space here test for last peer and wrap
      else prep = peerString.substring( peerString.indexOf(currpeer)+currpeer.length()+1, peerString.indexOf(' ', peerString.indexOf(currpeer)+currpeer.length()+1) );    //  advance to next peer in list so this is the next peer or local when no peers set


      xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){1, 1, currpeer + "P"}, 0);
      //xQueueSend(showQueue, ("part" + currpeer + "P").c_str(), 0);    //  show overlay of currentpeer profile
      if (prep == "slot6") prep = "slot0";
    }

    sendscreen = 1;  //  TODO make this a local var in showtas
  });

  // TODO test if this resets for multible key presses or if this fires multible times
  belowus.bind(Event_KeyPress, [](){    //  this is called after double click timeout so actually do the stuff here

    Serial.println("timeout reached will do stuff now");

      if (prep.indexOf("slot") == 0) {    //  when slot in prep then send slot to current peer
        if (!prefs.getBytes(prep.c_str(), volatileBuff, 15000 )) { Serial.println("timer up found garbage for " + prep); esp_fill_random(volatileBuff, sizeof(volatileBuff)); }    //  for invalid nvs lookups this returns null and leaves showBuff

        xQueueSend(sendmqttQueue, ("sendp " + currpeer).c_str(), 0);    //  send profile to currpeer
        xQueueSend(sendmqttQueue, ("sendv " + currpeer).c_str(), 0);    //  send prepared buffer to currpeer
      }

      else {    //  everything else is just a peer change so make prep the currpeer and display currpeer
        prep = currpeer;

        xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){0, 1, currpeer + "P"}, 0);
        xQueueSend(showQueue, &(struct { uint8_t sendscreen; uint8_t partial; char nvsalias[15]; }){0, 0, currpeer + "L"}, 0);

        //xQueueSend(showQueue, ("part" + currpeer + "P").c_str(), 0);    //  queue 'P'rofile picture of peer    for now first show profile of next peer then do a full refresh with 'L'atest foto of next peer to also clear the partial overlay
        //xQueueSend(showQueue, ("full" + currpeer + "L").c_str(), 0);    //  queue 'L'atest foto of peer
      }

      sendscreents = 0;    //  free the timer the zero value also means user not in sendscreen   //  TODO make this a local var in showtas
  });
  */




  /*
  belowus.bind(Event_KeyPress, [](){    //  feedlog inside here does chrash perhaps this is 'm_RTOSservicerStackDepth' see here https://github.com/rwmingis/InterruptButton/tree/main?tab=readme-ov-file#known-limitations

    if ( sendscreents) {    //  when already in sendscreen this preps next slot
      prep = "slot" + String(prep.substring(4).toInt() + 1);    //  prep slot to be sent when sendscreen timout runs out
      if (prep != "slot6" ) { xQueueSend(showQueue, ("part" + prep).c_str(), 0); return; }    //  show slot in overlay and return eraly aslong as max slot is not reached
    }

    if (!sendscreents || prep == "slot6") {    //  when not in sendscreen or last slot is reached this preps the next peer and resets slot
      String peerString = prefs.getString("peers", "local");
      if (peerString.indexOf(currpeer)+currpeer.length()+1+1 > peerString.length()) prep = "local";   //  account for trailing space here test for last peer and wrap
      else prep = peerString.substring( peerString.indexOf(currpeer)+currpeer.length()+1, peerString.indexOf(' ', peerString.indexOf(currpeer)+currpeer.length()+1) );    //  advance to next peer in list so this is the next peer or local when no peers set

      xQueueSend(showQueue, ("part" + currpeer + "P").c_str(), 0);    //  show overlay of currentpeer profile
      if (prep == "slot6") prep = "slot0";
    }

    unsigned long m = millis(); sendscreents = m ? m : 1;    //  catch unicorn case where millis is zero since sendscreen state is tracked with nonzero value

    //Serial.println("current peer is " + currpeer + " now queueing currpeer p and L");    //  print current peer to serial
    //xQueueSend(showQueue, ("full" + currpeer + "L").c_str(), 0);    //  queue 'L'atest foto of peer
    //xQueueSend(showQueue, ("part" + currpeer + "P").c_str(), 0);    //  queue 'P'rofile picture of peer

  });
  */

  //belowus.bind(Event_DoubleClick, [](){
  //  xQueueSend(sendmqttQueue, ("sendp " + currpeer).c_str(), 0);    //  just annoy peer with profile
  //});
}


//Preferences prefs;    //  commented so no redfinition error
void setup() {
  Serial.begin(115200);    //  serial requires delay or while(!Serial); so no output is lost
  prefs.begin("prefs", false);    //  open preferences with namespace prefs in read write mode this is for wifi creds and stuff

  xTaskCreate( showTas, "showTas", 32768, NULL, 1, &showTasHandle );    //  spawn show task to show stuff on epaper

  initWebSerial();   //  init wifi and webserial this is blocks until wifi is up

  tryair(prefs.getString("airlink", ""));    //  TODO this should be a command thing to an auto thing try to upgrade firmware from hardcoded url fails in ap mode this blocks aswell

  xTaskCreate( servoTas, "servoTas", 4096, NULL, 1, &servoTasHandle );    //  now spawn async tasks

  xTaskCreate( sendmqttTas, "sendmqttTas", 32768, NULL, 1, &sendmqttHandle );    //  spawn mqtt message sender task apparently task has to have enough stack for every buffer so here > 15KB

  xTaskCreate( flanksTas, "flanksTas", 4096, NULL, 1, &flanksTasHandle );    //  spawn flanks task to handle presses  // TODO make stackdepth smaller perhaps
  //initflanks();    //  this is asnyc per lib so no xTaskCreate nessesary


  //initmqtt();    //  init mqtt this is asnyc per lib so no xTaskCreate nessesary

  feedlog("init done");

  //delay(500);

  xQueueSend(showQueue, "fulllocalL", 0); // WHY DOES THIS NOT WORK????

  // TODO add a boot screen of some sort currently the showTas does not support this 
  //memcpy_P(volatileBuff, epd_bitmap_xpwallp, 15000);    //  copy boot foto from PROGMEM to volatile buffer for fast access
  //xQueueSend(showQueue, "showboot", 0);    //  add volatile foto to show queue
}

void loop() { }