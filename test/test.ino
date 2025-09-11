
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


      //  this hard coded finite length stresses me in python me no have to worry me miss python


//#include <Arduino.h>    // all this is arduino for an esp32    so compared to c some delacrations are missing but im not sure 
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


Preferences prefs;    //  first declaration of preferences as perfs

TaskHandle_t flanksTasHandle;


// ----------- TODO -----------
//
//  -> combine feedlog() + recv() + tryair() + watermarktask() + WebSerialInitialization into one task
//     ( auto updateds via tryair() can be done here too )
//
//  -> give wifi its own task including and dnsservtas()
//     eitehr keep it running while ap mode for dnsservtas() or suspend it when not in ap mode
//



TaskHandle_t showTasHandle;
QueueHandle_t showQueue;
struct showstct { char ocupado[5]; uint8_t partial; char nvsalias[16]; };    //  handle and struct for show queue
void showTas(void *parameter) {    //  this handles the epaper
  showQueue = xQueueCreate(5, sizeof(showstct));    // create queue with buffer of 5

  static GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(/*CS=D8*/ 45, /*DC=D3*/ 46, /*RST=D4*/ 47, /*BUSY=D2*/ 48));
  //char buff[20] = "";    //  buffer to read from queue has length of 15 nvs chars plus 4 for full/part

  pinMode(7, OUTPUT); digitalWrite(7, HIGH);   //  give power to the panel
  display.init(115200);    // init epd with 115200 baud rate
  display.setRotation(0);    //  TODO make this a setting in preferences but also change selection/ditthered overlay aspect accordingly

  static uint8_t showBuff[15000];
  char ocupado[5];    //  save the screen state either user or empty

  while(true){
    //if (!xQueueIsQueueEmptyFromISR( showQueue )){    //  just do sth when queue not empty
    if (!uxQueueMessagesWaiting( showQueue )){
      showstct show; xQueueReceive(showQueue, &show, 0);

      if ( ocupado[0] && strcmp(ocupado, show.ocupado) && strcmp(show.ocupado, "user") ) { xQueueSend(showQueue, &show, 0); continue;
        Serial.println("did a pushback");
      }    //  this passes when no ocupado, user requests, requests with same occupation  everything else is pushed back into queue

      strcpy(ocupado, show.partial ? show.ocupado : "");    //  clear ocupation for full refreshes for partial refreshes ocupie screen

      if (!prefs.getBytes( show.nvsalias, showBuff, 15000 )) { Serial.println("nothing found for " + String(show.nvsalias)); esp_fill_random(showBuff, sizeof(showBuff)); }    //  for invalid nvs lookups this fills the showBuff with noise

      //if (strncmp(buff, "full", 4) == 0) {    //  show with full refresh
      if (!show.partial) {    //  show with full refresh
        display.setFullWindow();
        display.firstPage();
        do {
          display.fillScreen(GxEPD_BLACK);
          display.drawBitmap(0, 0, showBuff, display.width(), display.height(), GxEPD_WHITE);
        } while (display.nextPage());
      }

      if ( show.partial) {    //  show picture in picture (center 100x100 of currently loaded showBuff)    //  check display.epd2.hasFastPartialUpdate with other screens
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
    //vTaskDelay(1);    //  befor one second so no flicker just show every thing fast user interactions
    taskYIELD();    //  more efficent vTaskDely(0)

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
  char buff[] = "sit";    //  why does char buf[4] error help
  
  while(true){
    //if(!xQueueIsQueueEmptyFromISR( servoQueue )){
    if(!uxQueueMessagesWaiting( servoQueue )){
      xQueueReceive(servoQueue, &buff, 0);    //  just do sth when queue not empty
      //ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); String(buf) == "top" ? ledcWrite(38, prefs.getInt("top", 0)) : ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0);    //  move servo to poses in preferences also cool c ternary operator
      if (!strcmp(buff, "top")) { ledcWrite(38, prefs.getInt("top", 0)); vTaskDelay(500); ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }   //  wigle servo to poses in preferences always top and back to sit pose
      if (!strcmp(buff, "sit")) { ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }  // move servo to sit pose
    }
    //vTaskDelay(1);
    taskYIELD();
  }
}




TaskHandle_t sendmqttHandle;
QueueHandle_t sendmqttQueue;
struct sendstct { char peer[16]; char load[16]; };
void sendmqttTas(void *parameter) {    //  this handles outgoing mqtt messages

  sendmqttQueue = xQueueCreate( 5, sizeof(sendstct) );    // create queue with buffer of 5

  PsychicMqttClient mqttClient;
  ChaChaPoly chachapoly;

  static uint8_t curriv[12];    //  this is written every outgoing message and read with every incoming so perhaps protect this with mutex/semaphore 

  String serverAddress = prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work

  //static bool seenecho;    //  otherwise echo rejection is racey  todo rewrite this with proper portMUX and a ring with timestaps to allow multiple messages and invalidate old ivs currently this only allows one message and waits for its echo


  static size_t size = prefs.getBytesLength("peers");    //  size of peer list this is to clac peer count

  static char (*peers)[16] = (char (*)[16]) malloc( size );    //  allocate memory for peer list

  prefs.getBytes("peers", peers, size);    //  read peer list into memory


  mqttClient.onTopic( prefs.getString("mqtop", "fpaper/").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer      wildcards should work here listen one level deep for now TODO change this to only subscribe to peers
    

    //  ------------ TODO -----------
    //  instead of filtering echos with curriv here perhaps add currive as mqtt topic and unsub from this topiv until next send
    //  potential issue is unsbub may takes some time
    //                  how would i resub
    //  this does not seem to be a good solution
    //
    //  i really want to avoid setting a permanent sender id so the conversation stays anonymous yes server has id but every other client does not get this id
    //
    //  perhaps collect ivs in a black-list and filter with this
    //    seems wastefull and also time consuming
    //
    //  also the curriv method is racey when sending multiple messages in a short time


    // ZERO CURRIV HERE!! does this even make sense? null iv still is a valid iv i guess so is as likely as any other iv
    //if ( !memcmp(curriv, payload, 12) ) { seenecho = true; memcpy( curriv, NULL) return;}    //  ignore echos just listen to messages of our peers no sens to decode echos  TODO implement some check to avoid mitigate spam here eg some chek for known phrase or sth
    

    if ( !memcmp(curriv, payload, 12) ) {  xTaskNotifyGive(sendmqttHandle); return;}    //  ignore echos no sens to decode echos  echos free the send task early  this seems racey but while publish there is no echo befor publish   TODO implement some check to avoid mitigate spam here eg some chek for known phrase or sth


    feedlog("got message start decoding");    //  TODO make this debug
    //char peer[16] = "0hkdf";    //  initally start with local hkdf so while trying all peers just increment the '0' literal

    //static char (*peers)[16] = NULL;    //  this is a pointer to an array of char arrays with length 16

    static uint32_t index;    //  static initialises to zero also remember last successful peer index

    //uint8_t retry = 1;    //  allow retry for every call

    // ------------ TODO -----------
    //  optimization possibly  reload peer list only after we tried all peers and found no sender
    //                         remember last successful peer and try that first
    //                         both of the above dont neccessarily make sense together !!
    
    //char peercount = prefs.getUChar("peercount");
    //static size_t size;    //  this is size of the peer list to calculate the count of peers

    static uint8_t rcvhkdf[32];    //  static to try message with previous successful peer first but this also cuases decryption to fail at first call
    uint8_t iv[12];  memcpy(iv, payload, 12);
    uint8_t rcvtag[16];   memcpy(rcvtag, payload + 12, 16);
    static uint8_t rcvcyphy[15000];    //  this is a bit large for stack so this is static perhaps better malloc and free int8_t* cyphy = (uint8_t*)malloc(15000);
    static uint8_t temp[15000];     //  see comment abouveus

    feedlog("Received message on topic: " + String(topic) );     //  TODO make this a debug log

    //while (peer[0] < peercount) {    //  iterate over all peers to find whos sender
    //while ( retry || index ) {    //  iterate over all peers to find whos sender plus allow one complete retry with reloaded peer list
    //for (uint8_t attempts = 0; attempts < (size/16 - 1); attempts++) {    //  iterate over all peers to find whos sender
    //for (uint8_t attempts = 1; attempts < (size/16) + 1; attempts++) {    //  iterate over all peers to find whos sender
    for (uint8_t attempts = (size/16); attempts; attempts--) {       // a =  4, 3, 2, 1
                                                                   // key =  x, 3, 2, 1
    
      //for (uint8_t attempts = 0; attempts < (size/16); attempts++) { // a  = 0, 1, 2, 3
                                                                    // key = x, 0, 1, 2 
                                                               // (new)key = x, x, 1, 2
                                                            // (newnew)key = x, 1, 2, 3
      
      // ----------- TODO ----------- this for loop and the on failure handleing have to be rethought! perhaps do stuff on success instead and revise the logic
      
      
      //peer[0]++;    //  peer initialises to '0hkdf' so just increment pos 0 here

      /*
      if (!index) {    //  reload peer list when no sender found after trying all peers
    
        //reload peer list    peers[16][] =  {{"peer0"},{"peer1"},{"peer2"},{} ... }
        //free(peers); peers = NULL;    //  free memory for peer list and set pointer to null so malloc works
        //size = prefs.getBytesLength("peers"); peers = (char (*)[16]) malloc( size );
        //if (!peers) { feedlog("failed to allocate memory for peers\n"); return; }    //  check if malloc worked

        size = prefs.getBytesLength("peers"); peers = (char (*)[16]) realloc( peers, size );    //  preseving old peers is useless here so just directly assign yes this leaky but next line errors out
        if (!peers) { feedlog("failed to allocate memory for peers\n"); ESP.restart(); }    //  error out

        prefs.getBytes("peers", peers, size);    //  read peer list into memory
        retry=0;    //  just one retry per message otherwise infinite loop
      }
      */


      //prefs.getBytes(peer, rcvhkdf, 32);    //  read incremented peer hkdf into buffer

      memcpy(rcvcyphy, payload + 12 + 16, 15000);    //  load original message every try this is located after iv and tag and is 15000 bytes long

      chachapoly.setIV(iv, 12);                                feedlog(" set iv");    //  TODO make all this debug logs
      chachapoly.setKey(rcvhkdf, 32);                             feedlog(" set key");
      chachapoly.decrypt(rcvcyphy, rcvcyphy, 15000);                 feedlog(" decrypted cypher text");

      if (!chachapoly.checkTag(rcvtag, 16)) {
        //index = ++index % ((size/16) + 1);    //  when decryption fails increment peer
        //index = ++index % (size/16);    //  when decryption fails increment peer
        //index++; if (index > (size/16) - 1) index = 1;    //  when decryption fails increment peer

        //if(!attempts) index = 1;
        //else index = attempts +1;

        //index = (attempts) ? attempts+1 : 1;

        index = attempts -1;

        //index = attempts;

        //if ( index < (size/16) ) prefs.getBytes(peers[index], rcvhkdf, 32);    //  only read incremented peer hkdf for valid indices
        if ( index ) prefs.getBytes(peers[index], rcvhkdf, 32);    //  read next peer hkdf.  when no peer matches this keeps first peer hkdf in memory but whatevs
        continue;    //  retry
      }
      
      if ( !memcmp("look here", payload + 12 + 16 + 15000, 9) ) {    //  here compare recieved profile to saved profile and perhpas overwrite    also show recieved profile    also move servo 
        //prefs.getBytes( strcpy(&peer[1], "profile"), temp, 15000 );    //  peer char array here still is  'index+hkdf' so make it 'index+profile' here and read the profile into temp to compare with message
        //prefs.getBytes( strcat( peers[index], "profile" ), temp, 15000);    //  this permanently adds 'profile' 
        sprintf(peers[0], "%sprofile", peers[index]); prefs.getBytes( peers[0], temp, 15000 );    //  use peer zero as scratch buffer here to read the profile of peer wich is at 'index+profile'

        feedlog("first decryption successfull");

        if ( memcmp(temp, rcvcyphy, 15000) ) prefs.putBytes( peers[0], rcvcyphy, 15000 );    //  when profile changes save recieved profile to peer wich is 'index+profile' see abouve

        struct showstct show={ "", 1, "" }; strcpy(show.ocupado, peers[index]); strcpy(show.nvsalias, peers[index]); xQueueSend(showQueue, &show, 0);    //  show recieved profile with picture in picture and occupie screen with sending peer so no other message interferes

        xQueueSend(servoQueue, "top", 0);    //  move servo to top position this wiggles screen
      }

      if ( !memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here save recieved foto to nvsalias+'L'    also show this
        //prefs.putBytes( (String(topic).substring(7) + "L").c_str(), cyphy, 15000 );    //  save foto to nvsalias+'L' so we can show it later
        //prefs.putBytes( strcpy(&peer[1], "latest"), rcvcyphy, 15000 );    //  save foto of peer wich is 'index+latest'
        sprintf(peers[0], "%slatest", peers[index]); prefs.putBytes( peers[0], rcvcyphy, 15000 );    //  use peer zero as scratch buffer here to save foto of peer wich is 'index+latest'
        
        feedlog("second decryption successfull");
        
        struct showstct show={ "", 0, "" }; strcpy(show.ocupado, peers[index]); strcpy(show.nvsalias, peers[index]); xQueueSend(showQueue, &show, 0);    //  show recieved latest foto with full refresh to clear profile this also frees occupation
      }

      chachapoly.clear(); return;    //  exit lambda
    }
  });

  mqttClient.connect();

  uint8_t sendhkdf[32];
  static uint8_t sendcyphy[15000];    //  this hold load wich then gets encrypted and sent
  uint8_t sendtag[16];

  while (true) {

    if ( ulTaskNotifyTake(pdTRUE, 0) ) { //reload peer list when notified by other tasks
      //free(peers); peers = NULL;    //  free memory for peer list so it is reallocated with press

      size = prefs.getBytesLength("peers"); peers = (char (*)[16]) realloc( peers, size );    //  preseving old peers is useless here so just directly assign yes this leaky but next line errors out
      if (!peers) { feedlog("failed to allocate memory for peers\n"); ESP.restart(); }    //  error out

      prefs.getBytes("peers", peers, size);    //  read peer list into memory


      // ---------- TODO -------------
      //  also load all hkdfs into memory for decryption in .ontopic here this would be faster but also use more memory
    }



    //if(!xQueueIsQueueEmptyFromISR( sendmqttQueue )){    //  just do sth when queue not empty
    if(!uxQueueMessagesWaiting( sendmqttQueue )){    //  just do sth when queue not empty


      //if (!seenecho) { feedlog("no wait for echo"); continue;}    //  do not pop queue when last messages echo was not seen do not push back into queue to preserved sequence   // optimization this technicaly doe not have to wait on local sends

      sendstct send; xQueueReceive(sendmqttQueue, &send, 0);    //  reads first word out of queue

      Serial.println("sending to " + String(send.peer));

      if (!prefs.getBytes( send.load, sendcyphy, 15000 )) { Serial.println("nothing found for " + String(send.load)); continue; }    //  for invalid nvs lookups this returns null and leaves cyphy

      //if ( !(send.peer[0]-'0') ) {    //  local is "0" so falsy
      if ( !strcmp(send.peer, "local") ) {    //  when recipient local just save to local latest
        prefs.putBytes( "locallatest", sendcyphy, sizeof(sendcyphy));    //  when recipient local save to local latest
      }

      //if ( send.peer[0]-'0' ) {    //  here when recipient not local actually do send stuff either answer to look here with profile or send profile plus volatileShow    // TODO somehow dont send full profile everytime you want to annoy
      else {        //  here when recipient not local actually do send stuff
        uint8_t *payload = (uint8_t*)malloc(sizeof(curriv) + sizeof(sendtag) + sizeof(sendcyphy) + 9);    //  allocate memory for payload
      
        esp_fill_random(curriv, sizeof(curriv));    //  fill curriv with noise here this only is to later in recieve mqtt determine wether message is a echo
        //seenecho = false;    //  reset echo flag

        prefs.getBytes( strcat(send.peer, "hkdf") , sendhkdf, 32);    //  find the peer hkdf

        chachapoly.setIV(curriv, 12);
        chachapoly.setKey(sendhkdf, 32);
        chachapoly.encrypt(sendcyphy, sendcyphy, 15000);    //  encrypt clear bytes of load this was loaded into cyphy befor

        chachapoly.computeTag(sendtag, 16);
        chachapoly.clear();

        memcpy(payload, curriv, sizeof(curriv));    //  pack payload with first iv
        memcpy(payload + sizeof(curriv), sendtag, sizeof(sendtag));    //  then tag
        memcpy(payload + sizeof(curriv) + sizeof(sendtag), sendcyphy, sizeof(sendcyphy));    //  then foto
        memcpy(payload + sizeof(curriv) + sizeof(sendtag) + sizeof(sendcyphy), strcmp(send.load, "0profile") ? "look here" : "see this ", 9);    //  send our profile with 'look here' appendix or send foto slot with 'see this'    TODO send hash of peers profile to minimize messages

        Serial.println("packed payload try sending now to " + String(send.peer));    //  TODO make this a feedlog message

        mqttClient.publish( prefs.getString("mqtop", "fpaper/").c_str() , 0, 0, reinterpret_cast<const char*>(payload), 12 + 16 + 15000 + 9, true);     //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer       publish full length message to base topic

        free(payload);


        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(4000)) == 0) { feedlog("seen echo or timeout over\n"); }    //  this releases the task until notivied or timeout runs out
        else { feedlog("timeout waiting for echo\n"); }    //  TODO make this a debug log

      }

    }
    taskYIELD();
    //vTaskDelay(1);    // just send every two second so we have enugh time to filter out our echos with curriv
                      // TODO change this back to two seconds
                      // but with a larege delay local sends do not get processed fast enough and when user send sth to himself alias cahnges his lates foto
                      //  then the show task is executed first and shows the old latest foto instead of the new one
  }
}




TaskHandle_t dnsServHandle;
void dnsServTas(void *parameter) {    //  this is the dns response task this only is called in ap mode
  DNSServer dnsServer;
  dnsServer.start(53, "*", WiFi.softAPIP());    //  init dns server on port 53 with wildcard domain to map all requests to ap ip for captive portal
  while(true){
    dnsServer.processNextRequest();
    feedlog("dns for ap mode", "debug");
    //vTaskDelay(10);
    taskYIELD();
  }
}




void tryair(String airlink) {    //  this works with redirects and insecure https source 'https://github.com/espressif/arduino-esp32/issues/9530#issuecomment-2090034699' improve this with checking here 'https://api.github.com/repos/crbyxwpzfl/mini/releases/latest' or 'https://api.github.com/repos/crbyxwpzfl/mini/tags' befor download and then use 'https://github.com/crbyxwpzfl/mini/releases/latest/download/adafruit-feather-esp32s3-4flash-2psram.bin'
  WiFiClientSecure secureClient;
  HTTPUpdate up;

  if( airlink ) {    //  only do this when airlink has value
    prefs.putString("airlink", "");    //  disable airlink for next boot
    //String airlink = prefs.getString("airlink", "https://github.com/crbyxwpzfl/mini/releases/download/v9/adafruit-feather-esp32s3-4flash-2psram.bin"); prefs.putString("airlink", "https://github.com/crbyxwpzfl/mini/releases/download/v9/adafruit-feather-esp32s3-4flash-2psram.bin" );  //  usually try fixed link or try custom link only once
    secureClient.setInsecure();    //  this is to ignore ssl so theoretically some one can spoof github this is not good 
    up.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);    //  this is to follw link redirects other options are eg 'up.rebootOnUpdate(false);' or 'secureClient.setTimeout(5);'
    up.onStart([]() { feedlog("overwrite firmware init download \n"); });
    up.onEnd([]() { feedlog("firmware download success so restart to overwrite \n"); });
    up.onError([&up](int err) { feedlog(  up.getLastErrorString() + " \n"); });
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




void recv( String msg ){    //  this uses string likely char array is better see https://github.com/asjdf/WebSerialLite/blob/545465b009a06a4a7d2da4247c9af2a821391beb/examples/demo/demo.ino#L27
  if ( msg.indexOf("help") >= 0 ) {
    String peerstring = "";
    
    size_t size = prefs.getBytesLength("peers"); char (*peers)[16] = (char (*)[16]) malloc( size );
    prefs.getBytes("peers", peers, size);    //  read peer list
    if (!peers) { feedlog("failed to allocate memory for peers\n"); return; }    //  check if malloc worked
    for (int i = 0; i < size/16; i++) { peerstring += String(peers[i]) + ", "; }    //  iterate over peers and add to peerstring
    free(peers);

    //char i[2] = {'0', '\0'}; while (prefs.isKey(i)) {
      //peerstring += String(i) + "-" + prefs.getString(i, "N.A.") + " ";
      //i[0]++;
    //}

    feedlog("\n \n"
         "\nwhen wlan fails an access point spawns \n"
         " ssid 'ssid'         sets wlan '" + prefs.getString("ssid", "N.A.") + "' \n"
         " pass 'password'     sets password \n"
                                
         "\nmqtt config. tell others to add '" + prefs.getString("publ", String(ESP.getEfuseMac()) ) + "' \n"
         " peer 'name' 'secret' adds peer '" + peerstring + "' \n"
         " serv 'mqtt://url'    sets server " + prefs.getString("mqserv", "mqtt://broker.hivemq.com") + " \n"
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
    feedlog("this is disabled fix this");
    //xQueueSend(sendmqttQueue, msg.substring(5).c_str(), 0); return;
  }
  if ( msg.indexOf("serv ") == 0 ) {
    prefs.putString("mqserv", msg.substring(5)); feedlog("mqtt server set to '" + msg.substring(5) + "'\n"); return;
  }

  // -------- TODO --------- add function to delete slot   just overwrite the slot with the top most slot and update slotcount and restart

  if ( msg.indexOf("peer ") == 0 ) {    //  this adds peer name to nvs with ASCII index so that its easy to iterate over peers also this does hkdf with secret and puts it into nvs with 'peer name + hkdf'


  // --------- TODO-------  when secret empty delete peer   delete all keys for peer like indexhkdf, indexprofile, index, indexlatest!   then move topmost peer to the index of deleted peer to keep iterable structure
  //                        overwrite secret here for already known peers




  //                        DO EVERYTHING WITH LISTS ISTEAD OF ASCII INDEXES
  //                      - but this would add complexety of malloc when ever we have to access the peers
  //                      + would sepperate all other keys for peers like hkdf, profile, latest from the index itself so no need to rewrite them for peer deletion
  //                      - requires more nvs reads one for size and one for data

    size_t size = prefs.getBytesLength("peers"); char (*peers)[16] = (char (*)[16]) malloc( size +16 );    //  allocate memory for peer list

    if (!peers) { feedlog("failed to allocate memory for peers\n"); ESP.restart(); }    //  check if malloc worked

    prefs.getBytes("peers", peers, size);    //  read peer list

    strncpy(peers[size/16], msg.substring(5, msg.indexOf(" ", 5)).c_str(), 15);    //  add peer

    //peers[size/16 - 1] = msg.substring(msg.indexOf(" ", 5)+1).c_str();    //  add new peer to end of list
    
    prefs.putBytes("peers", peers, size+16);    //  write back peer list with new peer

    uint32_t hkdfbuff[32]; hkdf<SHA256>( hkdfbuff, 32, msg.substring(msg.indexOf(" ", 5)+1).c_str(), msg.substring(5).length(), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 32 bytes as secret for encryption hkdf<SHA256>(outputbuff, sizeof(output), secret, sizeof(secret), salt, sizeof(salt), info, sizeof(info));

    prefs.putBytes( strcat(peers[size/16], "hkdf"), hkdfbuff, sizeof(hkdfbuff));    //  write back peer list with new peer

    free(peers);    //  free memory for peer list

    feedlog("added '" +  msg.substring(5, msg.indexOf(" ", 5)) + "' with '" + msg.substring(msg.indexOf(" ", 5)+1) + "'"); return;


    xTaskNotifyGive(sendmqttHandle);    //  notify other tasks to reload peer list
    xTaskNotifyGive(flanksTasHandle);

  //                        perhaps its just better to switch to uint32 enumeration this is infinite enough
  //                        then convert to key (char array) with sprintf(buffer, "%u", number);    //  this gives a char array with the number in it to use as key
  //char peerindexchar[10];
  //uint32_t peerindexuint = 0;





   /*
    if (!prefs.isKey("0")) prefs.putString("0", "local");    //  when local peer not found do initialise local here

    //  this limits peer count to dec 48/ASCII 0 to dec 126/ASCII ~     //  theoretically with for esp32 platform this could do dec 0/ASCII NULL to dec 255/ASCII nbsp see here https://forum.arduino.cc/t/char-is-not-signed-no-reference-in-the-documentation/1297470

    char i[] = { prefs.getUChar("peercount", '0') + 1 , '\0'};    //  read current peer count and add one
    prefs.putUChar("peercount", i[0]);    //  put incremented peer count

    prefs.putString(i, msg.substring(5, msg.indexOf(" ", 5)));    //  put alias into nvs with ASCII index this will overwrite peers when ASCII rolesover

    uint32_t hkdfbuff[32]; hkdf<SHA256>( hkdfbuff, 32, msg.substring(msg.indexOf(" ", 5)+1).c_str(), msg.substring(5).length(), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 32 bytes as secret for encryption hkdf<SHA256>(outputbuff, sizeof(output), secret, sizeof(secret), salt, sizeof(salt), info, sizeof(info));

    strcat(i, "hkdf");    //  adds hkdf to peer

    prefs.putBytes(i, hkdfbuff, sizeof(hkdfbuff));    //  store hkdf result in nvs under 'index + hkdf'
    feedlog("added '" +  msg.substring(5, msg.indexOf(" ", 5)) + "' with '" + msg.substring(msg.indexOf(" ", 5)+1) + "'"); return;

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


  server.on("/querySlots", HTTP_GET, [](AsyncWebServerRequest *request) {
    char buff[16]; snprintf(buff, sizeof(buff), "%u", prefs.getUInt("slots", 0) + 1); request->send(200, "text/plain", buff);    //  send slot count plus one so user can add new fotos
  });


  server.on("/file", HTTP_POST,
    [](AsyncWebServerRequest* request) {},    // empty request handler - no response sent
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    static size_t totalSize = 0;    //  static so this is not reset on each chunck
    static char slot[12];    // static to persist across chunks this max is 'profile'
    static uint8_t rcvbuff[15000];    // static buffer allocated once

    if (!index){
      totalSize = request->header("Content-Length").toInt();
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
      //prefs.putBytes( (strcmp(slot, "profile") ? strcat(slot, "slot") : "0profile") , rcvbuff, sizeof(rcvbuff));    //  save profile to 'local profile' or save foto to 0-7 for foto slots
      //feedlog("file saved to " + String(slot));
      
      prefs.putBytes( (strcmp(slot, "profile") ? slot : "localprofile") , rcvbuff, sizeof(rcvbuff));    //  save profile to 'local profile' or save foto to slot
      feedlog("file saved to " + String(slot));
      if (strcmp(slot, "profile") && strtoul(slot, NULL, 10) > prefs.getUInt("slots", 0)) {
        prefs.putUInt("slots", strtoul(slot, NULL, 10));    //  update slot count when new slot added
        xTaskNotifyGive(flanksTasHandle);    //  notify flanks task to reload slots
        feedlog("updated slot count to " + String(prefs.getUInt("slots", 0)) + "\n");
      }
    }
  });


  server.onNotFound([](AsyncWebServerRequest* request) {    //  redirect all requests to webserial for captive portal request->redirect("/webserial"); does not work for captive portal
    request->send(200, "text/html", "<!DOCTYPE html><html><meta http-equiv='refresh' content='0; url=http://fpaper.local/webserial' /><head><title>Captive Portal</title></head><body><p>auto redirect failed http://" + WiFi.softAPIP().toString() + "/webserial </p></body></html>");
  });
  server.begin();
  if (MDNS.begin("fpaper")) { feedlog("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
}




//TaskHandle_t flanksTasHandle;
void flanksTas(void *parameter) {    //  this is hopefully the same as using this lib in default Asynchronous
//void initflanks(){

  //static InterruptButton belowus(2, LOW, GPIO_MODE_INPUT, 750, 250, 5000, 8000);    //  ------  TODO switch to pin 20 here -------     pin, pressed low, pin mode, longpress ms, autorepeat ms, doubleclick ms, debounce us
  static InterruptButton belowus(2, LOW, GPIO_MODE_INPUT);

  InterruptButton::setMode(Mode_Synchronous);    // defaults to async wich executes immediate like an ISR, Synchronuse has to have a loop, hybrid does up/down events async and rest synchronous
  //InterruptButton::setMode(Mode_Hybrid);

  static volatile uint32_t lastpress = 0;

  //static const uint16_t slotcount = prefs.getUShort("slotcount", 8);    //  TODO replace with prefs.getUShort("slotcount", 4); and also update site accordingly in the future
  //static char slotcount = prefs.getUChar("slotcount", '8');
  //static char slots = prefs.getUInt("slots", 0);
  static uint32_t slots = prefs.getUInt("slots", 0);    //  slot count

  static size_t size = prefs.getBytesLength("peers");    //  size of peer list this is to clac peer count

  //static char (*peers)[16] = NULL;    //  this is a pointer to an array of char arrays with length 16
  static char (*peers)[16] = (char (*)[16]) malloc( size );    //  allocate memory for peer list

  prefs.getBytes("peers", peers, size);    //  read peer list into memory

  //static char prep[] = {'0'+slotcount, '\0'};    //  initially perp last slot so first increment shows peer
  //static char prep[] = {slotcount, '\0'};    //  initially perp last slot so first increment shows peer

  static uint32_t prep = slots;    //  this is the prepared thing to send or show when timer elapses initally perp last slot so first increment shows  current peer

  //static char currpeer[] = "0";    //  initially peer is local
  static uint32_t currpeer;    //  initially peer is local static initialises to zero

  belowus.bind(Event_KeyPress, [](){    //  this is called after double click timeout so actually do the stuff here

    /*
    if (!peers) {
      slots = prefs.getUInt("slots", 0);    //  reload slot count for first press after timeout

      size = prefs.getBytesLength("peers"); peers = (char (*)[16]) malloc( size );
      if (!peers) { feedlog("failed to allocate memory for peers\n"); ESP.restart(); }    //  check if malloc worked
      prefs.getBytes("peers", peers, size);    //  read peer list into memory
    }     //  when no peers allocate memory for peer list
    */

    lastpress = millis(); if (lastpress == 0) lastpress = 1;    //  record last press time but never zero to prevent initialisation to register as a press

    prep = ++prep % (slots+1);    //  cycle trough slots zero slot is free and is reserved for current peer
    if (!prep) {    //  for zero show current peer
      struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sprofile", peers[currpeer]); xQueueSend(showQueue, &show, 0);    //  show current peer with picture in picture onece every full cycle

      feedlog("prep:" + String(prep) + " currpeer:" + String(currpeer) + " trying to show " + String(show.nvsalias));

    }
    else {    //  for non zero show foto slot
      struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%u", prep); xQueueSend(showQueue, &show, 0);    //  show foto slot with picture in picture

      feedlog("prep:" + String(prep) + " currpeer:" + String(currpeer) + " trying to show " + String(show.nvsalias));

    }



     /*
    slotcount = prefs.getUChar("slotcount", '8');

    prep[0] = '0' + ((++prep[0] - '0') % (slotcount  - '0' +2));    //  cycle trough eight slots plus one for current peer           
    if (prep[0] > slotcount ) {
      struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sprofile", currpeer); xQueueSend(showQueue, &show, 0);    //  show current peer with picture in picture onece every full cycle

      feedlog("prep:" + String(prep) + " currpeer:" + String(currpeer) + " trying to show " + String(show.nvsalias));

    }
    else {
      struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sslot", prep); xQueueSend(showQueue, &show, 0);    //  show foto slot with picture in picture

      feedlog("prep:" + String(prep) + " currpeer:" + String(currpeer) + " trying to show " + String(show.nvsalias));

    }
    */
  });

  while(true){
    InterruptButton::processSyncEvents();    //  only here for synchronous or hybrid

    if ( ulTaskNotifyTake(pdTRUE, 0) ) { //reload peer list when notified by other tasks
      //free(peers); peers = NULL;    //  free memory for peer list so it is reallocated with press
      slots = prefs.getUInt("slots", 0);    //  reload slot count

      size = prefs.getBytesLength("peers"); peers = (char (*)[16]) realloc( peers, size );    //  preseving old peers is useless here so just directly assign yes this leaky but next line errors out
      if (!peers) { feedlog("failed to allocate memory for peers\n"); ESP.restart(); }    //  error out

      prefs.getBytes("peers", peers, size);    //  read peer list into memory
    }
    
    if ( lastpress && (millis() - lastpress > 2000)) {    //  when no press for two seconds actually do the stuff here
      lastpress = 0;    //  disarm this until real press

      if (!prep) {    //  when prep is a peer
        //currpeer = ++currpeer % ((size/16) + 1);    //  advance peer or wrap
        currpeer = ++currpeer % (size/16);    //  advance peer or wrap
        struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sprofile", peers[currpeer]); xQueueSend(showQueue, &show, 0);    //  show the advanced peers profile with picture in picture
      }
      else {    //  when perep is a foto slot
        if (currpeer) { struct sendstct sendprofile={ "", "localprofile" }; strcpy(sendprofile.peer, peers[currpeer]); xQueueSend(sendmqttQueue, &sendprofile, 0); }    //  first send our profile to current peer but not to local
        struct sendstct sendload; strcpy(sendload.peer,  peers[currpeer]); sprintf(sendload.load, "%u", prep); xQueueSend(sendmqttQueue, &sendload, 0);    //  then send prepped foto slot to current peer
      }
      //prep[0] = '0'+slotcount;    // reset prep so next time current peer shows up with first press

      //vTaskDelay(1);    //  wait some time to let send finish and then show

      // TODO this comes to fast so dely this some how or move this line into send task
      struct showstct show={ "user", 0, "" }; sprintf(show.nvsalias, "%slatest", peers[currpeer]); xQueueSend(showQueue, &show, 0);    //  show current peers latest foto with full refresh
    

      prep = slots;
      //free(peers); peers = NULL;    //  free memory for peer list so it is reallocated with press

      feedlog("timer up -> prep:" + String(prep) + " currpeer:" + String(currpeer) + " try to show " + String(show.nvsalias));

      /*
      if (prep[0] > slotcount ) {    //  when prep is a peer
        currpeer[0] = '0' + ((++currpeer[0] - '0') % (prefs.getUChar("peercount", '0') - '0' + 1));    //  advance peer or wrap. the literal '0' here is the ASCII offset since all this uses ASCII enumeration
        struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sprofile", currpeer); xQueueSend(showQueue, &show, 0);    //  show the advanced peers profile with picture in picture
      }
      else {    //  when perep is a foto slot
        if (currpeer[0]-'0') { struct sendstct sendprofile={ "", "0profile" }; strcpy(sendprofile.peer, currpeer); xQueueSend(sendmqttQueue, &sendprofile, 0); }    //  first send our profile to current peer but not never to local
        struct sendstct sendload={ "", "" }; strcpy(sendload.peer, currpeer); sprintf(sendload.load, "%sslot", prep); xQueueSend(sendmqttQueue, &sendload, 0);    //  then send prepped foto slot to current peer
      }
      //prep[0] = '0'+slotcount;    // reset prep so next time current peer shows up with first press
      prep[0] = slotcount;

      vTaskDelay(500);    //  wait some time to let send finish and then show
      // TODO this comes to fast so dely this some how or move this line into send task
      struct showstct show={ "user", 0, "" }; sprintf(show.nvsalias, "%slatest", currpeer); xQueueSend(showQueue, &show, 0);    //  show current peers latest foto with full refresh
    
      feedlog("timer up -> prep:" + String(prep) + " currpeer:" + String(currpeer) + " try to show " + String(show.nvsalias));
      */
    }

    //vTaskDelay(1);
    taskYIELD();
  }

      //   ------- TODO ---------
      //   restructure async queses to use structs and TRY TO REMOVE STRINGS where possible eg prep should be char[] or string (not String) i think
      //   build a timer to shutoff webpage after a time eg. server.end() MDNS.stopp() webserial.end() and so on
      //   package falnk into its own task perhaps this is not a good idea but it seems more consice with the rest of this and likely the lib does the same thing i think perhaps chekc this and perhaps use hybrid mode instead of syncronous wit own rtos task
      //   test if this (event key press) resets for multible key presses or if this fires multible times
      //   belowus.bind(Event_KeyPress, [](){    //  feedlog inside here does chrash perhaps this is 'm_RTOSservicerStackDepth' see here https://github.com/rwmingis/InterruptButton/tree/main?tab=readme-ov-file#known-limitations
}




void setup() {    //  when this int main() instead this does not compile
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


  if(!prefs.getBytesLength("peers")) { const char def[][16] = {{"local"}}; prefs.putBytes("peers", def, sizeof(def)); }

  feedlog("init done");

  //delay(500);

  //xQueueSend(showQueue, "fulllocalL", 0); // WHY DOES THIS NOT WORK????

  // TODO add a boot screen of some sort currently the showTas does not support this 
  //memcpy_P(volatileBuff, epd_bitmap_xpwallp, 15000);    //  copy boot foto from PROGMEM to volatile buffer for fast access
  //xQueueSend(showQueue, "showboot", 0);    //  add volatile foto to show queue
}




void loop() {vTaskSuspend(NULL);}    //  all done in tasks so suspend loop
