

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


#include <unordered_map>    //  c++ stuff
#include <functional>
//#include <format>    //  TODO this is huge remove this
#include <string>
//#include <string_view>




// ----------- global variables -----------

//static std::vector<std::array<char, 8>> peers;    //  for decoding in mqtt task and cycling fotos in flanks task  use array here to have continous storage in memory for nvs unlike std::string or list
//static std::vector<std::array<uint8_t, 32>> hkdfs;    //  for decoding in mqtt task and cycling fotos in flanks task
//static uint32_t slots = prefs.getUInt("slots", 0);    //  slot count this is one index based so slot zero does not exist
// perhaps add more here like servo positions
// consider removing dynamic changes to these and istead only applie them on restart perhaps
// pro - simplifies code a lot, 
//     - less risc for heap fragmentation and crashes,
//     - easy to create local copies for tasks so no globals needed
//     - more consitent with settings like wifi or mqtt server wich only take affect after restart
//
// con - less flexible, 
//     - local copies in each task use more ram (but psram is abundant anyway)
//     - already put work into dynamic changes sunken cost fallacy 



/* ----- moved into communication functions to avoid globals -----
QueueHandle_t servoQueue;    //  handle for servo queue this has no associated struct

QueueHandle_t sendmqttQueue;
struct sendstct { uint32_t peer; char load[16]; };

QueueHandle_t showQueue;
struct showstct { char ocupado[5]; uint8_t partial; char nvsalias[16]; };    //  handle and struct for show queue

QueueHandle_t logsQueue;
struct logsstct { uint8_t verbosity; char feed[40]; };    //  handle and struct for logs queue
---------------------- */


//Preferences prefs;    //  first declaration of preferences as perfs   --------- TODO -------- perhaps move this into first crated task and declare it as static !!!



// only used for get watermarks in wstas info cmd so not globaly needed
//TaskHandle_t servoTasHandle;
//TaskHandle_t sendmqttHandle;
//TaskHandle_t flanksTasHandle;
//TaskHandle_t networkTasHandle;
//TaskHandle_t wsTasHandle;
//TaskHandle_t showTasHandle;
// -------------------------------------




// ----------- TODO -----------
//
//  could i replace queues with direct to task notifications for servoQueue, sendmqttQueue, showQueue ?
//  would not require global queues and structs anymore but how would i actually queue stuff up ?
//
//  redo logging pass stuff with verobosity levels into logQueue but use timeouts so tasks do not block 
//
//  perhaps instead of global variables pass them as parameters to the tasks at creation
//
//  currently app uses ~1,2MB of flash perhaps reduce this to less than 1MB
//  currently 200KB internal Ram is remaining this is not too much
//
//  major restructuring concerns
//  - initialize vectors peers, hkdfs, slots correctly
//  - correct startup sequence eg wifi has to start first only then webserial and mqtt task can start
//  - webserial already is async so is it bad to put it into its own task?
//  - same goes for mqtt which is async too
//
//  NVS ACCESS IS NOT THREAD SAFE find a solution !! eg semaphore or mutex or seperate task wich sends values back to other tasks 
//  currently all the app code runs on core1 so this is not an issue
//
//  sartup order
//  - all tasks would like to send logs so logsQueue has to be created first and since it only is consumed with wstas add timout to xQueueSend
//  - network task is required for webserial and mqtt



QueueHandle_t logs(uint8_t verbosity = 0, char text[40] = "n.a.") {    //  this is to send logs to wstas task non blocking with timeout so tasks do not block forever
  struct logsstct { uint8_t verbosity = verbosity; char text[40] = text; } logs;
  static QueueHandle_t logsQueue = NULL;

  if (logsQueue == NULL ) {    //  create queue when not yet created
    logsQueue = xQueueCreate(5, sizeof(logsstct));    //  decided to use queue for coherence instead of a messagebuffer the variable length is not really a benefit here
  } else {
    logs.verbosity = verbosity;
    strncpy(logs.text, text, sizeof(logs.text));
    xQueueSend(logsQueue, &logs, 0);    //  do not block if queue is full
  }
  return logsQueue;
}


QueueHandle_t shows(char ocupado[5] = "N.A.", uint8_t partial = 0, char nvsalias[16] = "N.A.") {    //  this is to send logs to wstas task non blocking with timeout so tasks do not block forever
  struct showstct { char ocupado[5]; uint8_t partial; char nvsalias[16]; }; show;
  static QueueHandle_t showQueue = NULL;

  if (showQueue == NULL ) {    //  create queue when called with NULL and not yet created
    showQueue = xQueueCreate(5, sizeof(showstct));    //  decided to use queue for coherence instead of a messagebuffer the variable length is not really a benefit here
  } else {
    strncpy(show.ocupado, ocupado, sizeof(show.ocupado));
    show.partial = partial;
    strncpy(show.nvsalias, nvsalias, sizeof(show.nvsalias));
    xQueueSend(showQueue, &show, 0);    //  do not block
  }
  return showQueue;
}


QueueHandle_t mqsend(uint32_t peer = 0, char load[16] = "N.A.") {    //  this is to send logs to wstas task non blocking with timeout so tasks do not block forever
  struct sendstct { uint32_t peer; char load[16]; } send;
  static QueueHandle_t sendQueue = NULL;

  if (sendQueue == NULL ) {    //  create queue when called the first time
    sendQueue = xQueueCreate(5, sizeof(sendstct));    //  decided to use queue for coherence instead of a messagebuffer the variable length is not really a benefit here
  } else {
    send.peer = peer;
    strncpy(send.load, load, sizeof(send.load));
    xQueueSend(sendQueue, &send, 0);    //  do not block if queue is full
  }
  return sendQueue;
}


QueueHandle_t servo(char pos[4] = "N.A.") {    //  this is to send logs to wstas task non blocking with timeout so tasks do not block forever
  static QueueHandle_t servoQueue = NULL;

  if (servoQueue == NULL ) {    //  create queue when called the first time
    servoQueue = xQueueCreate(5, sizeof(char[4]));    //  decided to use queue for coherence instead of a messagebuffer the variable length is not really a benefit here
  } else {
    strncpy(pos, pos, sizeof(pos));
    xQueueSend(servoQueue, &pos, 0);    //  do not block if queue is full
  }
  return servoQueue;
}




Preferences& nvs() {    //  this initializes nvs namespace and provided direct access to it
  static Preferences prefs;
  static bool firstcall = false;

  if (!firstcall
    prefs.begin("prefs", false);
    firstcall = true;
  }
  return prefs;
}


char* peerscache (uint32 i, char* peername = nullptr, char* secret = nullptr){    //  fast accessor to peers list and manage its nvs validness    hopefully this is better than reading form nvs each time
  static std::vector<std::array<char, 8>> peers;

  static bool firstcall = true;    //  static initilizes to false

  if (firstcall) {
    size_t pb = nvs().getBytesLength("peers");
    if (pb) {    //  either load peers into cache
      peers.resize(pb / 8);
      nvs().getBytes("peers", peers.data(), pb);
    }
    else {    //  or initialize peers with default value at index 0
      peers.push_back("local");
      nvs().putBytes("peers", peers.data(), peers.size() * 8);
    }
    firstcall = false;
  }

  if (!peername && !secret) return ( i < peers.size() ) ? peers[i].data() : nullptr;    //  for out of range return nullptr

  size_t found; for (found = 1; found < peers.size(); ++found) {    //  find peer in list
    if ( !strcmp(peers[found].data(), peername) ) break;
    if ( found == peers.size() - 1 ) found = 0;    //  mark as not found
  }

  if (!found && secret) {    //  peer not in list and secret provided so add peer here
    std::array<char, 8> peer{};    //  zero initialise a fixed byte array for peer name ensures null termination and allowes continous storage in vector unlike std::string would this is necessary for nvs

    args.copy(peername, size_t(7), 0);   // copy directly from args and leave last byte for NUL
    peers.push_back(peer);    //  append peer to vector

    nvs().putBytes("peers", peers.data(), peers.size() * 8);    //  write back peer list with new peer to nvs

    hkdfscache(found, false, secret);    //  also add secret    // TODO check return here for success

    return peers[found].data();    //  return added peer
  }

  if (found && !secret) {    //  peer in list and no secret provided so delete peer plus associated data here
    peers.erase(peers.begin() + found );    //  this is slow but this is not a frequent operation
    peers.shrink_to_fit();    //  free unused memory technically not necessary perhaps even bad for fragmentation
    nvs().putBytes("peers", peers.data(), peers.size() * 8);    //  write back peer list without deleted peer to nvs

    nvs().remove( (peername + "latest").c_str() );    //  remove latest foto entry for this peer
    nvs().remove( (peername + "profile").c_str() );    //  remove profile foto of this peer

    hkdfscache(found, true, nullptr);    //  also delete secret

    return peername;    //  echo deleted peer name
  }

  return nullptr;    //  peer found and secret provided perhaps add this to overwrite secret    currently user first has to delte peer and then re add him/her/they/them/it/....
}


uint8_t* hkdfscache (uint32 i, bool found = false, char* secret = nullptr ){    //  fast accessor to hkdfs list and manage its nvs validness    hopefully this is better than reading form nvs each time
  static std::vector<std::array<uint8_t, 32>> hkdfs;

  static bool firstcall = true;    //  static initilizes to false

  if (firstcall) {
    size_t hb = nvs().getBytesLength("hkdfs");
    if (hb) {    //  either load peers into cache
      hkdfs.resize(hb / 32);
      nvs().getBytes("hkdfs", hkdfs.data(), hb);
    }
    else {    //  or initialize peers with default value at index 0
      hkdfs.push_back({});
      nvs().putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32);
    }
    firstcall = false;
  }

  if (!found && !secret) return ( i < hkdfs.size() ) ? hkdfs[i].data() : nullptr;    //  for out of range return nullptr

  if (!found && secret) {    //  add secret
    std::array<uint8_t, 32> hkdfout{};    //  zero initialise a fixed byte array for hkdf ensures null termination and allowes continous storage in vector unlike std::string would this is necessary for nvs
    
    hkdf<SHA256>(hkdfout.data(), 32, secret, strlen(secret), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 32 bytes as secret for encryption hkdf<SHA256>(outputbuff, sizeof(output), secret, sizeof(secret), salt, sizeof(salt), info, sizeof(info));
    hkdfs.push_back(hkdfout);    //  append hkdf to vector

    nvs().putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32);    //  write back hkdf list with new hkdf to nvs
    
    return hkdfs.back().data();    //  return added hkdf
  }

  if (found && !secret) {    //  delete secret
    hkdfs.erase(hkdfs.begin() + i );    //  this is slow but this is not a frequent operation
    hkdfs.shrink_to_fit();    //  free unused memory technically not necessary perhaps even bad for fragmentation
    nvs().putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32);    //  write back peer list without deleted peer to nvs
    return hkdfs[i].data();
  }

  return nullptr;
}


uint32_t slotscache (uint32_t delslot = 0){
  static uint32_t slots = 0;

  static bool firstcall = true;

  if (firstcall) {
    slots = nvs().getUInt("slots", 0);
    firstcall = false;
  }

  if (!delslot) return slots;    //  slots are one indexed so zero/default just wants slot count otherwise delete the provided slot but keep an interable/continuous order

  if (delslot > slots) return = 0;    //  slot out of range cant delete this is invalid

  char slotschar[12]; snprintf(slotschar, 12, "%u", slots);    //  hold the delslot/slots value as char this is required for nvs access
  char delslotchar[12]; snprintf(delslotchar, 12, "%u", delslot);

  if (delslot != slots) {    //  swap slot with top most slot
    uint8_t temp[15000];
    prefs.getBytes( slotschar , temp, sizeof(temp) );    //  read top most slot into temp
    prefs.putBytes( delslotchar , temp, sizeof(temp) );    //  copy temp to target slot
  }
  prefs.remove( slotschar );    //  remove top most slot
  prefs.putUInt("slots", slots -1);    //  adjust / save slots count
  
  return delslot;    //  echo deleted slot to confirm
}




void networkTas(void *parameter) {    //  this connects to wifi or spawns an access point for configuration
  WiFi.mode(WIFI_STA);
  WiFi.begin( prefs.getString("ssid", "fpaper"), prefs.getString("pass", "") );    //  return ssid from preferences nvs or return finger

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {    //  this waits for a default time and when not able to connect to ssid falls back to ap
    WiFi.mode(WIFI_AP);
    WiFi.softAP("fpaper", "");
    Serial.println(prefs.getString("ssid", "fpaper") + " failed so fallback soft ap fpaper up so access webserial at http://" + WiFi.softAPIP().toString().c_str() + "/webserial \n");
    //if (MDNS.begin("fpaper")) { Serial.println("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
  } else {
    Serial.println(prefs.getString("ssid", "fpaper") + " success so access webserial at http://" + WiFi.localIP().toString().c_str() + "/webserial \n");
    //if (MDNS.begin("fpaper")) { Serial.println("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
    vTaskDelete(NULL);    //  all done so delete this task
  }

  DNSServer dnsServer;
  dnsServer.start(53, "*", WiFi.softAPIP());    //  init dns server on port 53 with wildcard domain to map all requests to ap ip for captive portal
  while(true){
      dnsServer.processNextRequest();
      Serial.println("dns for ap mode");  // make this debug only
      //vTaskDelay(10);
      taskYIELD();
  }
}




void wstas() {    //  this spawns webserial and handles all web stuff

  // -------- TODO -----------
  // load the peers and hkdfs vectors from nvs
  // if not found initialize them with local peer and garbage for hkdf[0]
  // this is required for iterations over these vectors in mqtt task or flanks task !!!!!


  
  //logsQueue = xQueueCreate(1, sizeof(logsstct));    //  decided to use queue for coherence instead of a messagebuffer the variable length is not really a benefit here
  QueueHandle_t logsQueue = logs(NULL);    //  create logs queue



  static uint8_t prefsverbosity = 0; // TODO add verbosity command with nvs

  AsyncWebServer server(80);

  WebSerial ws;    //  first delclartion of webserial not static anymore since v8.0.0

  const std::unordered_map<std::string, std::function<void(std::string args)> > cmds = {    //  const map with compile-time initialization to reduce heap pressure/fragmentation
    {"delslot", [&](std::string args) {    //  all these conversions feel wrong
      if (args.empty()) { ws.print("eeee delslot requires args\n"); return; }

      uint32_t delslot = strtoul(args.c_str(), NULL, 10);    //  this returns zero value for invalid input this is acceptable since zero slot does not exist slots a one indexed

      if (!delslot) { ws.printf("eeee '%s' not a slot \n", args.c_str()); return; }    //  prevent zero slot since this does not exist

      uint32_t answer = slotscache(delslot);    //  delete and recache/rewrite slots into nvs

      if ( answer == delslot ) { ws.printf("deleted slot '%u'\n", delslot); return; }
      if (!answer) { ws.printf("eeee slot '%s' out of range\n", args.c_str()); return; }
    }},

    {"peer", [&](std::string args) {    //  with just peername this deletes peer and all associated data with aditional secret this adds/overwrites peer hkdf
      if (args.empty()) { ws.print("eeee peer requires args\n"); return; }
      if (strtoul(args.c_str(), NULL, 10)) { ws.print("eeee numbers not allowed\n"); return; }    //  prevent peer names which are numbers since these are reserved for slots

      std::string name = args.substr(0, args.find(' '));    //  this always is just the peer name either pos 0..7 or pos 0..nospc

      if (name.length() > 8) { ws.print("eeee peer name too long\n"); return; }    //  this is reached when peername too long

      char* answer = peerscache(0, name.c_str(), args.c_str() + name.length());    //  all the add/deletion and recache logic is there
      if ( !answer) { ws.print("eeee sth went wrong in peercache"); return; }    //  nullptr is some error inside peerscache()
      if ( !strcmp(answer, name.c_str()) ) ws.printf("deleted peer '%s'\n", args.c_str());    //  echo means deleted successfully
      else ws.printf("added peer '%s' with secret '%s'\n", answer, (args.c_str() + name.length()));

      return;
    }},

    // todo add 'wstime' cmd to set the uptime for the webserial

    // todo add 'publ' to publish to mqtt

    // todo add 'apt upgrade' to set frimware url

    // todo add 'rm -rf' to clear nvs prefs.clear();

    // todo add 'top' , 'sit' to set servo positions prefs.putInt("top",


    {"info", [&](std::string args) {
      // this only works after all tasks are created obviously so wstask has to be created last
      // get task handles for the info task wich tries to get the watermarks of other tasks
      // instead of keeping global task handles these could be retrieved once here. make sure the taskhndles are not used elsewhere to
      TaskHandle_t servoTasHandle = xTaskGetHandle("servoTas");    //  this takes some time hopfully this is fine inide a callback
      TaskHandle_t sendmqttHandle = xTaskGetHandle("sendmqtt");
      TaskHandle_t flanksTasHandle = xTaskGetHandle("flanksTas");
      TaskHandle_t networkTasHandle = xTaskGetHandle("networkTas");
      TaskHandle_t showTasHandle = xTaskGetHandle("showTas");
      //TaskHandle_t wsTasHandle = xTaskGetHandle("wsTas");   notrequired just do uxTaskGetStackHighWaterMark(NULL) for self
      
      // todo either put this inside help or in a seperate info cmd
      //
      // char nvsfree[30]; sprintf(nvsfree, "\n\nfree entries in nvs %d \n", prefs.freeEntries()); ws.print(nvsfree);
      // ws.print("PSRAM " + (psramFound() ? "found " + String(ESP.getPsramSize()) + " bytes total, " + String(ESP.getFreePsram()) + " bytes free \n" : "Not found\n"));
      // ws.print("auto firmware url is '" + prefs.getString("airlink", "error") + "' \n");
      // if(WiFi.getMode() == WIFI_MODE_AP) { ws.print("local ip " + WiFi.softAPIP().toString() + " \n"); }
      // if(WiFi.getMode() == WIFI_MODE_STA) { ws.print("local ip " + WiFi.localIP().toString() + " \n"); }
      // char macStr[30]; sprintf(macStr, "eFuse mac %012llX \n", ESP.getEfuseMac() ); ws.print(macStr);    //  this is so tiedious pls help me do not know how to string
      // ws.print("| Type | Sub |  Offset  |   Size   |       Label      | \n");    //  this prints current partition table just for your info
      // ws.print("| ---- | --- | -------- | -------- | ---------------- | \n");
      // esp_partition_iterator_t pi = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
      // if (pi != NULL) {
      //   do {
      //     const esp_partition_t* p = esp_partition_get(pi);
      //       char buffer[128]; sprintf(buffer, "|  %02x  | %02x  | 0x%06X | 0x%06X | %-16s | \n", p->type, p->subtype, p->address, p->size, p->label); ws.print(buffer);    //  this sucks i hate strings i miss python
      //   } while (pi = (esp_partition_next(pi)));
      // }
      // int count = msg.substring(5).toInt() ; xTaskCreate( printWatermarkTas, "printWatermarkTas", 2048, (void*) &count, 1, &watermarkHandle ); return;   //  determine stack size just for your info 'xTaskCreate( function, name, stack size bytes, parameter to pass, priority, handle )'
      //    TaskHandle_t watermarkHandle;
      //    void printWatermarkTas(void *count){
      //      int iter = *(int*) count; feedlog ("printing stack high watermark for tasks for " + String(iter) + " seconds \n");
      //      for (int i = 0; i < iter; i++) {
      //          feedlog(String(i+1) + "/" + String(iter) + ", dnsTas '" + String(uxTaskGetStackHighWaterMark(dnsServHandle)) + "', servoTas '" + String(uxTaskGetStackHighWaterMark(servoTasHandle)) + "', sendmqttTas '" + String(uxTaskGetStackHighWaterMark(sendmqttHandle)) + "'\n");
      //          vTaskDelay(1000);
      //      }
      //      feedlog("\n\n");
      //      vTaskDelete(watermarkHandle);
      //    }
    }},


    {"topic", [&](std::string args) {
      if (args.empty()) { ws.print("Error: topic requires a value\n"); return; }
      prefs.putString("mqtop", args.c_str());
      ws.printf("MQTT topic set to '%s'\n", args.c_str());
    }},
    
    {"debug", [&](std::string args) {
      if (args.empty()) { ws.print("Error: debug requires a level\n"); return; }
      prefs.putString("debuglevel", args.c_str());  //  TODO make this an enum !!!!
      ws.printf("Debug level set to '%s'\n", args.c_str());
    }},
    
    {"ssid", [&](std::string args) {
      if (args.empty()) { ws.print("Error: ssid requires a value\n"); return; }
      prefs.putString("ssid", args.c_str());
      ws.printf("SSID set to '%s'\n", args.c_str());
    }},
    
    {"pass", [&](std::string args) {
      if (args.empty()) { ws.print("Error: pass requires a value\n"); return; }
      prefs.putString("pass", args.c_str());
      ws.printf("Password set to '%s'\n", args.c_str());
    }},
    
    {"serv", [&](std::string args) {
      if (args.empty()) { ws.print("Error: serv requires a URL\n"); return; }
      prefs.putString("mqserv", args.c_str());
      ws.printf("MQTT server set to '%s'\n", args.c_str());
    }},
    
    {"restart", [&](std::string args) {
      ws.print("Restarting ESP...\n");
      ESP.restart();
    }},
    
    {"help", [&](std::string args) {
      String peerstring = "";
      size_t size = prefs.getBytesLength("peers");
      char (*peers)[16] = (char (*)[16]) malloc(size);
      prefs.getBytes("peers", peers, size);
      if (!peers) { ws.print("failed to allocate memory for peers\n"); return; }
      for (int i = 0; i < size/16; i++) { peerstring += String(peers[i]) + ", "; }
      free(peers);

      // TODO -------- add this Serial.println(__cplusplus); // Shows C++ standard version
      //  no filepath - small snippet
      //  Serial.printf("free heap: %u\n", ESP.getFreeHeap());
      //  Serial.printf("psram found: %d free psram: %u\n", psramFound(), ESP.getFreePsram());
      
      ws.print("\n \n"
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
          " rm -rf              chill this just clears preferences\n\n\n" );
    }}
  };


  ws.onMessage([&ws, &cmds](const std::string& stdstr) {    //  todo redo this with std::unordered_map<std::string, std::function<void(std::string_view args)> > cmds;
    auto pos = stdstr.find(' ');
    auto it = cmds.find(stdstr.substr(0, pos));
    if (it != cmds.end()) {
      it->second( (pos == std::string::npos) ? "" : stdstr.substr(pos + 1) );    //  call the function in the unordered map with arguments
    } else {
      ws.printf("'%s' is unknown try 'help'\n", stdstr.c_str() );
    }
  });    //  attach message callback

  ws.begin(&server);    //  init webserial


  server.on("/querySlots", HTTP_GET, [](AsyncWebServerRequest *request) {
    char buff[16]; snprintf(buff, sizeof(buff), "%u", prefs.getUInt("slots", 0) + 1); request->send(200, "text/plain", buff);    //  send slot count plus one so user can add new fotos
  });

  server.on("/file", HTTP_POST,
    [](AsyncWebServerRequest* request) {},    // empty request handler - no response sent
    [&ws](AsyncWebServerRequest *request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    static size_t totalSize = 0;    //  static so this is not reset on each chunck
    static char slot[12];    // static to persist across chunks this max is 'profile'
    static uint8_t rcvbuff[15000];    // static buffer allocated once

    if (!index){
      totalSize = request->header("Content-Length").toInt();
      strncpy(slot, request->getParam("slot")->value().c_str(), 8);    //  max copy eight chars for 'profile' here
      ws.print("file is for slot " + String(slot));
    }
    if (len + index > sizeof(rcvbuff)) {
      ws.print("aw thats to grande for me"); return;    //  this is to prevent buffer overflow
    }
    else if (len) {
      ws.print("file " + filename + " " + String(index + len) + "/" + String(totalSize) + " bytes\r\n");
      memcpy(rcvbuff + index, data, len);    //  copy data to volatile buffer
    }
    if (final){    //  just save the recieved buffer to nvs
      //prefs.putBytes( (strcmp(slot, "profile") ? strcat(slot, "slot") : "0profile") , rcvbuff, sizeof(rcvbuff));    //  save profile to 'local profile' or save foto to 0-7 for foto slots
      //ws.print("file saved to " + String(slot));
      
      prefs.putBytes( (strcmp(slot, "profile") ? slot : "localprofile") , rcvbuff, sizeof(rcvbuff));    //  save profile to 'local profile' or save foto to slot
      ws.print("file saved to " + String(slot));
      if (strcmp(slot, "profile") && strtoul(slot, NULL, 10) > prefs.getUInt("slots", 0)) {
        prefs.putUInt("slots", strtoul(slot, NULL, 10));    //  update slot count when new slot added
        
        
        // TODO remove this xTaskNotifyGive(flanksTasHandle);    //  notify flanks task to reload slots 
        
        
        ws.print("updated slot count to " + String(prefs.getUInt("slots", 0)) + "\n");
      }
    }
  });

  server.onNotFound([](AsyncWebServerRequest* request) {    //  redirect all requests to webserial for captive portal request->redirect("/webserial"); does not work for captive portal
    request->send(200, "text/html", "<!DOCTYPE html><html><meta http-equiv='refresh' content='0; url=http://fpaper.local/webserial' /><head><title>Captive Portal</title></head><body><p>auto redirect failed http://" + WiFi.softAPIP().toString() + "/webserial </p></body></html>");
  });

  if ( MDNS.begin("fpaper") ) { Serial.println("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add a service to mDNS use 'MDNS.addService("http", "tcp", 80);'

  server.begin();


  if ( prefs.getBytesLength("airlink") || prefs.getBool("autofw", false) ) {    //  this tries auto firmware upgrade or manual link once every boot    //  this works with redirects and insecure https source 'https://github.com/espressif/arduino-esp32/issues/9530#issuecomment-2090034699'
    char airlink[256] = "";    //  instead of hardcoding the link here improve this with checking here 'https://api.github.com/repos/crbyxwpzfl/fpaper/releases/latest' or 'https://api.github.com/repos/crbyxwpzfl/fpaper/tags' befor download and then use 'https://github.com/crbyxwpzfl/fpaper/releases/latest/download/not-merged-correct-board.bin'
    if ( prefs.getBytes("airlink", airlink, sizeof(airlink)) ) prefs.remove("airlink");     //  when read successfull rm the airlink so just try this once when not successfull leave buffer alone

    WiFiClientSecure secureClient;    //  replace all this with this here https://github.com/espressif/arduino-esp32/tree/master/libraries/Update/examples/HTTPS_OTA_Update
    HTTPUpdate up;

    secureClient.setInsecure();    //  this is to ignore ssl so theoretically some one can spoof github this is not good 
    up.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);    //  this is to follw link redirects other options are eg 'up.rebootOnUpdate(false);' or 'secureClient.setTimeout(5);'
    up.onStart([&ws]() { ws.print("overwrite firmware init download \n"); });
    up.onEnd([&ws]() { ws.print("firmware download success so restart to overwrite \n"); });
    up.onError([&ws, &up](int err) { ws.print(  up.getLastErrorString() + " \n"); });
    up.onProgress([&ws](int current, int total) { ws.print(  String(100.0 * current / total) + "% \n" ); });    //  to print percentage of download and pulse led yellow while updating perhaps prgressbar is cooler instead but have ro figure out how to do same line prints in webserial
    HTTPUpdateResult result = up.update(secureClient, airlink, "", [](HTTPClient *http) { });    //  to add sth to the http header use 'http->addHeader("Authorization", "{\"token\":\"noInitYet\"}");'
  
    //ws.print("auto firmware error (" + String(up.getLastError()) + ") " + up.getLastErrorString().c_str() + " check " + airlink.c_str() + " \n");
    ws.printf("auto firmware error %s link was %s \n", up.getLastErrorString().c_str(), airlink);    //  usually auto restart prevents this line so just prints when no restart cause error
  }


  //   ------- TODO ---------
  //   build a timer to shutoff webpage after a time eg. server.end() MDNS.stopp() webserial.end() and so on
  uint32_t livetime = prefs.getUInt("wsalivesec", 0) * 1000UL;    //  read webserial alive time in seconds from nvs or default to forever
  uint32_t inittimestamp = millis();
  while ( !livetime || (millis() - inittimestamp) < livetime ) {
    logsstct logs; if( xQueueReceive( logsQueue, &logs, 0 ) == pdPASS ) {    //  when something in logs queue do 
      if ( prefsverbosity > logs.verbosity ) { ws.printf("%s\n", logs.feed); }    //  print logs when verbosity matches
    }
    taskYIELD();
  }

  server.end();    //  stop server so callback are unregistered so ws is not used anymore
  vTaskDelete(NULL);     //  safe to delete task and destroy ws
}




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
    showstct show; if( xQueueReceive( showQueue, &show, 0 ) == pdPASS ) {    //  just pops when queue not empty and returns pdPASS when something was recieved else returns pdFAIL

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
    };
    taskYIELD();    //  more efficent vTaskDely(0)    //  befor one second so no flicker just show every thing fast user interactions

  }
}




void servoTas(void *parameter) {    //  this handles servo movement
  servoQueue = xQueueCreate(5, sizeof("sit"));    // create queue with buffer of 5 
  ledcAttach(38, 50, 12);    //  50hz pwm at pin 38 with 12 bit resolution so 0-4095
  
  while(true){
    char buff[4]; if( xQueueReceive( servoQueue, &buff, 0 ) == pdPASS ) {
      //ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); String(buf) == "top" ? ledcWrite(38, prefs.getInt("top", 0)) : ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0);    //  move servo to poses in preferences also cool c ternary operator
      if (!strcmp(buff, "top")) { ledcWrite(38, prefs.getInt("top", 0)); vTaskDelay(500); ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }   //  wigle servo to poses in preferences always top and back to sit pose
      if (!strcmp(buff, "sit")) { ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }  // move servo to sit pose
    }
    taskYIELD();
  }
}




void sendmqttTas(void *parameter) {    //  this handles all mqtt traffic

  sendmqttQueue = xQueueCreate( 5, sizeof(sendstct) );    // create queue with buffer of 5

  PsychicMqttClient mqttClient;
  ChaChaPoly chachapoly;

  static uint8_t sendcyphy[12+16+15000+9];    //  this is the last sent message the first 12iv plus 16tag plus 15000foto plus 9profile/slot    //  this is written every outgoing message and read with every incoming so perhaps protect this with mutex/semaphore 

  String serverAddress = prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work

  mqttClient.onTopic( prefs.getString("mqtop", "fpaper/").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer      wildcards should work here listen one level deep for now TODO change this to only subscribe to peers

    if ( !memcmp(sendcyphy, payload, 12) ) {  xTaskNotifyGive(sendmqttHandle); return;}    //  ignore echos no sens to decode echos  echos free the send task early  this seems racey but while publish there is no echo befor publish   TODO implement some check to avoid mitigate spam here eg some chek for known phrase or sth

    Serial.println("got message start decoding");    //  TODO make this debug

    static uint32_t index;    //  static initialises to zero also remember last successful peer index

    static uint8_t rcvcyphy[15000];    //  TODO perhaps move these to psram or this is a bit large for stack so this is static perhaps better malloc and free int8_t* cyphy = (uint8_t*)malloc(15000);
    static uint8_t temp[15000];     //  see comment abouveus

    Serial.println("Received message on topic: " + String(topic) );     //  TODO make this a debug log

    for (uint32_t attempts = hkdfs.size(); attempts; attempts--) {       // a =  4, 3, 2, 1
                                                                       // key =  x, 3, 2, 1
 
      chachapoly.setIV( payload , 12);                                Serial.println(" set iv");    //  TODO make all this debug logs
      chachapoly.setKey( hkdfs[index].data() , 32);                             Serial.println(" set key");
      chachapoly.decrypt(rcvcyphy, payload + 12 + 16, 15000);                 Serial.println(" decrypted cypher text");

      if (!chachapoly.checkTag( payload + 12, 16)) { index = attempts -1; continue; }    //  retry
      
      if ( !memcmp("look here", payload + 12 + 16 + 15000, 9) ) {    //  here compare recieved profile to saved profile and perhpas overwrite    also show recieved profile    also move servo 
        char nvsalias[16]; sprintf(nvsalias, "%sprofile", peers[index].data()); prefs.getBytes( nvsalias, temp, 15000 );    //  read the profile of peer wich is at 'peer+profile'
        //char nvsalias[16]; snprintf(nvsalias, sizeof(nvsalias), "%.*sprofile", 8, peers[index].data()); prefs.getBytes( nvsalias, temp, 15000 );    //  read the profile of peer wich is at 'peer+profile' safer version to prevent buffer overflow when peer is not null terminated this reads at most eight chars
        //prefs.getBytes( std::format("{}profile", peers[index]).c_str(), temp, 15000 );

        Serial.println("first decryption successfull");

        if ( memcmp(temp, rcvcyphy, 15000) ) prefs.putBytes( nvsalias, rcvcyphy, 15000 );    //  when profile changes save recieved profile to peer wich is 'peer+profile' see abouve

        struct showstct show={ "", 1, "" }; 
        strcpy(show.ocupado, peers[index].data()); 
        strcpy(show.nvsalias, peers[index].data()); 
        xQueueSend(showQueue, &show, 0);    //  make sure peers[index] is null terminated. .data() just points to beninging and str ops go untill '\0'.  show recieved profile with picture in picture and occupie screen with sending peer so no other message interferes
        //comm("showQueue", "param1", "param2");

        xQueueSend(servoQueue, "top", 0);    //  move servo to top position this wiggles screen
      }

      if ( !memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here save recieved foto to nvsalias+'L'    also show this
        char nvsalias[16]; snprintf(nvsalias, sizeof(nvsalias), "%slatest", peers[index].data()); prefs.putBytes( nvsalias, rcvcyphy, 15000 );    //  save foto of peer wich is 'peer+latest' safer version to prevent buffer overflow when peer is not null terminated this reads at most eight chars

        Serial.println("second decryption successfull");

        struct showstct show={ "", 0, "" }; strcpy(show.ocupado, peers[index].data()); strcpy(show.nvsalias, peers[index].data()); xQueueSend(showQueue, &show, 0);    //  show recieved latest foto with full refresh to clear profile this also frees occupation
      }

      chachapoly.clear(); return;    //  exit lambda
    }
  });

  mqttClient.connect();


  while (true) {
    sendstct send; if( xQueueReceive( sendmqttQueue, &send, 0 ) == pdPASS ) {    //  reads first word out of queue when sth in queue

      Serial.println("sending to " + String(peers[send.peer].data()) );

      if (!prefs.getBytes( send.load, 12+16+sendcyphy, 15000 )) { Serial.println("nothing found for " + String(send.load)); continue; }    //  for invalid nvs lookups this returns null and leaves cyphy

      if ( !send.peer ) {    //  when recipient local just save to local latest
        prefs.putBytes( "locallatest", sendcyphy, 15000);    //  when recipient local save to local latest
      }

      else {        //  here when recipient not local actually do send stuff    // TODO somehow dont send full profile everytime you want to annoy
        esp_fill_random(sendcyphy, 12);    //  fill first 12 bytes of sendcyphy with noise to use as iv for chachapoly also to later in recieve mqtt determine wether message is a echo

        chachapoly.setIV(sendcyphy, 12);    //  use first 12 bytes of sendcyphy as iv
        chachapoly.setKey(hkdfs[send.peer].data(), 32);    //  
        chachapoly.encrypt( 12+16+sendcyphy, 12+16+sendcyphy, 15000);    //  encrypt clear bytes of load this was loaded into cyphy befor

        chachapoly.computeTag( 12+sendcyphy, 16);
        chachapoly.clear();

        memcpy(payload + sizeof(curriv) + sizeof(sendtag) + sizeof(sendcyphy), strcmp(send.load, "localprofile") ? "look here" : "see this ", 9);    //  send our profile with 'look here' appendix or send foto slot with 'see this'    TODO send hash of peers profile to minimize messages

        memcpy(12 + 16 + 15000, strcmp(send.load, "localprofile") ? "look here" : "see this ", 9);    //  send our profile with 'look here' appendix or send foto slot with 'see this'    TODO send hash of peers profile to minimize messages

        Serial.println("packed payload try sending now to " + String(peers[send.peer].data()) );    //  TODO make this a feedlog message

        mqttClient.publish( prefs.getString("mqtop", "fpaper/").c_str() , 0, 0, reinterpret_cast<const char*>(sendcyphy), 12 + 16 + 15000 + 9, true);     //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer       publish full length message to base topic

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(4000)) == 0) { Serial.println("seen echo or timeout over\n"); }    //  this blocks the task until notivied or timeout runs out this is to filter echos  // TODO optimization this technicaly doe not have to wait on local sends
        else { Serial.println("timeout waiting for echo\n"); }    //  TODO make this a debug log

      }

    }
    taskYIELD();
  }
}




void flanksTas(void *parameter) {    //  this is hopefully the same as using this lib in default asynchronous
//void initflanks(){

  //static InterruptButton belowus(2, LOW, GPIO_MODE_INPUT, 750, 250, 5000, 8000);    //  ------  TODO switch to pin 20 here -------     pin, pressed low, pin mode, longpress ms, autorepeat ms, doubleclick ms, debounce us
  static InterruptButton belowus(2, LOW, GPIO_MODE_INPUT);

  InterruptButton::setMode(Mode_Synchronous);    // defaults to async wich executes immediate like an ISR, Synchronuse has to have a loop, hybrid does up/down events async and rest synchronous
  //InterruptButton::setMode(Mode_Hybrid);

  static volatile uint32_t lastpress = 0;

  static uint32_t prep = slots;    //  this is the prepared thing to send or show when timer elapses initally perp last slot so first increment shows  current peer

  static uint32_t currpeer;    //  initially peer is local static initialises to zero


  belowus.bind(Event_KeyPress, [](){    //  this is called after double click timeout so actually do the stuff here

    lastpress = millis(); if (!lastpress) lastpress = 1;    //  record last press time but never zero to prevent initialisation to register as a press

    prep = ++prep % (slots+1);    //  cycle trough slots zero slot is free and is reserved for current peer
    if (!prep) {    //  for zero show current peer
      struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sprofile", peers[currpeer].data()); xQueueSend(showQueue, &show, 0);    //  show current peer with picture in picture onece every full cycle

      Serial.println("prep:" + String(prep) + " currpeer:" + String(currpeer) + " trying to show " + String(show.nvsalias));

    }
    else {    //  for non zero show foto slot
      struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%u", prep); xQueueSend(showQueue, &show, 0);    //  show foto slot with picture in picture

      Serial.println("prep:" + String(prep) + " currpeer:" + String(currpeer) + " trying to show " + String(show.nvsalias));

    }
  });


  while(true){
    InterruptButton::processSyncEvents();    //  only here for synchronous or hybrid

    if ( lastpress && (millis() - lastpress > 2000)) {    //  when no press for two seconds actually do the stuff here
      lastpress = 0;    //  disarm this until real press

      if (!prep) {    //  when prep is a peer
        //currpeer = ++currpeer % ((size/16) + 1);    //  advance peer or wrap
        currpeer = ++currpeer % (size/16);    //  advance peer or wrap
        struct showstct show={ "user", 1, "" }; sprintf(show.nvsalias, "%sprofile", peers[currpeer].data()); xQueueSend(showQueue, &show, 0);    //  show the advanced peers profile with picture in picture
      }
      else {    //  when perep is a foto slot
        //if (currpeer) { struct sendstct sendprofile={ "", "localprofile" }; strcpy(sendprofile.peer, peers[currpeer]); xQueueSend(sendmqttQueue, &sendprofile, 0); }    //  first send our profile to current peer but not to local
        if (currpeer) { struct sendstct sendprofile{ currpeer, "localprofile" }; xQueueSend(sendmqttQueue, &sendprofile, 0); }    //  first send our profile to current peer but not to local
        //struct sendstct sendload; strcpy(sendload.peer,  peers[currpeer]); sprintf(sendload.load, "%u", prep); xQueueSend(sendmqttQueue, &sendload, 0);    //  then send prepped foto slot to current peer
        struct sendstct sendload{ currpeer, ""}; sprintf(sendload.load, "%u", prep); xQueueSend(sendmqttQueue, &sendload, 0);    //  then send prepped foto slot to current peer
      }

      //vTaskDelay(1);    //  wait some time to let send finish and then show
      // TODO this comes to fast so dely this some how or move this line into send task
      struct showstct show={ "user", 0, "" }; sprintf(show.nvsalias, "%slatest", peers[currpeer].data()); xQueueSend(showQueue, &show, 0);    //  show current peers latest foto with full refresh    

      prep = slots;

      Serial.println("timer up -> prep:" + String(prep) + " currpeer:" + String(currpeer) + " try to show " + String(show.nvsalias));    // todo make this debug
    }
    taskYIELD();
  }
}




void setup() {    //  when this int main() instead this does not compile
  


  /* no global task handles anymore only necessary for info cmd for high water mark.
  xTaskCreate( networkTas, "networkTas", 8192, NULL, 1, &networkTasHandle );    //  spawn network task to connect to wifi
  xTaskCreate( showTas, "showTas", 32768, NULL, 1, &showTasHandle );    //  spawn show task to show stuff on epaper
  xTaskCreate( wstas, "wstas", 32768, NULL, 1, &wstasHandle );    //  spawn web task to handle all web stuff
  xTaskCreate( servoTas, "servoTas", 4096, NULL, 1, &servoTasHandle );    //  now spawn async tasks
  xTaskCreate( sendmqttTas, "sendmqttTas", 32768, NULL, 1, &sendmqttHandle );    //  spawn mqtt message sender task apparently task has to have enough stack for every buffer so here > 15KB
  xTaskCreate( flanksTas, "flanksTas", 4096, NULL, 1, &flanksTasHandle );    //  spawn flanks task to handle presses  // TODO make stackdepth smaller perhaps
  */

  Serial.begin(115200);    //  serial requires delay or while(!Serial); so no output is lost
  //prefs.begin("prefs", false);    //  open preferences with namespace prefs in read write mode this is for wifi creds and stuff

  xTaskCreate( networkTas, "networkTas", 8192, NULL, 1, NULL );    //  spawn network task to connect to wifi
  xTaskCreate( wstas, "wstas", 32768, NULL, 1, NULL );    //  spawn web task to handle all web stuff
  xTaskCreate( servoTas, "servoTas", 4096, NULL, 1, NULL );    //  now spawn async tasks
  xTaskCreate( sendmqttTas, "sendmqttTas", 32768, NULL, 1, NULL );    //  spawn mqtt message sender task apparently task has to have enough stack for every buffer so here > 15KB
  xTaskCreate( flanksTas, "flanksTas", 4096, NULL, 1, NULL );    //  spawn flanks task to handle presses  // TODO make stackdepth smaller perhaps
  xTaskCreate( showTas, "showTas", 32768, NULL, 1, NULL );    //  spawn show task to show stuff on epaper



  // TODO MOVE THIS into wstas
  if(!prefs.getBytesLength("peers")) { const char def[][16] = {{"local"}}; prefs.putBytes("peers", def, sizeof(def)); }



  Serial.println("init done");
  // TODO add a boot screen of some sort currently the showTas does not support this 
  //memcpy_P(volatileBuff, epd_bitmap_xpwallp, 15000);    //  copy boot foto from PROGMEM to volatile buffer for fast access
}




void loop() {vTaskSuspend(NULL);}    //  all done in tasks so suspend loop
