

//  switched to arduinocli since platformio does not really support arduino core v3 eventually I really want to switch to idf and ditch arduino in the long term
//  `arduino-cli sketch new all-together-arduino`    init sketch
//  'arduino-cli core update-index'    fetches latest core index
//  'arduino-cli board search Adafruit Feather esp32s3'    find board
//  'arduino-cli core install esp32:esp32'    install matching core see Platform ID
//  'arduino-cli lib install -v --git-url 'url' '    install library from git set environment variable 'export ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true' use verbos to find liberary dir for edits
//  'arduino-cli compile -v --fqbn esp32:esp32:adafruit_feather_esp32s3 --build-path ./firmware -upload -p /dev/tty.usbmodem101 '    compile for fqbn form board search esp32:esp32:adafruit_feather_esp32s3 also puts binaries into firmware folder
//  use 'merged.bin' at adress 0x0 with https://espressif.github.io/esptool-js/ for web programming



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

#include <cstdarg>
#include <cstdio>

#include <unordered_map>    //  c++ stuff
#include <functional>
#include <string>


// ----------- TODO -----------
//
//  currently app uses ~1,4MB of flash perhaps reduce this to less than 1MB
//  currently 180KB internal Ram is remaining this is not too much
//
//  major restructuring concerns
//  - correct startup sequence eg wifi has to start first only then webserial and mqtt task can start
//
//  sartup order
//  - all tasks would like to send logs class has to be inited first
//  - network task is required for webserial and mqtt
//
// prpper boot screen would be nice eg. with  while (WiFi.status() != WL_CONNECTED) {delay(500); Serial.print("."); } and similar
//
// set a strong unique client id for mqtt to ensure broker wider unique id ids are not topic scoped!
// - mqttClient.setClientId(const char *clientId); // has to be <23bytes UTF-8 encoded string for MQTT 3.1.1 defaults to last 3 bytes of the MAC address in hex format
// - esp32s3 has only 6 bytes MAC see https://github.com/espressif/esp-idf/blob/800f141f94c0f880c162de476512e183df671307/components/efuse/esp32s3/esp_efuse_table.c#L694
// - what happens on session hijacking? is a last will message sent? will i just reconnect with same id and potentially fight indefinitely with hijacker?
//
// add ciritcal screen for very bad errors that require user intervention like dos detected, replay detected and so on ....
//
//
// ------------ obvious vulnerabilities ------------
//
// - replay/dos by publishing profilehashes wich then cause peers to register the true owner of this profilehash as offline
//   this is so cheap and is so impactful this has to be fixed
//   - perhaps add on receive of ownprofilehash in cleartext do full disconnect/reconnect cycle to reassert online status but this than allowes for dos amplification
//   - perhaps add on receive of ownprofilehash in cleartext show warning sceen and do nothing else and let user manually reconnect
//
// - dos by flooding topic with garbage
//   perhaps add a rate limit on processing messages and throw dos warning on screen
//   drop messages of wrong size without processing
//
// - replay old messages are not detected and are just accepted
//   add timestamp to messages and drop old messages
//
// - session hijack/takeover potentially suppresses last will message of true session owner or dublicates last will message when hijacker disconnects
//   causes other peers to send messages to a peer who is offline uneccessary traffic
//   this is not easy to fix and relies on mqtt broker configuration
//   best mitigation is to use a strong unique client id see above
//
//
// ------------- THIS SEEMS TO BE A WORKABLE SOLUTION -------------
//
// on connect
//  -> ask all peers for their profiles perhaps by simply sending own profile encrypted to each peer
//  -> remember who answered alias remember online peers
//
// on profile change
//  -> push profile to online peers
//
// on disconnect alias last will
//  -> send goodbye message to all online peers perhaps send own profilehash in cleartext since every peer online peer knows this without having to decrypt
//
// on recieve of known profilehash in cleartext
//  -> set this peer to offline
//
// on peer add
//  -> send own profile encrypted with new peers secret
//
// on recieve of encrypted profile and successfull decrypt and peer offline
//  -> set this peer to online
//  -> answer with own profile encrypted with this peers secret only answer to offline peers here other wise this is a endless loop ping pong of profiles
//
// issues
//  - curretnly A onconnect 1. message (ping) -> B on receive 2. message (answer to ping) -> A on receive 3. message (miss interpretation as ping) -> B has a as online so done
//    third message is redundant here since A already knows B is online by receiving the second message
//    differentiate between inital ping and ping answer e.g. append "you there" to inital ping, "sure sure" to ping answer, "look here" to lates foto, "me pretty" to profile changes
//    or more minimal just append "?" to inital ping and "!" to ping answer and profile changes, "" nothing to latest foto
//    so only answer with profile+! when message was profile+?
//    either way always include this appendix in message cypher and make all messages the same length to avoid leaking info whts beeing sent
//  - perhaps makes public who knows how many peers
//  - lots of messages on connect
//  - tracks online status of peers
//
// pros
//  - dont send unnecessary messages after initial sync
//  - tracks online status of peers
//
// ------------------------------------------------------------------
//
//
//
//
//

//  ---------- key insights while testing ----------
//
// - swithcing to     // if( xQueueReceive( comms::mqsendq, &send, 0 ) == pdPASS ) { ... } xTaskYIELD();
//   for all tasks makes the programm run in a very tight loop and starves the idle task so the watchdog is not reset and triggers
//   before the xTaskDelay(1) would allow the idle task to run
//   so for everytask without a queue use xTaskDelay(1) instead of taskYIELD() to allow idle task to run and reset watchdog
//
// - if( xQueueReceive( comms::mqsendq, &send, portMAX_DELAY ) == pdPASS ) { ... } does not block the cpu but just moves the task to waiting state until sth is in the queue
//   this is exactly what is wanted
//
// - xTaskCreate does, even with arduino, not place all tasks on core1 so use xTaskCreatePinnedToCore to ensure this
//   otherwiese make sure that all recources are thread safe e.g. nvs access use semaphores or mutexes
//   this RAII nvs::prefslock is very cool and so nice to use
//
// - MDNS is broken somehow!!
//
//


class comms {    //  this is for all the inter task communication via queues    essentially these are globals not sure this namespace thing is good practice
  public:
  inline static QueueHandle_t showq = nullptr;    //  provide public access to the queue handles instead of private and access functions
  inline static QueueHandle_t mqsendq = nullptr;    // c++17 inline variable avoids static member definition at global scope
  inline static QueueHandle_t servoq = nullptr;

  struct showstct { char ocupado[5]; uint8_t partial; char nvsalias[16]; };   //  make struct public so it can be used outside of class    this hard coded finite length stresses me in python me no have to worry me miss python
  struct mqsendstct { uint32_t peer; char load[16]; };
  struct servostct { char pos[4]; };    //  no struct required for servo queue

  static void init() {    //  no constructor instead have treat theses as global utils    also feels not so racey
    if (showq == nullptr) showq = xQueueCreate(5, sizeof(showstct));
    if (mqsendq == nullptr) mqsendq = xQueueCreate(5, sizeof(mqsendstct));
    if (servoq == nullptr) servoq = xQueueCreate(5, sizeof(servostct));
  }

  static bool toshowq(const char* ocupado, uint8_t partial, const char* format, ...) {
    if (!showq) return false;
    
    showstct show{};
    strncpy(show.ocupado, ocupado ? ocupado : "", sizeof(show.ocupado) - 1);
    show.partial = partial;

    va_list args;    //  this is basicly a snprintf into the struct member
    va_start(args, format);
    vsnprintf(show.nvsalias, sizeof(show.nvsalias), format, args);
    va_end(args);

    return xQueueSend(showq, &show, 0) == pdPASS;
  }

  static bool tomqsendq(uint32_t peer, const char* format, ...) {
    if (!mqsendq) return false;
    
    mqsendstct send{};
    send.peer = peer;

    va_list args;    //  this is basicly a snprintf into the struct member
    va_start(args, format);
    vsnprintf(send.load, sizeof(send.load), format, args);
    va_end(args);

    return xQueueSend(mqsendq, &send, 0) == pdPASS;
  }

  static bool toservoq(const char* pos) {
    if (!servoq) return false;
    servostct servo{};
    strncpy(servo.pos, pos ? pos : "", sizeof(servo.pos) - 1);
    return xQueueSend(servoq, &servo, 0) == pdPASS;
  }
};




class nvs {    //  this is mostly transparent and adds a cache for frequently accessed nvs values plus utils to edit these the cached values safely
  private:
  static inline SemaphoreHandle_t nvsmtx;
  static inline Preferences prefs;

  static void cachepeers() {    //  this rechaches from nvs or inits cache with default value
    size_t pb = prefs.getBytesLength("peers");
    if (pb) {    //  either load peers into cache
      peers.resize(pb / 8);
      //prefs.getBytes("peers", peers.data(), pb);
      saveprefs()->getBytes("peers", peers.data(), pb);
    }
    else {    //  or initialize peers with default value at index 0
      appendpeer("local");
    }
  }

  static void cachehkdfs() {
    //size_t hb = prefs.getBytesLength("hkdfs");
    size_t hb = saveprefs()->getBytesLength("hkdfs");
    if (hb) {    //  either load peers into cache
      hkdfs.resize(hb / 32);
      //prefs.getBytes("hkdfs", hkdfs.data(), hb);
      saveprefs()->getBytes("hkdfs", hkdfs.data(), hb);
    }
    else {    //  or initialize peers with default value at index 0
      appendhkdf("");    //  empty secret is this safe this feels like an undefinded read
    }
  }

  static void cacheslots() {
    //slots = prefs.getUInt("slots", 0);
    slots = saveprefs()->getUInt("slots", 0);
  }

  static void cachelogsverbosity() {
    //logsverbosity = prefs.getUShort("logsverbosity", 0);
    logsverbosity = saveprefs()->getUShort("logsverbosity", 0);
  }

  public:
  struct prefslock {    //  so cool this RAII helper this locks with contructor and unlocks with destructor    do not Preferences& p = *nvs::lock() this reference potentionally dangles    do not copy or move this perhpas add prefslock& operator=(const lockprefs&) = delete;
    prefslock()  { xSemaphoreTake(nvsmtx, portMAX_DELAY); }
    ~prefslock() { xSemaphoreGive(nvsmtx); }
    Preferences* operator->() { return &prefs; }    //  allows prefslock()->memberofprefs() syntax
  };

  static prefslock saveprefs() {    //  this returns a thread save proxy to prefs which unlocks when it goes out of scope
    return prefslock();
  }

  static inline std::vector<std::array<char, 8>> peers;    //  these have to be a continous array for nvs storage
  static inline std::vector<std::array<uint8_t, 32>> hkdfs;    
  static inline uint32_t slots;
  static inline uint16_t logsverbosity;    //  this is the verbosity level for logs to webserial

  static void init() {
    nvsmtx = xSemaphoreCreateMutex();
    prefs.begin("prefs", false);    //  open preferences with namespace prefs in read write mode
    cachepeers();
    cachehkdfs();
    cacheslots();
    cachelogsverbosity();
  }

  static char* appendpeer(const char* peername = nullptr) {    //  this appends and writes this vector to nvs
    std::array<char, 8> peer{};    //  zero initialise a fixed byte array for peer name ensures null termination and allowes continous storage in vector unlike std::string would this is necessary for nvs

    strncpy(peer.data(), peername ? peername : "", 7);   // copy directly from args and leave last byte for NUL
    peers.push_back(peer);    //  append peer to vector

    //prefs.putBytes("peers", peers.data(), peers.size() * 8);    //  write back peer list with new peer to nvs    todo check for write errors
    saveprefs()->putBytes("peers", peers.data(), peers.size() * 8);    //  write back peer list with new peer to nvs    todo check for write errors

    return peers.back().data();    //  echo added peer
  }

  static bool deletepeer(uint32_t i) {    //  this deletes a peer and writes this vector to nvs
    if (i > peers.size() || !i) return false;    //  invalid index
    peers.erase(peers.begin() + i );    //  this is slow but this is not a frequent operation
    peers.shrink_to_fit();    //  free unused memory technically not necessary perhaps even bad for fragmentation
    //return prefs.putBytes("peers", peers.data(), peers.size() * 8) == peers.size() * 8;    //  write back peer list without deleted peer to nvs
    return saveprefs()->putBytes("peers", peers.data(), peers.size() * 8) == peers.size() * 8;    //  write back peer list without deleted peer to nvs
  }

  static uint32_t findpeer(const char* peername) {    //  this finds a peer in the cached vector and returns its index or zero if not found
    if (!peername) return 0;
    
    for (size_t i = 1; i < peers.size(); ++i) {    //  find peer in list
        if (!strcmp(peers[i].data(), peername)) return i;
    }
    return 0;  // Not found
  }


  static uint8_t* appendhkdf(const char* secret = nullptr) {    //  this appends and writes this vector to nvs
    std::array<uint8_t, 32> hkdfout{};    //  zero initialise a fixed byte array for hkdf ensures null termination and allowes continous storage in vector unlike std::string would this is necessary for nvs
    
    hkdf<SHA256>(hkdfout.data(), 32, secret, strlen(secret), nullptr, 0, "nvsalias", strlen("nvsalias"));    //  derive 32 bytes as secret for encryption hkdf<SHA256>(outputbuff, sizeof(output), secret, sizeof(secret), salt, sizeof(salt), info, sizeof(info));
    hkdfs.push_back(hkdfout);    //  append hkdf to vector

    //prefs.putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32);    //  write back hkdf list with new hkdf to nvs    todo check for write errors
    saveprefs()->putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32);    //  write back hkdf list with new hkdf to nvs    todo check for write errors

    return hkdfs.back().data();    //  return added hkdf
  }

  static bool deletehkdf(uint32_t i) {    //  this deletes a hkdf and writes this vector to nvs
    hkdfs.erase(hkdfs.begin() + i );    //  this is slow but this is not a frequent operation
    hkdfs.shrink_to_fit();    //  free unused memory technically not necessary perhaps even bad for fragmentation
    //return prefs.putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32) == hkdfs.size() * 32;    //  write back peer list without deleted peer to nvs
    return saveprefs()->putBytes("hkdfs", hkdfs.data(), hkdfs.size() * 32) == hkdfs.size() * 32;    //  write back peer list without deleted peer to nvs
  }


  static uint32_t deleteslot(uint32_t delslot) {    //  this deletes a slot and writes this count into nvs
    if (delslot > slots) return 0;    //  slot out of range cant delete this is invalid
    
    char slotschar[12]; snprintf(slotschar, 12, "%u", slots);    //  hold the delslot/slots value as char this is required for nvs access
    char delslotchar[12]; snprintf(delslotchar, 12, "%u", delslot);

    if (delslot != slots) {    //  swap slot with top most slot
      uint8_t temp[15000];
      //prefs.getBytes( slotschar , temp, sizeof(temp) );    //  read top most slot into temp
      //prefs.putBytes( delslotchar , temp, sizeof(temp) );    //  copy temp to target slot
      saveprefs()->getBytes( slotschar , temp, sizeof(temp) );    //  read top most slot into temp
      saveprefs()->putBytes( delslotchar , temp, sizeof(temp) );    //  copy temp to target slot
    }
    //prefs.remove( slotschar );    //  remove top most slot
    saveprefs()->remove( slotschar );    //  remove top most slot
    slots--;
    //prefs.putUInt("slots", slots);    //  adjust / save slots count
    saveprefs()->putUInt("slots", slots);    //  adjust / save slots count

    return delslot;    //  echo deleted slot to confirm
  }

  static uint32_t appendslot(const char* slotchar, uint8_t* foto) {    //  this overwrites or adds a foto to nvs and writes corrected count to nvs
    uint32_t slotint = strtoul(slotchar, NULL, 10);
    if (slotint > slots) {
      slots = slotint;
      //prefs.putUInt("slots", slots);
      saveprefs()->putUInt("slots", slots);    //  todo this locks/unlocks a lot of times perhaps use 'auto saveprefs = lock()' then do 'saveprefs->....()' this  this locks once /unlocks once when going out of scope
    }
    //prefs.putBytes( slotchar , foto, 15000);    //  write foto to nvs at slot position
    saveprefs()->putBytes( slotchar , foto, 15000);    //  write foto to nvs at slot position
    return slots;    //  echo slot count to confirm
  }

  static bool putlogsverbosity(uint16_t verbosity) {    //  this sets the logs verbosity level and writes it to nvs
    logsverbosity = verbosity;
    //return prefs.putUShort("logsverbosity", logsverbosity) == logsverbosity;
    return saveprefs()->putUShort("logsverbosity", logsverbosity) == logsverbosity;
  }
};




class logs {    //  this is for logging to serial and perhaps webserial with verbosity levels
  private:
  inline static SemaphoreHandle_t logmtx;
  inline static WebSerial* wspointer = nullptr;    //  webserial instance is declared and initialized in wstas() logs may use this via reference to also log to webserial

  public:
  enum level : uint16_t { critical, warn, info, debug };    //  verbosity levels  critical is always shown

  static void init() {    //  no constructor instead have treat theses as global utils    also feels not so racey
    logmtx = xSemaphoreCreateMutex();
    Serial.begin(115200);    //  serial requires delay or while(!Serial) to not miss any output
  }

  static void attachws(WebSerial* ws = nullptr) {
    xSemaphoreTake(logmtx, portMAX_DELAY);    //  perhaps also check for pdTRUE here
    wspointer = ws;
    xSemaphoreGive(logmtx);
  }

  static void detachews() {
    xSemaphoreTake(logmtx, portMAX_DELAY);    //  perhaps also check for pdTRUE here
    wspointer = nullptr;
    xSemaphoreGive(logmtx);
  }

  static void feed(level verbosity, const char* format, ...) {    //  todo likly always add '\r\n' so most serial monitors and webserial show line brakes correctly
    if (verbosity > nvs::logsverbosity) return;    //  do not show meassages above user verbosity level

    va_list args;
    va_start(args, format);

    va_list msglen;
    va_copy(msglen, args);
    int len = vsnprintf(nullptr, 0, format, msglen);    //  find required size
    va_end(msglen);

    if (len < 0) { va_end(args); return; }    //  encoding error

    xSemaphoreTake(logmtx, portMAX_DELAY);    //  perhaps also check for pdTRUE here

    if (wspointer) {
      AsyncWebSocketMessageBuffer* wsBuffer = wspointer->makeBuffer(len + 1);    //  internal buffer avoids extra copy this requires plus one for null terminator    //  todo check for alloc success

      vsnprintf(reinterpret_cast<char*>(wsBuffer->get()), len + 1, format, args);    //  write directly into ws buffer

      Serial.write(reinterpret_cast<const uint8_t*>(wsBuffer->get()), len);    //  convenient this uses the same buffer no extra copy

      wspointer->send(wsBuffer);    //  send the exact length no trailing null terminator    todo is this correct without null terminator
    }
    else {    //  fallback for no webserial  todo consider a stack buffer here instead of dynamic std::string to relieve heap pressure
      std::string tmp;
      tmp.resize(len + 1);    //  resize std::string and leave space for null terminator this is required for vsnprintf
      vsnprintf(tmp.data(), len + 1, format, args);    //  format into tmp string

      Serial.write(reinterpret_cast<const uint8_t*>(tmp.data()), len);
    }
    va_end(args);

    xSemaphoreGive(logmtx);
  }
};




void networkTas(void *parameter) {    //  this connects to wifi or spawns an access point for configuration
  WiFi.mode(WIFI_STA);
  //WiFi.begin( nvs::prefs.getString("ssid", "fpaper"), nvs::prefs.getString("pass", "") );    //  return ssid from preferences nvs or return finger
  WiFi.begin( nvs::saveprefs()->getString("ssid", "fpaper"), nvs::saveprefs()->getString("pass", "") );    //  return ssid from preferences nvs or return finger

  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {    //  this waits for a default time and when not able to connect to ssid falls back to ap
    WiFi.mode(WIFI_AP);
    WiFi.softAP("fpaper", "");
    //logs::feed(logs::warn, "%s failed so fallback to soft ap fpaper. access webserial at http://%s/webserial \n\r", nvs::prefs.getString("ssid", "fpaper").c_str(), WiFi.softAPIP().toString().c_str());
    logs::feed(logs::warn, "%s failed so fallback to soft ap fpaper. access webserial at http://%s/webserial \n\r", nvs::saveprefs()->getString("ssid", "fpaper").c_str(), WiFi.softAPIP().toString().c_str());
    
    //Serial.println( nvs::prefs.getString("ssid", "fpaper") + " failed so fallback soft ap fpaper up so access webserial at http://" + WiFi.softAPIP().toString().c_str() + "/webserial \n");
    //if (MDNS.begin("fpaper")) { Serial.println("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
  } else {
    //logs::feed(logs::info, "%s success so access webserial at http://%s/webserial \n\r", nvs::prefs.getString("ssid", "fpaper").c_str(), WiFi.localIP().toString().c_str());
    logs::feed(logs::info, "%s success so access webserial at http://%s/webserial \n\r", nvs::saveprefs()->getString("ssid", "fpaper").c_str(), WiFi.localIP().toString().c_str());
    //Serial.println( nvs::prefs.getString("ssid", "fpaper") + " success so access webserial at http://" + WiFi.localIP().toString().c_str() + "/webserial \n");
    //if (MDNS.begin("fpaper")) { Serial.println("mDNS responder is up \n"); } //  this is to responde to fpaper.local for windows perhaps install bonjour to add service to mDNS use 'MDNS.addService("http", "tcp", 80);'
    vTaskDelete(NULL);    //  all done so delete this task
  }

  DNSServer dnsServer;    //  recently there was an api introduced for this see here https://github.com/espressif/arduino-esp32/blob/c2bd3c960dfcdc9dae8424452309aeef8a70a0eb/libraries/DNSServer/examples/CaptivePortal/CaptivePortal.ino#L39
  if ( dnsServer.start(53, "*", WiFi.softAPIP()) ) {    //  init dns server on port 53 with wildcard domain to map all requests to ap ip for captive portal
    logs::feed(logs::info, "dns server for captiveportal is up \n\r");
  } else {
    logs::feed(logs::critical, "eeee dns server for captiveportal failed \n\r");
  }
  while(true){
    dnsServer.processNextRequest();
    vTaskDelay(1);    //  unlike taskYIELD() this allows idle time this auto resets watchdog otherwise it timesout or we have to feed it manually
  }
}




void wstas(void *parameter) {    //  this spawns webserial and handles all web stuff
  AsyncWebServer server(80);
  static WebSerial ws;    //  make webserial static so it can be attached to logs class

  const std::unordered_map<std::string, std::function<void(std::string args)> > cmds = {    //  const map with compile time initialization to reduce heap pressure/fragmentation
    {"delslot", [&](std::string args) {    //  all these conversions feel wrong
      if (args.empty()) { ws.print("eeee delslot requires args\n"); return; }

      uint32_t delslot = strtoul(args.c_str(), NULL, 10);    //  this returns zero value for invalid input this is acceptable since zero slot does not exist slots a one indexed

      if (!delslot) { ws.printf("eeee '%s' not a slot \n", args.c_str()); return; }    //  prevent zero slot since this does not exist

      if (nvs::deleteslot(delslot)) ws.printf("deleted slot '%u'\n", delslot);
      else  ws.printf("eeee slot '%u' out of range\n", delslot);
      
      return;
    }},

    {"peer", [&](std::string args) {    //  with just peername this deletes peer and all associated data with aditional secret this adds/overwrites peer hkdf
      if (args.empty()) { ws.print("eeee peer requires args\n"); return; }
      if (strtoul(args.c_str(), NULL, 10)) { ws.print("eeee numbers not allowed\n"); return; }    //  prevent peer names which are numbers since these are reserved for slots

      std::string name = args.substr(0, args.find(' '));    //  this always is just the peer name either pos 0..7 or pos 0..nospc

      if (name.length() > 8) { ws.print("eeee peer name too long\n"); return; }    //  when peername too long return early

      uint32_t found = nvs::findpeer(name.c_str());    //  this checks for peer existence

      if (found && args.length() == name.length()) {    //  when peer exists and no secret provided so delete peer
        if ( !nvs::deletepeer(found) ) { ws.printf("eeee failed to delete peer '%s'\n", name.c_str()); return; }

        nvs::deletehkdf(found);    //  also delete associated hkdf
        //nvs::prefs.remove( (name + "latest").c_str() );    //  remove latest foto entry for this peer
        //nvs::prefs.remove( (name + "profile").c_str() );    //  remove profile foto of this peer
        nvs::saveprefs()->remove( (name + "latest").c_str() );    //  remove latest foto entry for this peer
        nvs::saveprefs()->remove( (name + "profile").c_str() );    //  remove profile foto of this peer

        ws.printf("deleted peer '%s'\n", name.c_str());
        return;
      }

      if (!found && args.length() != name.length() ) {    //  when peer does not exist and secret provided so add peer
        nvs::appendpeer(name.c_str());    //  add peer
        nvs::appendhkdf(args.c_str() + name.length() + 1);    //  add secret
      }
    }},

    {"wstime", [&](std::string args){    //  this sets time after wich webserial closes automatically    this is for this one friend who is on public wifi
      if (args.empty()) { ws.print("eeee wstime requires args\n"); return; }
      uint32_t wstime = strtoul(args.c_str(), NULL, 10);
      if (!wstime) { ws.printf("eeee '%s' not a number \n", args.c_str()); return; }
      //nvs::prefs.putUInt("wsalivesec", wstime);
      nvs::saveprefs()->putUInt("wsalivesec", wstime);

      ws.printf("closes after '%u' seconds\n", wstime);
    }},

    {"publ", [&](std::string args){    //  this manually publishes sth via the mqsendq
      if (args.empty()) { ws.print("eeee publ requires args\n"); return; }
      // TODO validate args and pass to mqsendq
    }},


    {"firmware", [&](std::string args){    // TODO either try link or set to auto for hardcoded link/default link
      if (args.empty()) { ws.print("eeee apt upgrade requires a link\n"); return; }
      //nvs::prefs.putBytes("airlink", args.c_str(), args.length());
      nvs::saveprefs()->putBytes("airlink", args.c_str(), args.length());
      ws.printf("set firmware url to '%s'\n", args.c_str());
    }},


    {"rm", [&](std::string args){    // todo add 'rm -rf' to clear nvs prefs.clear();
      if (args != "-rf") { ws.print("eeee rm requires args '-rf'\n"); return; }
      //nvs::prefs.clear();
      nvs::saveprefs()->clear();
      ws.print("cleared nvs \n");
    }},


    {"top", [&](std::string args){    // todo add 'top' , 'sit' to set servo positions prefs.putInt("top",
      if (args.empty()) { ws.print("eeee top requires args\n"); return; }
      uint32_t pos = strtoul(args.c_str(), NULL, 10);
      if (!pos) { ws.printf("eeee '%s' not a number \n", args.c_str()); return; }
      //nvs::prefs.putUInt("top", pos);
      nvs::saveprefs()->putUInt("top", pos);
      ws.printf("top set to '%u'\n", pos);
    }},

    {"sit", [&](std::string args){
      if (args.empty()) { ws.print("eeee sit requires args\n"); return; }
      uint32_t pos = strtoul(args.c_str(), NULL, 10);
      if (!pos) { ws.printf("eeee '%s' not a number \n", args.c_str()); return; }
      //nvs::prefs.putUInt("sit", pos);
      nvs::saveprefs()->putUInt("sit", pos);
      ws.printf("sit set to '%u'\n", pos);
    }},

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
      if (args.empty()) { ws.print("eeeee topic requires a value\n"); return; }
      //nvs::prefs.putString("mqtop", args.c_str());
      nvs::saveprefs()->putString("mqtop", args.c_str());
      ws.printf("MQTT topic set to '%s'\n", args.c_str());
    }},

    {"logs", [&](std::string args) {
      if (args.empty()) { ws.print("eeeee logs requires a level\n"); return; }
      nvs::putlogsverbosity(args == "critical" ? logs::critical : args == "warn" ? logs::warn : args == "info" ? logs::info : args == "debug" ? logs::debug : 0 );    //  default to error on invalid input  this is not pretty but simple
      ws.printf("logs level is '%u'\n", nvs::logsverbosity);
    }},
    
    {"ssid", [&](std::string args) {
      if (args.empty()) { ws.print("eeeee ssid requires a value\n"); return; }
      //nvs::prefs.putString("ssid", args.c_str());
      nvs::saveprefs()->putString("ssid", args.c_str());
      ws.printf("ssid set to '%s'\n", args.c_str());
    }},
    
    {"pass", [&](std::string args) {
      if (args.empty()) { ws.print("eeeee pass requires a value\n"); return; }
      //nvs::prefs.putString("pass", args.c_str());
      nvs::saveprefs()->putString("pass", args.c_str());
      ws.printf("Password set to '%s'\n", args.c_str());
    }},
    
    {"serv", [&](std::string args) {
      if (args.empty()) { ws.print("eeeee serv requires a URL\n"); return; }
      //nvs::prefs.putBytes("airlink", args.c_str(), args.length());    //  store firmware link as well
      nvs::saveprefs()->putBytes("mqserv", args.c_str(), args.length());
      ws.printf("MQTT server set to '%s'\n", args.c_str());
    }},
    
    {"restart", [&](std::string args) {
      ws.print("restarting ....\n");
      ESP.restart();
    }},
    
    {"help", [&](std::string args) {
      // TODO -------- add this Serial.println(__cplusplus); // Shows C++ standard version
      //  no filepath - small snippet
      //  Serial.printf("free heap: %u\n", ESP.getFreeHeap());
      //  Serial.printf("psram found: %d free psram: %u\n", psramFound(), ESP.getFreePsram());
      
      ws.print("\n \n"
          "\nwhen wlan fails an access point spawns \n"
          " ssid 'ssid'         sets wlan '" + nvs::saveprefs()->getString("ssid", "N.A.") + "' \n"
          " pass 'password'     sets password \n"
                                  
          "\nmqtt config \n");
      ws.print(
          " peer 'name' 'secret' adds peer '");

      for (const auto &p : nvs::peers) ws.printf("%s, ", p.data());

      ws.print("' \n"
          " serv 'mqtt://url'    sets server " + nvs::saveprefs()->getString("mqserv", "mqtt://broker.hivemq.com") + " \n"
          " topic 'mqtt/topic'   sets topic '" + nvs::saveprefs()->getString("mqtop", "fpaper/+") + "' \n"

          "\nservo config \n"
          " top  'servo pos'    sets top pos '"  + nvs::saveprefs()->getInt("top", 0) + "' \n"
          " sit  'servo pos'    sets sit pos '"  + nvs::saveprefs()->getInt("sit", 0) + "' \n"

          "\nother stuff \n"
          " help                prints this\n"
          " site 'seconds'      closes site zero infinite\n"
          " info                see some info \n"
          " publ 'text'         publish to mqtt \n"
          " logs 'level'        logs level '" + nvs::saveprefs()->getString("debuglevel", "info") + "' \n"
          " restart             well this restarts \n"
          " firmware 'link'     firmware url or auto \n"
          " rm 'flags'          chill this just clears preferences\n\n\n" );
    }}
  };


  ws.onMessage([&ws, &cmds](const std::string& stdstr) {    //  todo redo this with std::unordered_map<std::string, std::function<void(std::string_view args)> > cmds;

    logs::feed(logs::debug, "recieved '%s' and", stdstr.c_str());

    auto pos = stdstr.find(' ');
    auto it = cmds.find(stdstr.substr(0, pos));
    if (it != cmds.end()) {

      logs::feed(logs::debug, " found \n\r");

      it->second( (pos == std::string::npos) ? "" : stdstr.substr(pos + 1) );    //  call the function in the unordered map with arguments
    } else {
      logs::feed(logs::critical, " not found try 'help' \n\r");
    }
  });

  ws.begin(&server);    //  all callbacks are atteched so init webserial here

  logs::attachws(&ws);    //  attach webserial to logs


  server.on("/querySlots", HTTP_GET, [](AsyncWebServerRequest *request) {
    //char buff[16]; snprintf(buff, sizeof(buff), "%u", nvs::prefs.getUInt("slots", 0) + 1); request->send(200, "text/plain", buff);    //  send slot count plus one so user can add new fotos
    char buff[16]; snprintf(buff, sizeof(buff), "%u", nvs::saveprefs()->getUInt("slots", 0) + 1); request->send(200, "text/plain", buff);    //  send slot count plus one so user can add new fotos
  });

  server.on("/file", HTTP_POST,
    [](AsyncWebServerRequest* request) {},    // empty request handler - no response sent
    [&ws](AsyncWebServerRequest *request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
    static size_t totalSize = 0;    //  static so this is not reset on each chunck
    static char slot[12];    // static to persist across chunks this max is 'profile'
    static uint8_t rcvbuff[15000];    // static buffer allocated once  todo perhaps move this to psram later

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

      if ( !strcmp(slot, "profile") ) {    //  either save to local profile or to slot number
        //nvs::prefs.putBytes("localprofile", &rcvbuff, sizeof(rcvbuff));    //  write profile to nvs at 'profile' position
        nvs::saveprefs()->putBytes("localprofile", &rcvbuff, sizeof(rcvbuff));    //  write profile to nvs at 'profile' position
      } 
      else  {
        nvs::appendslot(slot, rcvbuff);    //  write foto to nvs at slot position alias &rcvbuff[0]
      }
      ws.print("foto saved to " + String(slot));
    }
  });

  server.onNotFound([](AsyncWebServerRequest* request) {    //  redirect all requests to webserial for captive portal request->redirect("/webserial"); does not work for captive portal
    request->send(200, "text/html", "<!DOCTYPE html><html><meta http-equiv='refresh' content='0; url=http://fpaper.local/webserial' /><head><title>Captive Portal</title></head><body><p>auto redirect failed http://" + WiFi.softAPIP().toString() + "/webserial </p></body></html>");
  });

  if ( MDNS.begin("fpaper") ) {    //  this is to responde to fpaper.local for windows perhaps install bonjour to add a service to mDNS use 'MDNS.addService("http", "tcp", 80);'
    logs::feed(logs::info, "mdns responder is up \n\r");
  } else {
    logs::feed(logs::warn, "mdns responder failed \n\r");
  }

  server.begin();

  MDNS.addService("http", "tcp", 80);    //  adds service to device discovery mdns DS this is just for compatibility

  logs::feed(logs::debug, "webserial site is up and attached to logs \n\r");

  //if ( nvs::prefs.getBytesLength("airlink") ) {    //  this tries auto firmware link or manual link once every boot    //  this works with redirects and insecure https source 'https://github.com/espressif/arduino-esp32/issues/9530#issuecomment-2090034699'
  if ( nvs::saveprefs()->getBytesLength("airlink") ) {    //  this tries auto firmware link or manual link once every boot    //  this works with redirects and insecure https source '
    //size_t len = nvs::prefs.getBytesLength("airlink");
    size_t len = nvs::saveprefs()->getBytesLength("airlink");
    char *airlink = (char*) malloc(len);    //  todo check for alloc success
    //nvs::prefs.getBytes("airlink", airlink, len);    //  read link from nvs
    nvs::saveprefs()->getBytes("airlink", airlink, len);    //  read link from nvs
    if ( !strcmp(airlink, "auto") ) { 
      const char* autolink = "hard coded default link here";  //  todo add default link here perhaps with
      airlink = (char*) realloc(airlink, sizeof(autolink) +1);
      strcpy(airlink, autolink);
    }
    else {
      //nvs::prefs.remove("airlink");    //  when not auto rm the airlink so just try this once
      nvs::saveprefs()->remove("airlink");    //  when not auto rm the airlink so just try this once
    }

    WiFiClientSecure secureClient;    //  replace all this with this here https://github.com/espressif/arduino-esp32/tree/master/libraries/Update/examples/HTTPS_OTA_Update
    HTTPUpdate up;

    secureClient.setInsecure();    //  this is to ignore ssl so theoretically some one can spoof github this is not good 
    up.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);    //  this is to follw link redirects other options are eg 'up.rebootOnUpdate(false);' or 'secureClient.setTimeout(5);'
    up.onStart([&ws]() { logs::feed(logs::critical, "overwrite firmware init download \n\r"); });
    up.onEnd([&ws]() { logs::feed(logs::critical, "firmware download success so restart to overwrite \n\r"); });
    up.onError([&ws, &up](int err) { logs::feed(logs::critical, "eeee %s \n\r", up.getLastErrorString().c_str()); });
    up.onProgress([&ws](int current, int total) { logs::feed(logs::info, "%d percent \n\r", 100.0 * current / total); });    //  to print percentage of download and pulse led yellow while updating perhaps prgressbar is cooler instead but have ro figure out how to do same line prints in webserial
    HTTPUpdateResult result = up.update(secureClient, airlink, "", [](HTTPClient *http) { });    //  to add sth to the http header use 'http->addHeader("Authorization", "{\"token\":\"noInitYet\"}");'
  
    //ws.print("auto firmware error (" + String(up.getLastError()) + ") " + up.getLastErrorString().c_str() + " check " + airlink.c_str() + " \n");
    logs::feed(logs::critical, "firmware error (%d) %s link was %s \n", up.getLastError(), up.getLastErrorString().c_str(), airlink);    //  usually auto restart prevents this line so just prints when no restart cause error
    //ws.printf("auto firmware error %s link was %s \n", up.getLastErrorString().c_str(), airlink);    //  usually auto restart prevents this line so just prints when no restart cause error
    free(airlink);
  } else { 
    logs::feed(logs::debug, "firmware not checked no airlink \n\r");
  }

  //uint32_t alivesecs = nvs::prefs.getUInt("wsalivesec", 0);    //  read alive seconds from nvs this is for the info cmd
  uint32_t alivesecs = nvs::saveprefs()->getUInt("wsalivesec", 0);    //  read alive seconds from nvs this is for the info cmd
  if ( alivesecs ) {    //  yield unitl wstime seconds elapsed then clean up webserial and stop web interface
    logs::feed(logs::info, "webserial closes after %u seconds \n\r", alivesecs);
    vTaskDelay( alivesecs * 1000UL );
    logs::feed(logs::info, "webserial closes immediately \n\r");
    logs::detachews();    //  detach webserial from logs so no more webserial calls    todo webserial is static so even wehn task is deleted webserial is still in scope right hope this is fine
    MDNS.end();    //  stop mDNS responder
    server.end();    //  stop server so callbacks are unregistered so ws is not used anymore
    vTaskDelete(NULL);     //  safe to delete task and destroy ws
  } else {    //  when zero do not timeout webserial suspend does not free memory so this is save for the callbacks aboveus perhaps vTaskDelete(NULL) is also fine here that also frees stack
    logs::feed(logs::info, "webserial stays alive forever \n\r");
    vTaskSuspend(NULL);
  }
}




void showTas(void *parameter) {    //  this handles the epaper
  static GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> display(GxEPD2_420_GDEY042T81(/*CS=D8*/ 45, /*DC=D3*/ 46, /*RST=D4*/ 47, /*BUSY=D2*/ 48));

  pinMode(7, OUTPUT); digitalWrite(7, HIGH);   //  give power to the panel
  display.init(115200);    // init epd with 115200 baud rate
  display.setRotation(0);    //  TODO make this a setting in preferences but also change selection/ditthered overlay aspect accordingly

  static uint8_t showBuff[15000];
  char ocupado[5];    //  save the screen ocupation state to prevent clashes

  logs::feed(logs::debug, "screen is initialised \n\r");
  comms::toshowq("user", false, "localprofile");    //  show local profile after restart

  while(true){
    comms::showstct show{}; if( xQueueReceive( comms::showq, &show, portMAX_DELAY ) == pdPASS ) {    //  just pops when queue not empty and returns pdPASS when something was recieved else returns pdFAIL

      if ( ocupado[0] && strcmp(ocupado, show.ocupado) && strcmp(show.ocupado, "user") ) {    //  this passes for no ocupado, user requests, requests with same occupation  everything else is pushed back into queue
        xQueueSend(comms::showq, &show, 0); continue;
        logs::feed(logs::warn, "pushback of show '%s' with occupation '%s' \n\r", show.nvsalias, show.ocupado);
      }

      strcpy(ocupado, show.partial ? show.ocupado : "");    //  clear ocupation for full refreshes for partial refreshes ocupie screen

      //if (!nvs::prefs.getBytes( show.nvsalias, showBuff, 15000 )) { Serial.println("nothing found for " + String(show.nvsalias)); esp_fill_random(showBuff, sizeof(showBuff)); }    //  for invalid nvs lookups this fills the showBuff with noise
      if (!nvs::saveprefs()->getBytes( show.nvsalias, showBuff, 15000 )) { logs::feed(logs::warn, "eeee nothing found for '%s' \n\r", show.nvsalias); esp_fill_random(showBuff, sizeof(showBuff)); }    //  for invalid nvs lookups this fills the showBuff with noise
      
      //if (strncmp(buff, "full", 4) == 0) {    //  show with full refresh
      if ( !show.partial ) {    //  show with full refresh
        display.setFullWindow();
        display.firstPage();
        do {
          display.fillScreen(GxEPD_BLACK);
          display.drawBitmap(0, 0, showBuff, display.width(), display.height(), GxEPD_WHITE);
        } while (display.nextPage());
      }

      if ( show.partial ) {    //  show picture in picture (center 100x100 of currently loaded showBuff)    //  check display.epd2.hasFastPartialUpdate with other screens
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
    taskYIELD();    //  more efficent than vTaskDelay(0) also fine here since this has a queue and allows idle time while waiting on queue    //  befor one second so no flicker just show every thing fast user interactions
  }
}




void servoTas(void *parameter) {    //  this handles servo movement
  ledcAttach(38, 50, 12);    //  50hz pwm at pin 38 with 12 bit resolution so 0-4095
  logs::feed(logs::debug, "servo is initialised \n\r");
  while(true){
    //char buff[4]; if( xQueueReceive( servoQueue, &buff, 0 ) == pdPASS ) {        // -------- TODO cache top and sit aswell !! these are used frequently better to cache them hope ram is enough perhaphs cache into psram  -----------------
    comms::servostct servo{}; if( xQueueReceive( comms::servoq, &servo, portMAX_DELAY ) == pdPASS ) {
      logs::feed(logs::debug, "servo move to '%s' \n\r", servo.pos);
      //ledcWrite(38, nvs::prefs.getInt("sit", 0)); vTaskDelay(500); String(buf) == "top" ? ledcWrite(38, nvs::prefs.getInt("top", 0)) : ledcWrite(38, prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0);    //  move servo to poses in preferences also cool c ternary operator
      //if (!strcmp(servo.pos, "top")) { ledcWrite(38, nvs::prefs.getInt("top", 0)); vTaskDelay(500); ledcWrite(38, nvs::prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }   //  wigle servo to poses in preferences always top and back to sit pose
      //if (!strcmp(servo.pos, "sit")) { ledcWrite(38, nvs::prefs.getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }  // move servo to sit pose
      if (!strcmp(servo.pos, "top")) { ledcWrite(38, nvs::saveprefs()->getInt("top", 0)); vTaskDelay(500); ledcWrite(38, nvs::saveprefs()->getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }   //  wigle servo to poses in preferences always top and back to sit pose
      if (!strcmp(servo.pos, "sit")) { ledcWrite(38, nvs::saveprefs()->getInt("sit", 0)); vTaskDelay(500); ledcWrite(38, 0); }  // move servo to sit pose
    
    }
    taskYIELD();
  }
}




void sendmqttTas(void *parameter) {    //  this handles all mqtt traffic
  PsychicMqttClient mqttClient;
  ChaChaPoly chachapoly;

  static uint8_t sendcyphy[12+16+15000+9];    //  this is the last sent message the first 12iv plus 16tag plus 15000foto plus 9profile/slot    //  this is written every outgoing message and read with every incoming so perhaps protect this with mutex/semaphore 

  //String serverAddress = nvs::prefs.getString("mqserv", "mqtt://broker.hivemq.com"); mqttClient.setServer(serverAddress.c_str());    // thanks chatgpt but why does this work but this 'mqttClient.setServer( prefs.getString("mqserv", "mqtt://broker.emqx.io").c_str() );' not work
  //if ( nvs::prefs.getBytesLength("mqserv") ) {    //  this uses the large snedcyphy buffer temporarily as a scratch buffer to read the server address from nvs
  //  nvs::prefs.getBytes("mqserv", sendcyphy, nvs::prefs.getBytesLength("mqserv"));
  if ( nvs::saveprefs()->getBytesLength("mqserv") ) {    //  this uses the large snedcyphy buffer temporarily as a scratch buffer to read the server address from nvs
    nvs::saveprefs()->getBytes("mqserv", sendcyphy, nvs::saveprefs()->getBytesLength("mqserv"));
  } else {
    strcpy( (char*)sendcyphy, "mqtt://broker.hivemq.com" );    //  default server
  }
  mqttClient.setServer( (char*)sendcyphy );    //  set server from nvs or default

  TaskHandle_t sendmqttHandle = xTaskGetCurrentTaskHandle();


  //mqttClient.onTopic( nvs::prefs.getString("mqtop", "fpaper/").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer      wildcards should work here listen one level deep for now TODO change this to only subscribe to peers
  mqttClient.onTopic( nvs::saveprefs()->getString("mqtop", "fpaper/").c_str() , 0, [&](const char *topic, const char *payload, int retain, int qos, bool dup) {    //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer      wildcards should work here listen one level deep for now TODO change this to only subscribe to peers
    logs::feed(logs::debug, "mqtt message preview '%.21s' at '%s' \n\r", payload, topic);    //  for binary this prints garbage and stops with first null terminator this is acceptable

    if ( !memcmp(sendcyphy, payload, 12) ) {    //  ignore echos no sens to decode echos  echos free the send task early  this seems racey but while publish there is no echo befor publish   TODO implement some check to avoid mitigate spam here eg some chek for known phrase or sth
      logs::feed(logs::debug, "plus this was an echo \n\r");
      xTaskNotifyGive(sendmqttHandle);    //  free the send task early
      return;    //  exit early for echos
    }

    // these non static to allow mutlible parallel callbacks perhaps better to serialize callbacks with a semaphore/mutex to relive stack pressure
    uint32_t index;    //  static initialises to zero also remember last successful peer index
    uint8_t rcvcyphy[15000];    //  TODO perhaps move these to psram or this is a bit large for stack so this is static perhaps better malloc and free int8_t* cyphy = (uint8_t*)malloc(15000);
    uint8_t temp[15000];     //  see comment abouveus
    //static uint32_t index;    //  static initialises to zero also remember last successful peer index
    //static uint8_t rcvcyphy[15000];    //  TODO perhaps move these to psram or this is a bit large for stack so this is static perhaps better malloc and free int8_t* cyphy = (uint8_t*)malloc(15000);
    //static uint8_t temp[15000];     //  see comment abouveus

    //Serial.println("Received message on topic: " + String(topic) );     //  TODO make this a debug log

    for (uint32_t attempts = nvs::hkdfs.size(); attempts; attempts--) {    //  first attempt is done with previous successful peer    peers.size() includes zero so first fail will then set index to top peer    following retries go through all peers in reverse    but dont try peer zero this is local peer so stop at peer one

      chachapoly.setIV( reinterpret_cast<const uint8_t*>(payload) , 12);
      chachapoly.setKey( nvs::hkdfs[index].data() , 32);
      chachapoly.decrypt(rcvcyphy, reinterpret_cast<const uint8_t*>(payload) + 12 + 16, 15000);    //  do not decrypt back to payload buffer since this is const

      if (!chachapoly.checkTag( payload + 12, 16)) {    //  for failed tag retry
        logs::feed(logs::debug, "decryption failed with peer '%s' \n\r", nvs::peers[index].data());
        index = attempts -1;  continue;  //  decrement index and retry
      }
      
      if ( !memcmp("look here", payload + 12 + 16 + 15000, 9) ) {    //  here compare recieved profile to saved profile and perhpas overwrite    also show recieved profile    also move servo 
        logs::feed(logs::info, "profile decryption successful of peer '%s' \n\r", nvs::peers[index].data());

        //char nvsalias[16]; sprintf(nvsalias, "%sprofile", nvs::peers[index].data()); nvs::prefs.getBytes( nvsalias, temp, 15000 );    //  read the profile of peer wich is at 'peer+profile'  TODO this or below decide!
        char nvsalias[16]; sprintf(nvsalias, "%sprofile", nvs::peers[index].data()); nvs::saveprefs()->getBytes( nvsalias, temp, 15000 ); 
        //char nvsalias[16]; snprintf(nvsalias, sizeof(nvsalias), "%.*sprofile", 8, peers[index].data()); prefs.getBytes( nvsalias, temp, 15000 );    //  read the profile of peer wich is at 'peer+profile' safer version to prevent buffer overflow when peer is not null terminated this reads at most eight chars
        
        if ( memcmp(temp, rcvcyphy, 15000) ) {
          //nvs::prefs.putBytes( nvsalias, rcvcyphy, 15000 );    //  when profile changes save recieved profile to peer wich is 'peer+profile' see abouve
          nvs::saveprefs()->putBytes( nvsalias, rcvcyphy, 15000 );    //  when profile changes save recieved profile to peer wich is 'peer+profile' see abouve
          logs::feed(logs::debug, "profile of peer '%s' overwritten \n\r", nvs::peers[index].data());
        } else {
          logs::feed(logs::debug, "profile of peer '%s' stayes \n\r", nvs::peers[index].data());
        }

        comms::toshowq(nvs::peers[index].data(), 1, "%s", nvs::peers[index].data());    //  show recieved profile with picture in picture and occupie screen with sending peer so no other message interferes
        comms::toservoq("top");    //  move servo to top position this wiggles screen
      }

      if ( !memcmp("see this ", payload + 12 + 16 + 15000, 9) ) {    //  here save recieved foto to nvsalias+'L'    also show this
        logs::feed(logs::info, "foto decryption successful of peer '%s' \n\r", nvs::peers[index].data());

        //char nvsalias[16]; snprintf(nvsalias, sizeof(nvsalias), "%slatest", nvs::peers[index].data()); nvs::prefs.putBytes( nvsalias, rcvcyphy, 15000 );    //  save foto of peer wich is 'peer+latest' safer version to prevent buffer overflow when peer is not null terminated this reads at most eight chars
        char nvsalias[16]; snprintf(nvsalias, sizeof(nvsalias), "%slatest", nvs::peers[index].data()); nvs::saveprefs()->putBytes( nvsalias, rcvcyphy, 15000 );    //  save foto of peer wich is 'peer+latest' safer version to prevent buffer overflow when peer is not null terminated this reads at most eight chars

        Serial.println("second decryption successfull");

        comms::toshowq(nvs::peers[index].data(), 0, "%s", nvs::peers[index].data());    //  show recieved foto with full refresh and ocupie screen with 'user' so no other message interferes
      }

      chachapoly.clear();  return;    //  exit lambda
    }
  });


  mqttClient.connect();
  logs::feed(logs::info, "initialised mqtt with address '%s' \n\r", (char*)sendcyphy);


  while (true) {
    comms::mqsendstct send{}; if( xQueueReceive( comms::mqsendq, &send, portMAX_DELAY ) == pdPASS ) {    //  reads first word out of queue when sth in queue

      logs::feed(logs::info, "mqtt send request to peer '%s' with load '%s' \n\r", nvs::peers[send.peer].data(), send.load);    //  this prints garbage for binary data in load but this is acceptable

      //if (!nvs::prefs.getBytes( send.load, 12+16+sendcyphy, 15000 )) {    //  for invalid nvs lookups this returns null and leaves cyphy
      if (!nvs::saveprefs()->getBytes( send.load, 12+16+sendcyphy, 15000 )) {    //  for invalid nvs lookups this returns null and leaves cyphy
        logs::feed(logs::warn, "nothing found for '%s' \n\r", send.load);  continue; 
      }

      if ( !send.peer ) {    //  when recipient local just save to local latest
        //nvs::prefs.putBytes( "locallatest", sendcyphy, 15000);    //  when recipient local save to local latest
        nvs::saveprefs()->putBytes( "locallatest", sendcyphy, 15000);    //  when recipient local save to local latest
      }

      else {        //  here when recipient not local actually do send stuff    // TODO somehow dont send full profile everytime you want to annoy
        esp_fill_random(sendcyphy, 12);    //  fill first 12 bytes of sendcyphy with noise to use as iv for chachapoly also to later in recieve mqtt determine wether message is a echo

        chachapoly.setIV(sendcyphy, 12);    //  use first 12 bytes of sendcyphy as iv
        chachapoly.setKey(nvs::hkdfs[send.peer].data(), 32);    //  
        chachapoly.encrypt( 12+16+sendcyphy, 12+16+sendcyphy, 15000);    //  encrypt clear bytes of load this was loaded into cyphy befor

        chachapoly.computeTag( 12+sendcyphy, 16);
        chachapoly.clear();

        memcpy(sendcyphy + 12 + 16 + 15000, strcmp(send.load, "localprofile") ? "look here" : "see this ", 9);    //  send our profile with 'look here' appendix or send foto slot with 'see this'    TODO send hash of peers profile to minimize messages

        logs::feed(logs::info, "packed payload try to send to peer '%s' \n\r", nvs::peers[send.peer].data());

        //mqttClient.publish( nvs::prefs.getString("mqtop", "fpaper/").c_str() , 0, 0, reinterpret_cast<const char*>(sendcyphy), 12 + 16 + 15000 + 9, true);     //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer       publish full length message to base topic
        mqttClient.publish( nvs::saveprefs()->getString("mqtop", "fpaper/").c_str() , 0, 0, reinterpret_cast<const char*>(sendcyphy), 12 + 16 + 15000 + 9, true);     //  TODO get dont use .c_str() here properly use a buffer and have getStrig read const char* into buffer       publish full length message to base topic

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(4000)) == 0) {    //  this blocks the task until notified or timeout this is to filter echos    todo optimization this technicaly does not have to wait on local sends
          logs::feed(logs::debug, "seen echo or timeout over \n\r");
        }
        else { 
          logs::feed(logs::debug, "timeout waiting for echo \n\r"); 
        }

      }

    }
    taskYIELD();
  }
}




void flanksTas(void *parameter) {    //  this is hopefully the same as using this lib in default asynchronous

  //static InterruptButton belowus(2, LOW, GPIO_MODE_INPUT, 750, 250, 5000, 8000);    //  ------  TODO switch to pin 20 here -------     pin, pressed low, pin mode, longpress ms, autorepeat ms, doubleclick ms, debounce us
  static InterruptButton belowus(2, LOW, GPIO_MODE_INPUT);

  InterruptButton::setMode(Mode_Synchronous);    // defaults to async wich executes immediate like an ISR, Synchronuse has to have a loop, hybrid does up/down events async and rest synchronous
  //InterruptButton::setMode(Mode_Hybrid);

  static volatile uint32_t lastpress = 0;
  static uint32_t prep = nvs::slots;    //  this is the prepared thing to send or show when timer elapses initally perp last slot so first increment shows  current peer
  static uint32_t currpeer;    //  initially peer is local static initialises to zero


  belowus.bind(Event_KeyPress, [](){    //  this is called after double click timeout so actually do the stuff here
    lastpress = millis(); if (!lastpress) lastpress = 1;    //  record last press time but never zero to prevent initialisation to register as a press

    logs::feed(logs::debug, "felt press \n\r");

    prep = ++prep % (nvs::slots+1);    //  cycle trough slots    zero slot is free and is reserved for current peer
    if (!prep) {    //  for zero show current peer
      logs::feed(logs::debug, "partial show '%sprofile' \n\r", nvs::peers[currpeer].data());
      comms::toshowq( "user", 1, "%sprofile", nvs::peers[currpeer].data());    //  show current peer with picture in picture onece every full cycle
    }
    else {    //  for non zero show foto slot
      logs::feed(logs::debug, "partial show slot '%u' \n\r", prep);
      comms::toshowq( "user", 1, "%u", prep );    //  show foto slot with picture in picture
    }
  });

  logs::feed(logs::info, "initialized flanksTas \n\r");

  while(true){
    InterruptButton::processSyncEvents();    //  only here for synchronous or hybrid

    if ( lastpress && (millis() - lastpress > 2000)) {    //  when no press for two seconds actually do the stuff here
      lastpress = 0;    //  disarm this until real press

      logs::feed(logs::debug, "confirm this \n\r");

      if (!prep) {    //  when prep is a peer advance peer and show this profile
        currpeer = ++currpeer % (nvs::peers.size()/16);    //  advance peer or wrap
        logs::feed(logs::debug, "advance to peer '%s' and show this profile \n\r", nvs::peers[currpeer].data());
        comms::toshowq( "user", 1, "%sprofile", nvs::peers[currpeer].data());
      }
      else {    //  when perep is a foto slot
        logs::feed(logs::debug, "send slot '%u' to peer '%s' \n\r", prep, nvs::peers[currpeer].data());
        if (currpeer) comms::tomqsendq( currpeer, "localprofile" );    //  first send our profile to current peer but not for local send
        comms::tomqsendq( currpeer, "%u", prep );    //  then send prepped foto slot to current peer
      }

      //vTaskDelay(1);    //  wait some time to let send finish and then show
      // TODO this comes to fast so dely this some how or move this line into send task
      logs::feed(logs::debug, "done with user input so return to '%slatest' \n\r", nvs::peers[currpeer].data());
      comms::toshowq( "user", 0, "%slatest", nvs::peers[currpeer].data() );    //  show current peers latest foto with full refresh
      prep = nvs::slots;
    }
    vTaskDelay(1);    //  unlike taskYIELD() this allows idle time this auto resets watchdog otherwise it timesout or we have to feed it manually
  }
}




void setup() {    //  when this int main() instead this does not compile


  //  TODO perhaps move these into tasks
  logs::init();    //  init logs and serial to not miss any messages this would require a delay or while(!Serial)
  nvs::init();    //  init prefs and loads cache values
  comms::init();    //  init inter task message queues

  //  this chooses cores freely so requires thread safety
  xTaskCreate( networkTas, "networkTas", 8192, NULL, 1, NULL );    //  spawn network task to connect to wifi
  xTaskCreate( wstas, "wstas", 32768, NULL, 1, NULL );    //  spawn web task to handle all web stuff
  xTaskCreate( servoTas, "servoTas", 4096, NULL, 1, NULL );    //  now spawn async tasks
  xTaskCreate( sendmqttTas, "sendmqttTas", 32768, NULL, 1, NULL );    //  spawn mqtt message sender task apparently task has to have enough stack for every buffer so here > 15KB
  xTaskCreate( flanksTas, "flanksTas", 4096, NULL, 1, NULL );    //  spawn flanks task to handle presses  // TODO make stackdepth smaller perhaps
  xTaskCreate( showTas, "showTas", 32768, NULL, 1, NULL );    //  spawn show task to show stuff on epaper

  /*
  // use this to test if crash potentially is related to thread safety
  xTaskCreatePinnedToCore(networkTas, "networkTas",   8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(wstas,      "wstas",       32768, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(servoTas,   "servoTas",     4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(sendmqttTas,"sendmqttTas", 32768, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(flanksTas,  "flanksTas",    4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(showTas,    "showTas",     32768, NULL, 1, NULL, 1);
  */

  logs::feed(logs::info, "done setup \n\r");
}




void loop() {vTaskSuspend(NULL);}    //  all done in tasks so suspend loop    todo is it possible to do vTaskDelete(NULL) here
