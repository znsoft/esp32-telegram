/*******************************************************************

   A Telegram bot for taking a photo with an ESP32Cam

   Parts used:
   ESP32-CAM module* - http://s.click.aliexpress.com/e/bnXR1eYs

    = Affiliate Links

   Note:
   - Make sure that you have either selected ESP32 Wrover Module,
           or another board which has PSRAM enabled
   - Choose "Huge App" partion scheme

   Some of the camera code comes from Rui Santos:
   https://randomnerdtutorials.com/esp32-cam-take-photo-save-microsd-card/

   Written by Brian Lough
    YouTube: https://www.youtube.com/brianlough
    Tindie: https://www.tindie.com/stores/brianlough/
    Twitter: https://twitter.com/witnessmenow
*******************************************************************/
//#define FORUPDATE 1
// ----------------------------
// Standard Libraries - Already Installed if you have ESP32 set up
// ----------------------------
#define USERGBMOTIONDETECT
#ifndef FORUPDATE
#include <Preferences.h>
Preferences prefs;
#endif
bool TBotNeedCamera;

TaskHandle_t MotionTask, TelegramBotTask;
SemaphoreHandle_t CameraReady;

//#include "SPIFS.h"

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
//#define FORMAT_SPIFFS_IF_FAILED true
//------- Replace the following! ------
#ifdef FORUPDATE
#undef USEWEBSERVER
#undef USETIMEZONE
#endif
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#ifdef USEWEBSERVER
#include <WebServer.h>
#endif
#include <HTTPUpdate.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#define TELEGRAM_DEBUG 1
#include <UniversalTelegramBot.h>
#ifndef FORUPDATE
#include "esp_camera.h"
framesize_t currentfsize;
camera_fb_t *fb = NULL;
uint8_t* fb_buffer;
size_t fb_length;
#endif
#include <ArduinoJson.h>


#ifndef FORUPDATE
#include "camera_pins.h"
#include "camera_code.h"
#ifdef USERGBMOTIONDETECT
#include "motion2.h"
#else
#include "motion.h"
#endif
#endif
#ifdef USEWEBSERVER
void startCameraServer();
#endif

// Wifi network station credentials
#include "passwords.h"

bool sendPhotos = true;
#define FLASH_LED_PIN 4

struct tm timeinfo;
time_t now;
#ifdef USEWDT
#include <Ticker.h>
// Loop WDT... please don't feed me...
// See lwdtcb() and lwdtFeed() below
Ticker lwdTimer;
#define LWD_TIMEOUT   60000

unsigned long lwdCurrentMillis = 0;
unsigned long lwdTimeOutMillis = LWD_TIMEOUT;
#endif
const unsigned long BOT_MTBS = 5500; // mean time between scan messages

unsigned long bot_lasttime; // last time messages' scan has been done
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

bool flashState = LOW;


int currentByte;

bool isMoreDataAvailable();
byte *getNextBuffer();
int getNextBufferLen();
uint8_t photoNextByte();
void  sendPhotoTask();

#ifdef USETIMEZONE
String getCurrentDateTimeToString();
#endif

void codeForMotionTask( void * parameter );
void codeForTBotTask( void * parameter );

bool dataAvailable = false;
// Motion Sensor
bool motionDetected = false;
bool ishwMotion = true;
bool isswMotion = true;
bool isUpdated = false;
bool isRestart = false;
bool debugToadmin = false;
bool debugToLed = false;
bool debugMotionDetect = false;


#ifdef USETIMEZONE
void StartTime() {

  //init_time();
  time(&now);
  setenv("TZ", "<+10>-10", 1);
  tzset();
  delay(1000);
  time(&now);
  Serial.print("After timezone : "); Serial.println(ctime(&now));

}

String getCurrentDateTimeToString() {

  char strftime_buf[64];

  time(&now);
  localtime_r(&now, &timeinfo);

  strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);
  return String(strftime_buf);
}
#endif
#ifndef FORUPDATE
void setupperf(String setValue) {
  prefs.begin("chatid"); // use "schedule" namespace
  if (setValue != "") {
    chatId = setValue;
    prefs.putString("chatid", chatId);
    return;
  }
  String _chatid = prefs.getString("chatid");
  if (_chatid == "")
    prefs.putString("chatid", chatId);
  else
    chatId = _chatid;

}

#endif

void StartTasks() {

  CameraReady = xSemaphoreCreateMutex();
#ifndef FORUPDATE
  xTaskCreatePinnedToCore(
    codeForMotionTask,
    "MotionTask",
    10000,
    NULL,
    1,
    &MotionTask,
    0);

  delay(500);
#endif
  xTaskCreatePinnedToCore(
    codeForTBotTask,
    "TBotTask",
    10000,
    NULL,
    2,
    &TelegramBotTask,
    1);

  delay(500);

}

void updateFW(String urlfile) {
  if (isUpdated == false)return;
  bot.sendMessage(chatadmin, "Start download file", "");
  isUpdated = false;
  Serial.println("Start update from " + urlfile);
  t_httpUpdate_return ret = httpUpdate.update(secured_client, urlfile);
  Serial.println("end download");
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      bot.sendMessage(chatadmin, httpUpdate.getLastErrorString(), "");
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      bot.sendMessage(chatadmin, "HTTP_UPDATE_NO_UPDATES", "");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      bot.sendMessage(chatadmin, "HTTP_UPDATE_OK", "");
      break;
  }

}

void bot_setup()
{
  const String commands = F("["
                            "{\"command\":\"help\",  \"description\":\"Get bot usage help\"},"
                            "{\"command\":\"start\", \"description\":\"Message sent when you open a chat with a bot\"},"
                            "{\"command\":\"photo\",\"description\":\"Answer device current status\"}" // no comma on last command
                            "]");
  bot.setMyCommands(commands);

  bot.sendMessage(chatadmin, "Start v1.95  "__DATE__, ""); //+" resetreason:"+(rtc_get_reset_reason(0)) , "");
  // bot.sendMessage(chatadmin, "rtc_get_reset_reason(0)
}

#ifndef FORUPDATE
void sendPhoto(String chat_id) {


  //sendDocumentTelegram(chat_id);
  fb = NULL;
  // Take Picture with Camera
  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    bot.sendMessage(chat_id, "Camera capture failed", "");
    return;
  }

  currentByte = 0;
  fb_length = fb->len;
  fb_buffer = fb->buf;

  dataAvailable = true;
  Serial.println("Sending");
  //bot.sendMessage(chat_id, getCurrentDateTimeToString(), "");
  if (sendPhotos) {
    bot.sendMultipartFormDataToTelegram("sendPhoto", "photo",  "photo.jpg",    "image/jpeg", chat_id, fb->len,
                                        isMoreDataAvailable,
                                        photoNextByte,
                                        nullptr,
                                        nullptr);
  } else {

    bot.sendMultipartFormDataToTelegram("sendDocument", "document",  "photo.jpg",    "image/jpeg", chat_id, fb->len,
                                        isMoreDataAvailable,
                                        photoNextByte,
                                        nullptr,
                                        nullptr);
  }
  /*photoNextByte
    bot.sendPhotoByBinary(chat_id, "image/jpeg", fb->len,
                        isMoreDataAvailable, nullptr,
                        getNextBuffer, getNextBufferLen);
  */
  Serial.println("done!");

  esp_camera_fb_return(fb);
  fb_length = NULL;
  fb_buffer = NULL;
}

void framesize(String value) {
  xSemaphoreTake( CameraReady, portMAX_DELAY );
  int val = atoi(value.c_str());
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, (framesize_t)val);
  currentfsize = (framesize_t)val;
  xSemaphoreGive( CameraReady );
}

bool isSendedFromInt = false;
// Indicates when motion is detected
static void IRAM_ATTR detectsMovement(void * arg) {
  //if(!ishwMotion)return;
  Serial.println("MOTION DETECTED!!!");

  motionDetected = ishwMotion;
  if (debugToadmin && debugMotionDetect && isSendedFromInt == false) {
    isSendedFromInt = true;
    bot.sendMessage(chatadmin, ".", "");
    isSendedFromInt = false;
  }
  if (debugToLed) {
    flashState = !flashState;
    digitalWrite(FLASH_LED_PIN, flashState);
  }
}
#endif
void handleNewMessages(int numNewMessages)
{
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++)
  {
    String chat_id = String(bot.messages[i].chat_id);
    String from_id = String(bot.messages[i].from_id);
    String file_path = String(bot.messages[i].file_path);
    String text = bot.messages[i].text;
    Serial.println(chat_id + " " + text);
    String from_name = bot.messages[i].from_name;
    if (from_name == "")
      from_name = "Guest";
    if (debugToadmin)bot.sendMessage(chatadmin, chat_id + " " + from_name + " " + from_id + " " + text +  " " + file_path, "");
#ifndef FORUPDATE
    if (chat_id != chatadmin && from_id == chatadmin) {
      if (text == "/setchanel")setupperf(chat_id);
    }
#endif
    if (chat_id == chatadmin && from_id == chatadmin) {
#ifndef FORUPDATE
      if (text.indexOf("/trashhold") >= 0) {
        text.replace("/trashhold", "");
        Image_thresholdL = atoi(text.c_str());;
      }

      if (text.indexOf("/framesize") >= 0) {
        text.replace("/framesize", "");
        framesize(text);
      }

      if (text == "/usehwmotion") {
        ishwMotion = !ishwMotion;
        bot.sendMessage(chatadmin, ishwMotion ? "On" : "Off", "");

      }


      if (text == "/useswmotion") {
        isswMotion = !isswMotion;
        bot.sendMessage(chatadmin, isswMotion ? "On" : "Off", "");

      }
      if (text == "/debugled") {
        debugToLed = !debugToLed;
        bot.sendMessage(chatadmin, debugToLed ? "On" : "Off", "");
      }
#endif
      if (text == "/debugadmin") {
        debugToadmin = !debugToadmin;
        bot.sendMessage(chatadmin, debugToadmin ? "On" : "Off", "");
      }
      if (text == "/debugmotion") {
        debugMotionDetect = !debugMotionDetect;
        bot.sendMessage(chatadmin, debugMotionDetect ? "On" : "Off", "");
      }
      if (text == "/restart" || text == "/reset")isRestart = true;
      if (text == "/fw" || text == "/update") {
        isUpdated = true;
        bot.sendMessage(chatadmin, "Ready to recv file for update firmware...", "");
      }
      if (file_path != "")updateFW(bot.messages[i].file_path);
      if (text == "/flash")
      {
        flashState = !flashState;
        digitalWrite(FLASH_LED_PIN, flashState);
        bot.sendMessage(chat_id, flashState ? "On" : "Off", "");
      }


      bot.messages[i].file_path = "";// bugfix lib

      if (text == "/start")
      {
        String welcome = "Welcome to bot.\n\n";
        welcome += "/restart : will restart\n";
        welcome += "/fw : after cmd upload binary firmware\n";
        welcome += "/debugmotion : debug motion\n";
        welcome += "/debugadmin : echo bot messages\n";
        welcome += "/debugled : debug motion detect by led\n";
        welcome += "/setchanel : remember chat as master\n";
        welcome += "/usehwmotion : use hardware motion detector or PIR sensor\n";
        welcome += "/useswmotion : use software motion detector \n";
        welcome += "/framesize : and number 0 to 15\n";
        welcome += "/trashhold : and number 0 to 100\n";
        welcome += "/photo : will take a photo\n";
        welcome += "/flash : toggle flash LED (VERY BRIGHT!)\n";
        bot.sendMessage(chat_id, welcome, "Markdown");
        return;
      }
    }
#ifndef FORUPDATE
    if (text.indexOf("/photo") >= 0)
    {
      sendPhotoTask();
      bot.sendMessage(chat_id,  "sending, wait...", "");
      //xSemaphoreTake( CameraReady, portMAX_DELAY );
      // sendPhoto(chat_id);
      // xSemaphoreGive( CameraReady );
    }
#endif
    if (text.indexOf("/start") >= 0)
    {
      String welcome = "Welcome to bot.\n\n";
      welcome += "/photo : will take a photo\n";
      //welcome += "/flash : toggle flash LED (VERY BRIGHT!)\n";
      bot.sendMessage(chat_id, welcome, "Markdown");
    }
  }
}
#ifndef FORUPDATE
bool isMoreDataAvailable() {
  return (fb_length - currentByte);
}

uint8_t photoNextByte() {
  currentByte++;
  return (fb_buffer[currentByte - 1]);
}


bool _isMoreDataAvailable()
{
  if (dataAvailable)
  {
    dataAvailable = false;
    return true;
  }
  else
  {
    return false;
  }
}

byte *getNextBuffer()
{
  if (fb)
  {
    return fb->buf;
  }
  else
  {
    return nullptr;
  }
}

int getNextBufferLen()
{
  if (fb)
  {
    return fb->len;
  }
  else
  {
    return 0;
  }
}
#endif

#ifdef USEWDT
// Our new WDT to help prevent freezes
// code concept taken from https://sigmdel.ca/michel/program/esp8266/arduino/watchdogs2_en.html
void ICACHE_RAM_ATTR lwdtcb(void) {
  if ((millis() - lwdCurrentMillis > LWD_TIMEOUT) || (lwdTimeOutMillis - lwdCurrentMillis != LWD_TIMEOUT)) ESP.restart();
   // RestartESP("Loop WDT Failed!");
}

void lwdtFeed(void) {
  lwdCurrentMillis = millis();
  lwdTimeOutMillis = lwdCurrentMillis + LWD_TIMEOUT;
}
#endif

void VerifyWifi() {
  while (WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0))
    WiFi.reconnect();
}


void setup()
{

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState); //defaults to low
#ifndef FORUPDATE
  setupperf("");
  if (!setupCamera())
  {
    Serial.println("Camera Setup Failed!");
    while (true)
    {
      delay(100);
    }
  }
#endif
  // attempt to connect to Wifi network:
  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
//  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
    i++;

    if (i > 500) {
      Serial.print("WIFI not found , restart");
      ESP.restart();
    }
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
#ifdef USETIMEZONE
  Serial.print("Retrieving time: ");
  configTime(0, 0, "pool.ntp.org"); // get UTC time via NTP
  now = time(nullptr);

  while (now < 24 * 3600)
  {
    Serial.print(".");
    delay(100);
    now = time(nullptr);
  }
  StartTime();
  Serial.println(now);
  Serial.println(getCurrentDateTimeToString());
#endif
  // Make the bot wait for a new message for up to 60seconds
  bot.longPoll = 1;
  //startCameraServer();
  bot_setup();
  // PIR Motion Sensor mode INPUT_PULLUP
  //err = gpio_install_isr_service(0);
  //gpio_pullup_dis(GPIO_NUM_13);
  //  gpio_pullup_en(GPIO_NUM_13);
#ifndef FORUPDATE
  gpio_pulldown_en(GPIO_NUM_13);
  esp_err_t  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);
  if (err != ESP_OK) {
    Serial.printf("handler add failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
#endif
  secured_client.setInsecure();
  //setupOTA();
#ifdef USEWDT
  lwdtFeed();
  lwdTimer.attach_ms(LWD_TIMEOUT, lwdtcb);
#endif  
#ifdef USEWEBSERVER
  startCameraServer();
#endif
#ifndef FORUPDATE
  isMotionDetected(currentfsize);
#endif
  //TRIGGERtimer = millis();
  StartTasks();
}



#ifndef FORUPDATE
void  sendPhotoTask()  {
  Serial.println("+");
  TBotNeedCamera = true;
  xSemaphoreTake( CameraReady, portMAX_DELAY );
  returnTheCamera(currentfsize);

  if (debugToLed) {
    flashState = LOW;
    digitalWrite(FLASH_LED_PIN, flashState);
  }
  sendPhoto(chatId);
  motionDetected = false;
  TBotNeedCamera = false;
  xSemaphoreGive( CameraReady );
}

void SendMotion() {
  // Serial.println("-");
  //if (TBotNeedCamera)return;
  //Serial.println("-");
  if (motionDetected && !isUpdated) {

    sendPhotoTask();
  }
}
#endif

void loop()
{
#ifdef USEWDT
lwdtFeed();
#endif
VerifyWifi();
 // if (WiFi.status() != WL_CONNECTED)ESP.restart();
  if (isRestart)ESP.restart();



}




void codeForMotionTask( void * parameter )
{
#ifndef FORUPDATE
  for (;;) {
    if (isswMotion && !isUpdated)
      if (!TBotNeedCamera) {
        xSemaphoreTake( CameraReady, portMAX_DELAY );
        motionDetected =  isMotionDetected(currentfsize);
        if (motionDetected) TBotNeedCamera = true;
        xSemaphoreGive( CameraReady );
        //if (motionDetected && !TBotNeedCamera) SendMotion();
      }
    delay(1);
    #ifdef USEWDT
    lwdtFeed();
    #endif
  }
#endif
}

void codeForTBotTask( void * parameter )
{



  for (;;) {
#ifndef FORUPDATE
    SendMotion();
#endif
    if (millis() - bot_lasttime > BOT_MTBS)
    {

      Serial.println("getUpdates");

      //SendMotion();
      int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      while (numNewMessages)
      {
        //SendMotion();
        Serial.print("numNewMessages=");
        Serial.println(numNewMessages);
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        Serial.println ("NewMessages handled");

      }
      /*
        bool needToRestart = true;
        for (int i = 0; i < 15; i++) {
         if (!bot.getMe()) continue;
        needToRestart = false;
        break;
        }
        if (needToRestart) {

        Serial.println ("reset by !bot.getMe()");
        ESP.restart();
        }*/
      bot_lasttime = millis();
    }
    delay(1);
    #ifdef USEWDT
    lwdtFeed();
    #endif
  }
}
