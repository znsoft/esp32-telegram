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

// ----------------------------
// Standard Libraries - Already Installed if you have ESP32 set up
// ----------------------------
#include <Preferences.h>
Preferences prefs;


//#include "SPIFS.h"

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
//#define FORMAT_SPIFFS_IF_FAILED true
//------- Replace the following! ------

//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <HTTPUpdate.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <UniversalTelegramBot.h>
#include "esp_camera.h"
#include <ArduinoJson.h>
#include "camera_pins.h"
#include "camera_code.h"

void startCameraServer();
// Wifi network station credentials
#include "passwords.h"
//#define WIFI_SSID ""
//#define WIFI_PASSWORD ""
//#define BOT_TOKEN ""
//String chatadmin = "";
//String chatId = "";
bool sendPhotos = true;
#define FLASH_LED_PIN 4
struct tm timeinfo;
time_t now;
const unsigned long BOT_MTBS = 5500; // mean time between scan messages

unsigned long bot_lasttime; // last time messages' scan has been done
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

bool flashState = LOW;

camera_fb_t *fb = NULL;
uint8_t* fb_buffer;
size_t fb_length;
int currentByte;

bool isMoreDataAvailable();
byte *getNextBuffer();
int getNextBufferLen();
uint8_t photoNextByte();
String getCurrentDateTimeToString();

bool dataAvailable = false;
// Motion Sensor
bool motionDetected = false;
bool ishwMotion = true;
bool isUpdated = false;
bool isRestart = false;
bool debugToadmin = false;
bool debugToLed = false;
bool debugMotionDetect = false;


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





void updateFW(String urlfile) {
  if (isUpdated == false)return;
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
  bot.sendMessage(chatadmin, "Start v1.3  " + getCurrentDateTimeToString(), "");
}


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
  bot.sendMessage(chat_id, getCurrentDateTimeToString(), "");
  if(sendPhotos){
  bot.sendMultipartFormDataToTelegram("sendPhoto", "photo", getCurrentDateTimeToString() + ".jpg",    "image/jpeg", chat_id, fb->len,
                                      isMoreDataAvailable,
                                      photoNextByte,
                                      nullptr,
                                      nullptr);
                                      }else{
  
  bot.sendMultipartFormDataToTelegram("sendDocument", "document", getCurrentDateTimeToString() + ".jpg",    "image/jpeg", chat_id, fb->len,
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

  int val = atoi(value.c_str());
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, (framesize_t)val);
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
    if (chat_id != chatadmin && from_id == chatadmin) {
      if (text == "/setchanel")setupperf(chat_id);
    }
    if (chat_id == chatadmin && from_id == chatadmin) {


      if (text.indexOf("/framesize") >= 0) {
        text.replace("/framesize", "");
        framesize(text);
      }
      if (text == "/usehwmotion") {
        ishwMotion = !ishwMotion;
        bot.sendMessage(chatadmin, ishwMotion ? "On" : "Off", "");
        
        }
      if (text == "/debugled") {
        debugToLed = !debugToLed;
        bot.sendMessage(chatadmin, debugToLed ? "On" : "Off", "");
      }
      if (text == "/debugadmin") {
        debugToadmin = !debugToadmin;
        bot.sendMessage(chatadmin, debugToadmin ? "On" : "Off", "");
      }
      if (text == "/debugmotion") {
        debugMotionDetect = !debugMotionDetect;
        bot.sendMessage(chatadmin, debugMotionDetect ? "On" : "Off", "");
      }
      if (text == "/restart" || text == "/reset")isRestart = true;
      if (text == "/fw" || text == "/update") isUpdated = true;
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
        welcome += "/framesize : and number 0 to 15\n";
        welcome += "/photo : will take a photo\n";
        welcome += "/flash : toggle flash LED (VERY BRIGHT!)\n";
        bot.sendMessage(chat_id, welcome, "Markdown");
        return;
      }
    }

    if (text.indexOf("/photo") >= 0)
    {
      sendPhoto(chat_id);

    }

    if (text.indexOf("/start")>= 0)
    {
      String welcome = "Welcome to bot.\n\n";
      welcome += "/photo : will take a photo\n";
      //welcome += "/flash : toggle flash LED (VERY BRIGHT!)\n";
      bot.sendMessage(chat_id, welcome, "Markdown");
    }
  }
}

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

void setup()
{

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState); //defaults to low
  setupperf("");
  if (!setupCamera())
  {
    Serial.println("Camera Setup Failed!");
    while (true)
    {
      delay(100);
    }
  }

  // attempt to connect to Wifi network:
  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

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
  // Make the bot wait for a new message for up to 60seconds
  bot.longPoll = 60;
  startCameraServer();
  bot_setup();
  // PIR Motion Sensor mode INPUT_PULLUP
  //err = gpio_install_isr_service(0);
  //gpio_pullup_dis(GPIO_NUM_13);
  //  gpio_pullup_en(GPIO_NUM_13);
  gpio_pulldown_en(GPIO_NUM_13);
  esp_err_t  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);
  if (err != ESP_OK) {
    Serial.printf("handler add failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  secured_client.setInsecure();
  //setupOTA();
  startCameraServer();


}

void SendMotion() {

  if (motionDetected) {
    if (debugToLed) {
      flashState = LOW;
      digitalWrite(FLASH_LED_PIN, flashState);
    }
    sendPhoto(chatId);
    motionDetected = false;
  }
}

void loop()
{

  if(WiFi.status() != WL_CONNECTED)ESP.restart();
  if (isRestart)ESP.restart();
  SendMotion();
  if (millis() - bot_lasttime > BOT_MTBS)
  {

  
    
    //SendMotion();
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages)
    {

      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);


    }

    if(!bot.getMe())ESP.restart();//ping 

    bot_lasttime = millis();
  }
}
