/**************************************************************************************************

    Motion detection from camera image - 08Sep20

    original code from: https://eloquentarduino.github.io/2020/01/motion-detection-with-esp32-cam-only-arduino-version/


   This works by capturing a greyscale image from the camera, splitting this up in to blocks of pixels (BLOCK_SIZE x BLOCK_SIZE)
   then reading all the pixel values inside each block and producing an average value for the block.
   The previous frames block values are then compared with the current and the number of blocks which have changed beyond
   a threshold (dayBlock_threshold) are counted.  If enough of the blocks have changed between two thresholds (dayImage_thresholdL and H)
   then motion is detected.
   - Many thanks to eloquentarduino for creating this code and for taking the time to answer my questions whilst I was
     developing this security camera sketch.

   For info on the camera module see: https://github.com/espressif/esp32-camera


 **************************************************************************************************/




#include "esp_camera.h"       // https://github.com/espressif/esp32-camera

//typedef void (*MotionStart)();
// Image Settings
#define FRAME_SIZE_MOTION FRAMESIZE_QVGA     // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA - Do not use sizes above QVGA when not JPEG
       // Image sizes: 160x120 (QQVGA), 128x160 (QQVGA2), 176x144 (QCIF), 240x176 (HQVGA), 320x240 (QVGA), 400x296 (CIF), 640x480 (VGA, default), 800x600 (SVGA), 1024x768 (XGA), 1280x1024 (SXGA), 1600x1200 (UXGA)
#define BLOCK_SIZE 20                        // size of image blocks used for motion sensing (20)
bool isDetectMode = false;
float cameraImageExposure = 900;             // Camera exposure (loaded from spiffs)
float cameraImageGain = 2;                 // Image gain (loaded from spiffs)
uint8_t cameraImageInvert = 0;                         // flip image vertically (i.e. upside down), 1 or 0

int8_t cameraImageBrightness = 2;                      // image brighness (-2 to 2) - Note: has no effect?

int8_t cameraImageContrast = 2;                        // image contrast (-2 to 2) - Note: has no effect?

float thresholdGainCompensation = 0.67;                // motion detection level compensation for increased noise in image when gain increased (i.e. in darker conditions)
uint32_t TRIGGERtimer = 0;                 // used for limiting camera motion trigger rate
uint16_t TriggerLimitTime = 2;             // min time between motion detection triggers (seconds)
//   ---------------------------------------------------------------------------------------------------------------------


// detection parameters (these are set by user and stored in Spiffs)
uint16_t targetBrightness = 120;      // Brightness level which is aimed to maintain by adjustment of camera settings
uint16_t Block_threshold = 20;        // average pixel variation in block required to count as changed - range 0 to 255
uint16_t Image_thresholdL = 15;       // min changed blocks in image required to count as motion detected in percent
uint16_t Image_thresholdH = 100;      // max changed blocks in image required to count as motion detected in percent

// misc
#define WIDTH 320                 // motion sensing frame size
#define HEIGHT 240
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)


uint16_t tCounter = 0;            // count number of consecutive triggers (i.e. how many times in a row movement has been detected)
uint16_t tCounterTrigger = 2;     // number of consequitive triggers required to count as movement detected
uint16_t AveragePix = 0;          // average pixel reading from captured image (used for nighttime compensation) - bright day = around 120
const uint16_t blocksPerMaskUnit = 16;    // number of blocks in each of the 12 detection mask units
// expected variables:  cameraImageBrightness, cameraImageInvert, cameraImageContrast, thresholdGainAdjust

// store most current motion detection reading for display on main page
uint16_t latestChanges = 0;

// frame stores (blocks)
uint16_t prev_frame[H][W] = { 0 };      // previously captured frame
uint16_t current_frame[H][W] = { 0 };   // current frame

// Image detection mask (i.e. if area of image is enabled for use when motion sensing, 1=active)
//   4 x 3 grid results in mask areas of 16 blocks (4x4) - image = 320x240 pixels, blocks = 20x20 pixels
const uint8_t mask_columns = 4;         // columns in detection mask
const uint8_t mask_rows = 3;            // rows in detection mask
uint16_t mask_active = 12;              // number of mask sections active
const uint16_t maskBlockWidth = W / 4;  // number of blocks in each mask area
const uint16_t maskBlockHeight = H / 3;
bool mask_frame[mask_columns][mask_rows] = { {1, 1, 1},
  {1, 1, 1},
  {1, 1, 1},
  {1, 1, 1}
};

// forward delarations

bool capture_still(framesize_t oldfsize);
float motion_detect();
void update_frame();
bool block_active(uint16_t x, uint16_t y);
bool isMotionDetected(framesize_t oldfsize);
bool cameraImageSettings(framesize_t fsize, bool isMotionDetect);
void RestartCamera(pixformat_t format, framesize_t fsize);

void RebootCamera(pixformat_t format, framesize_t fsize);

// ----------------------------------------------------------------
//              -restart the camera in different mode
// ----------------------------------------------------------------
// switches camera mode - format = PIXFORMAT_GRAYSCALE or PIXFORMAT_JPEG

void RestartCamera(pixformat_t format, framesize_t fsize) {

  esp_camera_deinit();
  config.frame_size = fsize;
  //else if (format == PIXFORMAT_GRAYSCALE) config.frame_size = FRAME_SIZE_MOTION;
  //else Serial.println("ERROR: Invalid image format");
  config.pixel_format = format;
  bool ok = esp_camera_init(&config);
  if (ok == ESP_OK) Serial.println("Camera mode switched ok");
  else {
    // failed so try again
    esp_camera_deinit();
    delay(50);
    ok = esp_camera_init(&config);
    if (ok == ESP_OK) Serial.println("Camera mode switched ok - 2nd attempt");
    else {
      Serial.println("Camera failed to restart so rebooting camera");        // store in bootlog
      RebootCamera(format, fsize);
    }
  }
  TRIGGERtimer = millis();        // reset last image captured timer (to prevent instant trigger)
}


// reboot camera (used if camera is failing to respond)
//      restart camera in motion mode, capture a test frame to check it is now responding ok
//      format = PIXFORMAT_GRAYSCALE or PIXFORMAT_JPEG

void RebootCamera(pixformat_t format, framesize_t fsize) {

  Serial.println("ERROR: Problem with camera detected so resetting it");
  // turn camera off then back on
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  delay(300);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(300);
  RestartCamera(PIXFORMAT_GRAYSCALE, FRAME_SIZE_MOTION);   // restart camera in motion mode
  delay(300);
  // try capturing a frame, if still problem reboot esp32
  if (!capture_still(fsize)) {
    Serial.println("Camera failed to reboot so rebooting esp32");    // store in bootlog
    delay(500);
    ESP.restart();
    delay(5000);      // restart will fail without this delay
  }
  if (format == PIXFORMAT_JPEG) RestartCamera(PIXFORMAT_JPEG, fsize);                 // if jpg mode required restart camera again
}



/**
   For serial debugging
   @param frame
*/
void print_frame(uint16_t frame[H][W]) {
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      Serial.print(frame[y][x]);
      Serial.print('\t');
    }

    Serial.println();
  }
}
//   runs every few seconds, called from loop
void AutoAdjustImage() {
  float exposureAdjustmentSteps = (cameraImageExposure / 25) + 0.2;    // adjust by higher amount when at higher level (25 / 0.2 = default)
  float gainAdjustmentSteps = 0.5;
  float hyster = 20.0;                                                 // Hysteresis on brightness level
  if (AveragePix > (targetBrightness + hyster)) {
    // too bright
    if (cameraImageGain > 0) cameraImageGain -= gainAdjustmentSteps;
    else cameraImageExposure -= exposureAdjustmentSteps;
  }
  if (AveragePix < (targetBrightness - hyster)) {
    // too dark
    if (cameraImageExposure >= 1200) cameraImageGain += gainAdjustmentSteps;
    else cameraImageExposure += exposureAdjustmentSteps;
  }
  Serial.println(cameraImageExposure);
    Serial.println(cameraImageGain);
  // check for over scale
  if (cameraImageExposure < 0) cameraImageExposure = 0;
  if (cameraImageExposure > 1200) cameraImageExposure = 1200;
  if (cameraImageGain < 0) cameraImageGain = 0;
  if (cameraImageGain > 30) cameraImageGain = 30;


}


bool isMotionDetected(framesize_t oldfsize) {
  Serial.println(" motion detector start");
  if (!capture_still(oldfsize)) return false;                                // capture image, if problem reboot camera and try again
  uint16_t changes = motion_detect();                                                   // find amount of change in current image frame compared to the last one
  update_frame();                                                                       // Copy current stored frame to previous stored frame
  if ( (changes >= Image_thresholdL) && (changes <= Image_thresholdH) ) {               // if enough change to count as motion detected
    if (tCounter >= tCounterTrigger) {                                                  // only trigger if movement detected in more than one consequitive frames
      tCounter = 0;
      //if ((unsigned long)(millis() - TRIGGERtimer) >= (TriggerLimitTime * 1000) ) {    // limit time between triggers
      TRIGGERtimer = millis();
      Serial.println("----Detected motion---");// update last trigger time
      return true;
      //}
    }
  }
  return false;
}

//   ---------------------------------------------------------------------------------------------------------------------

bool cameraSetAdjust() { 
   
    sensor_t *s = esp_camera_sensor_get();  

    if (s == NULL) {
      Serial.println("Error: problem getting camera sensor settings");
      return 0;
    } 


    // Image resolution / type  (may not be required?)
  //    s->set_framesize(s, fsize);                   // FRAME_SIZE_PHOTO , FRAME_SIZE_MOTION
  //    if (fsize == FRAME_SIZE_MOTION) s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
  //    if (fsize == FRAME_SIZE_PHOTO) s->set_pixformat(s, PIXFORMAT_JPEG);

      // If you enable gain_ctrl or exposure_ctrl it will prevent a lot of the other settings having any effect
      // more info on settings here: https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/
      s->set_gain_ctrl(s, 0);                       // auto gain off (1 or 0)
      s->set_exposure_ctrl(s, 0);                   // auto exposure off (1 or 0)
      s->set_agc_gain(s, cameraImageGain);          // set gain manually (0 - 30)
      s->set_aec_value(s, cameraImageExposure);     // set exposure manually  (0-1200)
      s->set_vflip(s, cameraImageInvert);           // Invert image (0 or 1)     
      s->set_quality(s, 10);                        // (0 - 63)
      s->set_gainceiling(s, GAINCEILING_32X);       // Image gain (GAINCEILING_x2, x4, x8, x16, x32, x64 or x128) 
      s->set_brightness(s, cameraImageBrightness);  // (-2 to 2) - set brightness
      s->set_lenc(s, 1);                            // lens correction? (1 or 0)
      s->set_saturation(s, 0);                      // (-2 to 2)
      s->set_contrast(s, cameraImageContrast);      // (-2 to 2)
      s->set_sharpness(s, 0);                       // (-2 to 2)  
      s->set_hmirror(s, 0);                         // (0 or 1) flip horizontally
      s->set_colorbar(s, 0);                        // (0 or 1) - show a testcard
      s->set_special_effect(s, 0);                  // (0 to 6?) apply special effect
//       s->set_whitebal(s, 0);                        // white balance enable (0 or 1)
//       s->set_awb_gain(s, 1);                        // Auto White Balance enable (0 or 1) 
//       s->set_wb_mode(s, 0);                         // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
//       s->set_dcw(s, 0);                             // downsize enable? (1 or 0)?
//       s->set_raw_gma(s, 1);                         // (1 or 0)
//       s->set_aec2(s, 0);                            // automatic exposure sensor?  (0 or 1)
//       s->set_ae_level(s, 0);                        // auto exposure levels (-2 to 2)
      s->set_bpc(s, 0);                             // black pixel correction
      s->set_wpc(s, 0);                             // white pixel correction
    return 1;
}





bool cameraImageSettings(framesize_t fsize, bool isMotionDetect) {


  if (isMotionDetect && !isDetectMode) {
    RestartCamera(PIXFORMAT_GRAYSCALE, FRAME_SIZE_MOTION);
    isDetectMode = true;
    // Serial.println("PIXFORMAT_GRAYSCALE");
    Serial.println("PIXFORMAT_GRAYSCALE");
  }
  if (!isMotionDetect && isDetectMode) {
    RestartCamera(PIXFORMAT_JPEG, fsize);
    isDetectMode = false;
    cameraSetAdjust();
    Serial.println("PIXFORMAT_JPEG");
  }

  
  
  
  
  
  Serial.println("fb get 1");
  camera_fb_t *frame_buffer = esp_camera_fb_get();    // capture frame from camera
  esp_camera_fb_return(frame_buffer);                 // return frame so memory can be released

  return true;

}


//   ---------------------------------------------------------------------------------------------------------------------


/**
   Capture image and down-sample in to blocks
     this sets all blocks to value zero then goes through each pixel in the greyscale image and adds its value to
     the relevant blocks total.  After this each blocks value is divided by the number of pixels in it
     resulting in each blocks value being the average value of all the pixels within it.
*/
bool capture_still(framesize_t oldfsize) {

   Serial.flush();   // wait for serial data to be sent first as I suspect this can cause problems capturing an image

  uint32_t TempAveragePix = 0;     // average pixel reading (used for calculating image brightness)
  uint32_t temp_frame[H][W] = { 0 };

  if (!cameraImageSettings(FRAME_SIZE_MOTION, true)) {
    Serial.println("error set FRAME_SIZE_MOTION");
    return 0;
  }                   // apply camera sensor settings
  //Serial.println("fb get 2");
  camera_fb_t *frame_buffer = esp_camera_fb_get();          // capture frame from camera

  if (!frame_buffer) {
    //Serial.println("fb get 3");// if there was a problem grabbing a frame try again
    camera_fb_t *frame_buffer = esp_camera_fb_get();
    if (!frame_buffer)return false;                         // failed to capture image
  }
  //Serial.println("downsample");
  // down-sample image in to blocks
  for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
    const uint16_t x = i % WIDTH;
    const uint16_t y = floor(i / WIDTH);
    const uint8_t block_x = floor(x / BLOCK_SIZE);
    const uint8_t block_y = floor(y / BLOCK_SIZE);
    const uint8_t pixel = frame_buffer->buf[i];

    // const uint16_t current = current_frame[block_y][block_x];
    
    // accumulate all the pixels in each block
    temp_frame[block_y][block_x] += (pixel);

    TempAveragePix += pixel;    // add all pixels together for calculation of image average brightness
  }
  esp_camera_fb_return(frame_buffer);    // return frame so memory can be released
  Serial.println("done");
  AveragePix = TempAveragePix / (WIDTH * HEIGHT);     // convert to average

  // average values for all pixels in each block
  bool frameChanged = 0;                            // flag if any change since last frame
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      uint16_t currentBlock = temp_frame[y][x] / (BLOCK_SIZE * BLOCK_SIZE );    // average pixel value in block
      if (current_frame[y][x] != currentBlock) frameChanged = 1;
      current_frame[y][x] = currentBlock;
    }
  }
  //    if (!frameChanged) log_system_message("Suspect camera problem as no change since last frame");

//Serial.println(" adjust ");
//AutoAdjustImage();
  Serial.println(" frame get ");
    print_frame(current_frame);
   // Serial.println(frameChanged);
  return true;
}

void returnTheCamera(framesize_t oldfsize) {
  cameraImageSettings(oldfsize, false);

}
//   ---------------------------------------------------------------------------------------------------------------------


/**
   Compute the number of different blocks in the frames.  If there are enough, then motion has happened
      returns the number of changed active blocks
*/
float motion_detect() {
  uint16_t changes = 0;
  const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);     // total number of blocks in image

  // adjust block_threshold for gain setting (to compensate for noise introduced with gain)
  uint16_t tThreshold = Block_threshold + (float)(cameraImageGain * thresholdGainCompensation);

  // go through all blocks in current frame and check for changes since previous frame
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      uint16_t current = current_frame[y][x];
      uint16_t prev = prev_frame[y][x];
      uint16_t pChange = abs(current - prev);          // modified code Feb20 - gives blocks average pixels variation in range 0 to 255
      // float pChange = abs(current - prev) / prev;   // original code
      if (pChange >= tThreshold) {                     // if change in block is enough to qualify as changed
        if (block_active(x, y)) changes += 1;        // if detection mask is enabled for this block increment changed block count

      }
    }
  }

  if (changes > latestChanges) latestChanges = changes;           // store highest reading for display on main page (it is zeroed when displayed)

  // Consecutive triggers counter (i.e. how many times in a row movement has been detected)
  if (changes >= Image_thresholdL && changes <= Image_thresholdH) tCounter ++;
  else tCounter = 0;

  Serial.print("Changes:");
  Serial.println(changes);
  //  Serial.println("---------------");

  return changes;                                                 // return number of changed blocks
}


//   ---------------------------------------------------------------------------------------------------------------------

/**
   Is this image block active in the detection mask  (mask area is a 4 x 3 grid)
      returns 1 for active, 0 for disabled
*/

bool block_active(uint16_t x, uint16_t y) {

  // Which mask area is this block in
  uint16_t Maskx = floor(x / maskBlockWidth);        // x mask area (0 to 3)
  uint16_t Masky = floor(y / maskBlockHeight);       // y mask area (0 to 2)

  return mask_frame[Maskx][Masky];

}


//   ---------------------------------------------------------------------------------------------------------------------


/**
   Copy current frame to previous
*/

void update_frame() {
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      prev_frame[y][x] = current_frame[y][x];
    }
  }
}



// ------------------------------------------------- end ----------------------------------------------------------------
