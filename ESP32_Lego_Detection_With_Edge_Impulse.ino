// Libraries
  #include <stdio.h>
  // Edge Impulse Libraries (vision AI)
  #include <FYP_inferencing.h>
  #include "edge-impulse-sdk/dsp/image/image.hpp"
  #include "esp_camera.h"
  // tft screen 
  #include <SPI.h>
  //e_SPI Libary for drivers, pin definitions, clock speed (27mhz) 
  #include <TFT_eSPI.h> // (edited dependant header "user_setup.h" for the project) 
  // JPEG decoder library
  #include <JPEGDecoder.h>

// Definitions
  #define Cam_Width 160                                     // Camera Width
  #define Cam_Height 120                                    // Camera Height
  #define Cam_Bytes 3                                       // Number of bytes per pixel (3 for max rgb888)
  #define minimum(a,b)     (((a) < (b)) ? (a) : (b))        // Return the minimum of two values a and b

  // Private variables
  static bool debug_nn = false;                             // Set this to true to see e.g. features generated from the raw signal
  uint8_t *snapshot_buf;                                    // points to the output of the capture
  TFT_eSPI tft = TFT_eSPI();                                // TFT screen driver instance
  uint8_t image_array[Cam_Width * Cam_Height * Cam_Bytes];  // image array for tft display

  // Functions
  bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

// FUNCTION : Inititalizes the camera with edited settings
void ei_camera_init() {
  camera_config_t config;               // Object to store the camera configuration parameters

  config.ledc_channel = LEDC_CHANNEL_0; // Camera Pin Defintions (all preset from esp32-cam example files)
  config.ledc_timer = LEDC_TIMER_0;     
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;

  // Adjustable configs
  config.xclk_freq_hz = 20000000;           // clock freq
  config.pixel_format = PIXFORMAT_JPEG;     // More image format choices in the header
  config.frame_size = FRAMESIZE_QQVGA;      // QQVGA (160 * 120)... Choices in the header
  config.jpeg_quality = 1;                  // Compression level ( 0 to 63 )
  config.fb_count = 1;                      // One frame buffer 

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);                     // Config the cam as defined above
  if (err != ESP_OK) {                                          // If it didn't conifg properly
    Serial.printf("Camera init failed with error 0x%x", err);   // print error tp serial
    tft.println("Camera failed to initialize... rebooting");    // print error to screen
    delay(1000);                                                // wait 1 second
    esp_restart();                                              // Give it the old off & on
    return;
  }

  sensor_t * s = esp_camera_sensor_get();   // config sensor as below

  s->set_brightness(s, 0);                  // BRIGHTNESS (-2 to 2)
  s->set_contrast(s, 0);                    // CONTRAST (-2 to 2)
  s->set_saturation(s, 0);                  // SATURATION (-2 to 2)
  s->set_special_effect(s, 0);              // SPECIAL EFFECTS (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);                    // WHITE BALANCE (0 = Disable , 1 = Enable)
  s->set_awb_gain(s, 1);                    // AWB GAIN (0 = Disable , 1 = Enable)
  s->set_wb_mode(s, 0);                     // WB MODES (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);               // EXPOSURE CONTROLS (0 = Disable , 1 = Enable)
  s->set_aec2(s, 0);                        // AEC2 (0 = Disable , 1 = Enable)
  s->set_ae_level(s, 0);                    // AE LEVELS (-2 to 2)
  s->set_aec_value(s, 300);                 // AEC VALUES (0 to 1200)
  s->set_gain_ctrl(s, 0);                   // GAIN CONTROLS (0 = Disable , 1 = Enable)
  s->set_agc_gain(s, 0);                    // AGC GAIN (0 to 30)
  s->set_gainceiling(s, (gainceiling_t)0);  // GAIN CEILING (0 to 6)
  s->set_bpc(s, 0);                         // BPC (0 = Disable , 1 = Enable)
  s->set_wpc(s, 1);                         // WPC (0 = Disable , 1 = Enable)
  s->set_raw_gma(s, 1);                     // RAW GMA (0 = Disable , 1 = Enable)
  s->set_lenc(s, 1);                        // LENC (0 = Disable , 1 = Enable)
  s->set_hmirror(s, 0);                     // HORIZ MIRROR (0 = Disable , 1 = Enable)
  s->set_vflip(s, 0);                       // VERT FLIP (0 = Disable , 1 = Enable)
  s->set_dcw(s, 1);                         // DCW (0 = Disable , 1 = Enable)
  s->set_colorbar(s, 0);                    // COLOR BAR PATTERN (0 = Disable , 1 = Enable)
}

// FUNCTION : Draw a JPEG on the TFT (calls renderJPEG)
void drawArrayJpeg(const uint8_t arrayname[], uint32_t array_size, int xpos, int ypos) {

  int x = xpos;
  int y = ypos;

  JpegDec.decodeArray(arrayname, array_size);
  
  renderJPEG(x, y);
}

// FUNCTION : Render Image (called from drawArrayJPEG)
void renderJPEG(int xpos, int ypos) {

  // retrieve infomration about the image
  uint16_t *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();


  // save the coordinate of the right and bottom edges to assist image cropping
  // to the screen size
  max_x += xpos;
  max_y += ypos;

  // read each MCU block until there are no more
  while (JpegDec.readSwappedBytes()) {
	  
    // save a pointer to the image block
    pImg = JpegDec.pImage ;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w + xpos;  // Calculate coordinates of top left corner of current MCU
    int mcu_y = JpegDec.MCUy * mcu_h + ypos;

    // check if the image block size needs to be changed for the right edge
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;

    // check if the image block size needs to be changed for the bottom edge
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // copy pixels into a contiguous block
    if (win_w != mcu_w)
    {
      uint16_t *cImg;
      int p = 0;
      cImg = pImg + win_w;
      for (int h = 1; h < win_h; h++)
      {
        p += mcu_w;
        for (int w = 0; w < win_w; w++)
        {
          *cImg = *(pImg + w + p);
          cImg++;
        }
      }
    }

    // draw image MCU block only if it will fit on the screen
    if (( mcu_x + win_w ) <= tft.width() && ( mcu_y + win_h ) <= tft.height())
    {
      tft.pushRect(mcu_x, mcu_y, win_w, win_h, pImg);
    }
    else if ( (mcu_y + win_h) >= tft.height()) {
      Serial.println("aborting image, no space");
      JpegDec.abort(); // Image has run off bottom of screen so abort decoding
    }
  }

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime;
  Serial.print(F(  "Total render time was    : ")); Serial.print(drawTime); Serial.println(F(" ms"));
  Serial.println(F(""));
}

// FUNCTION : capture new image & send info to display functions
// RETURNS  : false if not initialised, image captured, rescaled or cropped failed
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  bool do_resize = false;

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  memcpy(image_array, fb->buf, fb->len);                    // populate image_array using the frame buffer data  
  drawArrayJpeg(image_array, sizeof(image_array), 0, 4);    // Draw a jpeg image stored in memory at x,y

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if(!converted){
    ei_printf("Conversion failed\n");
    return false;
  }

  if ((img_width != Cam_Width) || (img_height != Cam_Height)) {
    do_resize = true;
  }

  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
    out_buf,
    Cam_Width,
    Cam_Height,
    out_buf,
    img_width,
    img_height);
  }

  return true;
}

// FUNCTION : Set
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix+=3;
    pixels_left--;
  }
  // and done!
  return 0;
}

// FUNCTION : Runs on boot
void setup() {
  Serial.begin(115200);                       // Start Serial at 115200 baud
  while (!Serial){};                          // wait for monitor to begin

  ei_camera_init();                           // Initialize the camera
  tft.begin();                                // Initialize screen
  tft.setRotation(1);                         // Set rotation | 0 & 2 Portrait | 1 & 3 landscape
  tft.fillScreen(ST7735_BLACK);               // Clear initialization screen
  tft.println("Starting...");                 // Write "starting..." to screen
  delay(1000);                                // wait for cam to adjust to light
  tft.fillScreen(ST7735_BLACK);               // Clear initialization screen
}

// FUNCTION : Runs after boot and loops
void loop() {
  // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }
  
  // Allocate memory for the image frame
  snapshot_buf = (uint8_t*)malloc(Cam_Width * Cam_Height * Cam_Bytes); 
    
  // check if allocation was successful
  if(snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  // Begin Object Classification
  ei::signal_t signal;                                                            // create new edge impulse signal
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;   // define the signal size
  signal.get_data = &ei_camera_get_data;                                          // set the signal to point to camera data

  // Get camera image frame, this function also calls the draw function for the screen
  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  // Clear classifier results
  ei_impulse_result_t result = { 0 };

  // run classifier
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions to serial montior
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

  // if any objects have been detected by the classifier
  #if EI_CLASSIFIER_OBJECT_DETECTION == 1
    bool bb_found = result.bounding_boxes[0].value > 0;
    tft.setCursor(0,4);
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
      auto bb = result.bounding_boxes[ix];
      if (bb.value == 0) {
        continue;
      }
      ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
      tft.print(bb.label);
      tft.print(" ");
      tft.print(bb.value, 2);
      tft.println("%");
    }
    if (!bb_found) {
      tft.setCursor(0,4);
      tft.print("No objects found");
      ei_printf("    No objects found\n");
    }
  #else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
      }
  #endif

  #if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
  #endif

  free(snapshot_buf);
}
