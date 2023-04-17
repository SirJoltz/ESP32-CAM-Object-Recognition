// Libraries & Definitions
  // Camera
  #include "esp_camera.h" 
  // Browout Control 
  #include "soc/soc.h"    
  #include "soc/rtc_cntl_reg.h"
  #include "driver/rtc_io.h"
  // MicroSD Libraries
  #include "FS.h"
  #include "SD_MMC.h"
  // Counter for picture number
  unsigned int pictureCount = 0;

// FUNCTION: Conifgures and Initalizes the camera
void configESPCamera() {
  // Create reference to store the camera configs
  camera_config_t config;            
  // Disable brownout detector (ignore low power issues)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  // Camera Pin Defintions (all preset from esp32-cam example files)
  config.ledc_channel = LEDC_CHANNEL_0;
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
  config.xclk_freq_hz = 20000000;           // set clock freq
  config.pixel_format = PIXFORMAT_JPEG;     // Set img capture format (More image format choices in the header esp_camera.h)
  config.frame_size = FRAMESIZE_QQVGA;      // Set fram size to QQVGA [160 * 120] (More choices in the header sensor.h)
  config.jpeg_quality = 0;                  // Set Compression level ( 0 to 63 )
  config.fb_count = 1;                      // Set one frame buffer (more can be allocated)

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config); // Initialize cam with the set configuration
  if (err != ESP_OK) {                      // If initialization failed
    esp_restart();                          // Restart Device
    return;
  }
  sensor_t * s = esp_camera_sensor_get();   // Set a pointer to sensor data for adjustment

  // Camera quality adjusted 01/03/23
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
  s->set_hmirror(s, 1);                     // HORIZ MIRROR (0 = Disable , 1 = Enable)
  s->set_vflip(s, 0);                       // VERT FLIP (0 = Disable , 1 = Enable)
  s->set_dcw(s, 1);                         // DCW (0 = Disable , 1 = Enable)
  s->set_colorbar(s, 0);                    // COLOR BAR PATTERN (0 = Disable , 1 = Enable)
}

// FUNCTION: Takes a new photo and saves it to a path
void takeNewPhoto(String path) {
  camera_fb_t  * fb = esp_camera_fb_get();                      // Capture new image (ignore failed caputres)
  Serial.print("Saving image: "); Serial.println(path.c_str()); // write a message to the monitor
  fs::FS &fs = SD_MMC;                                          // instanciate filesystem as SD_MMC
  File file = fs.open(path.c_str(), FILE_WRITE);                // open a file at the new path
  file.write(fb->buf, fb->len);                                 // save image buffer to file
  file.close();                                                 // Close the file
  esp_camera_fb_return(fb);                                     // Return the frame buffer back to the driver for reuse
} 

// FUNCTION: Runs on device boot
void setup() {
  Serial.begin(115200);   // start serial monitor
  pictureCount = 0;       // Reset picture count
  configESPCamera();      // Initialize the camera
  SD_MMC.begin();         // Initialize the MicroSD
}

// FUNCTION: Runs after Setup, then loops
void loop() {
  pictureCount += 1;                                      // iterate picture count
  String path = "/image" + String(pictureCount) + ".jpg"; // create new file path
  takeNewPhoto(path);                                     // Take and Save Photo
  delay(1000);                                            // Delay before looping
}
