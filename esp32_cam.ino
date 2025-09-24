#include "esp_camera.h"
#include <WebServer.h>
#include <WiFi.h>

const char* WIFI_SSID = "Dialog 4G G21";
const char* WIFI_PASS = "sena123d";

WebServer server(80);

// Camera configuration for AI Thinker ESP32-CAM
camera_config_t config;

void setupCamera() {
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

  config.xclk_freq_hz = 24000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size and quality settings
  config.frame_size = FRAMESIZE_QVGA;  // 640x480
  config.jpeg_quality = 20;
  config.fb_count = 1;

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Serial.println("Camera initialized successfully");
}

void serveJpg() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(503, "", "");
    return;
  }
  
  Serial.printf("CAPTURE OK %dx%d %db\n", fb->width, fb->height, fb->len);
  
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

void handleJpgLo() {
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA); // 320x240
  serveJpg();
}

void handleJpgMid() {
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA); // 640x480
  serveJpg();
}

void handleJpgHi() {
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_SVGA); // 800x600
  serveJpg();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  // Initialize camera
  setupCamera();
  
  // Initialize WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("  /cam-lo.jpg");
  Serial.println("  /cam-hi.jpg");
  Serial.println("  /cam-mid.jpg");

  // Setup server routes
  server.on("/cam-lo.jpg", handleJpgLo);
  server.on("/cam-hi.jpg", handleJpgHi);
  server.on("/cam-mid.jpg", handleJpgMid);

  server.begin();
  
}

void loop() {
  
  server.handleClient();
}
