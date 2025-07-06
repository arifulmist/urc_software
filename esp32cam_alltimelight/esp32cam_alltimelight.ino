#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"

const char* ssid = "hack_me";
const char* password = "arafatragib";

#define CAMERA_MODEL_AI_THINKER
#define LED_GPIO_NUM 4 // Flash LED

#define PART_BOUNDARY "123456789000000000000987654321"

// AI Thinker Pin Definition
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

httpd_handle_t stream_httpd = NULL;
httpd_handle_t index_httpd = NULL;

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// HTML page with live stream and LED control
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM Stream</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { text-align: center; font-family: Arial; background: #111; color: #ddd; }
      img { width: 100%%; max-width: 640px; border-radius: 10px; margin-top: 10px; }
      button { padding: 10px 20px; font-size: 18px; margin: 12px; border: none; background: #2f4468; color: white; border-radius: 6px; }
    </style>
  </head>
  <body>
    <h2>ESP32-CAM Live Stream</h2>
    <img src="/stream" id="cam">
    <br>
    <button onclick="toggleLED()">Toggle LED</button>
  <script>
  let ledOn = false;
  function toggleLED() {
    ledOn = !ledOn;
    fetch(`/led?state=${ledOn ? 'on' : 'off'}`)
      .then(response => console.log("LED toggled:", response.status))
      .catch(err => console.error("LED toggle failed:", err));
  }
</script>



  </body>
</html>
)rawliteral";

// Stream handler
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      if (!jpeg_converted) {
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

    if (fb->format == PIXFORMAT_JPEG) {
      esp_camera_fb_return(fb);
    } else if (_jpg_buf) {
      free(_jpg_buf);
    }

    if (res != ESP_OK) break;
  }
  return res;
}

// Main page handler
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// LED control handler
  static esp_err_t led_handler(httpd_req_t *req) {
  char* buf;
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  buf = (char*)malloc(buf_len);
  if (!buf) return httpd_resp_send_500(req);

  char param[8];               // ✅ Declare param here
  bool success = false;        // ✅ Declare success here

  if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
    if (httpd_query_key_value(buf, "state", param, sizeof(param)) == ESP_OK) {
      if (strcmp(param, "on") == 0) {
        digitalWrite(LED_GPIO_NUM, LOW);  // Inverted logic: LOW = ON
        Serial.println("LED ON");
        success = true;
      } else if (strcmp(param, "off") == 0) {
        digitalWrite(LED_GPIO_NUM, HIGH); // Inverted logic: HIGH = OFF
        Serial.println("LED OFF");
        success = true;
      }
    }
  }

  free(buf);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  if (success)
    return httpd_resp_send(req, "OK", 2);
  else
    return httpd_resp_send(req, "Bad Request", 11);
}




// Start the web servers
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t led_uri = {
    .uri       = "/led",
    .method    = HTTP_GET,
    .handler   = led_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&index_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(index_httpd, &index_uri);
    httpd_register_uri_handler(index_httpd, &led_uri);
    httpd_register_uri_handler(index_httpd, &stream_uri);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  pinMode(LED_GPIO_NUM, OUTPUT);
digitalWrite(LED_GPIO_NUM, HIGH);  // Start OFF (inverted logic)

  // Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Init camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("Access your ESP32-CAM at: http://");
  Serial.println(WiFi.localIP());

  startCameraServer();
}

void loop() {}
