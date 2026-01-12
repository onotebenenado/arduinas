#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"

// ====== Wi-Fi ======
const char* WIFI_SSID = "RT-GPON-0306";
const char* WIFI_PASS = "X43sdN5q";

// ====== AI-Thinker ESP32-CAM pins ======
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

httpd_handle_t stream_httpd = NULL;
bool camera_initialized = false;

// оптимизация для потока
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=123456789000000000000987654321";
static const char* _STREAM_BOUNDARY = "\r\n--123456789000000000000987654321\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// глобальные переменные для стабильности
static uint32_t client_count = 0;
static SemaphoreHandle_t xFrameSemaphore = NULL;

// сброс питания камеры
void camera_power_cycle() {
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);
  delay(200);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(200);
}

// обработчик потока с исправлениями для стабильности
static esp_err_t stream_handler(httpd_req_t* req) {
  esp_err_t res = ESP_OK;

  // увеличиваем счетчик клиентов
  xSemaphoreTake(xFrameSemaphore, portMAX_DELAY);
  client_count++;
  xSemaphoreGive(xFrameSemaphore);

  Serial.printf("New client connected. Total: %d\n", client_count);

  // устанавливаем заголовки
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "10");
  httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

  camera_fb_t* fb = NULL;
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

  while (true) {
    // чекаем соединение / 5 сек
    struct timeval now;
    gettimeofday(&now, NULL);
    if (now.tv_sec - last_time.tv_sec > 5) {
      Serial.println("Client timeout");
      break;
    }

    // получаем фрейм
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      delay(100);
      continue;
    }

    // отправляем boundary
    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    // отправляем заголовки кадра
    char part_buf[64];
    int part_len = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, part_len);
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    // отправляем JPEG данные
    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      Serial.printf("Send failed: %d\n", res);
      break;
    }

    // обновляем время последней активности
    gettimeofday(&last_time, NULL);

    // Задержка для контроля FPS (не более 10 FPS)
    delay(100);
  }

  // уменьшаем счетчик клиентов
  xSemaphoreTake(xFrameSemaphore, portMAX_DELAY);
  client_count--;
  xSemaphoreGive(xFrameSemaphore);

  Serial.printf("Client disconnected. Total: %d\n", client_count);
  return res;
}

// веб интерфейс
static esp_err_t index_handler(httpd_req_t* req) {
  const char* html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32-CAM Stream</title>
<style>
body, html {
  margin: 0;
  padding: 0;
  background: #000;
  height: 100%;
  overflow: hidden;
}
img {
  width: 100%;
  height: 100%;
  object-fit: contain;
}
.overlay {
  position: fixed;
  top: 10px;
  left: 10px;
  color: white;
  font-family: Arial, sans-serif;
  font-size: 12px;
  background: rgba(0,0,0,0.5);
  padding: 5px 10px;
  border-radius: 5px;
}
</style>
</head>
<body>
<div class="overlay">ESP32-CAM Live Stream</div>
<img id="stream" src="/stream">
<script>
let img = document.getElementById('stream');
let reconnectAttempts = 0;
let maxReconnect = 10;

function reconnect() {
  if (reconnectAttempts >= maxReconnect) {
    console.log('Max reconnection attempts reached');
    return;
  }
  
  reconnectAttempts++;
  console.log('Reconnecting attempt ' + reconnectAttempts);
  
  // создаем новый Image объект для переподключения
  let newImg = new Image();
  newImg.onload = function() {
    img.src = this.src;
    reconnectAttempts = 0;
    console.log('Reconnected successfully');
  };
  
  newImg.onerror = function() {
    console.log('Reconnection failed, retrying in 2s');
    setTimeout(reconnect, 2000);
  };
  
  // добавляем timestamp чтобы избежать кэширования
  newImg.src = '/stream?t=' + Date.now();
}

img.onerror = function() {
  console.log('Stream disconnected, attempting to reconnect...');
  setTimeout(reconnect, 1000);
};

// периодическая проверка соединения
setInterval(() => {
  if (img.naturalWidth === 0) {
    console.log('Stream seems dead, reconnecting...');
    img.src = img.src.split('?')[0] + '?t=' + Date.now();
  }
}, 10000);
</script>
</body>
</html>
)rawliteral";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

// запуск сервера
static void start_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.stack_size = 12288;  // Увеличиваем стек
  config.max_uri_handlers = 8;
  config.backlog_conn = 2;         // Ограничиваем количество одновременных соединений
  config.lru_purge_enable = true;  // Автоматически закрываем неактивные соединения

  // увеличиваем задержку
  config.recv_wait_timeout = 10;
  config.send_wait_timeout = 10;

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &index_uri);

    httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(stream_httpd, &stream_uri);

    Serial.println("HTTP server started on port 80");
  }
}

// инициализация камеры
bool init_camera() {
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
  config.xclk_freq_hz = 10000000; // если 20Mhz начинает моросить
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;  // 800x600 как компромисс
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Пробуем инициализировать 3 раза потомк что бог любит троицу
  for (int i = 0; i < 3; i++) {
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
      return true;
    }
    Serial.printf("Camera init attempt %d failed: 0x%x\n", i + 1, err);
    camera_power_cycle();
    delay(500);
  }

  // шалость не удалась, пробуем с более низким разрешением
  config.frame_size = FRAMESIZE_VGA;  // 640x480
  esp_err_t err = esp_camera_init(&config);
  if (err == ESP_OK) {
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-CAM Stable Stream ===\n");

  // семафор для синхронизации
  xFrameSemaphore = xSemaphoreCreateMutex();

  // Power cycle камеры
  camera_power_cycle();

  // ищем камеру
  if (init_camera()) {
    camera_initialized = true;
    Serial.println("Camera initialized successfully");

    // тестовый кадр
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      Serial.printf("Test frame: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);
      esp_camera_fb_return(fb);
    }
  } else {
    Serial.println("Camera initialization failed!");
    camera_initialized = false;
  }

  // подключение к вифи
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);

  for (int i = 0; i < 30; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // запуск сервера
    start_server();

    Serial.println("\nOpen in browser:");
    Serial.printf("  http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi connection failed!");
    // Работаем без WiFi (локально)
    start_server();
    Serial.println("Server started in AP mode");
    WiFi.softAP("ESP32-CAM", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  Serial.println("\n=== Ready ===");
}

void loop() {
  static uint32_t last_status = 0;

  if (millis() - last_status > 30000) {
    last_status = millis();

    // проверка WiFi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    }

    // проверка камеры
    if (camera_initialized) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) {
        Serial.printf("Camera alive: %dx%d, Clients: %d\n",
                      fb->width, fb->height, client_count);
        esp_camera_fb_return(fb);
      } else {
        Serial.println("Camera capture failed, trying to reinit...");
        esp_camera_deinit();
        camera_power_cycle();
        camera_initialized = init_camera();
      }
    }
  }
  // давай работай!!!
  delay(100);
}