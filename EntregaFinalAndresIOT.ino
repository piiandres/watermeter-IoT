#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include <ArduinoJson.h>
#include "FS.h"      //--> SD Card ESP32
#include "SD_MMC.h"  //--> SD Card ESP32
#include "SPI.h"
#include "fb_gfx.h"
#include "soc/soc.h"           //--> disable brownout problems
#include "soc/rtc_cntl_reg.h"  //--> disable brownout problems
#include "esp_http_server.h"
#include <watermeter_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
using namespace ei::image;
#include "edge-impulse-sdk/dsp/image/processing.hpp"
using namespace ei::image::processing;
#include <PubSubClient.h>


/* SET YOUR OWN WIFI SSID AND PASSWORD======================================= */
#include "secrets.h"


WiFiClient espClient;
PubSubClient client(espClient);

/* ======================================== Select camera model */
// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
#include "camera_pins.h"
// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT
/* ======================================== */


int tpic = 2;  //--> variable for the process status of saving images to MicroSD card.

int txt_Text_Count = 0;
/* ======================================== */
int pixel_size = 3;

uint8_t *resized_crop = NULL;
uint8_t *image_rgb888 = NULL;
int width;
int height;

boolean isConfig = false;

struct Crop {
  int cropX;
  int cropY;
  int width;
  int height;
};

struct CropsConfig {
  Crop dig[7];
};

CropsConfig cropsConfig;

unsigned long previousMillis = 0;
const long interval = 60000;


/* ======================================== */
typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;
/* ======================================== */

/* ======================================== */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
/* ======================================== */

/* ======================================== Empty handle to esp_http_server */
httpd_handle_t index_httpd = NULL;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
httpd_handle_t capture_httpd = NULL;
/* ======================================== */


/* ======================================== HTML code for index / main page */
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
  <html>

<head>

  <title>ESP32-CAM Stream Web Server</title>

  <meta name="viewport" content="width=device-width, initial-scale=1">

  <style>
    body {
      font-family: Arial;
      text-align: center;
      margin: 0px auto;
      padding-top: 30px;
    }

    /* ----------------------------------- Toggle Switch */
    .switch {
      position: relative;
      display: inline-block;
      width: 90px;
      height: 34px;
      top: 10px;
    }

    .switch input {
      display: none;
    }

    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: #D3D3D3;
      -webkit-transition: .4s;
      transition: .4s;
      border-radius: 34px;
    }

    .slider:before {
      position: absolute;
      content: "";
      height: 26px;
      width: 26px;
      left: 4px;
      bottom: 4px;
      background-color: #f7f7f7;
      -webkit-transition: .4s;
      transition: .4s;
      border-radius: 50%;
    }

    input:checked+.slider {
      background-color: #5b9dd8;
    }

    input:focus+.slider {
      box-shadow: 0 0 1px #2196F3;
    }

    input:checked+.slider:before {
      -webkit-transform: translateX(26px);
      -ms-transform: translateX(26px);
      transform: translateX(55px);
    }

    .slider:after {
      content: 'OFF';
      color: white;
      display: block;
      position: absolute;
      transform: translate(-50%, -50%);
      top: 50%;
      left: 70%;
      font-size: 10px;
      font-family: Verdana, sans-serif;
    }

    input:checked+.slider:after {
      left: 25%;
      content: 'ON';
    }

    /* ----------------------------------- */

    /* ----------------------------------- button1 / Capture Button */
    .button1 {
      display: inline-block;
      padding: 10px 20px;
      font-size: 14px;
      cursor: pointer;
      text-align: center;
      text-decoration: none;
      outline: none;
      color: #fff;
      background-color: #4CAF50;
      border: none;
      border-radius: 30px;
    }

    .button1:hover {
      background-color: #3e8e41
    }

    .button1:active {
      transform: scale(0.9, 0.9)
    }

    .button1:disabled {
      opacity: 0.6;
      cursor: not-allowed;
      pointer-events: none;
    }

    /* ----------------------------------- */

    /* ----------------------------------- button2 / Stream Button */
    .button2 {
      display: inline-block;
      padding: 10px 20px;
      font-size: 14px;
      cursor: pointer;
      text-align: center;
      text-decoration: none;
      outline: none;
      color: #fff;
      background-color: #935cfb;
      border: none;
      border-radius: 30px;
    }

    .button2:hover {
      background-color: #7c38fa
    }

    .button2:active {
      transform: scale(0.9, 0.9)
    }

    .button2:disabled {
      opacity: 0.6;
      cursor: not-allowed;
      pointer-events: none;
    }

    /* ----------------------------------- */
  </style>

</head>

<body>

  <h3>ESP32-CAM Stream Web Server </h3>

  <img id="source" src style="display: none;">

  <canvas id="canvas"></canvas>

  <br><br>
  <label for="dig-select">Elegir digito:</label>
  <select name="digits" id="dig-select" disabled>
    <option value>--Elegir una opcion--</option>
    <option value="dig1">Digito 1</option>
    <option value="dig2">Digito 2</option>
    <option value="dig3">Digito 3</option>
    <option value="dig4">Digito 4</option>
    <option value="dig5">Digito 5</option>
    <option value="dig6">Digito 6</option>
    <option value="dig7">Digito 7</option>
  </select>
  <button class="button1" onclick="saveCrop()" id="dig-button" disabled>Guardar
    Digito</button>
  <button class="button2" onclick="sendAllCrops()" id="save" disabled>Guardar configuracion</button>
  <br><br>
  <span style="font-size:15;">LED Flash : &nbsp;</span>

  <label class="switch">
    <input type="checkbox" id="togLEDFlash" onclick="LEDFlash()">
    <div class="slider round"></div>
  </label>

  <br><br>

  <button class="button1" onclick="viewer('capture')" id="Cptr">Capture</button>
  <button class="button2" onclick="viewer('stream')" id="Strm">Stream</button>

  <br><br>
  <p id="resultContainer" hidden>
    Resultado inferido: <span id="results"></span>
  </p>

  <script>
    /* ----------------------------------- Calls the video stream link and displays it */
    const source = document.getElementById('source');
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    source.src = window.location.href.slice(0, -1) + ":81/stream"
    var cropCoordinates = {}
    var isCapture = false;
    // Espera que cargue la primera imagen
    source.onload = function () {
      canvas.width = source.naturalWidth;
      canvas.height = source.naturalHeight;
      draw(); // empieza a dibujar
    };
    function draw() {
      if (!isCapture) {
        ctx.drawImage(source, 0, 0);
        requestAnimationFrame(draw);
      }
    }
    document.getElementById("Cptr").disabled = false;
    document.getElementById("Strm").disabled = true;
    send_cmd("stream");
    /* ----------------------------------- */


    /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to send commands to turn on or turn off the LED Flash */
    function LEDFlash() {
      var tgLEDFlash = document.getElementById("togLEDFlash");
      var tgState;
      if (tgLEDFlash.checked == true) tgState = 1;
      if (tgLEDFlash.checked == false) tgState = 0;
      send_cmd(tgState);
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */

    /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to choose to show streaming video or captured image */
    function viewer(x) {
      if (x == "capture") {
        document.getElementById("Cptr").disabled = true;
        document.getElementById("Strm").disabled = false;
        document.getElementById('dig-select').disabled = false;
        document.getElementById('dig-button').disabled = false;
        send_cmd(x);
        isCapture = true;
        window.stop();
        source.src = window.location.href.slice(0, -1) + ":81/capture";
        source.onload = function () {
          canvas.width = source.naturalWidth;
          canvas.height = source.naturalHeight;
          ctx.drawImage(source, 0, 0, canvas.width, canvas.height);
          canvas.addEventListener('mousedown', mouseDown, false);
          canvas.addEventListener('mouseup', mouseUp, false);
          canvas.addEventListener('mousemove', mouseMove, false);
        }
      }
      if (x == "stream") {
        isCapture = false;
        window.location.reload();
        document.getElementById('dig-select').disabled = true;
        document.getElementById('dig-button').disabled = true;
        document.getElementById("Cptr").disabled = false;
        document.getElementById("Strm").disabled = true;
        source.src = window.location.href.slice(0, -1) + ":81/stream";
        source.onload = function () {
          draw();
        };
      }
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */

    /* :::::::::::::::::::::::::::::::::::::::::::::::: Function for sending commands */
    function send_cmd(cmds) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/action?go=" + cmds, true);
      xhr.send();
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */


    /* :::::::::::::::::::::::::::::::::::::::::::::::: Function to get results */
    function sendAllCrops() {
      var cropsJSON = JSON.stringify(cropCoordinates);

      var xhttp = new XMLHttpRequest();
      xhttp.onreadystatechange = function () {
        if (this.readyState == 4) {
          if (this.status == 200) {
            console.log('Respuesta ESP32:', this.responseText);
            document.getElementById('results').innerText = JSON.parse(this.responseText).result;
            document.getElementById('resultContainer').hidden = false;
          } else {
            console.error('Error al enviar:', this.status);
          }
        }
      };
      xhttp.open("POST", "/saveCrop", true);
      xhttp.setRequestHeader("Content-Type", "application/json");
      xhttp.send(cropsJSON);
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */

    /* :::::::::::::::::::::::::::::::::::::::::::::::: Functions to set crop coordinates */

    let startX, startY, endX, endY;
    let isDrawing = false;
    const colors = {
      dig1: "red",
      dig2: "blue",
      dig3: "lightblue",
      dig4: "orange",
      dig5: "fuchsia",
      dig6: "yellow",
      dig7: "lightgreen",
    }

    canvas.onmousedown = e => {
      startX = e.offsetX; startY = e.offsetY; isDrawing = true;
    };
    canvas.onmousemove = e => {
      if (!isDrawing) return;
      endX = e.offsetX; endY = e.offsetY;
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(source, 0, 0, canvas.width, canvas.height);
      ctx.lineWidth = 2;
      ctx.strokeStyle = colors[document.getElementById('dig-select').value];
      ctx.strokeRect(startX, startY, endX - startX, endY - startY);
    };
    canvas.onmouseup = () => { isDrawing = false; };

    async function saveCrop() {
      if (!isCapture) {
        return;
      }
      cropCoordinates[document.getElementById('dig-select').value] = {
        cropX: Math.min(startX, endX),
        cropY: Math.min(startY, endY),
        width: Math.abs(endX - startX),
        height: Math.abs(endY - startY)
      };
      console.log(cropCoordinates);
      if (Object.keys(cropCoordinates).length === 7) {
        document.getElementById('save').disabled = false;
      } else {
        document.getElementById('save').disabled = true;
      }
    }
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */

    /* :::::::::::::::::::::::::::::::::::::::::::::::: Event to control digit select */
    document.getElementById('dig-select').addEventListener('change', function () {
      const key = this.value;
      if (cropCoordinates[key]) {
        const r = cropCoordinates[key];
        ctx.drawImage(source, 0, 0, canvas.width, canvas.height);
        ctx.strokeStyle = colors[this.value];
        ctx.lineWidth = 2;
        ctx.strokeRect(
          r.cropX,
          r.cropY,
          r.width,
          r.height
        );
        console.log(`Mostrando rectángulo para ${key}:`, r);
      } else {
        ctx.drawImage(source, 0, 0, canvas.width, canvas.height);
        console.log(`Sin rectángulo guardado para ${key}`);
      }
    });
    /* :::::::::::::::::::::::::::::::::::::::::::::::: */

  </script>

</body>

</html>
)rawliteral";
/* ======================================== */
void loadCropsConfig(StaticJsonDocument<768> &doc) {
  for (int i = 0; i < 7; i++) {
    String key = "dig" + String(i + 1);
    JsonObject dig = doc[key];
    cropsConfig.dig[i].cropX = dig["cropX"];
    cropsConfig.dig[i].cropY = dig["cropY"];
    cropsConfig.dig[i].width = dig["width"];
    cropsConfig.dig[i].height = dig["height"];
  }
}

void loadCropsConfigFile(const char *filename) {
  File file = SD_MMC.open(filename);
  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }

  StaticJsonDocument<768> doc;

  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.print(F("Error leyendo crops: "));
    Serial.println(error.f_str());
    file.close();
    return;
  }
  loadCropsConfig(doc);

  file.close();
  isConfig = true;
}



/* ________________________________________________________________________________ Index handler function to be called during GET or uri request */
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ jpg_encode_stream handler function to be called during GET or uri request */
static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ capture handler function to be called during GET or uri request */
static esp_err_t capture_handler(httpd_req_t *req) {
  Serial.println("Capture handler");
  tpic = 1;
  bool ledflashLastState = false;
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();
  if (image_rgb888 != NULL) {
    free(image_rgb888);
    image_rgb888 = NULL;
  }


  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  } else {
    Serial.printf("OK: %dx%d\n", fb->width, fb->height);
  }

  image_rgb888 = (uint8_t *)ps_malloc(fb->width * fb->height * 3);
  if (image_rgb888 == NULL) {
    Serial.println("image_rbg888 devolvió NULL.");
    return ESP_FAIL;
  }
  width = fb->width;
  height = fb->height;

  fmt2rgb888(fb->buf, fb->len, fb->format, image_rgb888);
  Serial.println('success img to rgb888');


  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  size_t out_len, out_width, out_height;
  uint8_t *out_buf;
  bool s;
  bool detected = false;
  int face_id = 0;
  if (true) {
    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
      jpg_chunking_t jchunk = { req, 0 };
      res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
      fb_len = jchunk.len;
    }
    Serial.println(fb->width, fb->height);
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    tpic = 0;
    return res;
    Serial.flush();
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ stream handler function to be called during GET or uri request. */
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if (fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ cmd handler function to be called during GET or uri request. */
static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int res = 0;

  if (!strcmp(variable, "1")) {
    digitalWrite(4, HIGH);
    Serial.println("LED Flash ON");
  } else if (!strcmp(variable, "0")) {
    digitalWrite(4, LOW);
    Serial.println("LED Flash OFF");
  } else if (!strcmp(variable, "capture")) {
    Serial.println("capture");
  } else if (!strcmp(variable, "stream")) {
    Serial.println("stream");
  } else {
    res = -1;
  }

  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}
/* ________________________________________________________________________________ */


/* ________________________________________________________________________________ */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    // Swap BGR to RGB here
    // due to https://github.com/espressif/esp32-camera/issues/379
    out_ptr[out_ptr_ix] = (resized_crop[pixel_ix + 2] << 16) + (resized_crop[pixel_ix + 1] << 8) + resized_crop[pixel_ix];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  // and done!
  return 0;
}

esp_err_t do_inference(char *resultNum) {
  resultNum[0] = '\0';
  for (int i = 1; i <= 7; i++) {
    Crop crop = cropsConfig.dig[i - 1];

    int cropX = crop.cropX;
    int cropY = crop.cropY;
    int cropWidth = crop.width;
    int cropHeight = crop.height;



    uint8_t *crop_rgb888 = (uint8_t *)malloc(cropWidth * cropHeight * 3);
    if (!crop_rgb888) {
      Serial.println("No se pudo reservar crop_rgb888");
      return ESP_FAIL;
    }

    crop_image_rgb888_packed(
      image_rgb888,
      width, height,
      cropX,
      cropY,
      crop_rgb888,
      cropWidth,
      cropHeight);

    Serial.printf("Heap DRAM libre: %d bytes | PSRAM libre: %d bytes \n",
                  ESP.getFreeHeap(), ESP.getFreePsram());

    resize_image(
      crop_rgb888,
      cropWidth, cropHeight,
      resized_crop,
      EI_CLASSIFIER_INPUT_WIDTH,
      EI_CLASSIFIER_INPUT_HEIGHT,
      3);

    free(crop_rgb888);
    Serial.printf("Heap DRAM libre: %d bytes | PSRAM libre: %d bytes \n",
                  ESP.getFreeHeap(), ESP.getFreePsram());


    /* char filename2[32];
    sprintf(filename2, "/crop_ei_dig%d.jpg", i);
    Serial.printf("Guardando %s en SD...\n", filename2);

    File file2 = SD_MMC.open(filename2, FILE_WRITE);
    if (file2) {
      uint8_t *temp_jpeg_buf2 = NULL;
      size_t temp_jpeg_len2 = 0;
      size_t max_jpeg_size2 = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
      temp_jpeg_buf2 = (uint8_t *)malloc(max_jpeg_size2);
      Serial.printf("Guardando %s en SD...\n", filename2);


      if (temp_jpeg_buf2) {
        Serial.printf("Guardando %s en SD...\n", filename2);

        fmt2jpg(resized_crop, EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3,
                EI_CLASSIFIER_INPUT_WIDTH,
                EI_CLASSIFIER_INPUT_HEIGHT,
                PIXFORMAT_RGB888,
                80,
                &temp_jpeg_buf2,
                &temp_jpeg_len2);
        if (temp_jpeg_len2 > 0) {
          file2.write(temp_jpeg_buf2, temp_jpeg_len2);
          Serial.printf("Guardado %s (%u bytes)\n", filename2, (unsigned int)temp_jpeg_len2);
        } else {
          Serial.printf("ERROR: Falló conversión %s a JPEG.\n", filename2);
        }
        free(temp_jpeg_buf2);
      } else {
        Serial.println("ERROR: No se pudo reservar buffer JPEG.");
      }
      file2.close();
    } else {
      Serial.println("ERROR: No se pudo abrir archivo en SD.");
    } */

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    printf("run_classifier returned: %d\n", res);


    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

    int maxIndex = 0;
    float maxValue = 0.0f;

    ei_printf("Predictions:\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("%s: %.5f\n", result.classification[ix].label, result.classification[ix].value);

      if (result.classification[ix].value > maxValue) {
        maxValue = result.classification[ix].value;
        maxIndex = ix;
      }
    }
    const char *label = result.classification[maxIndex].label;
    strcat(resultNum, label);

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
  }
  free(resized_crop);
  resized_crop = NULL;
  if (image_rgb888 != NULL) {
    free(image_rgb888);
    image_rgb888 = NULL;
  }
  Serial.println("Inference complete");
  return ESP_OK;
}

/* ________________________________________________________________________________ crop_handler to save config. */
static esp_err_t crop_handler(httpd_req_t *req) {
  Serial.println("Crop handler");
  if (image_rgb888 == NULL) {
    Serial.println("Error: image_rgb888 no está inicializado");
    return ESP_FAIL;
  } else {
    Serial.println("image_rgb888 está inicializado");
  }
  resized_crop = (uint8_t *)ps_malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3);
  if (!resized_crop) {
    Serial.println("No se pudo reservar resized_crop");
    return ESP_FAIL;
  }


  Serial.printf("Heap DRAM libre: %d bytes | PSRAM libre: %d bytes \n",
                ESP.getFreeHeap(), ESP.getFreePsram());

  int total_len = req->content_len;

  char *buf = (char *)malloc(total_len + 1);
  if (buf == NULL) {
    return ESP_FAIL;
  }
  Serial.println("OK BUF");

  int received = 0;
  while (received < total_len) {
    int ret = httpd_req_recv(req, buf + received, total_len - received);
    if (ret <= 0) {
      Serial.println("Error: Falló la lectura del cuerpo JSON.");
      free(buf);
      free(image_rgb888);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Error al leer JSON");
      return ESP_FAIL;
    }
    received += ret;
  }
  buf[total_len] = '\0';


  StaticJsonDocument<768> doc;
  DeserializationError error = deserializeJson(doc, buf);
  if (error) {
    free(image_rgb888);
    free(buf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON inválido");
    return ESP_FAIL;
  }

  File file = SD_MMC.open("/cropsConfig.json", FILE_WRITE);
  if (file) {
    serializeJson(doc, file);
    file.close();
    Serial.println("cropsConfig.json guardado");
  } else {
    Serial.println("error al guardar cropsConfig.json");
  }
  loadCropsConfig(doc);
  StaticJsonDocument<1024> jsonResponse;
  free(buf);
  char resultNum[8];
  esp_err_t status = do_inference(resultNum);
  if (status != ESP_OK) {
    return ESP_FAIL;
  }

  jsonResponse["result"] = resultNum;
  char responseStr[128];
  size_t len = serializeJson(jsonResponse, responseStr, sizeof(responseStr));

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, responseStr, len);
  //httpd_resp_set_type(req, "text/plain");
  //httpd_resp_send(req, "Crops recibidos OK", HTTPD_RESP_USE_STRLEN);
  isConfig = true;
  previousMillis = millis();

  return ESP_OK;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ Subroutine for starting the web server / startCameraServer. */
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/action",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };

  httpd_uri_t crop_uri = {
    .uri = "/saveCrop",
    .method = HTTP_POST,
    .handler = crop_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&index_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(index_httpd, &index_uri);
    httpd_register_uri_handler(index_httpd, &cmd_uri);
    httpd_register_uri_handler(index_httpd, &crop_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
  }

  Serial.println();
  Serial.println("Camera Server started successfully");
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.print(WiFi.localIP());
  Serial.println();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ VOID SETUP() */
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  //--> disable brownout detector

  Serial.begin(115200);
  Serial.println('new');
  delay(1000);
  Serial.setDebugOutput(false);

  /* ---------------------------------------- Camera configuration. */
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;  //UXGA;
    /*
     * From the results of the tests I did:
     * - config.jpeg_quality = 10; --> the captured images are good for indoors or in low light conditions.
     *   But the image file is "corrupted" for capturing images outdoors or in bright light conditions.
     * - config.jpeg_quality = 20; --> there is no "corrupt" of image files for image capture both indoors and outdoors.
     * 
     * I don't know if this only happens to my ESP32 Cam module. Please test the settings above.
     * 
     * From source: https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/ , The image quality (jpeg_quality) can be a number between 0 and 63.
     * A lower number means a higher quality. However, very low numbers for image quality,
     * specially at higher resolution can make the ESP32-CAM to crash or it may not be able to take the photos properly. 
     */
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  /* ---------------------------------------- */

  /* ---------------------------------------- Camera init. */
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_ae_level(s, -1);
  s->set_brightness(s, 0); 
  s->set_contrast(s, 1);


  Serial.printf("XCLK: %d\n", s->xclk_freq_hz);
  Serial.printf("PSRAM found: %s\n", psramFound() ? "Yes" : "No");
  /* ---------------------------------------- */

  /* ---------------------------------------- Start accessing and checking SD card. */
  /*
   * The "SD_MMC.h" library supports 4-bit and 1-bit modes for accessing MicroSD cards.
   * 
   * The 4-bit mode uses GPIO 2, 4, 12, 13, 14, and 15. That means the LED Flash on Borad on the ESP32 Cam cannot be used, because the LED Flash uses GPIO_4.
   * 
   * The 1-bit mode does not use GPIO 4, 12 and 13 (based on some references I read).
   * So GPIO 4, 12 and 13 can be used for other purposes.
   * But in the tests that I tried on the ESP32 Cam that I have (ESP32-CAM AI-Thinker), it seems that GPIO 4, 12 and 13 are still being used.
   * However, if you use 1-bit mode, GPIO 4, 12 and 13 can still be used for other purposes with a few programming tricks.
   * Unlike the 4-bit mode, where GPIO 4, 12 and 13 cannot be used for other purposes at all.
   * 
   * I CONFIRM THE ABOVE DESCRIPTION IS BASED ON TESTING I HAVE DONE ON THE ESP32 CAM I HAVE.
   * I might be wrong. Please leave a comment about it in the comments section of this video project (on Youtube).
   * 
   * So in this project I'm using 1-bit mode.
   */


  /* ::::::: 1-bit mode */
  Serial.println("Starting SD Card");
  pinMode(13, INPUT_PULLUP);  //--> This is done to resolve an "error" in 1-bit mode when SD_MMC.begin("/sdcard", true). Reference: https://github.com/espressif/arduino-esp32/issues/4680

  Serial.println("Start accessing SD Card 1-bit mode");
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  Serial.println("Started accessing SD Card 1-bit mode successfully");

  pinMode(13, INPUT_PULLDOWN);
  /* ::::::: */
  /* ---------------------------------------- */

  /* ---------------------------------------- Checking SD card type */
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }
  /* ---------------------------------------- */

  /* ---------------------------------------- Loads previous configuration*/
  loadCropsConfigFile("/cropsConfig.json");
  /* ---------------------------------------- */

  delay(1000);

  pinMode(4, OUTPUT);  //--> LED Flash on Board

  /* ---------------------------------------- Wi-Fi connection */
  Serial.println();
  WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(4, HIGH);
    delay(250);
    digitalWrite(4, LOW);
    delay(250);
    Serial.print(".");
  }
  /* ---------------------------------------- */

  client.setServer(MQTT_BROKER_ADRESS, MQTT_PORT);
  client.setKeepAlive(120);

  delay(1000);
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();  //--> Start camera web server
}
/* ________________________________________________________________________________ */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print(" MQTT connection...");

    if (client.connect(MQTT_CLIENT_NAME)) {  //OJO credenciales MOSQUITTO MQTT_USER, MQTT_PASS
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
static esp_err_t recurrently_inference(long now) {
  Serial.printf("Heap DRAM libre: %d bytes | PSRAM libre: %d bytes \n",
                ESP.getFreeHeap(), ESP.getFreePsram());
  Serial.println("LEDON");
  digitalWrite(4, HIGH);
  delay(100);
  for (int i = 0; i < 10; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    delay(100);
  }
  camera_fb_t *fb = NULL;
  if (image_rgb888 != NULL) {
    free(image_rgb888);
    image_rgb888 = NULL;
  }
  if (resized_crop != NULL) {
    free(resized_crop);
    resized_crop = NULL;
  }


  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return ESP_FAIL;
  } else {
    Serial.printf("OK: %dx%d\n", fb->width, fb->height);
  }
  /* char filename2[32];
  sprintf(filename2, "/photo.jpg");
  Serial.printf("Guardando %s en SD...\n", filename2);

  File file2 = SD_MMC.open(filename2, FILE_WRITE);
  if (file2) {
    file2.write(fb->buf, fb->len);
    file2.close();
  } else {
    Serial.println("ERROR: No se pudo abrir archivo en SD.");
  } */


  image_rgb888 = (uint8_t *)ps_malloc(fb->width * fb->height * 3);
  if (image_rgb888 == NULL) {
    Serial.println("image_rbg888 devolvió NULL.");
    return ESP_FAIL;
  }
  width = fb->width;
  height = fb->height;

  fmt2rgb888(fb->buf, fb->len, fb->format, image_rgb888);
  esp_camera_fb_return(fb);
  Serial.println("success img to rgb888");
  digitalWrite(4, LOW);
  Serial.println("LEDOFF");
  Serial.flush();
  resized_crop = (uint8_t *)ps_malloc(EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3);
  if (!resized_crop) {
    Serial.println("No se pudo reservar resized_crop");
    return ESP_FAIL;
  }
  Serial.flush();
  delay(1500);
  char resultNum[8];
  esp_err_t status = do_inference(resultNum);
  if (status != ESP_OK) {
    return ESP_FAIL;
  }
  client.publish("watermeterResults", resultNum);
  previousMillis = now;
  Serial.println("Recurrently Inference finished");

  return ESP_OK;
}
/* ________________________________________________________________________________ VOID LOOP() */
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (isConfig && now - previousMillis >= interval) {
    Serial.println("Recurrently Inference");
    recurrently_inference(now);
  }
  delay(1);
}