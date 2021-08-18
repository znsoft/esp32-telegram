#include "esp_http_server.h"
#include "esp_camera.h"


httpd_handle_t camera_httpd = NULL;
int file_number;

static esp_err_t stream_handler(httpd_req_t *req) {

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char fname[100];
  
//  xSemaphoreTake( baton, portMAX_DELAY );
  fb = esp_camera_fb_get();

  if (!fb) {
    //Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    //xSemaphoreGive( baton );
    return ESP_FAIL;
  }

  file_number++;

  sprintf(fname, "inline; filename=capture_%d.jpg", file_number);
  
  char refstr[100];
   
//  sprintf(refstr, "1;url=http://%s/", localip);
//  httpd_resp_set_hdr(req, "Refresh",refstr);// "1;url=http://192.168.1.189/stream");
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", fname);
  
  

  size_t out_len, out_width, out_height;
  size_t fb_len = 0;
  fb_len = fb->len;
  res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  return res;
}



void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}
