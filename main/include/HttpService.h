/*

    A file with http server related functions.

*/

#pragma once

#include "esp_http_server.h"

extern const httpd_uri_t set_ssid_cfg;

extern const httpd_uri_t home_cfg;


//----Endpoints

esp_err_t home(httpd_req_t *req);

esp_err_t set_ssid(httpd_req_t *req);