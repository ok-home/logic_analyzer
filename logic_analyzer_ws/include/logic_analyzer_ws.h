#pragma once

#ifdef __cplusplus
extern "C" {
#endif
// register uri handlers on runing server
esp_err_t logic_analyzer_register_uri_handlers(httpd_handle_t server);

#ifdef __cplusplus
}
#endif