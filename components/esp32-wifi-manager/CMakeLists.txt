cmake_minimum_required(VERSION 3.5)
idf_component_register(SRC_DIRS src
    REQUIRES log nvs_flash mdns wpa_supplicant lwip esp_http_server
    INCLUDE_DIRS src
    EMBED_FILES src/style.css src/code.js src/index.html)
