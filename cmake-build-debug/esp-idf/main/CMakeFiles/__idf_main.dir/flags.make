# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.20

# compile CXX with C:/Users/Justin/esp/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++.exe
CXX_DEFINES = -DHAVE_CONFIG_H -DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\" -DUNITY_INCLUDE_CONFIG_H -DWITH_POSIX

CXX_INCLUDES = -IC:\AllFiles\programming\MachineDog\FocX_v_0_5\cmake-build-debug\config -IC:\Users\Justin\esp\esp-idf\components\newlib\platform_include -IC:\Users\Justin\esp\esp-idf\components\freertos\include -IC:\Users\Justin\esp\esp-idf\components\freertos\port\xtensa\include -IC:\Users\Justin\esp\esp-idf\components\esp_hw_support\include -IC:\Users\Justin\esp\esp-idf\components\esp_hw_support\port\esp32\. -IC:\Users\Justin\esp\esp-idf\components\heap\include -IC:\Users\Justin\esp\esp-idf\components\log\include -IC:\Users\Justin\esp\esp-idf\components\lwip\include\apps -IC:\Users\Justin\esp\esp-idf\components\lwip\include\apps\sntp -IC:\Users\Justin\esp\esp-idf\components\lwip\lwip\src\include -IC:\Users\Justin\esp\esp-idf\components\lwip\port\esp32\include -IC:\Users\Justin\esp\esp-idf\components\lwip\port\esp32\include\arch -IC:\Users\Justin\esp\esp-idf\components\soc\include -IC:\Users\Justin\esp\esp-idf\components\soc\esp32\. -IC:\Users\Justin\esp\esp-idf\components\soc\esp32\include -IC:\Users\Justin\esp\esp-idf\components\hal\esp32\include -IC:\Users\Justin\esp\esp-idf\components\hal\include -IC:\Users\Justin\esp\esp-idf\components\esp_rom\include -IC:\Users\Justin\esp\esp-idf\components\esp_rom\esp32 -IC:\Users\Justin\esp\esp-idf\components\esp_common\include -IC:\Users\Justin\esp\esp-idf\components\esp_system\include -IC:\Users\Justin\esp\esp-idf\components\esp32\include -IC:\Users\Justin\esp\esp-idf\components\driver\include -IC:\Users\Justin\esp\esp-idf\components\driver\esp32\include -IC:\Users\Justin\esp\esp-idf\components\esp_ringbuf\include -IC:\Users\Justin\esp\esp-idf\components\efuse\include -IC:\Users\Justin\esp\esp-idf\components\efuse\esp32\include -IC:\Users\Justin\esp\esp-idf\components\xtensa\include -IC:\Users\Justin\esp\esp-idf\components\xtensa\esp32\include -IC:\Users\Justin\esp\esp-idf\components\espcoredump\include -IC:\Users\Justin\esp\esp-idf\components\esp_timer\include -IC:\Users\Justin\esp\esp-idf\components\esp_ipc\include -IC:\Users\Justin\esp\esp-idf\components\esp_pm\include -IC:\Users\Justin\esp\esp-idf\components\vfs\include -IC:\Users\Justin\esp\esp-idf\components\esp_wifi\include -IC:\Users\Justin\esp\esp-idf\components\esp_wifi\esp32\include -IC:\Users\Justin\esp\esp-idf\components\esp_event\include -IC:\Users\Justin\esp\esp-idf\components\esp_netif\include -IC:\Users\Justin\esp\esp-idf\components\esp_eth\include -IC:\Users\Justin\esp\esp-idf\components\tcpip_adapter\include -IC:\Users\Justin\esp\esp-idf\components\app_trace\include -IC:\Users\Justin\esp\esp-idf\components\mbedtls\port\include -IC:\Users\Justin\esp\esp-idf\components\mbedtls\mbedtls\include -IC:\Users\Justin\esp\esp-idf\components\mbedtls\esp_crt_bundle\include -IC:\Users\Justin\esp\esp-idf\components\app_update\include -IC:\Users\Justin\esp\esp-idf\components\spi_flash\include -IC:\Users\Justin\esp\esp-idf\components\bootloader_support\include -IC:\Users\Justin\esp\esp-idf\components\nvs_flash\include -IC:\Users\Justin\esp\esp-idf\components\pthread\include -IC:\Users\Justin\esp\esp-idf\components\esp_gdbstub\include -IC:\Users\Justin\esp\esp-idf\components\esp_gdbstub\xtensa -IC:\Users\Justin\esp\esp-idf\components\esp_gdbstub\esp32 -IC:\Users\Justin\esp\esp-idf\components\wpa_supplicant\include -IC:\Users\Justin\esp\esp-idf\components\wpa_supplicant\port\include -IC:\Users\Justin\esp\esp-idf\components\wpa_supplicant\include\esp_supplicant -IC:\Users\Justin\esp\esp-idf\components\perfmon\include -IC:\Users\Justin\esp\esp-idf\components\asio\asio\asio\include -IC:\Users\Justin\esp\esp-idf\components\asio\port\include -IC:\Users\Justin\esp\esp-idf\components\cbor\port\include -IC:\Users\Justin\esp\esp-idf\components\unity\include -IC:\Users\Justin\esp\esp-idf\components\unity\unity\src -IC:\Users\Justin\esp\esp-idf\components\cmock\CMock\src -IC:\Users\Justin\esp\esp-idf\components\coap\port\include -IC:\Users\Justin\esp\esp-idf\components\coap\port\include\coap -IC:\Users\Justin\esp\esp-idf\components\coap\libcoap\include -IC:\Users\Justin\esp\esp-idf\components\coap\libcoap\include\coap2 -IC:\Users\Justin\esp\esp-idf\components\console -IC:\Users\Justin\esp\esp-idf\components\nghttp\port\include -IC:\Users\Justin\esp\esp-idf\components\nghttp\nghttp2\lib\includes -IC:\Users\Justin\esp\esp-idf\components\esp-tls -IC:\Users\Justin\esp\esp-idf\components\esp-tls\esp-tls-crypto -IC:\Users\Justin\esp\esp-idf\components\esp_adc_cal\include -IC:\Users\Justin\esp\esp-idf\components\esp_hid\include -IC:\Users\Justin\esp\esp-idf\components\tcp_transport\include -IC:\Users\Justin\esp\esp-idf\components\esp_http_client\include -IC:\Users\Justin\esp\esp-idf\components\esp_http_server\include -IC:\Users\Justin\esp\esp-idf\components\esp_https_ota\include -IC:\Users\Justin\esp\esp-idf\components\protobuf-c\protobuf-c -IC:\Users\Justin\esp\esp-idf\components\protocomm\include\common -IC:\Users\Justin\esp\esp-idf\components\protocomm\include\security -IC:\Users\Justin\esp\esp-idf\components\protocomm\include\transports -IC:\Users\Justin\esp\esp-idf\components\mdns\include -IC:\Users\Justin\esp\esp-idf\components\esp_local_ctrl\include -IC:\Users\Justin\esp\esp-idf\components\sdmmc\include -IC:\Users\Justin\esp\esp-idf\components\esp_serial_slave_link\include -IC:\Users\Justin\esp\esp-idf\components\esp_websocket_client\include -IC:\Users\Justin\esp\esp-idf\components\expat\expat\expat\lib -IC:\Users\Justin\esp\esp-idf\components\expat\port\include -IC:\Users\Justin\esp\esp-idf\components\wear_levelling\include -IC:\Users\Justin\esp\esp-idf\components\fatfs\diskio -IC:\Users\Justin\esp\esp-idf\components\fatfs\vfs -IC:\Users\Justin\esp\esp-idf\components\fatfs\src -IC:\Users\Justin\esp\esp-idf\components\freemodbus\common\include -IC:\Users\Justin\esp\esp-idf\components\idf_test\include -IC:\Users\Justin\esp\esp-idf\components\idf_test\include\esp32 -IC:\Users\Justin\esp\esp-idf\components\jsmn\include -IC:\Users\Justin\esp\esp-idf\components\json\cJSON -IC:\Users\Justin\esp\esp-idf\components\libsodium\libsodium\src\libsodium\include -IC:\Users\Justin\esp\esp-idf\components\libsodium\port_include -IC:\Users\Justin\esp\esp-idf\components\mqtt\esp-mqtt\include -IC:\Users\Justin\esp\esp-idf\components\openssl\include -IC:\Users\Justin\esp\esp-idf\components\spiffs\include -IC:\Users\Justin\esp\esp-idf\components\ulp\include -IC:\Users\Justin\esp\esp-idf\components\wifi_provisioning\include

CXX_FLAGS = -mlongcalls -Wno-frame-address -g -ffunction-sections -fdata-sections -Wall -Werror=all -Wno-error=unused-function -Wno-error=unused-variable -Wno-error=deprecated-declarations -Wextra -Wno-unused-parameter -Wno-sign-compare -ggdb -Og -fstrict-volatile-bitfields -Wno-error=unused-but-set-variable -std=gnu++11 -fno-exceptions -fno-rtti -D_GNU_SOURCE -DIDF_VER=\"v4.3\" -DESP_PLATFORM

