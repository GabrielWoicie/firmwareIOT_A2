#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "MFRC522.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#define LED_STRIP_GPIO 8
#define LED_STRIP_LED_NUM 1

// Wi-Fi credentials
#define WIFI_SSID "Bagre"
#define WIFI_PASS "grupo123"

// MQTT Broker URI
#define MQTT_URI "mqtt://192.168.87.168:1883" 

static const char *TAG = "RFID_MQTT";

// Estados
typedef enum {
    IDLE,
    READ_CARD,
    ERROR,
    VERIFY
} State;

static State currentState = IDLE;
static State previousState = IDLE;
static spi_device_handle_t spi;
static esp_timer_handle_t verify_timer_handle;
static int error_count = 0;
static esp_mqtt_client_handle_t mqtt_client;

led_strip_handle_t led_strip;

void led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void led_clear() {
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}

// Wi-Fi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Wi-Fi conectado");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi desconectado, reconectando...");
        esp_wifi_connect();
    }
}

void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

// Timer callback para estado VERIFY
void verify_timer_callback(void *arg) {
    if (currentState != VERIFY) {
        previousState = currentState;
        currentState = VERIFY;
    }
}

void init_verify_timer() {
    esp_timer_create_args_t verify_timer_args = {
        .callback = &verify_timer_callback,
        .arg = NULL,
        .name = "verify_timer"
    };

    esp_timer_create(&verify_timer_args, &verify_timer_handle);
    esp_timer_start_periodic(verify_timer_handle, 5000000); // 5 segundos
}

void handle_verify_state() {
    uint8_t version = PCD_ReadRegister(spi, VersionReg);

    if (version == 0x92 || version == 0x91 || version == 0xB2) {
        error_count = 0;
        currentState = IDLE;
    } else {
        error_count++;
        if (error_count >= 5) {
            esp_mqtt_client_publish(mqtt_client, "rfid/erro", "MFRC522 falha", 0, 1, 0);
            currentState = ERROR;
            return;
        }
    }
    currentState = previousState;
}

void handle_idle_state() {
    led_clear();
    led_set_color(0, 0, 255); // LED azul aceso
    // permanece ocioso até receber comando via MQTT
}

void handle_read_card_state() {
    led_set_color(0, 255, 0); // LED verde aceso durante leitura

    if (PICC_IsNewCardPresent(spi) && PICC_ReadCardSerial(spi)) {
        char payload[64];
        char uid_str[32] = {0};


        for (uint8_t i = 0; i < uid.size; i++) {
            char byte_str[4];
            sprintf(byte_str, "%02X", uid.uidByte[i]);
            strcat(uid_str, byte_str);
        }
        snprintf(payload, sizeof(payload), "{\"uid\":\"%s\"}", uid_str);
        ESP_LOGI(TAG, "Publicando no MQTT: %s", payload);

        esp_mqtt_client_publish(mqtt_client, "rfid/leitura", payload, 0, 1, 0);
        
        PICC_HaltA(spi);
        //blink 2x led purple to finish reading
        led_set_color(255, 0, 255); // LED roxo aceso
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_clear();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_set_color(255, 0, 255); // LED roxo aceso
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        currentState = IDLE;
    }
}

void handle_error_state() {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    error_count = 0;
    currentState = IDLE;
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT conectado");
            esp_mqtt_client_subscribe(event->client, "rfid/comando", 1);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT dado recebido: topico=%.*s dado=%.*s",
                     event->topic_len, event->topic, event->data_len, event->data);
            if (strncmp(event->topic, "rfid/comando", event->topic_len) == 0) {
                if (strncmp(event->data, "ler", event->data_len) == 0) {
                    ESP_LOGI(TAG, "Comando de leitura recebido");
                    currentState = READ_CARD;
                } else if (strncmp(event->data, "parar", event->data_len) == 0) {
                    ESP_LOGI(TAG, "Comando de parar recebido");
                    if (currentState == READ_CARD) {
                        currentState = IDLE;
                        ESP_LOGI(TAG, "Estado alterado para IDLE");
                        esp_mqtt_client_publish(mqtt_client, "rfid/leitura", "interrompido", 0, 1, 0);
                    }
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    mqtt_event_handler_cb(event_data);
}

void fsm_task() {
    while (1) {
        switch (currentState) {
            case IDLE:
                handle_idle_state(); break;
            case READ_CARD:
                handle_read_card_state(); break;
            case ERROR:
                handle_error_state(); break;
            case VERIFY:
                handle_verify_state(); break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void app_main(void) {
    esp_err_t ret;
    nvs_flash_init();

    wifi_init_sta();


    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_STRIP_LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    PCD_Init(spi);

    init_verify_timer();
    fsm_task();
}