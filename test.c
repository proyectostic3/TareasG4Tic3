#include <stdio.h>
#include <string.h>


#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "lwip/sockets.h" // Para sockets

#include "math.h" //biblioteca para crear datos matematicos

//Credenciales de WiFi

#define WIFI_SSID "LAB.SISTEMAS DE COMUNICACIONES"
#define WIFI_PASSWORD "Comunicaciones"
#define SERVER_IP     "192.168.0.170" // IP del servidor
#define SERVER_PORT   1234

// Pin del LED
#define LED_PIN GPIO_NUM_2


// Variables de WiFi
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static const char* TAG = "WIFI";
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

// Funciones para generar datos simulados
float get_random_float(float min, float max) {
    return min + (float)rand() / (float)(RAND_MAX) * (max - min);
}

void generate_sensor_data(int n, char *buffer) {
    // Simular acelerómetro
    float acc_x = 2 * sin(2 * M_PI * 0.001 * n);
    float acc_y = 3 * cos(2 * M_PI * 0.001 * n);
    float acc_z = 10 * sin(2 * M_PI * 0.001 * n);

    // Simular sensor THCP
    float temp = get_random_float(5.0, 30.0);
    float hum = get_random_float(30.0, 80.0);
    float pres = get_random_float(1000.0, 1200.0);
    float co = get_random_float(30.0, 200.0);

    // Simular sensor de batería
    int batt = rand() % 100 + 1;

    // Crear el mensaje en formato JSON
    snprintf(buffer, 256, 
        "{\"acc_x\": %.2f, \"acc_y\": %.2f, \"acc_z\": %.2f, \"temp\": %.2f, \"hum\": %.2f, \"pres\": %.2f, \"co\": %.2f, \"batt\": %d}",
        acc_x, acc_y, acc_z, temp, hum, pres, co, batt);
}

void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(char* ssid, char* password) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));

    // Set the specific fields
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", ssid,
                 password);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", ssid,
                 password);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}


void socket_tcp(){
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr.s_addr);

    // Crear un socket
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Error al crear el socket");
        return;
    }
    
    // Conectar al servidor
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Error al conectar");
        close(sock);
        return;
    }

    // Encender el LED al iniciar la comunicación
    gpio_set_level(LED_PIN, 1);

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        ESP_LOGE(TAG, "Error al conectar");
        close(sock);
        gpio_set_level(LED_PIN, 0);  // Apagar el LED si falla la conexión
        return;
    }
    // Recibir la frecuencia de muestreo desde el servidor
    char frecuencia[4];
    int len = recv(sock, frecuencia, sizeof(frecuencia) - 1, 0);
    if (len > 0) {
        frecuencia[len] = '\0';
        int frec = atoi(frecuencia);  // Convertir a entero
        if (frec == 100) {
            sampling_delay = 10000;  // 100 Hz
        } else if (frec == 500) {
            sampling_delay = 2000;   // 500 Hz
        } else if (frec == 1000) {
            sampling_delay = 1000;   // 1000 Hz
        }
        ESP_LOGI(TAG, "Frecuencia recibida: %d Hz", frec);
    } else {
        ESP_LOGE(TAG, "Error al recibir la frecuencia");
        close(sock);
        gpio_set_level(LED_PIN, 0);
        return;
    }
    char buffer[256];
    int n = 0;
    while (1) {
        generate_sensor_data(n, buffer);
        n++;

        send(sock, buffer, strlen(buffer), 0); // enviar datos al servidor
        ESP_LOGI(TAG, "Datos enviados: %s", buffer);
        // gacer parpadear el LED durante el envioo de datos
        gpio_set_level(LED_PIN, 0);
        usleep(50000);  // apagar durante 50ms
        gpio_set_level(LED_PIN, 1);

        usleep(sampling_delay);
    }

    close(sock);
    gpio_set_level(LED_PIN, 0); // Apagar el LED al terminar
}



void app_main(void){
    nvs_init();
    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);

    // Configurar el LED
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG,"Conectado a WiFi!\n");
    socket_tcp();
}
