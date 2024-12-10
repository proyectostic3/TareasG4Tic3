#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <fcntl.h>  
#include "esp_timer.h" 

#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "sdkconfig.h"

//Paquetes de la conexion TCP

#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <stdlib.h>
#include <time.h>
#include <math.h> //biblioteca para crear datos matematicos
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
#include "driver/gpio.h" //para el led
#include "rom/gpio.h"  // permite inicializar el LED

#include <string.h> // permiote comparar strings



//Credenciales de WiFi

#define WIFI_SSID "CEFOP LAB 206" //"Telsur_MAER" //  "LAB.SISTEMAS DE COMUNICACIONES"
#define WIFI_PASSWORD  "00holo206"  //"C86C879F39F6X" // "Comunicaciones"
#define SERVER_IP     "192.168.1.113" // IP del servidor
#define SERVER_PORT   1234

// Pin del LED
#define LED_PIN GPIO_NUM_12




// Codigo del Sensor de Humedad:

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BUF_SIZE (128)       // buffer size
#define TXD_PIN 1            // UART TX pin
#define RXD_PIN 3            // UART RX pin
#define UART_NUM UART_NUM_0  // UART port number
#define BAUD_RATE 115200     // Baud rate
#define M_PI 3.14159265358979323846

#define I2C_MASTER_SCL_IO GPIO_NUM_22  // GPIO pin
#define I2C_MASTER_SDA_IO GPIO_NUM_21  // GPIO pin
#define I2C_MASTER_FREQ_HZ 10000
#define BME_ESP_SLAVE_ADDR 0x76
#define WRITE_BIT 0x0
#define READ_BIT 0x1
#define ACK_CHECK_EN 0x0
#define EXAMPLE_I2C_ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

esp_err_t ret = ESP_OK;
esp_err_t ret2 = ESP_OK;

uint16_t val0[6];

float task_delay_ms = 1000;

esp_err_t sensor_init(void) {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

esp_err_t bme_i2c_read(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme_i2c_write(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_wr, size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------ BME 688 ------------- //
uint8_t calc_gas_wait(uint16_t dur) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1176
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur >> 2;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

uint8_t calc_res_heat(uint16_t temp) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1145
    uint8_t heatr_res;
    uint8_t amb_temp = 25;

    uint8_t reg_par_g1 = 0xED;
    uint8_t par_g1;
    bme_i2c_read(I2C_NUM_0, &reg_par_g1, &par_g1, 1);

    uint8_t reg_par_g2_lsb = 0xEB;
    uint8_t par_g2_lsb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_lsb, &par_g2_lsb, 1);
    uint8_t reg_par_g2_msb = 0xEC;
    uint8_t par_g2_msb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_msb, &par_g2_msb, 1);
    uint16_t par_g2 = (int16_t)(CONCAT_BYTES(par_g2_msb, par_g2_lsb));

    uint8_t reg_par_g3 = 0xEE;
    uint8_t par_g3;
    bme_i2c_read(I2C_NUM_0, &reg_par_g3, &par_g3, 1);

    uint8_t reg_res_heat_range = 0x02;
    uint8_t res_heat_range;
    uint8_t mask_res_heat_range = (0x3 << 4);
    uint8_t tmp_res_heat_range;

    uint8_t reg_res_heat_val = 0x00;
    uint8_t res_heat_val;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) {
        temp = 400;
    }

    bme_i2c_read(I2C_NUM_0, &reg_res_heat_range, &tmp_res_heat_range, 1);
    bme_i2c_read(I2C_NUM_0, &reg_res_heat_val, &res_heat_val, 1);
    res_heat_range = (mask_res_heat_range & tmp_res_heat_range) >> 4;

    var1 = (((int32_t)amb_temp * par_g3) / 1000) * 256;
    var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (res_heat_range + 4));
    var5 = (131 * res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

int bme_get_chipid(void) {
    uint8_t reg_id = 0xd0;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_id, &tmp, 1);
    printf("Valor de CHIPID: %2X \n\n", tmp);

    if (tmp == 0x61) {
        printf("Chip BME688 reconocido.\n\n");
        return 0;
    } else {
        printf("Chip BME688 no reconocido. \nCHIP ID: %2x\n\n", tmp);  // %2X
    }

    return 1;
}

int bme_softreset(void) {
    uint8_t reg_softreset = 0xE0, val_softreset = 0xB6;

    ret = bme_i2c_write(I2C_NUM_0, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("\nError en softreset: %s \n", esp_err_to_name(ret));
        return 1;
    } else {
        printf("\nSoftreset: OK\n\n");
    }
    return 0;
}

void bme_forced_mode(void) {
    /*
    Fuente: Datasheet[19]
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=19

    Para configurar el BME688 en forced mode los pasos son:

    1. Set humidity oversampling to 1x     |-| 0b001 to osrs_h<2:0>
    2. Set temperature oversampling to 2x  |-| 0b010 to osrs_t<2:0>
    3. Set pressure oversampling to 16x    |-| 0b101 to osrs_p<2:0>

    4. Set gas duration to 100 ms          |-| 0x59 to gas_wait_0
    5. Set heater step size to 0           |-| 0x00 to res_heat_0
    6. Set number of conversion to 0       |-| 0b0000 to nb_conv<3:0> and enable gas measurements
    7. Set run_gas to 1                    |-| 0b1 to run_gas<5>

    8. Set operation mode                  |-| 0b01  to mode<1:0>

    */

    // Datasheet[33]
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0
    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // Configuramos el sensor de gas

    // 4. Seteamos gas_wait_0 a 100ms
    uint8_t gas_duration = calc_gas_wait(100);

    // 5. Seteamos res_heat_0
    uint8_t heater_step = calc_res_heat(300);

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000000;
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    bme_i2c_write(I2C_NUM_0, &gas_wait_0, &gas_duration, 1);
    bme_i2c_write(I2C_NUM_0, &res_heat_0, &heater_step, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 01, pasando primero por sleep
    uint8_t mode = 0b00000001;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) {
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03);
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03;
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_check_forced_mode(void) {
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t tmp, tmp2, tmp3, tmp4, tmp5;

    ret = bme_i2c_read(I2C_NUM_0, &ctrl_hum, &tmp, 1);
    ret = bme_i2c_read(I2C_NUM_0, &gas_wait_0, &tmp2, 1);
    ret = bme_i2c_read(I2C_NUM_0, &res_heat_0, &tmp3, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_gas_1, &tmp4, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp5, 1);
    vTaskDelay(task_delay_ms / portTICK_PERIOD_MS);
    return (tmp == 0b001 && tmp2 == 0x59 && tmp3 == 0x00 && tmp4 == 0b100000 && tmp5 == 0b01010101);
}

int bme_temp_celsius(uint32_t temp_adc) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion
    uint8_t addr_par_t1_lsb = 0xE9, addr_par_t1_msb = 0xEA;
    uint8_t addr_par_t2_lsb = 0x8A, addr_par_t2_msb = 0x8B;
    uint8_t addr_par_t3_lsb = 0x8C;
    uint16_t par_t1;
    uint16_t par_t2;
    uint16_t par_t3;

    uint8_t par[5];
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t3_lsb, par + 4, 1);

    par_t1 = (par[1] << 8) | par[0];
    par_t2 = (par[3] << 8) | par[2];
    par_t3 = par[4];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int t_fine;
    int calc_temp;

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = (int32_t)(var2 + var3);
    calc_temp = (((t_fine * 5) + 128) >> 8);
    return calc_temp;
}


//Funcion que entrega los valores de humedad 
int bme_hum(uint32_t hum_adc, int calc_temp) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion
    uint8_t addr_par_h1_lsb = 0xE2, addr_par_h1_msb = 0xE3;
    uint8_t addr_par_h2_lsb = 0xE2, addr_par_h2_msb = 0xE1;
    uint8_t addr_par_h3_lsb = 0xE4;
    uint8_t addr_par_h4_lsb = 0xE5;
    uint8_t addr_par_h5_lsb = 0xE6;
    uint8_t addr_par_h6_lsb = 0xE7;
    uint8_t addr_par_h7_lsb = 0xE8;

    uint8_t par_h1;
    uint8_t par_h2;


    uint8_t par[9];
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h3_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h4_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h5_lsb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h6_lsb, par + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h7_lsb, par + 8, 1);

    par_h1 = ( (par[1] & 0x0F)  >> 4 ) | par[0];
    par_h2 = (par[3] >> 4) | par[2];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    int64_t var5;
    int64_t var6;
    int hum_calc;
    int32_t scaled_temp = (int32_t)calc_temp ;

    var1 = ((int32_t)hum_adc >> 3) - (int32_t)((int32_t)par_h1 << 4) -((( scaled_temp*(int32_t)par[4] )/(int32_t)100) >> 1 );
    var2 = ((int32_t)par_h2 * ( (( scaled_temp*(int32_t)par[5] )/((int32_t)100)) + ((( scaled_temp*(( scaled_temp* (int32_t)par[6] )/((int32_t)100))) >> 6  ) / ((int32_t)100)) + ((int32_t)(1 << 14 )))) >>10 ;
    var3 = var1*var2;
    var4 = (((int32_t)par[7] << 7 ) + ((scaled_temp*(int32_t)par[8]/ ((int32_t)100) ))     ) >> 4;
    var5 = ((var3 >> 14)*(var3 >> 14)) >> 10 ;
    var6 = var4*var5 >> 1;
    hum_calc = (var3 + var6) >> 12;
    hum_calc = (((var3 + var6) >>10 ) * ((int32_t) 1000)) >> 12;
    return hum_calc;
}









void bme_get_mode(void) {
    uint8_t reg_mode = 0x74;
    uint8_t tmp;

    ret = bme_i2c_read(I2C_NUM_0, &reg_mode, &tmp, 1);

    tmp = tmp & 0x3;

    printf("Valor de BME MODE: %2X \n\n", tmp);
}

uint32_t bme_read_data(void) {  //  void bme_read_data(void)
    // Datasheet[23:41]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    uint8_t tmp;

    // Se obtienen los datos de temperatura
    uint8_t forced_temp_addr[] = {0x22, 0x23, 0x24};
    
        uint32_t temp_adc = 0;
        bme_forced_mode();
        // Datasheet[41]
        // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41

        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[0], &tmp, 1);
        temp_adc = temp_adc | tmp << 12;
        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[1], &tmp, 1);
        temp_adc = temp_adc | tmp << 4;
        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[2], &tmp, 1);
        temp_adc = temp_adc | (tmp & 0xf0) >> 4;

        uint32_t temp = bme_temp_celsius(temp_adc);
        return temp;
        //printf("Temperatura2s: %f\n", (float)temp / 100);
    
}

//Lee los valores de humedad:
uint32_t bme_read_hum_data(void) { //void bme_read_hum_data(void)
    // Datasheet[23:41]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    uint8_t tmp;

    // Se obtienen los datos de temperatura
    uint8_t forced_temp_addr[] = {0x25, 0x26};

        uint32_t temp_adc = 0;
        bme_forced_mode();
        // Datasheet[41]
        // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41

        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[0], &tmp, 1);
        temp_adc = temp_adc | tmp << 12;
        bme_i2c_read(I2C_NUM_0, &forced_temp_addr[1], &tmp, 1);
        temp_adc = temp_adc | tmp >> 4;

        uint32_t temp1 = bme_read_data();
        

        uint32_t hum = bme_hum( temp_adc, temp1 ); // bme_temp_celsius(temp_adc);
        //printf("Humedad: %f\n", (float)temp );
    return hum;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//Codigo de la comunicación TCP


// Variables de WiFi
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static const char* TAG = "WIFI";
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;



//esto queda igual
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
//esto tambien queda igual
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
// esta fun queda igual
void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}



// Función TCP principal:
//Variables Globales 
bool SendData = true;
bool DataMode = false;
int sampling_delay = 2000;



void socket_tcp_CicloPrincipal() {
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

    // Configurar el socket como no bloqueante
    fcntl(sock, F_SETFL, O_NONBLOCK);

    // Conectar al servidor
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        if (errno != EINPROGRESS) {
            ESP_LOGE(TAG, "Error al conectar");
            close(sock);
            return;
        }
    }

    char buffer[256];
    char rx_buffer[256];
    int rx_len;

    // Loop para mantener la conexión activa y recibir datos continuamente
    while (1) {
        rx_len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

        if (rx_len < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // No se recibieron datos en este ciclo (sin bloquear)
                ESP_LOGW(TAG, "No se recibieron datos (no bloqueante)");
                //vTaskDelay(10 / portTICK_PERIOD_MS); // Ajustar según la necesidad
            } else {
                ESP_LOGE(TAG, "Error al recibir datos");
                break; // Salir del bucle si hay un error grave
            }
        } else if (rx_len == 0) {
            // Conexión cerrada por el servidor
            ESP_LOGI(TAG, "Conexión cerrada por el servidor");
            break;
        } else {
            rx_buffer[rx_len] = '\0';  // Asegura cadena válida
            ESP_LOGI(TAG, "Datos recibidos: %s", rx_buffer); // Imprime lo que se recibió

            // Procesar respuesta
            if (strcmp(rx_buffer, "100") == 0) {
                sampling_delay = 10000;  // 100 Hz
                ESP_LOGI(TAG, "100 Hz\n");
            } 
            else if (strcmp(rx_buffer, "500") == 0) {
                sampling_delay = 2000;   // 500 Hz
                ESP_LOGI(TAG, "500 Hz\n");
            } 
            else if (strcmp(rx_buffer, "1000") == 0) {
                sampling_delay = 1000;   // 1000 Hz
                ESP_LOGI(TAG, "1000 Hz\n");
            } 
            else if (strcmp(rx_buffer, "si") == 0) {
                SendData = true;   
                ESP_LOGI(TAG, "Se activó el envío\n");
            }
            else if (strcmp(rx_buffer, "no") == 0) {
                SendData = false;   
                ESP_LOGI(TAG, "Se desactivó el envío\n");
            }
                        else if (strcmp(rx_buffer, "prom") == 0) {
                DataMode = true;   
                ESP_LOGI(TAG, "Se cambio a modo promedio\n");
            }
            else if (strcmp(rx_buffer, "inst") == 0) {
                DataMode = false;   
                ESP_LOGI(TAG, "Se cambio a modo instantaneo\n");
            }
        }

    
if (SendData && !DataMode) {
    // Se ejecuta si SendData es verdadero y DataMode falso
    uint32_t temp1 = bme_read_data();
    uint32_t hum1  = bme_read_hum_data();

    
    snprintf(buffer, 256, 
        "{\"temp: %.2f hum: %.2f \"}",
         (float)temp1 /100, (float)hum1 /100 );


        send(sock, buffer, strlen(buffer), 0); // enviar datos al servidor
        ESP_LOGI(TAG, "Datos enviados: %s", buffer);
        // gacer parpadear el LED durante el envioo de datos
        gpio_set_level(LED_PIN, 1);
        usleep(sampling_delay);  // apagar durante el tiempo de muestreo
        gpio_set_level(LED_PIN, 0);

        //
    
    } else if (SendData && DataMode) {
    uint32_t temp_sum = 0, hum_sum = 0;
    int sample_count = 0;
    gpio_set_level(LED_PIN, 1);
    // Periodo de promediación ( 2 segundos)
    uint32_t sampling_period = 2000000;  
    uint32_t start_time = esp_timer_get_time();  // Tiempo inical
    uint32_t end_time = esp_timer_get_time();

    while (( end_time  - start_time) < sampling_period) {
        temp_sum += bme_read_data();
        hum_sum  += bme_read_hum_data();
        sample_count++;
        end_time =  esp_timer_get_time();
        usleep(sampling_delay);  // Periodo entre muestras 
    }

    // Calculo de los promedios:
    float avg_temp = (float)temp_sum / sample_count / 100;
    float avg_hum  = (float)hum_sum  / sample_count / 100;

    // Format and send the averaged data
    snprintf(buffer, 256, 
        "{\"temp: %.2f hum: %.2f \"}", avg_temp, avg_hum);

    send(sock, buffer, strlen(buffer), 0);  // Send data to the server
    ESP_LOGI(TAG, "Datos enviados, Promedio sobre 5 segundos: %s", buffer);

    gpio_set_level(LED_PIN, 0);
}

    //gpio_set_level(LED_PIN, 0); // Apagar el LED al terminar



    }

    // Cerrar el socket
    close(sock);
}









// Funcion Principal:
void app_main(void) {

    nvs_init();
    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);

    // Configurar el LED
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    //COmunica que se logro conectar:
    ESP_LOGI(TAG,"Conectado a WiFi!\n");


    ESP_ERROR_CHECK(sensor_init());
    bme_get_chipid();
    bme_softreset();
    bme_get_mode();
    bme_forced_mode();
    printf("Comienza lectura\n\n");
   while(1){
    uint32_t temp1 = bme_read_data();
    printf("Temperatura: %f\n", (float)temp1 / 100);
    uint32_t hum1  = bme_read_hum_data();
    printf("Humedad: %f\n ", (float)hum1 /100);
 
    socket_tcp_CicloPrincipal();
    //usleep(sampling_delay);
        

}
}


