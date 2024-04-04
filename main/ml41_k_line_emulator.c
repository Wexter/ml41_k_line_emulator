/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_system.h"
#include <string.h>

static const char *TAG = "example";

#define LED_GPIO 2
#define MS_TICKS(ms) (ms / portTICK_PERIOD_MS)
#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS);

#define UART_BAUD_RATE 8860
#define UART_RX_BUF_SIZE 256

#define K_LINE_UART_NUMBER UART_NUM_0
#define K_LINE_TDX_PIN GPIO_NUM_1
#define K_LINE_RXD_PIN GPIO_NUM_3

// #define K_LINE_UART_NUMBER UART_NUM_2
// #define K_LINE_TDX_PIN GPIO_NUM_17
// #define K_LINE_RXD_PIN GPIO_NUM_16

#define LED_BLINK_DELAY 100
#define ECU_MAX_REQUEST 0x1F

uint8_t interrupts_count = 0,
    ecu_state = 0;

TickType_t last_isr_call_time = 0;
const uint8_t ECU_REQUESTS[][8] = {
   { 0x03, 0x00, 0x09, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x00
   { 0x03, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x01
   { 0x06, 0x00, 0x03, 0x0D, 0x00, 0x00, 0x03, 0x00 }, // 0x02
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x03
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x04
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x05
   { 0x04, 0x00, 0x08, 0x00, 0x03, 0x00, 0x00, 0x00 }, // 0x06
   { 0x04, 0x00, 0x08, 0x01, 0x03, 0x00, 0x00, 0x00 }, // 0x07
   { 0x04, 0x00, 0x08, 0x02, 0x03, 0x00, 0x00, 0x00 }, // 0x08
   { 0x04, 0x00, 0x08, 0x03, 0x03, 0x00, 0x00, 0x00 }, // 0x09
   { 0x03, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x0A
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x0B
   { 0x04, 0x00, 0x08, 0x04, 0x03, 0x00, 0x00, 0x00 }, // 0x0C
   { 0x04, 0x00, 0x08, 0x05, 0x03, 0x00, 0x00, 0x00 }, // 0x0D
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x0E
   { 0x04, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x00 }, // 0x0F
   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 0x10
   { 0x03, 0x00, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x11
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, 0x00 }, // 0x12
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, 0x00 }, // 0x13
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, 0x00 }, // 0x14
   { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, 0x00 }, // 0x15

   // ML1.5 commands
   { 0x03, 0x00, 0x12, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x16
   { 0x03, 0x00, 0x1c, 0x03, 0x00, 0x00, 0x00, 0x00 }, // 0x17
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xF8, 0x03, 0x00 }, // 0x18

   // ML4.1 additional parameters
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x22, 0x03, 0x00 }, // 0x19
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x29, 0x03, 0x00 }, // 0x1a
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0x90, 0x03, 0x00 }, // 0x1b
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xb0, 0x03, 0x00 }, // 0x1c

   // ML4.1 devices test
   { 0x04, 0x00, 0x04, 0x0e, 0x03, 0x00, 0x00, 0x00 }, // 0x1d
   { 0x04, 0x00, 0x04, 0x1f, 0x03, 0x00, 0x00, 0x00 }, // 0x1e
   { 0x04, 0x00, 0x04, 0x21, 0x03, 0x00, 0x00, 0x00 }, // 0x1f

   // ML1.5 devices test
   /*
   { 0x04, 0x00, 0x04, 0x10, 0x03, 0x00, 0x00, 0x00 }, // 0x20
   { 0x04, 0x00, 0x04, 0x05, 0x03, 0x00, 0x00, 0x00 }, // 0x21
   { 0x04, 0x00, 0x04, 0x04, 0x03, 0x00, 0x00, 0x00 }, // 0x22
   { 0x04, 0x00, 0x04, 0x17, 0x03, 0x00, 0x00, 0x00 }, // 0x23
   */
};

uint8_t ecu_eprom_code[]    = { 0x0D, 0x01, 0xF6, 0x31, 0x33, 0x31, 0x30, 0x30, 0x32, 0x31, 0x36, 0x32, 0x30, 0x03, };
uint8_t ecu_bosch_code[]    = { 0x0D, 0x03, 0xF6, 0x33, 0x34, 0x34, 0x36, 0x35, 0x33, 0x37, 0x36, 0x32, 0x31, 0x03, };
uint8_t ecu_gm_code[]       = { 0x0D, 0x05, 0xF6, 0x30, 0x33, 0x33, 0x34, 0x32, 0x33, 0x30, 0x39, 0x54, 0x46, 0x03, };
uint8_t ecu_no_data[]       = { 0x03, 0x07, 0x09, 0x03 };
uint8_t ecu_rpm_data[]      = { 0x04, 0x09, 0xFE, 0x00, 0x03, };
uint8_t ecu_errors_data[]   = { 0x08, 0x00, 0xFC, 0x41, 0x60, 0x11, 0x3D, 0x14, 0x03, }; // 0x40 0x60 0xA0 0xE0

uint8_t ecu_tps_data[]      = { 0x04, 0x00, 0xFE, 0x10 | 0x04, 0x03, }; // [3] & 3: 0 - MID, 1 - FULL, 2 - IDLE, 3 - ERR. [3] & 0x04 == 0: Manual transmission. [3] & 0x20 == 0: O2 sensor present
uint8_t ecu_ac_data[]           = { 0x04, 0x00, 0xFE, 0x08 | 0x10, 0x03, }; // [3] & 0x08 == 0: AC drive off, [3] & 0x10 == 0: AC switch off
uint8_t ecu_lambda_reg_data[]   = { 0x04, 0x00, 0xFE, 0x20, 0x03, }; // [3] & 0x20 == 0: open
uint8_t ecu_engine_power_data[] = { 0x04, 0x00, 0xFE, 0x20, 0x03, }; // [3] & 0x04 == 0: Fuel pump on, [3] & 0x20 == 0: engine torque control off
uint8_t ecu_adsorber_data[]     = { 0x04, 0x00, 0xFE, 0x20, 0x03, }; // [3] & 0x20 == 0: Valve open


static void enable_led(void)
{
    gpio_set_level(LED_GPIO, 1);
}

static void disable_led(void)
{
    gpio_set_level(LED_GPIO, 0);
}

static void configure_led(void)
{
    gpio_reset_pin(LED_GPIO);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

uint8_t find_packet_idx(const uint8_t *packet)
{
    for (uint8_t request_idx = 0; request_idx <= ECU_MAX_REQUEST; request_idx++)
    {
        if (packet[0] != ECU_REQUESTS[request_idx][0])
            continue;

        uint8_t byte_idx = 1;

        for (; byte_idx <= packet[0]; byte_idx++)
            if (packet[byte_idx + 1] != ECU_REQUESTS[request_idx][byte_idx + 1])
                break;

        if (byte_idx == packet[0] + 1)
            return request_idx;
    }

    return 0xFF;
}

bool k_line_send_byte(uint8_t byte, bool wait_echo)
{
    uart_write_bytes(K_LINE_UART_NUMBER, &byte, 1);

    if (!wait_echo)
        return true;

    uint8_t rx_buff;

    if (1 > uart_read_bytes(K_LINE_UART_NUMBER, &rx_buff, 1, 1000 / portTICK_PERIOD_MS))
        return false;

    // emulate k-line
    uart_write_bytes(K_LINE_UART_NUMBER, &rx_buff, 1);

    return (byte + rx_buff) == 0xFF;
}

bool k_line_read_byte(uint8_t* rx_buff, bool send_echo)
{
    if (1 > uart_read_bytes(K_LINE_UART_NUMBER, rx_buff, 1, 1000 / portTICK_PERIOD_MS))
        return false;

    // emulate k-line with duplicating host-sent byte on host RX
    uart_write_bytes(K_LINE_UART_NUMBER, rx_buff, 1);

    if (!send_echo)
        return true;

    uint8_t echo_byte = ~(*rx_buff);
    uart_write_bytes(K_LINE_UART_NUMBER, &echo_byte, 1);

    return true;
}

void blink_led(uint8_t count)
{
    disable_led();

    do {
        delay(LED_BLINK_DELAY);
        enable_led();
        delay(LED_BLINK_DELAY);
        disable_led();
    } while (--count > 0);
}

void k_line_send_packet(uint8_t* packet)
{
    for (uint8_t idx = 0; idx <= packet[0]; idx++)
        k_line_send_byte(packet[idx], idx != packet[0]);
}

void k_line_recv_packet(uint8_t* rx_buff)
{
    k_line_read_byte(rx_buff, true);

    for (uint8_t idx = 1; idx <= rx_buff[0]; idx++)
        k_line_read_byte(rx_buff + idx, idx < rx_buff[0]);
}

void init_full_speed_uart()
{
    delay(750);

    ESP_LOGI(TAG, "init_full_speed_uart");

    blink_led(1);

    uint8_t rx_buffer[64] = { 0x00 };

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    gpio_reset_pin(K_LINE_RXD_PIN);
    uart_driver_install(K_LINE_UART_NUMBER, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(K_LINE_UART_NUMBER, &uart_config);
    uart_set_rx_full_threshold(K_LINE_UART_NUMBER, 1);
    uart_set_pin(K_LINE_UART_NUMBER, K_LINE_TDX_PIN, K_LINE_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_flush(K_LINE_UART_NUMBER);

    blink_led(1);

    delay(200);

    k_line_send_byte( 0x55, false);

    delay(50);

    if (!k_line_send_byte(0x38, true))
    {
        blink_led(2);
        goto task_end;
    }

    delay(10);

    if (!k_line_send_byte(0x80, true))
    {
        blink_led(3);
        goto task_end;
    }

    enable_led();

    delay(50);

    k_line_send_packet(ecu_eprom_code);

    delay(10);

    k_line_recv_packet(rx_buffer);

    delay(50);

    k_line_send_packet(ecu_bosch_code);

    delay(10);

    k_line_recv_packet(rx_buffer);

    delay(50);

    k_line_send_packet(ecu_gm_code);

    uint8_t response_data[64] = { 0x00 };
    uint8_t ecu_connection_sequence_number = 7;

    while (true) {
        delay(20);

        k_line_recv_packet(rx_buffer);

        delay(50);

        uint8_t packet_idx = find_packet_idx(rx_buffer);

        switch (packet_idx)
        {
        case 0x11:
            memcpy(response_data, ecu_errors_data, ecu_errors_data[0] + 1);
            break;
        case 0x12:
            memcpy(response_data, ecu_rpm_data, ecu_rpm_data[0] + 1);
            break;
        case 0x13:
            memcpy(response_data, ecu_tps_data, ecu_tps_data[0] + 1);
            break;
        case 0x19:
            memcpy(response_data, ecu_ac_data, ecu_ac_data[0] + 1);
            break;
        case 0x1a:
            memcpy(response_data, ecu_lambda_reg_data, ecu_lambda_reg_data[0] + 1);
            break;
        case 0x1b:
            memcpy(response_data, ecu_engine_power_data, ecu_engine_power_data[0] + 1);
            break;
        case 0x1c:
            memcpy(response_data, ecu_adsorber_data, ecu_adsorber_data[0] + 1);
            break;
        default:
            memcpy(response_data, ecu_no_data, ecu_no_data[0] + 1);
            break;
        }

        response_data[1] = ecu_connection_sequence_number;

        k_line_send_packet(response_data);

        // update sequence number
        ecu_connection_sequence_number += 2;
    }

    // delay(5000);

task_end:
    uart_driver_delete(K_LINE_UART_NUMBER);
    disable_led();

    ecu_state = 0;

    vTaskDelete(NULL);
}

static void gpio_isr_handler(void* arg)
{
    if (xTaskGetTickCount() - last_isr_call_time < MS_TICKS(500))
        return;

    last_isr_call_time = xTaskGetTickCount();

    if (ecu_state == 0 && ++interrupts_count == 2) {
        ecu_state = 1;
        interrupts_count = 0;
        gpio_isr_handler_remove(K_LINE_RXD_PIN);
        xTaskCreate(init_full_speed_uart, "init_full_speed_uart", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
    }
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    gpio_reset_pin(K_LINE_TDX_PIN);
    gpio_reset_pin(K_LINE_RXD_PIN);
    gpio_set_direction(K_LINE_RXD_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(K_LINE_RXD_PIN, GPIO_PULLUP_ENABLE);
    gpio_set_intr_type(K_LINE_RXD_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(K_LINE_RXD_PIN, gpio_isr_handler, (void *) K_LINE_RXD_PIN);

    // xTaskCreate(init_full_speed_uart, "init_full_speed_uart", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
    while (1) {
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        // blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // gpio_state = gpio_get_level(GPIO_NUM_0);
        // ESP_LOGI(TAG, "GPIO0: %d", gpio_state);
        // blink_led(5);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
