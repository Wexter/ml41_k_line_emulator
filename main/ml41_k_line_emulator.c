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

uint8_t interrupts_count = 0,
    ecu_state = 0;

TickType_t last_isr_call_time = 0;

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

uint8_t ecu_connection_sequence_number = 9;

uint8_t ecu_eprom_code[] = { 0x0D, 0x01, 0xF6, 0x31, 0x33, 0x31, 0x30, 0x30, 0x32, 0x31, 0x36, 0x32, 0x30, 0x03, };
uint8_t ecu_bosch_code[] = { 0x0D, 0x03, 0xF6, 0x33, 0x34, 0x34, 0x36, 0x35, 0x33, 0x37, 0x36, 0x32, 0x31, 0x03, };
uint8_t ecu_gm_code[] =    { 0x0D, 0x05, 0xF6, 0x30, 0x33, 0x33, 0x34, 0x32, 0x33, 0x30, 0x39, 0x54, 0x46, 0x03, };
uint8_t ecu_no_data[] = { 0x03, 0x07, 0x09, 0x03 };
uint8_t ecu_rpm_data[] = { 0x04, 0x09, 0xfe, 0x00, 0x03, };

void k_line_send_packet(uint8_t* packet)
{
    for (uint8_t idx = 0; idx <= packet[0]; idx++)
        k_line_send_byte(packet[idx], idx != packet[0]);
}

void k_line_recv_packet()
{
    uint8_t rx_buff;

    k_line_read_byte(&rx_buff, true);

    for (uint8_t idx = rx_buff; idx > 0; idx--)
        k_line_read_byte(&rx_buff, idx > 1);
}

void init_full_speed_uart()
{
    delay(750);

    ESP_LOGI(TAG, "init_full_speed_uart");

    blink_led(1);

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

    k_line_recv_packet();

    delay(50);

    k_line_send_packet(ecu_bosch_code);

    delay(10);

    k_line_recv_packet();

    delay(50);

    k_line_send_packet(ecu_gm_code);

    int ecu_rpm_increment = 5;

    while (true) {
        delay(20);

        k_line_recv_packet();

        delay(50);

        // ecu_rpm_data[1] = ecu_connection_sequence_number;
        ecu_rpm_data[3] = rand() % 160;
        // ecu_rpm_data[3] += ecu_rpm_increment;
        k_line_send_packet(ecu_rpm_data);

        // update sequence number
        ecu_connection_sequence_number += 2;

        if (ecu_rpm_data[3] >= 160 )
            ecu_rpm_increment = -5;

        if (ecu_rpm_data[3] == 0)
            ecu_rpm_increment = 5;
    }

    delay(5000);

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
