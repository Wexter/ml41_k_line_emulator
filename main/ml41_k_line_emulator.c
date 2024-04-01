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
#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS);

#define GPIO_INIT_PIN 0
#define GPIO_TX_PIN 1
#define GPIO_RX_PIN 3

#define K_LINE_UART_NUMBER UART_NUM_0
#define UART_BAUD_RATE 9600
#define UART_TXD_PIN GPIO_TX_PIN
#define UART_RXD_PIN GPIO_RX_PIN
#define UART_RX_BUF_SIZE 256

#define LED_BLINK_DELAY 100

uint8_t interrupts_count = 0,
    ecu_state = 0;

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

    // uart_write_bytes(K_LINE_UART_NUMBER, &rx_buff, 1);

    return (byte + rx_buff) == 0xFF;
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

void init_full_speed_uart()
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(K_LINE_UART_NUMBER, UART_RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(K_LINE_UART_NUMBER, &uart_config);
    uart_set_rx_full_threshold(K_LINE_UART_NUMBER, 1);
    uart_set_pin(K_LINE_UART_NUMBER, UART_TXD_PIN, UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_flush(K_LINE_UART_NUMBER);

    blink_led(1);

    delay(200);

    k_line_send_byte( 0x55, false);

    delay(50);

    if (!k_line_send_byte(0x38, true))
        goto task_end;

    if (!k_line_send_byte(0x80, true))
        goto task_end;

    // blink connection success
    enable_led();
    // blink_led(3);

    delay(5000);

task_end:
    uart_driver_delete(UART_NUM_0);
    disable_led();

    ecu_state = 0;

    vTaskDelete(NULL);
}

static void gpio_isr_handler(void* arg)
{
    if (ecu_state == 0 && ++interrupts_count == 4) {
        ecu_state = 1;
        interrupts_count = 0;
        xTaskCreate(init_full_speed_uart, "init_full_speed_uart", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
    }
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    gpio_reset_pin(GPIO_NUM_0);
    gpio_reset_pin(UART_TXD_PIN);
    gpio_reset_pin(UART_RXD_PIN);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ENABLE);
    gpio_set_intr_type(GPIO_NUM_0, GPIO_INTR_ANYEDGE);
    // gpio_isr_register()
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0, gpio_isr_handler, (void *) GPIO_NUM_0);

    // gpio_state = gpio_get_level(GPIO_NUM_0);

    while (1) {
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        // blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // gpio_state = gpio_get_level(GPIO_NUM_0);
        // ESP_LOGI(TAG, "GPIO0: %d", gpio_state);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
