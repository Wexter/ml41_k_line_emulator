#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_system.h"
#include <string.h>

static const char *TAG = "ml41-emulator";

#define LED_GPIO 2
#define MS_TICKS(ms) (ms / portTICK_PERIOD_MS)
#define delay(ms) vTaskDelay(ms / portTICK_PERIOD_MS)

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
   { 0x03, 0x00, 0x09, 0x03, },                     // 0x00 NoData
   { 0x03, 0x00, 0x06, 0x03, },                     // 0x01 EndSession
   { 0x06, 0x00, 0x03, 0x0D, 0x00, 0x00, 0x03, },   // 0x02 ReadEPROM
   { 0x04, 0x00, 0x08, 0x00, 0x03, },               // 0x06 GetAFR
   { 0x04, 0x00, 0x08, 0x01, 0x03, },               // 0x07 GetVBat
   { 0x04, 0x00, 0x08, 0x02, 0x03, },               // 0x08 GetIntakeAirTemp
   { 0x04, 0x00, 0x08, 0x03, 0x03, },               // 0x09 GetCoolantTemp
   { 0x03, 0x00, 0x05, 0x03, },                     // 0x0A EraseErrorCodes
   { 0x04, 0x00, 0x08, 0x04, 0x03, },               // 0x0C GetCOPot
   { 0x04, 0x00, 0x08, 0x05, 0x03, },               // 0x0D GetO2Sensor
   { 0x04, 0x00, 0x08, 0x07, 0x03, },               // 0x0F GetIgnitionTime
   { 0x03, 0x00, 0x07, 0x03, },                     // 0x11 GetErrorCodes
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x3a, 0x03, },   // 0x12 GetRPM
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x20, 0x03, },   // 0x13 GetTPS
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x42, 0x03, },   // 0x14 GetEngineLoad
   { 0x06, 0x00, 0x01, 0x02, 0x00, 0x62, 0x03, },   // 0x15 GetInjectionTime

   // ML4.1 additional parameters
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x22, 0x03, },   // 0x19 GetACParams
   { 0x06, 0x00, 0x01, 0x01, 0x00, 0x29, 0x03, },   // 0x1a GetO2Params
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0x90, 0x03, },   // 0x1b GetFuelPumpParams
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xb0, 0x03, },   // 0x1c GetAdsorberParams

   // ML4.1 devices test
   { 0x04, 0x00, 0x04, 0x0e, 0x03, },               // 0x1d EnableInjector
   { 0x04, 0x00, 0x04, 0x1f, 0x03, },               // 0x1e EnableAdsorberValve
   { 0x04, 0x00, 0x04, 0x21, 0x03, },               // 0x1f EnableIdleValve

   // ML1.5 commands
   /*
   { 0x03, 0x00, 0x12, 0x03, },                     // 0x16
   { 0x03, 0x00, 0x1c, 0x03, },                     // 0x17
   { 0x06, 0x00, 0x01, 0x01, 0x01, 0xF8, 0x03, },   // 0x18
   */

   // ML1.5 devices test
   /*
   { 0x04, 0x00, 0x04, 0x10, 0x03, },               // 0x20
   { 0x04, 0x00, 0x04, 0x05, 0x03, },               // 0x21
   { 0x04, 0x00, 0x04, 0x04, 0x03, },               // 0x22
   { 0x04, 0x00, 0x04, 0x17, 0x03, },               // 0x23
   */
};

uint8_t ECU_RESPONSES[][24] = {
    { 0x03, 0x00, 0x09, 0x03, }, // NoData, // 0x00
    { 0x03, 0x00, 0x06, 0x03, }, // EndSession, // 0x01
    { 0x03, 0x00, 0x09, 0x03, }, // ReadEPROM, // 0x02
    { 0x05, 0x00, 0xFB, 0x00, 0x06, 0x03, }, // GetAFR, // 0x06
    { 0x05, 0x00, 0xFB, 0x00, 0xad, 0x03, }, // GetVBat, // 0x07
    { 0x05, 0x00, 0xFB, 0x00, 0xfd, 0x03, }, // GetIntakeAirTemp, // 0x08
    { 0x05, 0x00, 0xFB, 0x00, 0xfd, 0x03, }, // GetCoolantTemp, // 0x09
    { 0x03, 0x00, 0x09, 0x03, }, // EraseErrorCodes, // 0x0A
    { 0x05, 0x00, 0xFB, 0x00, 0x04, 0x03, }, // GetCOPot, // 0x0C
    { 0x05, 0x00, 0xFB, 0x00, 0x5c, 0x03, }, // GetO2Sensor, // 0x0D
    { 0x05, 0x00, 0xFB, 0x00, 0xfd, 0x03, }, // GetIgnitionTime, // 0x0F
    { 0x04, 0x00, 0xFC, 0x00, 0x03, }, // GetErrorCodes, // 0x11
    { 0x04, 0x00, 0xFE, 0x00, 0x03, }, // GetRPM, // 0x12
    // [3] & 3: 0 - MID, 1 - FULL, 2 - IDLE, 3 - ERR. [3] & 0x04 == 0: Manual transmission. [3] & 0x20 == 0: O2 sensor present
    { 0x04, 0x00, 0xFE, 0x10, 0x03, }, // GetTPS, // 0x13
    // [3] & 0x04 == 0: Fuel pump on, [3] & 0x20 == 0: engine torque control off
    { 0x04, 0x00, 0xFE, 0x00, 0x03, }, // GetEngineLoad, // 0x14
    { 0x05, 0x00, 0xFE, 0x00, 0x00, 0x03, }, // GetInjectionTime, // 0x15
    // [3] & 0x08 == 0: AC drive off, [3] & 0x10 == 0: AC switch off
    { 0x04, 0x00, 0xFE, 0x00, 0x03, }, // GetACParams, // 0x19
    // [3] & 0x20 == 0: open
    { 0x04, 0x00, 0xFE, 0x04, 0x03, }, // GetO2Params, // 0x1A
    { 0x04, 0x00, 0xFE, 0x07, 0x03, }, // GetFuelPumpParams, // 0x1B
    // [3] & 0x20 == 0: Valve open
    { 0x04, 0x00, 0xFE, 0xF7, 0x03, }, // GetAdsorberParams, // 0x1C
    { 0x03, 0x00, 0x09, 0x03, }, // EnableInjector, // 0x1D
    { 0x03, 0x00, 0x09, 0x03, }, // EnableAdsorberValve, // 0x1E
    { 0x03, 0x00, 0x09, 0x03, }, // EnableIdleValve, // 0x1F
};

enum EcuRequestID {
    NoData, // 0x00
    EndSession, // 0x01
    ReadEPROM, // 0x02
    GetAFR, // 0x06
    GetVBat, // 0x07
    GetIntakeAirTemp, // 0x08
    GetCoolantTemp, // 0x09
    EraseErrorCodes, // 0x0A
    GetCOPot, // 0x0C
    GetO2Sensor, // 0x0D
    GetIgnitionTime, // 0x0F
    GetErrorCodes, // 0x11
    GetRPM, // 0x12
    GetTPS, // 0x13
    GetEngineLoad, // 0x14
    GetInjectionTime, // 0x15
    GetACParams, // 0x19
    GetO2Params, // 0x1A
    GetFuelPumpParams, // 0x1B
    GetAdsorberParams, // 0x1C
    EnableInjector, // 0x1D
    EnableAdsorberValve, // 0x1E
    EnableIdleValve, // 0x1F
    EcuRequestMax
};

uint8_t ecu_eprom_code[]    = { 0x0D, 0x01, 0xF6, 0x31, 0x33, 0x31, 0x30, 0x30, 0x32, 0x31, 0x36, 0x32, 0x30, 0x03, };
uint8_t ecu_bosch_code[]    = { 0x0D, 0x03, 0xF6, 0x33, 0x34, 0x34, 0x36, 0x35, 0x33, 0x37, 0x36, 0x32, 0x31, 0x03, };
uint8_t ecu_gm_code[]       = { 0x0D, 0x05, 0xF6, 0x30, 0x33, 0x33, 0x34, 0x32, 0x33, 0x30, 0x39, 0x54, 0x46, 0x03, };

void gpio_isr_handler(void* arg);

void enable_led(void)
{
    gpio_set_level(LED_GPIO, 1);
}

void disable_led(void)
{
    gpio_set_level(LED_GPIO, 0);
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

void configure_led(void)
{
    gpio_reset_pin(LED_GPIO);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

void setup_k_line_init_isr()
{
    gpio_reset_pin(K_LINE_TDX_PIN);
    gpio_reset_pin(K_LINE_RXD_PIN);
    gpio_set_direction(K_LINE_RXD_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(K_LINE_RXD_PIN, GPIO_PULLUP_ENABLE);
    gpio_set_intr_type(K_LINE_RXD_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(K_LINE_RXD_PIN, gpio_isr_handler, (void *) K_LINE_RXD_PIN);

    ecu_state = 0;

    blink_led(3);
}

uint8_t find_request_packet_idx(const uint8_t *packet)
{
    for (uint8_t request_idx = 0; request_idx < EcuRequestMax; request_idx++)
    {
        if (packet[0] != ECU_REQUESTS[request_idx][0]) // compapre length first
            continue;

        uint8_t byte_idx = 2;

        for (; byte_idx < packet[0]; byte_idx++)
            if (packet[byte_idx] != ECU_REQUESTS[request_idx][byte_idx])
                break;

        if (byte_idx == packet[0])
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

bool ml41_send_packet(uint8_t* packet)
{
    for (uint8_t idx = 0; idx <= packet[0]; idx++)
        if (!k_line_send_byte(packet[idx], idx != packet[0]))
            return false;

    return true;
}

bool ml41_recv_packet(uint8_t* rx_buff)
{
    if (!k_line_read_byte(rx_buff, true))
        return false;

    for (uint8_t idx = 1; idx <= rx_buff[0]; idx++)
        if (!k_line_read_byte(rx_buff + idx, idx < rx_buff[0]))
            return false;

    return true;
}

bool init_full_speed_uart()
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
        return false;
    }

    delay(10);

    if (!k_line_send_byte(0x80, true))
    {
        blink_led(3);
        return false;
    }

    return true;
}

void start_session()
{
    uint8_t rx_buffer[32] = { 0x00 };

    if (!init_full_speed_uart()) goto task_end;

    enable_led();

    delay(50);

    if (!ml41_send_packet(ecu_eprom_code)) goto task_end;

    delay(10);

    if (!ml41_recv_packet(rx_buffer)) goto task_end;

    delay(50);

    if (!ml41_send_packet(ecu_bosch_code)) goto task_end;

    delay(10);

    if (!ml41_recv_packet(rx_buffer)) goto task_end;

    delay(50);

    if (!ml41_send_packet(ecu_gm_code))
        goto task_end;

    uint8_t response_data[64] = { 0x00 };
    uint8_t ecu_connection_sequence_number = 7;

    while (true) {
        if (!ml41_recv_packet(rx_buffer)) break;

        uint8_t packet_idx = find_request_packet_idx(rx_buffer);

        if (packet_idx >= EcuRequestMax)
            packet_idx = 0;

        memcpy(response_data, ECU_RESPONSES[packet_idx], ECU_RESPONSES[packet_idx][0] + 1);

        response_data[1] = ecu_connection_sequence_number;

        if (!ml41_send_packet(response_data)) break;

        if (packet_idx == EndSession)
        {
            blink_led(5);
            break;
        }

        // update sequence number
        ecu_connection_sequence_number += 2;
    }

task_end:
    delay(1000);

    uart_driver_delete(K_LINE_UART_NUMBER);

    disable_led();

    setup_k_line_init_isr();

    vTaskDelete(NULL);
}

void gpio_isr_handler(void* arg)
{
    if (xTaskGetTickCount() - last_isr_call_time < MS_TICKS(100))
        return;

    last_isr_call_time = xTaskGetTickCount();

    if (ecu_state == 0 && ++interrupts_count == 2) {
        ecu_state = 1;
        interrupts_count = 0;
        gpio_isr_handler_remove(K_LINE_RXD_PIN);
        xTaskCreate(start_session, "start_session", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
    }
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();

    setup_k_line_init_isr();

    // xTaskCreate(init_full_speed_uart, "init_full_speed_uart", 16384, NULL, configMAX_PRIORITIES - 2, NULL);
    while (1) {
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        // blink_led();
        // /* Toggle the LED state */
        // s_led_state = !s_led_state;
        // gpio_state = gpio_get_level(GPIO_NUM_0);
        // ESP_LOGI(TAG, "GPIO0: %d", gpio_state);
        // blink_led(5);

        delay(1000);
    }
}
