#include "voice.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h" // For esp_rom_delay_us
#include "driver/gpio.h"
#include "driver/uart.h"
#include <string.h>
#include <ctype.h>
#include "nvs_flash.h"
#include "nvs.h"

#define TAG "SKOEGLE ESP32:"
#define NVS_NAMESPACE2 "phone_numbers"
#define PRIMARY_NUM_KEY "primary_number"
#define SECONDARY_NUM_KEY "secondary"
#define THIRD_NUM_KEY "third"

char PRIMARY[20] = "+919689343153";   // Default number
char SECONDARY[20] = "+919689343153"; // Default number
char THIRD[20] = "+919689343153";     // Default number

#define AUDIO_PIN 25   // Using GPIO25 (works on all ESP32 models)
#define PWM_FREQ 64000 // 64kHz gives good quality for 8kHz audio
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define SOS_BUTTON 12
#define CPU_LED 27

#define UART_GSM UART_NUM_1 // For GSM Module
#define UART_GPS UART_NUM_2 // For GPS Module

#define GPS_TX_PIN 18 // GPS TX → ESP32 RX (UART2)
#define GPS_RX_PIN 19 // GPS RX → ESP32 TX (UART2)
#define GSM_TX_PIN 22 // GSM TX → ESP32 RX (UART1)
#define GSM_RX_PIN 23 // GSM RX → ESP32 TX (UART1)

// Baud Rates
#define GPS_BAUD_RATE 9600 // Standard GPS baud rate
#define GSM_BAUD_RATE 115200 // Common GSM baud rate

// Buffer Sizes
#define UART_BUF_SIZE 1024

volatile bool sos_status = false;

bool send_command(char *cmd, const char *expected_resp, uint32_t timeout_ms);
bool parse_sms(const char *raw_sms, char *sender, char *content, size_t max_len);
bool extract_target_number(const char *sms_content, char *number, size_t max_len,char *sender);

void IRAM_ATTR button_isr_handler(void *arg)
{
    static uint32_t last_isr_time = 0;
    uint32_t now = xTaskGetTickCountFromISR();

    // crude software debounce: ignore if within 200ms
    if (now - last_isr_time > pdMS_TO_TICKS(200))
    {
        sos_status = true;
        last_isr_time = now;
    }
}
void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        ESP_LOGE(TAG, "NVR IS NOW FREE");
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGE(TAG, "NVR IS ALREADY FREE");
}
void send_sms(char *msg, const char *number)
{
    send_command("AT+CMGF=1\r","OK",2000);
    char sms_cmd[64];
    snprintf(sms_cmd, sizeof(sms_cmd), "AT+CMGS=\"%s\"\r", number);
    send_command(sms_cmd, ">", 1000);
    // Send message (terminated with Ctrl+Z)
    uart_write_bytes(UART_GSM, msg, strlen(msg));
    uart_write_bytes(UART_GSM, "\x1A", 1); // Ctrl+Z to send    
    printf("SMS sent to %s\n", number);
}
bool save_number(const char *key, const char *number)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE2, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_str(handle, key, number);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save number: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }
    err = nvs_commit(handle);
    nvs_close(handle);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Saved number: %s", number);
        if(strcmp(key, PRIMARY_NUM_KEY) == 0)
        {
            send_sms("YOUR NUMBER IS SET AS PRIMARY",number);
        }
        else if(strcmp(key, SECONDARY_NUM_KEY) == 0)
        {
            send_sms("YOUR NUMBER IS SET AS SECONDARY",number);
        }
        else if (strcmp(key, THIRD_NUM_KEY) == 0)
        {
            send_sms("YOUR NUMBER IS SET AS EMERGENCY",number);
        }        
        return true;
    }
    return false;
}
bool read_number(const char *key, char *buffer, size_t length)
{
    nvs_handle_t handle;
    size_t required_size = 0;

    esp_err_t err = nvs_open(NVS_NAMESPACE2, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_get_str(handle, key, NULL, &required_size);
    if (err != ESP_OK || required_size == 0)
    {
        ESP_LOGE(TAG, "Error reading key: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }
    if (required_size > length)
    {
        ESP_LOGE(TAG, "Buffer too small. Need %d bytes", required_size);
        nvs_close(handle);
        return false;
    }
    err = nvs_get_str(handle, key, buffer, &required_size);
    nvs_close(handle);
    if (err == ESP_OK)
    {
        // ESP_LOGI(TAG, "Read number: %s", buffer);
        return true;
    }
    ESP_LOGE(TAG, "Failed to read: %s", esp_err_to_name(err));
    return false;
}
void check_number()
{
    // First try to read existing number
    char number_buffer[20];
    if (read_number(PRIMARY_NUM_KEY, number_buffer, sizeof(number_buffer)))
    {
        // Only update if read succeeded
        strlcpy(PRIMARY, number_buffer, sizeof(PRIMARY));
        ESP_LOGI(TAG, "PRIMARY NUMBER FOUND IN MEMORY");
    }
    if (read_number(SECONDARY_NUM_KEY, number_buffer, sizeof(number_buffer)))
    {
        // Only update if read succeeded
        strlcpy(SECONDARY, number_buffer, sizeof(SECONDARY));
        ESP_LOGI(TAG, "SECONDARY NUMBER FOUND IN MEMORY");
    }
    if (read_number(THIRD_NUM_KEY, number_buffer, sizeof(number_buffer)))
    {
        // Only update if read succeeded
        strlcpy(THIRD, number_buffer, sizeof(THIRD));
        ESP_LOGI(TAG, "THIRD NUMBER FOUND IN MEMORY");
    }
}
void init_uart(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
}
bool extract_target_number(const char *sms_content, char *number, size_t max_len,char *sender)
{
    const char *prefix = "SET-NO@";
    char new_numbers[14] = {0};
    char all_number[90]="FOLLOW NUMBER SET AS\n";    
    const char *ptr = strstr(sms_content, prefix);
    int set_count=0;    
    if (!ptr)
        return false;

    ptr += strlen(prefix);

    while (*ptr == ' ')
        ptr++;

    // Copy only digits (international numbers may start with +)
    int i = 0;
    while (i < max_len - 1 && *ptr != '@' && *ptr != '\0')
    {
        if (isdigit((unsigned char)*ptr) || (i == 0 && *ptr == '+'))
        {
            new_numbers[i++] = *ptr;
        }
        ptr++;
    }
    ESP_LOGI(TAG, "FIRST N0: %s--AT %d", new_numbers, i);
    if (i == 13)
    {
        if (save_number(PRIMARY_NUM_KEY, new_numbers))
        {
            ESP_LOGI(TAG, "FIRST Number updated successfully");
        }
        set_count++;
        strcat(all_number, "PRIMARY:");
        strcat(all_number, new_numbers);
        strcat(all_number, ",\n");

    }
    else
    {
        ESP_LOGI(TAG, "FIRST NUMBER NOT FOUND");
    }
    /// skip send @
    ptr++;
    // remove white space
    while (*ptr == ' ')
        ptr++;
    i = 0;
    memset(new_numbers, 0, sizeof(new_numbers));
    while (i < max_len - 1 && *ptr != '@' && *ptr != '\0')
    {
        if (isdigit((unsigned char)*ptr) || (i == 0 && *ptr == '+'))
        {
            new_numbers[i++] = *ptr;
        }
        ptr++;
    }
    ESP_LOGI(TAG, "SECOND N0: %s--AT %d", new_numbers, i);
    if (i == 13)
    {
        if (save_number(SECONDARY_NUM_KEY, new_numbers))
        {
            ESP_LOGI(TAG, "SECOND Number updated successfully");
        }
        set_count++;
        strcat(all_number, "SECONDARY:");
        strcat(all_number, new_numbers);
        strcat(all_number, ",\n");
    }
    else
    {
        ESP_LOGI(TAG, "SECOND NUMBER NOT FOUND");
    }

    /// skip send @
    ptr++;
    // remove white space
    while (*ptr == ' ')
        ptr++;
    i = 0;
    memset(new_numbers, 0, sizeof(new_numbers));
    while (i < max_len - 1 && *ptr != '@' && *ptr != '\0')
    {
        if (isdigit((unsigned char)*ptr) || (i == 0 && *ptr == '+'))
        {
            new_numbers[i++] = *ptr;
        }
        ptr++;
    }
    ESP_LOGI(TAG, "THIRD N0: %s--AT %d", new_numbers, i);
    if (i == 13)
    {
        if (save_number(THIRD_NUM_KEY, new_numbers))
        {
            ESP_LOGI(TAG, "THIRD Number updated successfully");
        }
        set_count++;
        strcat(all_number, "THIRD:");
        strcat(all_number, new_numbers);
        strcat(all_number, ",\n");
    }
    else
    {
        ESP_LOGI(TAG, "THIRD NUMBER NOT FOUND");
    }    
    if(set_count>0)
    {
        send_sms(all_number,sender);
    }
    else
    {
        send_sms("NOT ABLE TO STORE NUMBER..WRONG SMS FORMAT.\n SEND AS SET-NO@XX@XX@XX",sender);
    }
    ESP_LOGI(TAG, "NUMBER CONFIGURATION IS OVER");
    return true;
}
bool parse_sms(const char *raw_sms, char *sender, char *content, size_t max_len)
{
    // Format: +CMT: "+919876543210","","22/05/30,16:45:23+22"\r\nSET-PRYNOSET +123456789
    // --- Extract Sender Number ---
    const char *num_start = strchr(raw_sms, '"');
    if (!num_start)
        return false;
    num_start++; // Skip opening quote

    const char *num_end = strchr(num_start, '"');
    if (!num_end || (size_t)(num_end - num_start) >= max_len)
        return false;

    // Copy sender number (e.g., "+919876543210")
    size_t sender_len = num_end - num_start;
    strncpy(sender, num_start, sender_len);
    sender[sender_len] = '\0';

    // --- Extract SMS Content ---
    // Find last occurrence of \r\n or \n
    const char *msg_start = strstr(raw_sms, "\r\n");
    if (!msg_start)
        msg_start = strchr(raw_sms, '\n');
    if (!msg_start)
        return false;

    msg_start += (msg_start[0] == '\r' && msg_start[1] == '\n') ? 2 : 1; // Skip \r\n or \n

    // Copy content (ensure null-terminated)
    strncpy(content, msg_start, max_len - 1);
    content[max_len - 1] = '\0';

    return true;
}
void read_sms_or_call()
{
    uint8_t resp_buffer[256] = {0};
    int len = uart_read_bytes(UART_GSM, resp_buffer, sizeof(resp_buffer) - 1, pdMS_TO_TICKS(100));
    if (len > 0)
    {
        resp_buffer[len] = '\0';
        ESP_LOGI(TAG, "Received: %s", resp_buffer);

        if (strstr((char *)resp_buffer, "+CMT:"))
        {
            char sender[20] = {0};
            char content[100] = {0};
            if (parse_sms((char *)resp_buffer, sender, content, sizeof(content)))
            {
                ESP_LOGI(TAG, "From: %s", sender);
                ESP_LOGI(TAG, "Content: %s", content);

                // Extract target number from SET-PRYNOSET
                char target_number[20] = {0};
                if (extract_target_number(content, target_number, sizeof(target_number),sender))
                {
                    ESP_LOGI(TAG, "Extracted number: %s", target_number);
                    // SEND SMS
                }
                else
                {
                    ESP_LOGE(TAG, "INVALID SMS FORMAT");
                }
            }
        }
    }
}
void make_call(char number)
{
    char sms_cmd[64];
    snprintf(sms_cmd, sizeof(sms_cmd), "ATD\"%s\";\r", number);
    send_command(sms_cmd, "OK", 1000);
    // Send message (terminated with Ctrl+Z)
    printf("CALLING to %s\n", number);
}
void read_gps_location()
{

}
void RUNNIG_task(void *arg)
{
    while (1)
    {
        read_sms_or_call();
        read_gps_location();
        if(sos_status==true)
        {
            //sos pressed send sms and make call;
            send_sms("ALERT-SOS DETECTED",PRIMARY);
            send_sms("ALERT-SOS DETECTED",SECONDARY);
            send_sms("ALERT-SOS DETECTED",THIRD);
            make_call(PRIMARY);            
            sos_status=false;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void STATUS_LED_task(void *arg)
{
    while (1)
    {
        gpio_set_level(CPU_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(CPU_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void PLAY_VOICE(void *arg)
{
    while (1)
    {
        if (sos_status)
        {
            for (int i = 0; i < voice_audio_len; i += 800)
            { // Process 100ms chunks
                for (int j = 0; j < 800 && (i + j) < voice_audio_len; j++)
                {
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, voice_audio[i + j]);
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                    esp_rom_delay_us(125); // 8000Hz = 125µs/sample
                }
                vTaskDelay(1); // Yield after each chunk
            }
            sos_status = false;
        }
        vTaskDelay(10); // Small delay when idle
    }
}
void ini_port()
{
    gpio_set_direction(AUDIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SOS_BUTTON, GPIO_MODE_INPUT);
    gpio_set_direction(CPU_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(AUDIO_PIN, 0);

    // Button setup
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SOS_BUTTON),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE};
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SOS_BUTTON, button_isr_handler, NULL);

    // PWM COFIG FOR AUDIO

    // 3. PWM Configuration
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .gpio_num = AUDIO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 128, // Start at midpoint (50% duty)
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}
void init_gsm()
{
    send_command("AT\r\n", "OK", 2000);
    send_command("ATE0\r\n", "OK", 2000);
    send_command("AT+CMGD=1,4\r\n", "OK", 2000);
    send_command("AT+CPIN?\r\n", "READY", 2000);
    send_command("AT+CMGF=1\r\n", "OK", 2000);
    send_command("AT+CNMI=2,1\r\n", "OK", 2000);
    send_command("AT+CGATT=1\r\n", "OK", 4000);
    send_command("AT+CGACT=1,1\r\n", "OK", 4000);
}
bool send_command(char *cmd, const char *expected_resp, uint32_t timeout_ms)
{
    uint8_t resp_buffer[256] = {0};
    bool success = false;
    // Clear UART buffer before sending
    uart_flush_input(UART_GSM);
    ESP_LOGI(TAG, "Sending: %s", cmd);
    uart_write_bytes(UART_GSM, cmd, strlen(cmd));

    // Wait for response
    uint32_t start_time = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms)
    {
        int len = uart_read_bytes(UART_GSM, resp_buffer, sizeof(resp_buffer) - 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            resp_buffer[len] = '\0';
            ESP_LOGI(TAG, "Received: %s", resp_buffer);

            if (strstr((char *)resp_buffer, expected_resp))
            {
                success = true;
                ESP_LOGI(TAG, "PASS");
                break;
            }
        }
    }
    return success;
}
void app_main()
{
    // 1. Hardware verification
    ESP_LOGI(TAG, "Running hardware test...");
    // esp_task_wdt_init(30);
    init_nvs();
    ini_port();
    xTaskCreate(STATUS_LED_task, "STATUS LED task", 1024, NULL, 1, NULL);
    check_number();
    init_uart(UART_GPS, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD_RATE); // GPS on UART2
    init_uart(UART_GSM, GSM_TX_PIN, GSM_RX_PIN, GSM_BAUD_RATE); // GSM on UART1
    init_gsm();
    //send_sms("hi i m krishna","+919689343153");
    //send_sms("tesing msg","+919689343153");
    xTaskCreate(RUNNIG_task, "SOS task", 4096, NULL, 1, NULL);
    xTaskCreate(PLAY_VOICE, "VOICE task", 1024, NULL, 1, NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}