#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "py/mphal.h"
//#include "portmodules.h"

#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "esp_sleep.h"
#include "extmod/vfs_native.h"




#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include <stdio.h>
#include "esp_wifi.h"

#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"


#include "nvs_flash.h"
#include "nvs.h"



//espnow stuff
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"
#include "espnow_example.h"


#define GPIO_OUTPUT_IO_0 22
//#define TAG "main"
//#define TAG "main"

int do_record = 1;
int is_recording = 0;
int threaded_task = 1;
i2s_bits_per_sample_t bits_per_sample = 32;
int sample_rate = 44100;





static const char *TAG = "espnow_example";
int CONFIG_ESPNOW_CHANNEL = 1;
int CONFIG_ESPNOW_SEND_COUNT = 100;
int CONFIG_ESPNOW_SEND_DELAY = 1000;
int CONFIG_ESPNOW_SEND_LEN= 200;
char* CONFIG_ESPNOW_PMK="pmk1234567890123";
char* CONFIG_ESPNOW_LMK="lmk1234567890123";



static xQueueHandle example_espnow_queue;

static uint8_t example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

static void example_espnow_deinit(example_espnow_send_param_t *send_param);


static esp_err_t example_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(example_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

    /* In order to simplify example, channel is set after WiFi started.
     * This is not necessary in real application if the two devices have
     * been already on the same channel.
     */
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void example_espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    example_espnow_event_t evt;
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
//    printf("recieved data\n");
    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(example_espnow_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

/* Parse received ESPNOW data. */
int example_espnow_data_parse(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(example_espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

/* Prepare ESPNOW data to be sent. */
void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
{
    example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;
    int i = 0;

    assert(send_param->len >= sizeof(example_espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = s_example_espnow_seq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    for (i = 0; i < send_param->len - sizeof(example_espnow_data_t); i++) {
        buf->payload[i] = (uint8_t)esp_random();
    }
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

static void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t evt;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    example_espnow_send_param_t *send_param = (example_espnow_send_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error");
        example_espnow_deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
        switch (evt.id) {
            case EXAMPLE_ESPNOW_SEND_CB:
            {
                example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_addr);

                ESP_LOGD(TAG, "Send data to "MACSTR", status1: %d", MAC2STR(send_cb->mac_addr), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        example_espnow_deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));

                memcpy(send_param->dest_mac, send_cb->mac_addr, ESP_NOW_ETH_ALEN);
                example_espnow_data_prepare(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error");
                    example_espnow_deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case EXAMPLE_ESPNOW_RECV_CB:
            {

                example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

                ret = example_espnow_data_parse(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);

                free(recv_cb->data);
                if (ret == EXAMPLE_ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    printf("Recieved data \n");


                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_addr) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            example_espnow_deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = CONFIG_ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(recv_cb->mac_addr));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_addr, ESP_NOW_ETH_ALEN);
                            example_espnow_data_prepare(send_param);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error");
                                example_espnow_deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == EXAMPLE_ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_addr), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", evt.id);
                break;
        }
    }
}

static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
    if (example_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vSemaphoreDelete(example_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memcpy(send_param->dest_mac, example_broadcast_mac, ESP_NOW_ETH_ALEN);
    example_espnow_data_prepare(send_param);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, send_param, 4, NULL);

    return ESP_OK;
}

static void example_espnow_deinit(example_espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(example_espnow_queue);
    esp_now_deinit();
}

void esp_now_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    example_wifi_init();
    example_espnow_init();
    printf("ESPnow init done\n");
}



//ulp stuff
#include <stdio.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "main/ulp_main.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program();

void count_pulses()
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        printf("Not ULP wakeup, initializing ULP\n");
        //init_ulp_program();
    } else {
        printf("ULP wakeup, saving pulse count\n");
    }



    uint32_t pulse_count_from_ulp_old = -1;
    while(1) {
	    uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
	    uint32_t touch_counter = ulp_touch_counter;
	    /* In case of an odd number of edges, keep one until next time */
	    printf("Touch from ULP: %5d\n", touch_counter);

            if (pulse_count_from_ulp_old == pulse_count_from_ulp) {
	        ulp_edge_count = ulp_edge_count % 2;

	        printf("Pulse count from ULP: %5d\n", pulse_count_from_ulp);

		break;



	    }
	    else {
            pulse_count_from_ulp_old = pulse_count_from_ulp;
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
	//printf("Entering deep sleep\n\n");
	//ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
	//esp_deep_sleep_start();

}

static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_next_edge = 0;
    ulp_io_number = 11; /* GPIO0 is RTC_IO 11 */
    ulp_edge_count_to_wake_up = 2;

    /* Initialize GPIO0 as RTC IO, input, disable pullup and pulldown */
    gpio_num_t gpio_num = GPIO_NUM_0;
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO15 may be connected to ground to suppress boot messages.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);

    /* Set ULP wake up period to T = 20ms (3095 cycles of RTC_SLOW_CLK clock).
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
    //REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 3095);
    REG_SET_FIELD(SENS_ULP_CP_SLEEP_CYC0_REG, SENS_SLEEP_CYCLES_S0, 1000);

    /* Start the program */
    err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);
}





//recording stuff

int32_t get_and_increment_rec_count()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading rec-counter from NVS ... ");
        int32_t rec_counter = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(my_handle, "rec_counter", &rec_counter);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("Rec counter = %d\n", rec_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        printf("Updating rec counter in NVS ... ");
        rec_counter++;
        err = nvs_set_i32(my_handle, "rec_counter", rec_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
        return rec_counter;
    }
    return 0;
}

static void enter_ulp_deepsleep()
{
	printf("Entering deep sleep\n\n");
	ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
	esp_deep_sleep_start();
}


static void init_i2s()
{
	int i2s_num = 1;
	int bck = 26;
	int ws = 25;
	int data_in = 23;

    /* RX: I2S_NUM_1 */
    i2s_config_t i2s_config_rx = {
   .mode = I2S_MODE_MASTER | I2S_MODE_RX, // Only TX
   .sample_rate = sample_rate,
   .bits_per_sample = bits_per_sample,    // Only 8-bit DAC support
   .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // 2-channels
   .communication_format = I2S_COMM_FORMAT_I2S_MSB,
   .dma_buf_count = 64,                            // number of buffers, 128 max.
   .dma_buf_len = 32 * 2,                          // size of each buffer
   .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1        // Interrupt level 1
   };

   i2s_pin_config_t pin_config_rx = {
      .bck_io_num = bck,
      .ws_io_num = ws,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = data_in
   };

   i2s_driver_install(i2s_num, &i2s_config_rx, 0, NULL);
   i2s_set_pin(i2s_num, &pin_config_rx);

}


void set_do_recod(int state){
	do_record=state;
}

void task_record()
{
   is_recording = 1;
   gpio_pad_select_gpio(GPIO_OUTPUT_IO_0);
   gpio_set_direction(GPIO_OUTPUT_IO_0, GPIO_MODE_OUTPUT);
   gpio_set_level(GPIO_OUTPUT_IO_0, 0);
   uint16_t buf_len = 2048;
   char *buf = calloc(buf_len, sizeof(char));

   struct timeval tv = {0};
   struct timezone *tz = {0};
   gettimeofday(&tv, &tz);
   uint64_t micros = tv.tv_usec + tv.tv_sec * 1000000;
   uint32_t secs = (uint32_t)(tv.tv_sec);
   uint64_t micros_prev = micros;
   uint64_t delta = 0;

   int cnt = 0;
   int sec = 0;
   int bytes_2_write = 0;
   int bytes_written = 0;

   int file_count = 0;
   int inc_filename = 1;

    // Check if destination file exists before renaming
    struct stat st;

    char file_name[32]; // The filename buffer.
    int32_t rec_counter = get_and_increment_rec_count();
    printf("%d is new rec count\n", rec_counter);


    char buffer[32]; // The filename buffer.
    // Put "file" then k then ".txt" in to filename.
    snprintf(buffer, sizeof(char) * 64, "/_#!#_sdcard/rec_%i_%i.raw", rec_counter,secs);
    printf(buffer);
    printf("\n");

   FILE* f = fopen(buffer, "wb");
   while (f==NULL){
	   f = fopen(buffer, "wb");
	   vTaskDelay(100 / portTICK_RATE_MS);
   }
   if (f == NULL) {
       ESP_LOGE(TAG, "Failed to open file for writing");
       ESP_LOGE(TAG,VFS_NATIVE_SDCARD_MOUNT_POINT);
       printf('SD not mounted');
       printf(VFS_NATIVE_SDCARD_MOUNT_POINT);

       fclose(f);
       gpio_set_level(GPIO_OUTPUT_IO_0, 1);
       do_record=1;
       if (threaded_task==1) {
    	   vTaskDelete(NULL);
       }
       return;

   }

   while((sec<58) && (do_record==1))
   {
      char *buf_ptr_read = buf;
      char *buf_ptr_write = buf;

      // read whole block of samples
      int bytes_read = 0;
      while(bytes_read == 0) {
         bytes_read = i2s_read_bytes(I2S_NUM_0, buf, buf_len, 2048);
      }
      uint32_t samples_read = bytes_read / 2 / (bits_per_sample / 8);
      for(int i = 0; i < samples_read; i++) {
         // const char samp32[4] = {ptr_l[0], ptr_l[1], ptr_r[0], ptr_r[1]};

         // left
         buf_ptr_write[0] = buf_ptr_read[2]; // mid
         buf_ptr_write[1] = buf_ptr_read[3]; // high

         buf_ptr_write += (I2S_BITS_PER_SAMPLE_16BIT / 8);
         buf_ptr_read += 2 * (bits_per_sample / 8);
      }

      // local echo
      bytes_2_write = samples_read * (I2S_BITS_PER_SAMPLE_16BIT / 8);

      cnt += samples_read;
	  bytes_written  = fwrite(buf , sizeof(char), bytes_2_write , f );




      if(cnt >= sample_rate) {
   	     esp_task_wdt_reset();
		 sec+=1;
         gettimeofday(&tv, &tz);
         micros = tv.tv_usec + tv.tv_sec * 1000000;
         delta = micros - micros_prev;
         micros_prev = micros;
         printf("%d samples in %" PRIu64 " usecs - last %d bytes written, last read %d\n", cnt, delta,bytes_2_write,bytes_read);
         cnt = 0;
      }
   }
   fclose(f);



   gpio_set_level(GPIO_OUTPUT_IO_0, 1);
   do_record=1;
   is_recording = 0;
   if (threaded_task==1) {
	   vTaskDelete(NULL);
   }
   return;



}





STATIC mp_obj_t mymodule_record(void) {



    printf("starting record thread\n");
    do_record=1;
    TaskHandle_t xHandle = NULL;
    if (threaded_task==1) {
    	if (is_recording == 0){
    		xTaskCreatePinnedToCore(&task_record, "task_record", 16384, NULL, 20, &xHandle,0);
    		//xTaskCreatePinnedToCore(&task_record, "task_record", 16384, NULL, configMAX_PRIORITIES-2, &xHandle,0);

    		//configASSERT( xHandle );
    	}
    }
    else{
    	task_record();
    }


    printf("Done\n");

   return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_record_obj, mymodule_record);

/*
 * https://www.eevblog.com/forum/microcontrollers/esp32-deep-sleep/
 * sleep deeper
 * 		ESP_LOGI(TAG, "Awake\n");
		WRITE_PERI_REG(RTC_CNTL_PWC_REG, 0x00120240);
		ESP_LOGI(TAG, "RTC_CNTL_OPTIONS0_REG       0x%08x", READ_PERI_REG(RTC_CNTL_OPTIONS0_REG));
		ESP_LOGI(TAG, "RTC_CNTL_ANA_CONF_REG       0x%08x", READ_PERI_REG(RTC_CNTL_ANA_CONF_REG));
		ESP_LOGI(TAG, "RTC_CNTL_PWC_REG            0x%08x", READ_PERI_REG(RTC_CNTL_PWC_REG));
		ESP_LOGI(TAG, "RTC_CNTL_DIG_PWC_REG        0x%08x", READ_PERI_REG(RTC_CNTL_DIG_PWC_REG));
		sleep(2);
		ESP_LOGI(TAG, "Entering deep sleep\n");
		sleep(1);
		esp_deep_sleep(15*MHZ);
		ESP_LOGI(TAG, "Sleep failed\n");
		sleep(1);
 */



//

//wifi stuff
STATIC mp_obj_t mymodule_wifistop(void) {
	esp_wifi_stop();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_wifistop_obj, mymodule_wifistop);



STATIC mp_obj_t mymodule_countPulses(void) {
    count_pulses();
    return mp_const_none;

}
MP_DEFINE_CONST_FUN_OBJ_0(mymodule_countPulses_obj, mymodule_countPulses);

STATIC mp_obj_t mymodule_initULP(void) {
	init_ulp_program();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mymodule_initULP_obj, mymodule_initULP);



STATIC mp_obj_t mymodule_ULPDeepSleep(void) {
	enter_ulp_deepsleep();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mymodule_ULPDeepSleep_obj, mymodule_ULPDeepSleep);


STATIC mp_obj_t mymodule_stop(void) {
	set_do_recod(0);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_stop_obj, mymodule_stop);

STATIC mp_obj_t mymodule_start(void) {
	set_do_recod(1);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_start_obj, mymodule_start);

STATIC mp_obj_t mymodule_initi2s(void) {
	init_i2s();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_initi2s_obj, mymodule_initi2s);

STATIC mp_obj_t mymodule_deiniti2s(void) {
	i2s_driver_uninstall(I2S_NUM_0);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_deiniti2s_obj, mymodule_deiniti2s);

STATIC mp_obj_t mymodule_isrecording(void) {
	return MP_OBJ_NEW_SMALL_INT(is_recording);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_isrecording_obj, mymodule_isrecording);

STATIC mp_obj_t mymodule_setSamplerate(mp_obj_t set_sample_rate) {
    sample_rate =  mp_obj_get_int(set_sample_rate);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(mymodule_setSamplerate_obj, mymodule_setSamplerate);


STATIC mp_obj_t mymodule_espnow(void) {
	esp_now_main();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mymodule_espnow_obj, mymodule_espnow);




STATIC const mp_map_elem_t mymodule_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_mymodule) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_record), (mp_obj_t)&mymodule_record_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&mymodule_stop_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_start), (mp_obj_t)&mymodule_start_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_initi2s), (mp_obj_t)&mymodule_initi2s_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_deiniti2s), (mp_obj_t)&mymodule_deiniti2s_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_isrecording), (mp_obj_t)&mymodule_isrecording_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_wifistop), (mp_obj_t)&mymodule_wifistop_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_setSamplerate), (mp_obj_t)&mymodule_setSamplerate_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_countPulses), (mp_obj_t)&mymodule_countPulses_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_initULP), (mp_obj_t)&mymodule_initULP_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_ULPDeepSleep), (mp_obj_t)&mymodule_ULPDeepSleep_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_espnow), (mp_obj_t)&mymodule_espnow_obj},

};


STATIC MP_DEFINE_CONST_DICT (
    mp_module_mymodule_globals,
    mymodule_globals_table
);

const mp_obj_module_t mp_module_mymodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_mymodule_globals,
};
