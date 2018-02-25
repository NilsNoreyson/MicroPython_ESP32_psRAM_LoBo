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

#define TAG "main"


#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include <stdio.h>

#include "driver/gpio.h"


#define GPIO_OUTPUT_IO_0 27

int do_record = 1;
int threaded_task = 1;

static void init_i2s()
{
   const int sample_rate = 44100;


    /* RX: I2S_NUM_1 */
    i2s_config_t i2s_config_rx = {
   .mode = I2S_MODE_MASTER | I2S_MODE_RX, // Only TX
   .sample_rate = sample_rate,
   .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,    // Only 8-bit DAC support
   .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // 2-channels
   .communication_format = I2S_COMM_FORMAT_I2S_MSB,
   .dma_buf_count = 32,                            // number of buffers, 128 max.
   .dma_buf_len = 32 * 2,                          // size of each buffer
   .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1        // Interrupt level 1
   };

   i2s_pin_config_t pin_config_rx = {
      .bck_io_num = GPIO_NUM_26,
      .ws_io_num = GPIO_NUM_25,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = GPIO_NUM_23
   };

   i2s_driver_install(I2S_NUM_0, &i2s_config_rx, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config_rx);

}


void set_do_recod(int state){
	do_record=state;
}

void task_record()
{

	ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = flase,
        .max_files = 5
    };


    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%d). "
                "Make sure SD card lines have pull-up resistors in place.", ret);
        }
        return mp_const_none;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f_c = fopen("/sdcard/communicator.txt", "w");
    if (f_c == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return mp_const_none;
    }
    fprintf(f_c, "If you found this, please write to NilsNoreyso@gmail.com\n %s\n", card->cid.name);
    fclose(f_c);
    ESP_LOGI(TAG, "File written");

   gpio_pad_select_gpio(GPIO_OUTPUT_IO_0);
   gpio_set_direction(GPIO_OUTPUT_IO_0, GPIO_MODE_OUTPUT);
   gpio_set_level(GPIO_OUTPUT_IO_0, 1);
   uint16_t buf_len = 1024;
   //int *buf = calloc(buf_len, sizeof(int));
   char *buf = calloc(buf_len, sizeof(char));

   struct timeval tv = {0};
   struct timezone *tz = {0};
   gettimeofday(&tv, &tz);
   uint64_t micros = tv.tv_usec + tv.tv_sec * 1000000;
   uint64_t micros_prev = micros;
   uint64_t delta = 0;

   init_i2s();

   int cnt = 0;
   int sec = 0;
   int bytes_written = 0;

   int file_count = 0;
   int inc_filename = 1;

    // Check if destination file exists before renaming
    struct stat st;

    char file_name[32]; // The filename buffer.
    /*
    while (inc_filename==1){

        snprintf(file_name, sizeof(char) * 32, "/sdcard/audio_%i.raw", file_count);

	    if (stat(file_name, &st) == 0) {
		printf("%s exists \n",file_name);
		file_count += 1;
	    }
	    else{
		printf("Writing to %s \n",file_name);
		inc_filename=0;
		}
    }


   FILE* f = fopen(file_name, "wb");
   */
   FILE* f = fopen("/sdcard/rec.raw", "wb");

   if (f == NULL) {
       ESP_LOGE(TAG, "Failed to open file for writing");
       ESP_LOGE(TAG,VFS_NATIVE_SDCARD_MOUNT_POINT);
       printf('SD not mounted');
       printf(VFS_NATIVE_SDCARD_MOUNT_POINT);

       fclose(f);
       gpio_set_level(GPIO_OUTPUT_IO_0, 0);
       do_record=1;
       vTaskDelete(NULL);
       return;

   }

   while((sec<13) && (do_record==1))
   {
      char *buf_ptr_read = buf;
      char *buf_ptr_write = buf;

      // read whole block of samples
      int bytes_read = 0;
      while(bytes_read == 0) {
         bytes_read = i2s_read_bytes(I2S_NUM_0, buf, buf_len, 0);
      }

      uint32_t samples_read = bytes_read / 2 / (I2S_BITS_PER_SAMPLE_32BIT / 8);



      cnt += samples_read;

    //fprintf(f, "%.*s", bytes_read, buf);
//fprintf(f, "%s",buf);
//bytes_written  = fwrite(buf , sizeof(char), sizeof(buf) , f );
bytes_written  = fwrite(buf , sizeof(char), bytes_read , f );




      if(cnt >= 44100) {
		 //printf("%d\n",bytes_read);
		 //printf("%d\n",bytes_written);
		 sec+=1;
         gettimeofday(&tv, &tz);
         micros = tv.tv_usec + tv.tv_sec * 1000000;
         delta = micros - micros_prev;
         micros_prev = micros;
         printf("%d samples in %" PRIu64 " usecs\n", cnt, delta);
         //printf("Do record %d\n",do_record);

         cnt = 0;
      }
   }
   fclose(f);



   // All done, unmount partition and disable SDMMC or SPI peripheral
   esp_vfs_fat_sdmmc_unmount();
   vTaskDelay(100 / portTICK_RATE_MS);
   ESP_LOGI(TAG, "Card unmounted");
   gpio_set_level(GPIO_OUTPUT_IO_0, 0);
   do_record=1;
   if (threaded_task==1) {
	   vTaskDelete(NULL);
   }
   //return;



}





STATIC mp_obj_t mymodule_record(void) {



    printf("starting record thread\n");
    do_record=1;
    TaskHandle_t xHandle = NULL;
    if (threaded_task==1) {
    	xTaskCreatePinnedToCore(&task_record, "task_record", 16384, NULL, 25, &xHandle,0);
    	//xTaskCreate(&task_record, "task_record", 16384, NULL, 20, &xHandle);
    	configASSERT( xHandle );
    }
    else{
    	task_record();
    }


    //printf("Done\n");

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


STATIC const mp_map_elem_t mymodule_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_mymodule) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_hello), (mp_obj_t)&mymodule_record_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&mymodule_stop_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_start), (mp_obj_t)&mymodule_start_obj },
};


STATIC MP_DEFINE_CONST_DICT (
    mp_module_mymodule_globals,
    mymodule_globals_table
);

const mp_obj_module_t mp_module_mymodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_mymodule_globals,
};
