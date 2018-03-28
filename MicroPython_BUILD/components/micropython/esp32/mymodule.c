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
#include "esp_wifi.h"

#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "nvs.h"


#include "nvs_flash.h"
#include "nvs.h"

#define GPIO_OUTPUT_IO_0 22

int do_record = 1;
int is_recording = 0;
int threaded_task = 1;


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
   is_recording = 1;
   gpio_pad_select_gpio(GPIO_OUTPUT_IO_0);
   gpio_set_direction(GPIO_OUTPUT_IO_0, GPIO_MODE_OUTPUT);
   gpio_set_level(GPIO_OUTPUT_IO_0, 0);
   uint16_t buf_len = 1024;
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
         bytes_read = i2s_read_bytes(I2S_NUM_0, buf, buf_len, 0);
      }
      uint32_t samples_read = bytes_read / 2 / (I2S_BITS_PER_SAMPLE_32BIT / 8);
      for(int i = 0; i < samples_read; i++) {
         // const char samp32[4] = {ptr_l[0], ptr_l[1], ptr_r[0], ptr_r[1]};

         // left
         buf_ptr_write[0] = buf_ptr_read[2]; // mid
         buf_ptr_write[1] = buf_ptr_read[3]; // high

         buf_ptr_write += (I2S_BITS_PER_SAMPLE_16BIT / 8);
         buf_ptr_read += 2 * (I2S_BITS_PER_SAMPLE_32BIT / 8);
      }

      // local echo
      bytes_2_write = samples_read * (I2S_BITS_PER_SAMPLE_16BIT / 8);

      cnt += samples_read;

	  bytes_written  = fwrite(buf , sizeof(char), bytes_2_write , f );




      if(cnt >= 44100) {
   	     esp_task_wdt_reset();
		 sec+=1;
         gettimeofday(&tv, &tz);
         micros = tv.tv_usec + tv.tv_sec * 1000000;
         delta = micros - micros_prev;
         micros_prev = micros;
         printf("%d samples in %" PRIu64 " usecs\n", cnt, delta);
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

STATIC mp_obj_t mymodule_wifistop(void) {
	esp_wifi_stop();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_wifistop_obj, mymodule_wifistop);

//


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

STATIC mp_obj_t mymodule_isrecording(void) {
	return MP_OBJ_NEW_SMALL_INT(is_recording);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mymodule_isrecording_obj, mymodule_isrecording);


STATIC const mp_map_elem_t mymodule_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_mymodule) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_record), (mp_obj_t)&mymodule_record_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&mymodule_stop_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_start), (mp_obj_t)&mymodule_start_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_initi2s), (mp_obj_t)&mymodule_initi2s_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_isrecording), (mp_obj_t)&mymodule_isrecording_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_wifistop), (mp_obj_t)&mymodule_wifistop_obj },
};


STATIC MP_DEFINE_CONST_DICT (
    mp_module_mymodule_globals,
    mymodule_globals_table
);

const mp_obj_module_t mp_module_mymodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_mymodule_globals,
};
