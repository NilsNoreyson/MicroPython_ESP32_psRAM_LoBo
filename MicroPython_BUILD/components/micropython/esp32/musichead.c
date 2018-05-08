#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
//#include "portmodules.h"

#include "wm8978.h"
#include "hal_i2c.h"
#include "hal_i2s.h"
#include "audio.h"
#include "aplay.h"

//////////////////////

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
//#include "wm8978.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include <sys/socket.h>
#include "nvs.h"
#include "nvs_flash.h"
//#include "hal_i2c.h"
//#include "hal_i2s.h"
//#include "wm8978.h"
//#include "http.h"
//#include "audio.h"
//#include <dirent.h>
//#include "esp_heap_caps.h"
//#include "esp_heap_caps.h"
//#include "aplay.h"

#define TAG "musichead:"

//int PLAY_AUDIO = 1;

#define GPIO_OUTPUT_IO_0    5
#define GPIO_OUTPUT_PIN_SEL  ((1<<GPIO_OUTPUT_IO_0))




void init_music()
{
    esp_err_t err;
    /*init gpio*/
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    /*init codec */
    hal_i2c_init(0,19,18);
    hal_i2s_init(0,48000,16,2);
    WM8978_Init();
    WM8978_ADDA_Cfg(1,1);
    WM8978_Input_Cfg(1,0,0);
    WM8978_Output_Cfg(1,0);
    WM8978_MIC_Gain(25);
    WM8978_AUX_Gain(0);
    WM8978_LINEIN_Gain(0);
    WM8978_SPKvol_Set(0);
    WM8978_HPvol_Set(50,50);
    WM8978_EQ_3D_Dir(0);
    WM8978_EQ1_Set(0,12);
    WM8978_EQ2_Set(0,12);
    WM8978_EQ3_Set(0,12);
    WM8978_EQ4_Set(0,12);
    WM8978_EQ5_Set(0,12);
    /*init sd card*/
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 10
    };
    sdmmc_card_t* card;
    err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            printf("Failed to mount filesystem. If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            printf("Failed to initialize the card (%d). Make sure SD card lines have pull-up resistors in place.", err);
        }
        return;
    }
    sdmmc_card_print_info(stdout, card);
    /*print ram info*/
    size_t free8start=heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t free32start=heap_caps_get_free_size(MALLOC_CAP_32BIT);
    ESP_LOGI(TAG,"free mem8bit: %d mem32bit: %d\n",free8start,free32start);
}

void play_mp3_tread(char*filename){
	aplay_mp3(filename);
	vTaskDelete(NULL);
}

void play_music(char *filename){

		printf(filename);

		PLAY_AUDIO=1;
		//filename = "/sdcard/music.mp3";
		TaskHandle_t xHandle = NULL;
		xTaskCreatePinnedToCore(&play_mp3_tread, "play_audio", 16384, filename, 20, &xHandle,0);
		//aplay_mp3("/sdcard/music.mp3");

}

void stop_music(){

	PLAY_AUDIO=0;

}



STATIC mp_obj_t musichead_init(void) {
	init_music();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(musichead_init_obj, musichead_init);


STATIC mp_obj_t musichead_stop(void) {
	stop_music();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(musichead_stop_obj, musichead_stop);

STATIC mp_obj_t musichead_play(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args, uint8_t type) {
	enum {ARG_filename};
	const mp_arg_t allowed_args[] = {
        { MP_QSTR_filename,  	MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
	};

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    char *filename = NULL;

	filename = (char *)mp_obj_str_get_str(args[ARG_filename].u_obj);

	printf('wassss');
	printf(filename);

	play_music(*filename);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_KW(musichead_play_obj,1, musichead_play);


STATIC const mp_map_elem_t musichead_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_musichead) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&musichead_init_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&musichead_stop_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_play), (mp_obj_t)&musichead_play_obj},
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_musichead_globals,
	musichead_globals_table
);

const mp_obj_module_t mp_module_musichead = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_musichead_globals,
};

