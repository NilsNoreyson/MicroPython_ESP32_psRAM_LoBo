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
    /*
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
    */
    /*print ram info*/
}

void play_mp3_tread(char*filename){
	printf(filename);
    char file_name[128]; // The filename buffer.

    char buffer[128]; // The filename buffer.
    // Put "file" then k then ".txt" in to filename.
    snprintf(buffer, sizeof(char) * 64, "/_#!#_sdcard/%s", filename);
    aplay_mp3(buffer);
	//aplay_mp3(filename);
    PLAY_AUDIO=0;
	vTaskDelete(NULL);
}

void play_music(char *filename){

		printf("Starting play-tread.");

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


STATIC mp_obj_t musichead_playstate(void) {
	return MP_OBJ_NEW_SMALL_INT(PLAY_AUDIO);
}
MP_DEFINE_CONST_FUN_OBJ_0(musichead_playstate_obj, musichead_playstate);

STATIC mp_obj_t musichead_play(mp_obj_t mp3_path) {

    const char *filename = mp_obj_str_get_str(mp3_path);


	printf("wassss\n");
	printf("%s",filename);

	play_music(filename);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_1(musichead_play_obj, musichead_play);

STATIC mp_obj_t musichead_vol(mp_obj_t vol) {
    int hpvol =  mp_obj_get_int(vol);
	WM8978_HPvol_Set(hpvol,hpvol);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(musichead_vol_obj, musichead_vol);


STATIC const mp_map_elem_t musichead_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_musichead) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&musichead_init_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_stop), (mp_obj_t)&musichead_stop_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_play), (mp_obj_t)&musichead_play_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_vol), (mp_obj_t)&musichead_vol_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_state), (mp_obj_t)&musichead_playstate_obj},
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_musichead_globals,
	musichead_globals_table
);

const mp_obj_module_t mp_module_musichead = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_musichead_globals,
};

