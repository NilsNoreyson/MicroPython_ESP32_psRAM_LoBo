#include "freertos/FreeRTOS.h"
#include "driver/i2s.h"
#include "hal_i2s.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"

void hal_i2s_init(uint8_t i2s_num,uint32_t rate,uint8_t bits,uint8_t ch)
{

	i2s_channel_fmt_t chanel;
	if(ch==2)
		chanel=I2S_CHANNEL_FMT_RIGHT_LEFT;
	else
		chanel=I2S_CHANNEL_FMT_ONLY_LEFT;
	/*
	i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX|I2S_MODE_DAC_BUILT_IN,
        .sample_rate = rate,
        .bits_per_sample = bits,                                              
        .channel_format = chanel,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.use_apll = false
    };
    */
	i2s_config_t i2s_config = {
		  //.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN, //build in dac
		 .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_PDM,
	     .sample_rate = rate,
	     .bits_per_sample = bits, /* the DAC module will only take the 8bits from MSB */
	     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	     //.communication_format = I2S_COMM_FORMAT_I2S_MSB, //internal dac
		 .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
	     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // default interrupt priority
	     .dma_buf_count = 32,
	     .dma_buf_len = 64,
	     .use_apll = false,
	};

    i2s_pin_config_t pin_config = {
        .bck_io_num = 33,
        .ws_io_num = 25,
        .data_out_num = 26,
        .data_in_num = 27                                                       //Not used
    };
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    //i2s_set_pin(i2s_num, &pin_config);
    i2s_set_pin(i2s_num, NULL); //for internal DAC, this will enable both of the internal channels
    //You can call i2s_set_dac_mode to set built-in DAC output mode.
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    i2s_set_sample_rates(i2s_num, rate);
    i2s_set_clk(i2s_num, rate, bits, ch);
    //clk out
    //PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1); //peter:don't know, came from esp32 snow
    //REG_SET_FIELD(PIN_CTRL, CLK_OUT1, 0)//peter:don't know, came from esp32 snow
    //REG_WRITE(PIN_CTRL, 0xFFFFFFF0);;//peter:don't know, came from esp32 snow
    //i2s_set_clk(i2s_num, rate, bits, ch);//peter:don't know, came from esp32 snow
}
int hal_i2s_read(uint8_t i2s_num,char* dest,size_t size,TickType_t timeout)
{
    return i2s_read_bytes(i2s_num,  dest, size, timeout);
}
int hal_i2s_write(uint8_t i2s_num,char* dest,size_t size,TickType_t timeout)
{
    return i2s_write_bytes(i2s_num,  dest, size, timeout);
}



