#include <stdio.h>
// #include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_spi_flash.h"
#include "led_strip.h"
#include "driver/rmt.h"
static led_strip_t *strip;
static led_strip_t *strip2;

#define LIGHTNESS_MAX 255
#define LIGHT_INTERVAL 2
uint8_t get_reverse_value(uint8_t val)
{
    return (0xFF - val);
}

#define LED_NUM 19

void app_main(void)
{

    /* Print chip information */
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
    //         CONFIG_IDF_TARGET,
    //         chip_info.cores,
    //         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
    //         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    // printf("silicon revision %d, ", chip_info.revision);

    // printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
    //         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    // WS2812 Neopixel driver with RMT peripheral
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(8, RMT_CHANNEL_0);
    config.clk_div = 2; // set counter clock to 40MHz

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_NUM, (led_strip_dev_t) config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    strip->clear(strip, 100); // off led

    rmt_config_t config2 = RMT_DEFAULT_CONFIG_TX(9, RMT_CHANNEL_1);
    config2.clk_div = 2; // set counter clock to 40MHz

    rmt_config(&config2);
    rmt_driver_install(config2.channel, 0, 0);

    led_strip_config_t strip_config2 = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t) config2.channel);
    strip2 = led_strip_new_rmt_ws2812(&strip_config2);
    strip2->clear(strip2, 100); // off led

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // for (int i = 10; i > 0; i--) {
    //     printf("Hello world!\n");
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    while(1)
    {
        static uint8_t reverse_flag = 0;
        static int16_t lightness = 0;
        static uint8_t state_cnt = 1;

        static uint8_t r = 0;
        static uint8_t g = 0;
        static uint8_t b = 0;
        if(reverse_flag == 0)
        {
            lightness += LIGHT_INTERVAL;
            if(lightness > LIGHTNESS_MAX)
            {
                lightness = LIGHTNESS_MAX;
                reverse_flag = 1;
            }
        }
        else
        {
            lightness -= LIGHT_INTERVAL;
            if(lightness < 0)
            {
                lightness = 0;
                reverse_flag = 0;
                state_cnt++;
                if(state_cnt > 7)
                {
                    state_cnt = 1;
                }
            }
        }
        r=0;
        g=0;
        b=0;
        if(state_cnt&0x04)
        {
            r = lightness;
        }
        if(state_cnt&0x02)
        {
            g = lightness;
        }
        if(state_cnt&0x01)
        {
            b = lightness;
        }
        for(uint8_t i=0;i<LED_NUM;i++)
        {
            strip->set_pixel(strip, i, r, g, b);
        }

        strip->refresh(strip, 100);

        strip2->set_pixel(strip2, 0, lightness, 0, 0);
        // strip->set_pixel(strip, 1, get_reverse_value(lightness), get_reverse_value(lightness), get_reverse_value(lightness));
        strip2->refresh(strip2, 100);


        printf("set rgb state_cnt:%d, lightness:%d...\n", state_cnt, lightness);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}
