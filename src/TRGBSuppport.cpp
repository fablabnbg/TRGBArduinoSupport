/*
 * TRGBSuppport.cpp
 *
 *  Created on: 29.12.2022
 *      Author: ian
 *
 *      Code is copied from official examples (https://github.com/Xinyuan-LilyGO/T-RGB/tree/main/example/factory).
 *      No logic changes, but restructured for easier use.
 *
 *      License: MIT
 */

#include <TRGBSuppport.h>

#include "img.h"  //unsern Logo

// WiFi include needed for deep sleep support.
#include "WiFi.h"


static bool touch_pin_get_int=false;

static RTC_DATA_ATTR uint16_t bootCount = 0;

static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (touch_pin_get_int) {
    uint8_t touch_points_num;
    uint16_t x, y;
    ft3267_read_pos(&touch_points_num, &x, &y);
    data->point.x = x;
    data->point.y = y;
    data->state = (touch_points_num > 0) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    touch_pin_get_int = false;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}


TRGBSuppport::TRGBSuppport() {
	bootCount++;
}



void TRGBSuppport::init() {
	Serial.print("Boot count: ");
	Serial.println(bootCount);
	//Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t) 400000);
	xl.begin();
	uint8_t pin = (1 << PWR_EN_PIN)  | (1 << LCD_CS_PIN)  | (1 << TP_RES_PIN)
			    | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) | (1 << LCD_RST_PIN)
			    | (1 << SD_CS_PIN);

	xl.pinMode8(0, pin, OUTPUT);
	xl.digitalWrite(PWR_EN_PIN, 1);

	// Enable CS for SD card
	xl.digitalWrite(SD_CS_PIN, 1); // To use SDIO one-line mode, you need to pull the CS pin high


	// Reset and init Touch devuce
	xl.digitalWrite(TP_RES_PIN, 0);
	delay(200);
	xl.digitalWrite(TP_RES_PIN, 1);
	delay(200);
	ft3267_init(Wire);

	pinMode(TP_INT_PIN, INPUT_PULLUP);
	attachInterrupt(TP_INT_PIN, [] { touch_pin_get_int = true; }, FALLING);

	tft_init();
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_rgb_panel_config_t panel_config = {
	      .clk_src = LCD_CLK_SRC_PLL160M,
	      .timings =
	          {
	              .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
	              .h_res = EXAMPLE_LCD_H_RES,
	              .v_res = EXAMPLE_LCD_V_RES,
	              // The following parameters should refer to LCD spec
	              .hsync_pulse_width = 1,
	              .hsync_back_porch = 30,
	              .hsync_front_porch = 50,
	              .vsync_pulse_width = 1,
	              .vsync_back_porch = 30,
	              .vsync_front_porch = 20,
	              .flags =
	                  {
	                      .pclk_active_neg = 1,
	                  },
	          },
	      .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
	      .psram_trans_align = 64,
	      .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
	      .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
	      .de_gpio_num = EXAMPLE_PIN_NUM_DE,
	      .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
	      .data_gpio_nums =
	          {
	              // EXAMPLE_PIN_NUM_DATA0,
	              EXAMPLE_PIN_NUM_DATA13,
	              EXAMPLE_PIN_NUM_DATA14,
	              EXAMPLE_PIN_NUM_DATA15,
	              EXAMPLE_PIN_NUM_DATA16,
	              EXAMPLE_PIN_NUM_DATA17,

	              EXAMPLE_PIN_NUM_DATA6,
	              EXAMPLE_PIN_NUM_DATA7,
	              EXAMPLE_PIN_NUM_DATA8,
	              EXAMPLE_PIN_NUM_DATA9,
	              EXAMPLE_PIN_NUM_DATA10,
	              EXAMPLE_PIN_NUM_DATA11,
	              // EXAMPLE_PIN_NUM_DATA12,

	              EXAMPLE_PIN_NUM_DATA1,
	              EXAMPLE_PIN_NUM_DATA2,
	              EXAMPLE_PIN_NUM_DATA3,
	              EXAMPLE_PIN_NUM_DATA4,
	              EXAMPLE_PIN_NUM_DATA5,
	          },
	      .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
	      .on_frame_trans_done = NULL,
	      .user_ctx = NULL,
	      .flags =
	          {
	              .fb_in_psram = 1, // allocate frame buffer in PSRAM
	          },
	  };
	  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
	  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

	  // Draw a start logo (before init of LVGL)
	  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 480, logo_img);

	  lv_init();
	  // alloc draw buffers used by LVGL from PSRAM
	  lv_color_t *buf1 =
	      (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	  assert(buf1);
	  lv_color_t *buf2 =
	      (lv_color_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	  assert(buf2);
	  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);

	  Serial.println("Register display driver to LVGL");
	  lv_disp_drv_init(&disp_drv);
	  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
	  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
	  disp_drv.flush_cb = example_lvgl_flush_cb;
	  disp_drv.draw_buf = &disp_buf;
	  disp_drv.user_data = panel_handle;
	  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

	  lv_indev_drv_init(&indev_drv);
	  indev_drv.type = LV_INDEV_TYPE_POINTER;
	  indev_drv.read_cb = lv_touchpad_read;
	  lv_indev_drv_register(&indev_drv);
}


void TRGBSuppport::deepSleep(void) {
  WiFi.disconnect();
  detachInterrupt(TP_INT_PIN);
  xl.pinMode8(0, 0xff, INPUT);
  xl.pinMode8(1, 0xff, INPUT);
  xl.read_all_reg();
  // If the SD card is initialized, it needs to be unmounted.
  if (SD_MMC.cardSize())
    SD_MMC.end();

  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

  Serial.println("Enter deep sleep");
  delay(1000);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_INT_PIN, 0);
  esp_deep_sleep_start();
}


void TRGBSuppport::lcd_send_data(uint8_t data) {
  uint8_t n;
  for (n = 0; n < 8; n++) {
    if (data & 0x80)
      xl.digitalWrite(LCD_SDA_PIN, 1);
    else
      xl.digitalWrite(LCD_SDA_PIN, 0);

    data <<= 1;
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
  }
}

void TRGBSuppport::lcd_cmd(const uint8_t cmd) {
  xl.digitalWrite(LCD_CS_PIN, 0);
  xl.digitalWrite(LCD_SDA_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 1);
  lcd_send_data(cmd);
  xl.digitalWrite(LCD_CS_PIN, 1);
}

void TRGBSuppport::lcd_data(const uint8_t *data, int len) {
  uint32_t i = 0;
  if (len == 0)
    return; // no need to send anything
  do {
    xl.digitalWrite(LCD_CS_PIN, 0);
    xl.digitalWrite(LCD_SDA_PIN, 1);
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(*(data + i));
    xl.digitalWrite(LCD_CS_PIN, 1);
    i++;
  } while (len--);
}

void TRGBSuppport::tft_init(void) {
  xl.digitalWrite(LCD_CS_PIN, 1);
  xl.digitalWrite(LCD_SDA_PIN, 1);
  xl.digitalWrite(LCD_CLK_PIN, 1);

  // Reset the display
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  int cmd = 0;
  while (st_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    cmd++;
  }
  Serial.println("Register setup complete");
}
