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
uint16_t TRGBSuppport::getBootCount() { return bootCount; }


static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (touch_pin_get_int) {
    uint8_t touch_points_num;
    uint16_t x, y;
    ft3267_read_pos(&touch_points_num, &x, &y);
    if (touch_points_num > 0) {
    	data->point.x = x;
    	data->point.y = y;
    	data->state = LV_INDEV_STATE_PRESSED;
    } else {
    	data->state =  LV_INDEV_STATE_RELEASED;
    }
    touch_pin_get_int = false;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
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

esp_lcd_panel_handle_t TRGBSuppport::register_tft() {
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_rgb_panel_config_t panel_config = { .clk_src = LCD_CLK_SRC_PLL160M,
			.timings = { .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ, .h_res =
					EXAMPLE_LCD_H_RES,
					.v_res = EXAMPLE_LCD_V_RES, // The following parameters should refer to LCD spec
					.hsync_pulse_width = 1, .hsync_back_porch = 30,
					.hsync_front_porch = 50, .vsync_pulse_width = 1,
					.vsync_back_porch = 30, .vsync_front_porch = 20, .flags = {
							.pclk_active_neg = 1 } },
			.data_width = 16, // RGB565 in parallel mode, thus 16bit in width
			.psram_trans_align = 64, .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
			.vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC, .de_gpio_num =
					EXAMPLE_PIN_NUM_DE, .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
			.data_gpio_nums = { // EXAMPLE_PIN_NUM_DATA0,
					EXAMPLE_PIN_NUM_DATA13, EXAMPLE_PIN_NUM_DATA14,
							EXAMPLE_PIN_NUM_DATA15, EXAMPLE_PIN_NUM_DATA16,
							EXAMPLE_PIN_NUM_DATA17, EXAMPLE_PIN_NUM_DATA6,
							EXAMPLE_PIN_NUM_DATA7, EXAMPLE_PIN_NUM_DATA8,
							EXAMPLE_PIN_NUM_DATA9, EXAMPLE_PIN_NUM_DATA10,
							EXAMPLE_PIN_NUM_DATA11, // EXAMPLE_PIN_NUM_DATA12,
							EXAMPLE_PIN_NUM_DATA1, EXAMPLE_PIN_NUM_DATA2,
							EXAMPLE_PIN_NUM_DATA3, EXAMPLE_PIN_NUM_DATA4,
							EXAMPLE_PIN_NUM_DATA5 }, .disp_gpio_num =
					EXAMPLE_PIN_NUM_DISP_EN, .on_frame_trans_done = NULL,
			.user_ctx = NULL, .flags = { .fb_in_psram = 1 } };
	// allocate frame buffer in PSRAM
	ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	return panel_handle;
}

void TRGBSuppport::init() {
	Serial.begin(115200);
	Serial.setTxTimeoutMs(1);	// workaround to minimize blocking time for output to HWCDCSerial (Serial), if no host is connected.
	Serial.print("Init T-RGB device. Bootcount:");
	Serial.println(getBootCount());
	Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t) 400000);
	xl.begin();
	uint8_t pin = (1 << PWR_EN_PIN)  | (1 << LCD_CS_PIN)  | (1 << TP_RES_PIN)
			    | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) | (1 << LCD_RST_PIN)
			    | (1 << SD_CS_PIN);

	xl.pinMode8(0, pin, OUTPUT);
	xl.digitalWrite(PWR_EN_PIN, 1);

	// Enable CS for SD card
	xl.digitalWrite(SD_CS_PIN, 1); // To use SDIO one-line mode, you need to pull the CS pin high

	tft_init();
	esp_lcd_panel_handle_t panel_handle = register_tft();

	// Draw a start logo (before init of LVGL)
	esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 480, logo_img);

	pinMode(TP_INT_PIN, INPUT_PULLUP);
	attachInterrupt(TP_INT_PIN, [] { touch_pin_get_int = true; }, FALLING);
    pinMode(BAT_VOLT_PIN, ANALOG);

	lv_init();
	// alloc draw buffers used by LVGL from PSRAM
	lv_color_t *buf1 = (lv_color_t*) heap_caps_malloc(
			EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	assert(buf1);
	lv_color_t *buf2 = (lv_color_t*) heap_caps_malloc(
			EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	assert(buf2);
	lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);

	Serial.println("Register display driver to LVGL");
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = EXAMPLE_LCD_H_RES;
	disp_drv.ver_res = EXAMPLE_LCD_V_RES;
	disp_drv.flush_cb = lvgl_flush_cb;
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
  if (SD_MMC.cardSize()) SD_MMC.end();

  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL);

  Serial.println("Enter deep sleep");
  delay(2000);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_INT_PIN, 0);		// Wakeup by touch
  pinMode(GPIO_NUM_0, INPUT_PULLUP);
  //esp_sleep_enable_ext0_wakeup((gpio_num_t) GPIO_NUM_0, 0);	// Wakeup by boot-pin (TODO: not really useful for now, because display is not initialized correctly)
  esp_deep_sleep_start();
}

void TRGBSuppport::restart(void) {
	  WiFi.disconnect();
	  if (SD_MMC.cardSize()) SD_MMC.end();
	  Serial.println("Restart");
	  delay(500);
	  esp_restart();
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

  // Reset the display and touch

//  xl.digitalWrite(LCD_RST_PIN, 1);
//  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 0);
  xl.digitalWrite(TP_RES_PIN, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 1);
  xl.digitalWrite(TP_RES_PIN, 1);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  ft3267_init(Wire);

  // Switch on backlight
  pinMode(EXAMPLE_PIN_NUM_BK_LIGHT, OUTPUT);
  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

  int cmd = 0;
  while (st_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80) {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    cmd++;
  }
  Serial.println("Register setup complete");
  // Switch on backlight
  pinMode(EXAMPLE_PIN_NUM_BK_LIGHT, OUTPUT);
  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

}


void TRGBSuppport::SD_init(void) {
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true, true, BOARD_MAX_SDMMC_FREQ, 10)) { // max 10 open files (need more than the default 5 due to logging, replay, webserver etc.)
    Serial.println("Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");

  if (cardType == CARD_MMC)
    Serial.println("MMC");
  else if (cardType == CARD_SD)
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC)
    Serial.println("SDHC");
  else
    Serial.println("UNKNOWN");

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}


void TRGBSuppport::scan_iic(void) {
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error != 2) {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  }
}

void TRGBSuppport::print_chip_info(void) {
  Serial.print("Chip: ");
  Serial.println(ESP.getChipModel());
  Serial.print("ChipRevision: ");
  Serial.println(ESP.getChipRevision());
  Serial.print("Psram size: ");
  Serial.print(ESP.getPsramSize() / 1024);
  Serial.println("KB");
  Serial.print("Flash size: ");
  Serial.print(ESP.getFlashChipSize() / 1024);
  Serial.println("KB");
  Serial.print("CPU frequency: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println("MHz");
}
