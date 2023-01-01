#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

extern lv_obj_t * ui_S1Main;
extern lv_obj_t * ui_SWifiLabelIPAddr;
extern lv_obj_t * ui_S1LabelClock;

//void ui_event_ScreenWifi_Slider1(lv_event_t * e);
//extern lv_obj_t * ui_ScreenWifi_Slider1;
//extern lv_obj_t * ui_ScreenWifi_Label8;
void ui_event_SWifiButtonStandby(lv_event_t * e);
extern lv_obj_t * ui_SWifiButtonStandby;
extern lv_obj_t * ui_ScreenWifiButtonLabelStandby;
void ui_event_ScreenChart(lv_event_t * e);
extern lv_obj_t * ui_ScreenChart_Chart1;

void ui_ev_bright(lv_event_t * e);
void ui_ev_standby(lv_event_t * e);

void chart_init();
void chart_add_voltage(const float v);

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif
