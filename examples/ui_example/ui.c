#include "ui.h"
//#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t * ui_S1Main;
lv_obj_t * ui_S1LabelTitle;
lv_obj_t * ui_S1LabelClock;
lv_obj_t * ui_SWifiLabelIPAddr;
void ui_event_SWifiButtonStandby(lv_event_t * e);
lv_obj_t * ui_SWifiButtonStandby;
lv_obj_t * ui_ScreenWifiButtonLabelStandby;
lv_obj_t * ui_ScreenChart_Chart1;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////

void ui_event_SWifiButtonStandby(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_LONG_PRESSED) {
        ui_ev_standby(e);
    }
}

///////////////////// SCREENS ////////////////////
void ui_S1Main_screen_init(void)
{
    ui_S1Main = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_S1Main, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_S1LabelTitle = lv_label_create(ui_S1Main);
    lv_obj_set_width(ui_S1LabelTitle, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_S1LabelTitle, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_S1LabelTitle, 0);
    lv_obj_set_y(ui_S1LabelTitle, -200);
    lv_obj_set_align(ui_S1LabelTitle, LV_ALIGN_CENTER);
    lv_label_set_text(ui_S1LabelTitle, "T-RGB Demo");
    lv_obj_set_style_text_font(ui_S1LabelTitle, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
//    lv_obj_clear_flag(ui_ScreenWifi, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
//    lv_obj_set_style_bg_img_src(ui_ScreenWifi, &ui_img_1672050212073_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SWifiLabelIPAddr = lv_label_create(ui_S1Main);
    lv_obj_set_width(ui_SWifiLabelIPAddr, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_SWifiLabelIPAddr, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SWifiLabelIPAddr, 0);
    lv_obj_set_y(ui_SWifiLabelIPAddr, lv_pct(-35));
    lv_obj_set_align(ui_SWifiLabelIPAddr, LV_ALIGN_CENTER);
    lv_label_set_text(ui_SWifiLabelIPAddr, "IP: n/a");
    lv_obj_set_style_text_font(ui_SWifiLabelIPAddr, &lv_font_montserrat_36, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_S1LabelClock = lv_label_create(ui_S1Main);
    lv_obj_set_width(ui_S1LabelClock, 180);
    lv_obj_set_height(ui_S1LabelClock, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_S1LabelClock, 0);
    lv_obj_set_y(ui_S1LabelClock, 133);
    lv_obj_set_align(ui_S1LabelClock, LV_ALIGN_CENTER);
    lv_label_set_long_mode(ui_S1LabelClock, LV_LABEL_LONG_DOT);
    lv_label_set_text(ui_S1LabelClock, "xx:yy:zz");
    lv_obj_set_style_text_align(ui_S1LabelClock, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_S1LabelClock, &lv_font_montserrat_36, LV_PART_MAIN | LV_STATE_DEFAULT);


//    ui_ScreenWifi_Slider1 = lv_slider_create(ui_ScreenWifi);
//    lv_slider_set_range(ui_ScreenWifi_Slider1, 45, 255);
//    lv_slider_set_value(ui_ScreenWifi_Slider1, 220, LV_ANIM_OFF);
//    if(lv_slider_get_mode(ui_ScreenWifi_Slider1) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_ScreenWifi_Slider1, 0,
//                                                                                                       LV_ANIM_OFF);
//    lv_obj_set_width(ui_ScreenWifi_Slider1, 320);
//    lv_obj_set_height(ui_ScreenWifi_Slider1, 25);
//    lv_obj_set_x(ui_ScreenWifi_Slider1, 0);
//    lv_obj_set_y(ui_ScreenWifi_Slider1, lv_pct(25));
//    lv_obj_set_align(ui_ScreenWifi_Slider1, LV_ALIGN_CENTER);
//    lv_obj_clear_flag(ui_ScreenWifi_Slider1, LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE);      /// Flags

//    ui_ScreenWifi_Label8 = lv_label_create(ui_ScreenWifi);
//    lv_obj_set_width(ui_ScreenWifi_Label8, LV_SIZE_CONTENT);   /// 1
//    lv_obj_set_height(ui_ScreenWifi_Label8, LV_SIZE_CONTENT);    /// 1
//    lv_obj_set_x(ui_ScreenWifi_Label8, 0);
//    lv_obj_set_y(ui_ScreenWifi_Label8, lv_pct(25));
//    lv_obj_set_align(ui_ScreenWifi_Label8, LV_ALIGN_CENTER);
//    lv_label_set_text(ui_ScreenWifi_Label8, "Helligkeit");
//    lv_obj_set_style_text_color(ui_ScreenWifi_Label8, lv_color_hex(0x00ECBB), LV_PART_MAIN | LV_STATE_DEFAULT);
//    lv_obj_set_style_text_opa(ui_ScreenWifi_Label8, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
//    lv_obj_set_style_text_font(ui_ScreenWifi_Label8, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
//    lv_obj_set_style_blend_mode(ui_ScreenWifi_Label8, LV_BLEND_MODE_ADDITIVE, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SWifiButtonStandby = lv_btn_create(ui_S1Main);
    lv_obj_set_width(ui_SWifiButtonStandby, 169);
    lv_obj_set_height(ui_SWifiButtonStandby, 50);
    lv_obj_set_x(ui_SWifiButtonStandby, 0);
    lv_obj_set_y(ui_SWifiButtonStandby, lv_pct(38));
    lv_obj_set_align(ui_SWifiButtonStandby, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_SWifiButtonStandby, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_SWifiButtonStandby, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SWifiButtonStandby, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_SWifiButtonStandby, lv_color_hex(0x5D0000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SWifiButtonStandby, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_SWifiButtonStandby, lv_color_hex(0x595959), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_SWifiButtonStandby, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_SWifiButtonStandby, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ScreenWifiButtonLabelStandby = lv_label_create(ui_SWifiButtonStandby);
    lv_obj_set_width(ui_ScreenWifiButtonLabelStandby, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ScreenWifiButtonLabelStandby, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ScreenWifiButtonLabelStandby, LV_ALIGN_CENTER);
    lv_label_set_text(ui_ScreenWifiButtonLabelStandby, "Standby");
    lv_obj_set_style_text_font(ui_ScreenWifiButtonLabelStandby, &lv_font_montserrat_36, LV_PART_MAIN | LV_STATE_DEFAULT);

//    lv_obj_add_event_cb(ui_ScreenWifi_Slider1, ui_event_ScreenWifi_Slider1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SWifiButtonStandby, ui_event_SWifiButtonStandby, LV_EVENT_ALL, NULL);

    ui_ScreenChart_Chart1 = lv_chart_create(ui_S1Main);
    lv_obj_set_width(ui_ScreenChart_Chart1, lv_pct(85));
    lv_obj_set_height(ui_ScreenChart_Chart1, lv_pct(50));
    lv_obj_set_align(ui_ScreenChart_Chart1, LV_ALIGN_CENTER);

    lv_obj_set_style_line_color(ui_ScreenChart_Chart1, lv_color_hex(0x4040FF), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui_ScreenChart_Chart1, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_color(ui_ScreenChart_Chart1, lv_color_hex(0x4040FF), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_opa(ui_ScreenChart_Chart1, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_width(ui_ScreenChart_Chart1, 5, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_line_width(ui_ScreenChart_Chart1, 5, LV_PART_ITEMS | LV_STATE_DEFAULT);
}

static lv_chart_series_t * ser_v;

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_S1Main_screen_init();
    lv_disp_load_scr(ui_S1Main);

	lv_obj_t* chart = ui_ScreenChart_Chart1;
	lv_chart_set_point_count(chart, 120);		// Size 120 data points (1 per 30sec)
	lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_SHIFT);
	lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 400, 460);
	ser_v = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
}


void chart_add_voltage(const float v) {
	lv_obj_t* chart = ui_ScreenChart_Chart1;
	lv_chart_set_next_value(chart, ser_v, (int16_t) (v * 100));
    lv_chart_refresh(chart);
}



