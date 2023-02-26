#include <TRGBSuppport.h>

#include "ui.h"
#include "WiFi.h"
#include <DateTime.h>

const char* ntpServer = "pool.ntp.org";
const char* ssid = "fablabnbg";
const char* password = "askForIt!";

TRGBSuppport trgb;

// Callback from UI if Standby-Button is pressed
void ui_ev_standby(lv_event_t * e) {
	trgb.deepSleep();
}

void updateIP(const String &str) {
	lv_label_set_text(ui_SWifiLabelIPAddr, ("IP: " + str).c_str());
}

void updateClock(const String &str) {
	lv_label_set_text(ui_S1LabelClock, str.c_str());
}


void setup() {  
  delay(100);   // Rumors say it helps avoid sporadical crashes after wakeup from deep-sleep
  trgb.init();
  
  // Print some info to Serial
  TRGBSuppport::print_chip_info();
  TRGBSuppport::scan_iic();

  // Initialize SD Card. It can be accessed by SD_MMC object.
  trgb.SD_init();

  // Connect to WiFi
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
  configTime(3600, 3600, ntpServer);   // Configure Timezone and NTP

  // load UI
  ui_init();

  // add your own stuff ..
}

void loop() {
	static uint32_t Millis = 0;
	static bool wifiInitialized = false;
	static uint16_t time_counter = 0;
	lv_timer_handler();
	if (millis() - Millis > 500) {	// every 500ms
		time_counter++;
		Millis=millis();
		float v = trgb.getBatVoltage();
		chart_add_voltage(v);
	    Serial.print(WiFi.status());
		// Handle Wifi
		if (!wifiInitialized) {
			if (WiFi.status() == WL_CONNECTED) {
				wifiInitialized = true;
				updateIP(WiFi.localIP().toString());
			} else if (time_counter > 30 /*15s*/) {
//				wifiInitialized = true;
//				Serial.println("Not connected to Wifi - disabling it");
//				updateIP(String("WiFi disabled"));
//				WiFi.mode(WIFI_MODE_NULL);
//				WiFi.setSleep(WIFI_PS_MAX_MODEM);
//				WiFi.setSleep(true);
				WiFi.begin(ssid, password);		// retry
				updateIP(String("Timeout - retrying.."));
			} else {
			    delay(10);
			}
		} else if (WiFi.status() == WL_CONNECTION_LOST) {
			updateIP(String("connection lost"));
			Serial.println("Wifi connection lost - disabling it to save power");
			WiFi.mode(WIFI_MODE_NULL);
			WiFi.setSleep(WIFI_PS_MAX_MODEM);
			WiFi.setSleep(true);
		}
	    time_t now;
	    time(&now);

	    // To use the DateFormatter, add to platformio.ini lib_deps the library ESPDateTime
		String time_str = DateFormatter::format(DateFormatter::TIME_ONLY,now);
	    updateClock(time_str);
	} else {
		delay(2);
	}
}




