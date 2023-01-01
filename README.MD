This repo adds support for T-RGB device to use with arduino framework and platformio.

It's just a copy of the official example code into a sufficient structure.



## Why I created this library?

While looking for a better display for me bluetooth bicycle computer (see euphi/ESP32BTTacho), I found the
LilyGO T-RGB device which has almost everthing I need:

* An ESP32 chip (so Wifi and Bluetooth)
* a SD Card reader
* I2C connector for more peripherals (like acceleration sensor)
* A lithium-battery connector, so I can make use of the ESP32-integrated RTC.
* a great 480px round touch display

### What I'm missing:

* I still think that it would be useful to also have some RGB leds or input buttons connected.
The T-RGB does not have any spare GPIO (except the ones used for groove I2C).
There are 8 spare inputs on the XL9535 device, maybe I'll solder a cable with connector to it.


## How to use

The idea is that you can start right away and don't need to care about copying code from the examples
to build your own applications. Therefore I copied the necessary code together and encapsulated it in
this libary.

You need _platformio_ to install and run this libary.

So, create your own project with:

`platformui init --board=esp32-s3-devkitc-1`

Adapt the `platformio.ini` file to contain this section:

```
[env:esp32-s3-devkitc-1]
platform = platformio/espressif32
board = esp32-s3-devkitc-1
framework = arduino
platform_packages = 
	framework-arduinoespressif32@https://github.com/espressif/arduino-esp32.git#2.0.5
build_flags = 
	-DARDUINO_USB_MODE=1
	-DARDUINO_USB_CDC_ON_BOOT=1
	-DBOARD_HAS_PSRAM
	-DLV_CONF_PATH={$PLATFORMIO_WORKSPACE_DIR}/lv_conf.h
board_build.cpu = esp32s3
board_build.arduino.memory_type = qio_opi
lib_deps = 
        git@github.com:euphi/TRGBArduinoSupport.git
``` 

Important options:

* `-DARDUINO_USB_CDC_ON_BOOT=1`: Use onboard USB as serial device (for debug output). (See Caveeats below!!)
* `-DBOARD_HAS_PSRAM` - default platformio configuration for esp32-s3 does not enable PSRAM support, so enable it here
* `git@github.com:euphi/TRGBArduinoSupport.git` - intgegrate this library

In your arduino code (`src/main.cpp`):

```
#include <TRGBSuppport.h>

TRGBSuppport trgb;
void setup() {
  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t)400000);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);  // workaround for blocking output if no host is connected to native USB CDC
  delay(100);   // Rumors say it helps avoid sporadical crashes after wakeup from deep-sleep
  trgb.init();

}

void loop() {

// your loop code

}

```






