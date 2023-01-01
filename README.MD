This repo adds support for T-RGB device to use with arduino framework and platformio.

It's just a copy of the official example code into a sufficient structure. Some functions were reordered are newly combined. 

## Why we created this library?

While looking for a better display for Ian's bluetooth bicycle computer (see https://github.com/euphi/ESP32_BTTacho), Ian discovered the
LilyGO T-RGB device which has almost everthing needed already on-board:

* An ESP32 chip (so Wifi and Bluetooth, a lot of computing power, RAM, Flash etc.)
* 8 MB PSRAM (so even more RAM!)
* a SD Card reader
* I2C connector (groove) for more peripherals (like acceleration sensor)
* A lithium-battery connector, so the ESP32-integrated RTC can be used as clock.
* a great 480px round touch display

### What's missing:

* Ian still thinks that it would be useful to also have some RGB leds or input buttons connected to his bike computer.
The T-RGB does not have any spare GPIO (except the ones used for groove I2C).
There are 8 spare inputs on the XL9535 device, maybe he will solder a cable with connector to it.

## License

The code is using the same license as the original files:
* [MIT License](https://choosealicense.com/licenses/mit/) for the initilization code and XL9535 driver
* [Apache 2.0 licence](https://www.apache.org/licenses/LICENSE-2.0) for the touchpad ft3267 driver (by Espressif Systems (Shanghai) Co. Ltd.)

## How to use

The idea is that you can start right away and don't need to care about copying code from the examples
to build your own applications. Therefore we copied the necessary code together and encapsulated it in
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
        git@github.com:fablabnbg/TRGBArduinoSupport.git
``` 

Important options:

* `-DARDUINO_USB_CDC_ON_BOOT=1`: Use onboard USB as serial device (for debug output). (See Caveeats below!!)
* `-DBOARD_HAS_PSRAM` - default platformio configuration for esp32-s3 does not enable PSRAM support, so enable it here
* `-DLV_CONF_PATH={$PLATFORMIO_WORKSPACE_DIR}/lv_conf.h`- If you want to use the lvgl library there are issues to find the configuration file. So explicitly state where it is located

* `git@github.com:fablabnbg/TRGBArduinoSupport.git` - intgegrate this library

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
  
  // load your UI etc.
}

void loop() {

// your loop code

}

```

## Caveeats

### USB CDC

The T-RGB has no UART/UART to USB interface. Instead it uses ESP32's native USB support. The onboard USB-C connector is connected directly to corresponding GPIOS. Firmware upload is possible using this USB interface in any case (at least in boot mode which can be manually entered with the Boot-pushbutton pressed during reset).

To use the USB as Serial device, you must set these two configuration options:

	-DARDUINO_USB_MODE=1
	-DARDUINO_USB_CDC_ON_BOOT=1

The first one activates the USB support in general, the second one activates the support for USB as `Serial` device.
Unfortunately the direct USB support has  big disadvantage: 
  **Output to `Serial` blocks when the output buffer is full.** As long as there is no terminal connected to the ESP32, the buffer will run full quickly and the device seems to be frozen.
  
  So, you must connect a terminal always OR set `ARDUINO_USB_CDC_ON_BOOT` to 0.


