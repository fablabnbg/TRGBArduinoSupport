Installation: [![PlatformIO Registry](https://badges.registry.platformio.org/packages/ianh/library/TRGBArduinoSupport.svg)](https://registry.platformio.org/libraries/ianh/TRGBArduinoSupport)

This repo adds support for T-RGB device to use with arduino framework and platformio.

It's just a copy of the official example code into a sufficient structure. Some functions were reordered or newly combined.
There is also one bugfix to the touchpad-integration. 

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

You need [platformio](https://platformio.org/platformio-ide) to install and run this libary.

So, create your own project with:

`platformui init --board=esp32-s3-devkitc-1`

Adapt the `platformio.ini` file to contain this section:

```
[env:esp32-s3-devkitc-1]
platform = platformio/espressif32
board = esp32-s3-devkitc-1
framework = arduino
build_flags = 
	-DARDUINO_USB_MODE=1
	-DARDUINO_USB_CDC_ON_BOOT=1
	-DBOARD_HAS_PSRAM
	-DLV_CONF_PATH=${platformio.include_dir}/lv_conf.h
board_build.cpu = esp32s3
board_build.arduino.memory_type = qio_opi
lib_deps = 
        IanH/TRGBArduinoSupport
``` 

Important options:

* `-DARDUINO_USB_CDC_ON_BOOT=1`: Use onboard USB as serial device (for debug output). (See Caveats below!)
* `-DBOARD_HAS_PSRAM` - default platformio configuration for esp32-s3 does not enable PSRAM support, so enable it here
* `-DLV_CONF_PATH=${platformio.include_dir}/lv_conf.h`- If you want to use the lvgl library there are issues to find the configuration file. So explicitly state where it is located

Create `src/main.cpp`:

```C++
#include <TRGBSuppport.h>

TRGBSuppport trgb;
void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(1);  // workaround for blocking output if no host is connected to native USB CDC
  delay(100);   // Rumors say it helps avoid sporadical crashes after wakeup from deep-sleep
  trgb.init();
  
  // load your UI etc. (see example https://github.com/fablabnbg/TRGBArduinoSupport/tree/main/examples/ui_example)
}

void loop() {

// your loop code

}

```

Create `include/lv_conf.h`:

* The library comes with integrated dependencies to [lvgl graphics library](https://lvgl.io) to simplify using it.
* Therefore it is necessary to have an `lv_conf.h` file. So please copy the example from the example directory to your include path. There is no direct need to adapt it, but you can if you want so (e.g. to enable/disable other fonts).

* Under Linux, the easiest way to do this: (in the projects main directory)

```
cp .pio/libdeps/esp32-s3-devkitc-1/TRGBArduinoSupport/examples/lv_conf.h include/
```

* You also can download the file (https://raw.githubusercontent.com/fablabnbg/TRGBArduinoSupport/main/examples/lv_conf.h) and save it to the include directory. 

## UI Example

There is an example UI in https://github.com/fablabnbg/TRGBArduinoSupport/tree/main/examples/ui_example .
It shows a chart with battery voltage, the current time and IP adress of WiFi connection. There is also a "Standby" button to send the device into deep-sleep. It can be woken up again by touch.
You can copy it to your platformio project `src`directory, e.g. (Linux) by
```
cp .pio/libdeps/esp32-s3-devkitc-1/TRGBArduinoSupport/examples/ui_example/* src/
```
There is also an example `platformio.ini` file:
```
cp .pio/libdeps/esp32-s3-devkitc-1/TRGBArduinoSupport/examples/platformio.ini .
```

Adapt `src/main.cpp` to your needs (change Wifi SSID and password) and then build and upload it with `platformio run -t upload`.

## Caveats

### USB CDC

The T-RGB has no UART/UART to USB interface. Instead it uses ESP32's native USB support. The onboard USB-C connector is connected directly to corresponding GPIOS. Firmware upload is possible using this USB interface in any case (at least in boot mode which can be manually entered with the Boot-pushbutton pressed during reset).

To use the USB as Serial device, you must set these two configuration options:

	-DARDUINO_USB_MODE=1
	-DARDUINO_USB_CDC_ON_BOOT=1

The first one activates the USB support in general, the second one activates the support for USB as `Serial` device.
Unfortunately the direct USB support has  big disadvantage: 
  **Output to `Serial` blocks when the output buffer is full.** As long as there is no terminal emulator (like miniterm.py) connected to the ESP32, the buffer will run full quickly and the device gets laggy. If you use `Serial.flush()` somewhere in your code, it will block until the Serial device is read from terminal.
  
  * So, you should connect a terminal emulator OR set `ARDUINO_USB_CDC_ON_BOOT` to 0. 
  * As a workaround you can set the tx timeout to a low value (`Serial.setTxTimeoutMs(1);`) and you must avoid using `Serial.flush()`.

CDC also means that not all debug output from ESP-IDF/freeRTOS can be written to Serial output. This especially affects crashes/resets:
**You won't see a reset reason**, so it can be hard to find issues like stack overflows, heap corruption or watchdog timeouts.
You can install the Espressif IDF locally and use the coredump.py utility provided by Espressif. We recommend to install it in a seperate directory, e.g. by following the instructions from Espressif: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html

When IDF is installed and setup, you can get debug output with: `esp-coredump --port /dev/ttyACM0 info_corefile .pio/build/trgb-esp32-s3/firmware.elf`

This will show a long dump of registers, threads and stack trace per thread.

Example for crash due to nullptr access ("LoadProhibitedCause" as reset cause) in `src/BLEDevices.cpp:142`

```
espcoredump.py v1.4.2
INFO: Invoke parttool to read image.
INFO: esptool.py v4.5.1
Serial port /dev/ttyACM0
Connecting...
Detecting chip type... ESP32-S3
Chip is ESP32-S3 (revision v0.1)

[..more device info..]

===============================================================
==================== ESP32 CORE DUMP START ====================

Crashed task handle: 0x3fcf34f4, name: 'esp_timer', GDB name: 'process 1070544116'

================== CURRENT THREAD REGISTERS ===================
exccause       0x1c (LoadProhibitedCause)
excvaddr       0x34
epc1           0x420e2fd1
epc2           0x0
epc3           0x0
epc4           0x0
epc5           0x0
epc6           0x0
eps2           0x0
eps3           0x0
eps4           0x0
eps5           0x0
eps6           0x0
pc             0x42049ff6          0x42049ff6 <BLERemoteService::getCharacteristic(BLEUUID)+22>

[.. more registers..]
[.. Stack trace of crashed thread followed by a list of all threads:]

==================== CURRENT THREAD STACK =====================
#0  BLERemoteService::getCharacteristic (this=0x0, uuid=...) at /home/ian/.platformio/packages/framework-arduinoespressif32/libraries/BLE/src/BLERemoteService.cpp:142
#1  0x42005080 in BLEDevices::connectToServer (this=<optimized out>, ctype=<optimized out>) at src/BLEDevices.cpp:142
#2  0x420059d5 in BLEDevices::connCheckLoop (this=0x3fca0eb8 <bleDevs>) at src/BLEDevices.cpp:305
#3  BLEDevices::connCheckLoop (this=0x3fca0eb8 <bleDevs>) at src/BLEDevices.cpp:300
#4  0x42005a80 in BLEDevices::<lambda(BLEDevices*)>::operator() (__closure=0x0, thisInstance=0x3fca0eb8 <bleDevs>) at src/BLEDevices.cpp:183
#5  BLEDevices::<lambda(BLEDevices*)>::_FUN(BLEDevices *) () at src/BLEDevices.cpp:183
#6  0x42076d51 in timer_process_alarm (dispatch_method=ESP_TIMER_TASK) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/esp_timer/src/esp_timer.c:360
#7  timer_task (arg=<optimized out>) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/esp_timer/src/esp_timer.c:386

======================== THREADS INFO =========================
  Id   Target Id          Frame 
* 1    process 1070544116 BLERemoteService::getCharacteristic (this=0x0, uuid=...) at /home/ian/.platformio/packages/framework-arduinoespressif32/libraries/BLE/src/BLERemoteService.cpp:142
  2    process 1070319624 0x400559e0 in ?? ()
  3    process 1070516268 0x400559e0 in ?? ()
  4    process 1070552712 0x4215b05e in esp_pm_impl_waiti () at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/esp_pm/pm_impl.c:839
  5    process 1070551312 0x4215b05e in esp_pm_impl_waiti () at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/esp_pm/pm_impl.c:839
  6    process 1070274428 UIFacade::updateHandler (this=0x3fca0bb0 <ui>) at src/UIFacade.cpp:89
  7    process 1070523172 0x40381e9a in vPortEnterCritical (mux=0x3fced67c) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  8    process 1070536660 0x40382002 in vPortEnterCritical (mux=0x3fcf13ac) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  9    process 1070524764 0x40381e9a in vPortEnterCritical (mux=0x3fcee6e0) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  10   process 1070538360 vPortEnterCritical (mux=0x3fcf1a50) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  11   process 1070525124 0x40381e9a in vPortEnterCritical (mux=0x3fcee5d4) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  12   process 1070324868 0x40382004 in vPortEnterCritical (mux=0x3fcbd194) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  13   process 1070306488 0x40382004 in vPortEnterCritical (mux=0x3fcb8414) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  14   process 1070333008 0x40382004 in vPortEnterCritical (mux=0x3fcbe960) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578
  15   process 1070559724 0x40381e9a in vPortEnterCritical (mux=0x3fcf6b94) at /Users/ficeto/Desktop/ESP32/ESP32S2/esp-idf-public/components/freertos/port/xtensa/include/freertos/portmacro.h:578


[.. stack trace of all threads + information about memory regions]

```

### Touchpad

The examples from LilyGo for touchpad are quite confusing. I changed the code to use only the ft3267 touch controller and fixed a bug in the callback functions. The original code does not correctly behave when touch is released: It sends a position 0,0 once after release. This is fixed in the  TRGBArduinoSupport code.

**It seems that the FT3267 is only used on the "oval" variant of the T-RGB device.** The round variant is therefore not yet supported.
