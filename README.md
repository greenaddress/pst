This is a firmware for an ESP32 based dev board (ideally with psram) to pass through the PSX Link cable traffic to a pc uart or via tcp using WiFi

This was tested on a Lolin 32 Pro but should work on all ESP32

The wiring was done following https://unirom.github.io/serial_psx_cable/ (I've used a 6.6k resistor though as that's all I had around that was in the alleged range 1k-8k)


If you don't have esp-idf already setup please follow the [get-started](https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32/get-started/index.html) official Espressif guide.


to flash use;
```
idf.py flash
```

to configure use:
```
idf.py menuconfig
```

Notes on configuration

The ESP32 has 3 uarts.

uart 0 in most ESP32 dev board is pre-wired to a USB tty converter chip
so you shouldn't need to change it.

What uart you use for the PSX shouldn't matter as long as it is not the same one used by the PC.


For more information on pinout SIO see https://psx-spx.consoledev.net/pinouts/#pinouts-sio-pinouts

Additional info also: https://thp.io/2020/psxserial.html
and https://psx0.wordpress.com/2013/08/08/playstation-usb-link-cable/
http://www.psxdev.net/forum/viewtopic.php?t=760


For more information on esp32 pinout see https://www.upesy.com/blogs/tutorials/esp32-pinout-reference-gpio-pins-ultimate-guide


ESP32 hardware note:

The esp32 will log to uart0 some output on boot (from rom) - regardless of logging configuration - unless gpio 15 is strapped to ground.
This doesn't affect you if you are remapping the TX and RX gpio (from -1)

How to use the tcp port with nops (pass /tmp/virtualcom0 to nops) ?

No, don't use socat with nops - nops supports already both real uart and tcp natively.

https://github.com/JonathanDotCel/NOTPSXSerial/blob/30e907433678d0e0b358c7ea3ea702ad2e542761/NOTPSXSERIAL.CS#L722
