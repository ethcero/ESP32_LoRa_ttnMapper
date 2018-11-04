# ESP32_LoRa_ttnMapper
Just another TTN Mapper

Its works with the TTGO LORA32 ESP32 OLED
https://tienda.bricogeek.com/arduino-compatibles/1122-ttgo-lora32-esp32-con-oled-900-mhz.html

The authorization is done with OTAA. You need to rename the file `keys.h.template` to `keys.h` and set your IDs from the TTN console.

## Features
Uses the PRG integrated button to control the node.
 - Short press: Change spread factor.
 - Long press (<0.5s): Send one uplink packet at time.
 - Long Long press (>2s): Start the Auto modo that send uplink packets each 30 seconds.
 - Short press while in Auto mode: Stop automatic mode.
