// MIT License
//
// Based on examples from:
// https://github.com/matthijskooijman/arduino-lmic
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// https://github.com/G4lile0/LoRa_LCD_TTN_Mapper

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include <keys.h>

#define LEDPIN 2

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
#define BUTTON_PIN 0

unsigned int counter = 0;
char TTN_response[30];

enum { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS, EV_LONGLONGPRESS };
enum { MANUAL=0, AUTO};
int mode = MANUAL;
bool was_pressed = false;
const int timeLongThreshold = 500;
const int timeLongLongThreshold = 2000;
long timeCounter = 0;
int spreadFactor = DR_SF7;
String lora_msg = "";


SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};



void do_send(osjob_t* j){
    // Payload to send (uplink)

    static uint8_t message[] = "H";

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, message, sizeof(message)-1, 0);
        Serial.println(F("Sending uplink packet..."));
        digitalWrite(LEDPIN, HIGH);
        lora_msg = "Sending uplink packet...";

    }
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
           case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
              lora_msg =  "Received ACK.";
            }

            lora_msg = "rssi:" + String(LMIC.rssi) +" snr: " + String(LMIC.snr);

            if (LMIC.dataLen) {
             // int i = 0;
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
              Serial.println(LMIC.rssi);

              // display.drawString (0, 9, "Received DATA.");
              // for ( i = 0 ; i < LMIC.dataLen ; i++ )
              //   TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
              // TTN_response[i] = 0;
              // display.drawString (0, 22, String(TTN_response));
              // display.drawString (0, 32, String(LMIC.rssi));
              // display.drawString (64,32, String(LMIC.snr));
            }

            // Schedule next transmission
            if(mode == AUTO) {
                os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            }
            digitalWrite(LEDPIN, LOW);
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            lora_msg = "OTAA joining....";
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
            lora_msg = "OTAA Joining failed";
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            lora_msg = "Joined!";
            delay(3);
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);

            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }

}

void lora_init() {
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only2 three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.




    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(spreadFactor,14);

    // Start job
    //do_send(&sendjob);     // Will fire up also the join
    //LMIC_startJoining();
}



void drawMain() {
    display.clear();
    display.drawString (0, 0, "The Things Network");
    display.drawString (0, 12, "Mode: ");
    if(mode == MANUAL) {
        display.drawString (30, 12, "Manual");
    } else {
        display.drawString (30, 12, "Auto");
    }
    display.drawString (0, 24, "SF:");
    display.drawString (20, 24, String(12-spreadFactor));
    display.drawString (0, 36, lora_msg);

    if (LMIC.opmode & OP_TXRXPEND) {
        display.drawString (0, 48, "Busy");
    }else {
        display.drawString (0, 48, "Idle");
    }

    display.display ();
}

int buttonHandle() {
    int event;
    int millis_now = millis();
    bool now_pressed = !digitalRead(BUTTON_PIN);

    if (!now_pressed && was_pressed) {
    // handle release event
        if ((millis_now - timeCounter) < timeLongThreshold){
            event = EV_SHORTPRESS;
        }else if ((millis_now - timeCounter) < timeLongLongThreshold){
            event = EV_LONGPRESS;
        }else {
            event = EV_LONGLONGPRESS;
        }
        timeCounter = 0;
    } else {
        event = EV_NONE;
    }
    // new pressed
    if (!was_pressed) {
         timeCounter = millis_now;
    }
    // remember state, and we're done
    was_pressed = now_pressed;
    return event;

}


void setup() {
    Serial.begin(115200);
    delay(2500);                      // Give time to the serial monitor to pick up
    Serial.println(F("Starting..."));

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

    //Set up and reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    pinMode(BUTTON_PIN, INPUT);
    digitalWrite(BUTTON_PIN, INPUT_PULLUP); // pull-up

    display.init ();
    display.flipScreenVertically ();
    display.setFont (ArialMT_Plain_10);

    display.setTextAlignment (TEXT_ALIGN_LEFT);

    lora_msg = "Starting....";

    lora_init();
    lora_msg = "Ready!";
}

void loop() {

    drawMain();


    int event = buttonHandle();

    // send single uplink
    if(event == EV_LONGPRESS) {
        os_setCallback(&sendjob, do_send);
    }

    //change spread factor. If auto mode, short press cancel mode
    if (event == EV_SHORTPRESS)   {
        if(mode == AUTO) {
            mode = MANUAL;
            os_clearCallback(&sendjob);
        }else {
            spreadFactor--;
            if (spreadFactor==-1)  spreadFactor = 5 ;
            LMIC_setDrTxpow(spreadFactor,14);
        }
    }

    //auto mode. Send ulplink every TX_INTERVAL
    if (event == EV_LONGLONGPRESS)   {
        mode = AUTO;
        os_setCallback(&sendjob, do_send);
    }

    os_runloop_once();
    vTaskDelay(1); // yield to CPU
}
