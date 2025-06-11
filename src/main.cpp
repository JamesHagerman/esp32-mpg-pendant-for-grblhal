#include <Arduino.h>
#include "driver/pcnt.h"
#include <HardwareSerial.h>

HardwareSerial grblSerial(1);
#define GRBL_TX_PIN 17
#define GRBL_RX_PIN 16

#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35
#define ESTOP_PIN     25

#define MULT_X1_PIN   14
#define MULT_X10_PIN  27
#define MULT_X100_PIN 26

#define AXIS_X_PIN    33
#define AXIS_Y_PIN    32
#define AXIS_Z_PIN    23
#define AXIS_A_PIN    22

#define LED_PIN       2

// COMMON_PIN powers the switch matrix for axis/multiplier — keep LOW to enable
#define COMMON_PIN    4

#define PCNT_UNIT     PCNT_UNIT_0
#define DEBUG_OUTPUT 0

// State tracking
enum MpgState { IDLE, READY, JOGGING, ESTOP };
MpgState mpg_state = IDLE;

enum Axis { AXIS_NONE = -1, AXIS_X, AXIS_Y, AXIS_Z, AXIS_A };
Axis current_axis = AXIS_NONE;
const char* axis_name = "NONE";

int current_multiplier = 1;

volatile bool raw_estop_signal = false;
volatile unsigned long last_estop_time = 0;
const unsigned long debounce_delay = 50;  // ms

static unsigned long last_blink_time = 0;
static bool blink_on = false;
const unsigned long blink_interval = 250;

void setup_pcnt() {
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = ENCODER_A_PIN;
    pcnt_config.ctrl_gpio_num = ENCODER_B_PIN;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;

    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

void IRAM_ATTR handle_estop() {
    raw_estop_signal = true;
}

void check_encoder() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);

    if (mpg_state != READY || current_axis == AXIS_NONE) {
        pcnt_counter_clear(PCNT_UNIT);  // Always clear out garbage, even during ESTOP
        return;
    }

    if (abs(count) >= 2) {
        pcnt_counter_clear(PCNT_UNIT);
        int detents = count / 2;
        float step = detents * 0.001f * current_multiplier;

        uint8_t axis_mask = (current_axis == AXIS_X) ? 0x01 :
                            (current_axis == AXIS_Y) ? 0x02 :
                            (current_axis == AXIS_Z) ? 0x04 : 0x08;

        int16_t microns = step * 1000.0f;
        grblSerial.write(0x91); // Command: MPG jog
        grblSerial.write(axis_mask); // Axis bitmask: X = 0x01
        grblSerial.write((uint8_t)(microns & 0xFF)); // LSB: 50 microns = 0x0032
        grblSerial.write((uint8_t)((microns >> 8) & 0xFF)); // MSB: high byte of 0x0032

        Serial.print("G91 G0 ");
        Serial.print(axis_name);
        Serial.println(step, 3);
    }
}

void setup() {
    Serial.begin(115200);
    grblSerial.begin(115200, SERIAL_8N1, GRBL_RX_PIN, GRBL_TX_PIN);

    setup_pcnt();

    pinMode(ESTOP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), handle_estop, CHANGE);

    pinMode(MULT_X1_PIN, INPUT_PULLUP);
    pinMode(MULT_X10_PIN, INPUT_PULLUP);
    pinMode(MULT_X100_PIN, INPUT_PULLUP);

    pinMode(AXIS_X_PIN, INPUT_PULLUP);
    pinMode(AXIS_Y_PIN, INPUT_PULLUP);
    pinMode(AXIS_Z_PIN, INPUT_PULLUP);
    pinMode(AXIS_A_PIN, INPUT_PULLUP);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(COMMON_PIN, OUTPUT);
    digitalWrite(COMMON_PIN, LOW);
}

void loop() {
    static unsigned long last_debug_time = 0;
    unsigned long now = millis();

    static Axis last_stable_axis = AXIS_NONE;
    static unsigned long axis_change_time = 0;
    const unsigned long axis_debounce_delay = 50;

    if (!digitalRead(MULT_X1_PIN)) current_multiplier = 1;
    else if (!digitalRead(MULT_X10_PIN)) current_multiplier = 10;
    else if (!digitalRead(MULT_X100_PIN)) current_multiplier = 100;

    Axis new_axis = AXIS_NONE;
    if (!digitalRead(AXIS_X_PIN)) new_axis = AXIS_X;
    else if (!digitalRead(AXIS_Y_PIN)) new_axis = AXIS_Y;
    else if (!digitalRead(AXIS_Z_PIN)) new_axis = AXIS_Z;
    else if (!digitalRead(AXIS_A_PIN)) new_axis = AXIS_A;

    if (new_axis != last_stable_axis) {
        axis_change_time = now;
        last_stable_axis = new_axis;
    }

    if ((now - axis_change_time) > axis_debounce_delay && current_axis != last_stable_axis) {
        current_axis = last_stable_axis;
        axis_name = (current_axis == AXIS_X) ? "X" :
                    (current_axis == AXIS_Y) ? "Y" :
                    (current_axis == AXIS_Z) ? "Z" :
                    (current_axis == AXIS_A) ? "A" : "NONE";
    }

    if (raw_estop_signal) {
        raw_estop_signal = false;
        if (now - last_estop_time > debounce_delay) {
            last_estop_time = now;
            if (digitalRead(ESTOP_PIN) == HIGH && mpg_state != ESTOP) {
                mpg_state = ESTOP;
                pcnt_counter_clear(PCNT_UNIT); // Clear any accumulated steps
                current_axis = AXIS_NONE; // Deselect all axes

                // Send estop back over usb serial for help debugging
                Serial.println("!");

                // Send actual estop command characters
                // grblSerial.write(0x8B);
                // grblSerial.write(0x94);

                // Test for status response
                grblSerial.write('?');
            }
        }
    }

    // Update LED status
    if (mpg_state != ESTOP) {
        digitalWrite(LED_PIN, current_axis != AXIS_NONE ? HIGH : LOW);
    }
    if (mpg_state == ESTOP) {
        if (digitalRead(ESTOP_PIN) == LOW) {
            // E-stop cleared
            digitalWrite(LED_PIN, LOW);
            pcnt_counter_clear(PCNT_UNIT);
            mpg_state = IDLE;
        } else {
            // Non-blocking LED blink
            if (now - last_blink_time >= blink_interval) {
                last_blink_time = now;
                blink_on = !blink_on;
                digitalWrite(LED_PIN, blink_on ? HIGH : LOW);
            }
        }
    }

    static Axis last_axis = AXIS_NONE;
    bool axis_selected = (current_axis != AXIS_NONE);

    // Only take action when axis selection actually changes
    if (current_axis != last_axis) {
        if (current_axis != AXIS_NONE && mpg_state == IDLE) {
            grblSerial.write(0x8B);  // Enter MPG mode
            mpg_state = READY;
        } else if (current_axis == AXIS_NONE && mpg_state == READY) {
            grblSerial.write(0x8B);  // Exit MPG mode
            mpg_state = IDLE;
        }
        last_axis = current_axis;
    }

    // Always run to discard any movement, but only acts when state is READY and axis selected
    check_encoder();

    // Mirror data from grblSerial to Serial for debugging
    while (grblSerial.available()) {
        uint8_t b = grblSerial.read();
        if (isPrintable(b)) {
            Serial.write(b);
        } else {
            Serial.print("[0x");
            if (b < 0x10) Serial.print("0");
            Serial.print(b, HEX);
            Serial.print("]");
        }
    }

#if DEBUG_OUTPUT
    if (now - last_debug_time > 500) {
        Serial.print("Axis: ");
        Serial.print(axis_name);
        Serial.print("  Multiplier: ");
        Serial.println(current_multiplier);
        last_debug_time = now;
    }
#endif
}
