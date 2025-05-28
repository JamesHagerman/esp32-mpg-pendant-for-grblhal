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
long step_counter = 0;


volatile bool raw_estop_signal = false;
volatile unsigned long last_estop_time = 0;
const unsigned long debounce_delay = 50;  // ms

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

void update_switches() {
    bool axis_selected = false;

    if (!digitalRead(MULT_X1_PIN)) current_multiplier = 1;
    else if (!digitalRead(MULT_X10_PIN)) current_multiplier = 10;
    else if (!digitalRead(MULT_X100_PIN)) current_multiplier = 100;

    if (!digitalRead(AXIS_X_PIN)) { current_axis = AXIS_X; axis_name = "X"; axis_selected = true; }
    else if (!digitalRead(AXIS_Y_PIN)) { current_axis = AXIS_Y; axis_name = "Y"; axis_selected = true; }
    else if (!digitalRead(AXIS_Z_PIN)) { current_axis = AXIS_Z; axis_name = "Z"; axis_selected = true; }
    else if (!digitalRead(AXIS_A_PIN)) { current_axis = AXIS_A; axis_name = "A"; axis_selected = true; }
    else { current_axis = AXIS_NONE; axis_name = "NONE"; }

    digitalWrite(LED_PIN, axis_selected ? HIGH : LOW);
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
        step_counter += detents;
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

    update_switches();

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

    if (mpg_state == ESTOP) {
        while (digitalRead(ESTOP_PIN) == HIGH) {
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(250);
        }
        step_counter = 0;
        digitalWrite(LED_PIN, LOW);
        pcnt_counter_clear(PCNT_UNIT);  // Clear any steps counted during estop
        mpg_state = IDLE;
        return;
    }

    bool axis_selected = (current_axis != AXIS_NONE);
    if (axis_selected && mpg_state == IDLE) {
        grblSerial.write(0x8B);
        mpg_state = READY;
    } else if (!axis_selected && mpg_state == READY) {
        grblSerial.write(0x8B);
        mpg_state = IDLE;
    }

    // Always run encoder check to clear garbage when axis is NONE
    check_encoder();

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
