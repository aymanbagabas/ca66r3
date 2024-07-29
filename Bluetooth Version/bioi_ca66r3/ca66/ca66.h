#pragma once

#include "quantum.h"
#include "led.h"

/* G60 LEDs
 *   GPIO pads
 *   0 F7 not connected
 *   1 F6 RGB PWM Underglow
 *   2 F5 Backlight LED
 *   3 F4 not connected
 *   F0 Capslock LED
 *   B0 not connected
 */
//inline void g60_caps_led_on(void)    { DDRF |=  (1<<0); PORTF &= ~(1<<0); }
//inline void g60_bl_led_on(void)      { DDRB |=  (1<<0); PORTB &= ~(1<<0); }

//inline void g60_caps_led_off(void)   { DDRF &= ~(1<<0); PORTF &= ~(1<<0); }
//inline void g60_bl_led_off(void)     { DDRB &= ~(1<<0); PORTB &= ~(1<<0); }

/* G60 Keymap Definition Macro */

#define ___ KC_NO

#define LAYOUT_all( \
      K00,  K01,  K02,  K03,  K04,  K05,  K06,  K07,  K08,  K09,  K0A,  K0B,  K0C,  K0D,  K49,  K0E, \
      K10,  K11,  K12,  K13,  K14,  K15,  K16,  K17,  K18,  K19,  K1A,  K1B,  K1C,  K1D,        K1E, \
      K20,  K21,  K22,  K23,  K24,  K25,  K26,  K27,  K28,  K29,  K2A,  K2B,  K2C,  K2D,        K2E, \
      K30,  K31,  K32,  K33,  K34,  K35,  K36,  K37,  K38,  K39,  K3A,  K3B,  K3C,  K3D,        K3E, \
      K40,        K42,  K43,                    K47,              K4A,  K4B,  K4C,  K4D,  K4E        \
) { \
    { K00,  K01,  K02,  K03,   K04,   K05,  K06,   K07,  K08,  K09,  K0A,  K0B,  K0C,  K0D,  K0E }, \
    { K10,  K11,  K12,  K13,   K14,   K15,  K16,   K17,  K18,  K19,  K1A,  K1B,  K1C,  K1D,  K1E }, \
    { K20,  K21,  K22,  K23,   K24,   K25,  K26,   K27,  K28,  K29,  K2A,  K2B,  K2C,  K2D,  K2E }, \
    { K30,  K31,  K32,  K33,   K34,   K35,  K36,   K37,  K38,  K39,  K3A,  K3B,  K3C,  K3D,  K3E }, \
    { K40,  ___,  K42,  K43,   ___,   ___,  ___,   K47,  ___,  K49,  K4A,  K4B,  K4C,  K4D,  K4E }  \
}

