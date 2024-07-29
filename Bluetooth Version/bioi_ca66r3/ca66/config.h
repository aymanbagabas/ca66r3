/*
Copyright 2012 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */

//#define VENDOR_ID       0xFEED
//#define PRODUCT_ID      0x8101

#define VENDOR_ID       0xb101
#define PRODUCT_ID      0x4366    // CA66=C66=0x43 66
#define DEVICE_VER      0x0001
#define MANUFACTURER    Basic IO Instruments
#define PRODUCT         BIOI CA66

/* key matrix size */
#define MATRIX_ROWS 5
#define MATRIX_COLS 15

#define BLE_NAME        "BIOI CA66 BLE"

/*
 * Keyboard Matrix Assignments
 *
 * Change this to how you wired your keyboard
 * COLS: AVR pins used for columns, left to right
 * ROWS: AVR pins used for rows, top to bottom
 * DIODE_DIRECTION: COL2ROW = COL = Anode (+), ROW = Cathode (-, marked on diode)
 *         ROW2COL = ROW = Anode (+), COL = Cathode (-, marked on diode)
 *
*/


// Pin define of first version (green board)

//                        R1  R2  R3  R4  R5
#define MATRIX_ROW_PINS { B0, E6, F1, F6, F4 }

//                        C1   C2   C3   C4   C5   C6   C7   C8   C9   C10  C11  C12  C13  C14  C15
#define MATRIX_COL_PINS { F7,  C7,  C6,  B6,  B5,  B4,  D7,  D6,  D4,  D5,  B3,  B1,  D1,  D0,  B2 }

//#define QMK_KEYS_PER_SCAN 4

/* Backlight Setup */
//#define BACKLIGHT_PIN B7
//#define BACKLIGHT_PIN B7
//#define BACKLIGHT_LEVELS 12


/* COL2ROW or ROW2COL */
#define DIODE_DIRECTION COL2ROW

#define NO_BAT_LEVEL


/* RGB Underglow
 * F6 PIN for XD60v2 that has pre-soldered WS2812 LEDs
*/



#define RGB_UG_CONTROL_PIN B7
#define BLE_CONTROL_PIN E2

#define RGBLIGHT_LAYERS

#define RGB_DI_PIN F5
#define WS2812_BYTE_ORDER WS2812_BYTE_ORDER_GRB
#define RGBLIGHT_ANIMATIONS
#define RGBLED_NUM 12  // Number of LEDs
#define RGBLIGHT_HUE_STEP 8
#define RGBLIGHT_SAT_STEP 8
#define RGBLIGHT_VAL_STEP 8
#define RGBLIGHT_LIMIT_VAL 225



//#define QMK_ESC_OUTPUT F0
//#define QMK_ESC_INPUT D0
//#define QMK_LED B2
//#define QMK_SPEAKER C6

//#define BOOTMAGIC_KEY_SALT KC_NO
//#define BOOTMAGIC_KEY_BOOTLOADER KC_B
/* Mechanical locking support. Use KC_LCAP, KC_LNUM or KC_LSCR instead in keymap */
#define LOCKING_SUPPORT_ENABLE
/* Locking resynchronize hack */
#define LOCKING_RESYNC_ENABLE

/* key combination for magic key command */
//#define IS_COMMAND() ( \/
//  keyboard_report->mods == (MOD_BIT(KC_LSHIFT) | MOD_BIT(KC_RSHIFT)) \/
// )

//(get_mods() == (MOD_BIT(KC_LSHIFT) | MOD_BIT(KC_RSHIFT)))
#define KEYBOARD_LOCK_ENABLE
#define MAGIC_KEY_LOCK L
#define IS_COMMAND() (get_mods() == (MOD_BIT(KC_LSHIFT) | MOD_BIT(KC_RSHIFT)))

#define VIA_EEPROM_LAYOUT_OPTIONS_SIZE 3
#define VIA_EEPROM_CUSTOM_CONFIG_SIZE 1


//#define DYNAMIC_KEYMAP_LAYER_COUNT 4

// EEPROM usage

// TODO: refactor with new user EEPROM code (coming soon)
// #define EEPROM_MAGIC 0x451F
// #define EEPROM_MAGIC_ADDR 32
// Bump this every time we change what we store
// This will automatically reset the EEPROM with defaults
// and avoid loading invalid data from the EEPROM
// #define EEPROM_VERSION 0x08
// #define EEPROM_VERSION_ADDR 34

// Dynamic keymap starts after EEPROM version
// #define DYNAMIC_KEYMAP_EEPROM_ADDR 35
// Dynamic macro starts after dynamic keymaps (35+(4*5*14*2)) = (35+560)
// #define DYNAMIC_KEYMAP_MACRO_EEPROM_ADDR 595
// #define DYNAMIC_KEYMAP_MACRO_EEPROM_SIZE 429
// #define DYNAMIC_KEYMAP_MACRO_COUNT 16
