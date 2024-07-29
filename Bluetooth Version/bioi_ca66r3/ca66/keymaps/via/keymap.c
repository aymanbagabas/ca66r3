#include QMK_KEYBOARD_H
//#include "action_layer.h"
#include "print.h"
#include "../../../ble.h"

#if (defined(RGB_MIDI) | defined(RGBLIGHT_ANIMATIONS)) & defined(RGBLIGHT_ENABLE)
    #include "rgblight.h"
#endif

ble_led_stat ble_led;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  // 0: Base Layer
  [0] = LAYOUT_all(
      KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,   KC_5,   KC_6,   KC_7,   KC_8,   KC_9,    KC_0,    KC_MINS,  KC_EQL,  KC_BSPACE,  KC_DEL, KC_HOME, \
      KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,   KC_T,   KC_Y,   KC_U,   KC_I,   KC_O,    KC_P,    KC_LBRC,  KC_RBRC,           KC_BSLS,  KC_PGUP, \
      KC_CAPSLOCK, KC_A,    KC_S,    KC_D,    KC_F,   KC_G,   KC_H,   KC_J,   KC_K,   KC_L,    KC_SCLN, KC_QUOT,  KC_NONUS_HASH, KC_ENT,   KC_PGDN, \
      KC_LSFT, KC_NONUS_BSLASH,   KC_Z,    KC_X,    KC_C,   KC_V,   KC_B,   KC_N,   KC_M,   KC_COMM, KC_DOT,  KC_SLSH,  KC_RSFT, KC_UP,    KC_END, \
      KC_LCTL, KC_LGUI, KC_LALT,                          KC_SPC,                          KC_RALT, KC_RCTL,   KC_LEFT,  KC_DOWN,  KC_RIGHT),

  // 1: Function Layer
  [1] = LAYOUT_all(
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, RGB_TOG, RGB_MOD, BL_TOGG, BL_STEP, _______, _______, KC_MUTE, KC_VOLD, KC_VOLU, _______, _______, _______, _______, \
      _______,  _______,  _______,                          _______,                                     _______, _______, _______, _______, _______),

  [2] = LAYOUT_all(
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______,  _______,  _______,                          _______,                                     _______, _______, _______, _______, _______),

  [3] = LAYOUT_all(
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, \
      _______,  _______,  _______,                          _______,                                     _______, _______, _______, _______, _______),

};

const rgblight_segment_t PROGMEM my_capslock_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 12, HSV_WHITE}
);

const rgblight_segment_t PROGMEM my_layer1_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 12, HSV_SPRINGGREEN}
);
const rgblight_segment_t PROGMEM my_layer2_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 12, HSV_PURPLE}
);
const rgblight_segment_t PROGMEM my_layer3_layer[] = RGBLIGHT_LAYER_SEGMENTS(
    {0, 12, HSV_MAGENTA}
);

const rgblight_segment_t* const PROGMEM my_rgb_layers[] = RGBLIGHT_LAYERS_LIST(
  my_capslock_layer,
  my_layer1_layer,
  my_layer2_layer,
  my_layer3_layer
);

void keyboard_pre_init_user(void) {
  setPinOutput(BLE_CONTROL_PIN);
  writePinLow(BLE_CONTROL_PIN);

  setPinOutput(F0);
  writePinLow(F0);

  setPinOutput(RGB_UG_CONTROL_PIN);
  writePinLow(RGB_UG_CONTROL_PIN);
}

void keyboard_post_init_user(void) {
  rgblight_layers = my_rgb_layers;
}

layer_state_t layer_state_set_user(layer_state_t state){
  dprint("Get layer_state_set_user()\r\n");
  if(!layer_state_cmp(state, 0) && !rgblight_is_enabled()){
    rgblight_enable_noeeprom();
    writePinLow(RGB_UG_CONTROL_PIN);
  }

  
  if(ble_on && layer_state_cmp(state, 0)){
    rgblight_reload_from_eeprom();
    if(!rgblight_is_enabled()){
        writePinHigh(RGB_UG_CONTROL_PIN);
        rgblight_disable_noeeprom();
      }else{
        writePinLow(RGB_UG_CONTROL_PIN);
        rgblight_enable_noeeprom();
      }
  }
  
  
  rgblight_set_layer_state(1, layer_state_cmp(state, 1));
  rgblight_set_layer_state(2, layer_state_cmp(state, 2));
  rgblight_set_layer_state(3, layer_state_cmp(state, 3));
  return state;
}

void led_set_user(uint8_t usb_led) {
  if (!ble_on)
  {
    rgblight_set_layer_state(0, IS_LED_ON(usb_led, USB_LED_CAPS_LOCK));
    if (IS_LED_ON(usb_led, USB_LED_CAPS_LOCK)) {
      ble_led.caplck = true;
    } else {
      ble_led.caplck = false;
    }
    if (IS_LED_ON(usb_led, USB_LED_CAPS_LOCK)){
      rgblight_enable_noeeprom();
    }
    if (!IS_LED_ON(usb_led, USB_LED_CAPS_LOCK) && layer_state_is(0)){
      rgblight_reload_from_eeprom();
      if(!rgblight_is_enabled()){
        writePinHigh(RGB_UG_CONTROL_PIN);
        rgblight_disable_noeeprom();
      }else{
        writePinLow(RGB_UG_CONTROL_PIN);
        rgblight_enable_noeeprom();
      }
      dprint("Get led_set_user()\r\n");
    }
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
		case KC_F21:
			if (record->event.pressed) {
				//writePinLow(F0);
				//ble_config.init = 0; // Toggles the status
				//eeconfig_update_kb(ble_config.raw);
				module_reset();
        
			} else {

      }
      break;
    case KC_F19:
      if (record->event.pressed) {
				//writePinLow(F0);
				//ble_config.init = 0; // Toggles the status
				//eeconfig_update_kb(ble_config.raw);
				reset_ble_batt();
        
			} else {

      }
      break;
    case KC_F20:
      if (record->event.pressed) {
				//writePinLow(F0);
				//ble_config.init = 0; // Toggles the status
				//eeconfig_update_kb(ble_config.raw);
				update_ble_batt();
        wait_ms(100);
        
			} else {
        
      }
      break;
    case KC_CAPSLOCK:
      if (record->event.pressed) {
				//writePinLow(F0);
				//ble_config.init = 0; // Toggles the status
				//eeconfig_update_kb(ble_config.raw);
        if(ble_on){
          ble_led.caplck = !ble_led.caplck;
          if(ble_led.caplck){
            writePinHigh(F0);
          }else{
            writePinLow(F0);
          }
        }
			}
      break;
    default:
      return true;
	//return process_record_user(keycode, record);
  }
  return true;
}
/**
// Custom Actions
const uint16_t PROGMEM fn_actions[] = {
    [0] = ACTION_LAYER_MOMENTARY(1),  // to Fn overlay
};

// Macros
const macro_t *action_get_macro(keyrecord_t *record, uint8_t id, uint8_t opt) {

  // MACRODOWN only works in this function
  switch(id) {
    case 0:
      if (record->event.pressed) { register_code(KC_RSFT); }
      else { unregister_code(KC_RSFT); }
      break;
  }

  return MACRO_NONE;
};

// Loop
void matrix_scan_user(void) {
  // Empty
  //rgblight_task();
};

**/

void matrix_scan_user(void) {
  // Empty
  //rgblight_task();
}
