#include "ca66.h"
#include "print.h"
#include "quantum.h"
#include "action.h"



//extern inline void g60_caps_led_on(void);
//extern inline void g60_bl_led_on(void);

//extern inline void g60_caps_led_off(void);
//extern inline void g60_bl_led_off(void);

void led_set_kb(uint8_t usb_led) {
	led_set_user(usb_led);
}

void keyboard_pre_init_kb(void){
	keyboard_pre_init_user();
}

void keyboard_post_init_kb(void)
{
	keyboard_post_init_user();
}

layer_state_t layer_state_set_kb(layer_state_t state) { return layer_state_set_user(state); }


bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
	return process_record_user(keycode, record);
}

/*
void matrix_init_kb(void){
	debug_enable = true;
}
*/
